package decad_test

import (
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// extrudePlate is the one-body document most verify tests start from.
func extrudePlate(t *testing.T) (*decad.Document, *decad.Body) {
	t.Helper()
	s, p := plateSketch(t)
	doc := decad.New()
	body, err := doc.Extrude(s, p, decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	require.NoError(t, err)
	return doc, body
}

func TestReportZeroValueIsUnverified(t *testing.T) {
	var report decad.Report
	require.Equal(t, decad.Unverified, report.Status)
	require.Equal(t, "Unverified", report.Status.String())
	require.False(t, report.Trustworthy())

	var bodyReport decad.BodyReport
	require.Equal(t, decad.Unverified, bodyReport.Status)
}

func TestVerifySoundPlate(t *testing.T) {
	doc, body := extrudePlate(t)
	report, err := doc.Verify(t.Context())
	require.NoError(t, err)

	require.NotEqual(t, decad.Unverified, report.Status)
	require.Equal(t, decad.Sound, report.Status)
	require.True(t, report.Trustworthy())
	require.Empty(t, report.Interferences)
	require.Nil(t, report.Clearances, `clearances were not asked for`)
	require.Len(t, report.Bodies, 1)

	br := report.Bodies[0]
	require.Same(t, body, br.Body)
	require.NotEqual(t, decad.Unverified, br.Status)
	require.Equal(t, decad.Sound, br.Status)
	require.True(t, br.Solid)
	require.True(t, br.Watertight)
	require.True(t, br.Manifold)
	require.False(t, br.SelfIntersecting)
	require.Equal(t, 1, br.Lumps)
	require.Equal(t, 0, br.Voids)
	require.Equal(t, decad.Exact, br.Exactness)

	require.NotNil(t, br.Volume)
	require.True(t, br.Volume.Value.Equal(units.CubicMillimeters(60000), 1e-9))
	require.Equal(t, decad.Exact, br.Volume.Exactness)
	require.NotNil(t, br.Centroid)
	require.InDelta(t, 50.0, br.Centroid.Value.X, 1e-9)
	require.InDelta(t, 30.0, br.Centroid.Value.Y, 1e-9)
	require.InDelta(t, 5.0, br.Centroid.Value.Z, 1e-9)
	require.True(t, br.Area.Value.Equal(units.SquareMillimeters(15200), 1e-9))

	require.Nil(t, br.MinWallThickness)
	require.Nil(t, br.Undercuts)
	require.Nil(t, br.MinRadius)
}

func TestVerifyEmptyDocument(t *testing.T) {
	doc := decad.New()
	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.NotEqual(t, decad.Unverified, report.Status)
	require.Equal(t, decad.Sound, report.Status)
	require.True(t, report.Trustworthy())
	require.Empty(t, report.Bodies)
}

func TestVerifyDisjointPairIsSound(t *testing.T) {
	doc, body := extrudePlate(t)
	shift, err := r3.Translation(r3.NewVec(500, 0, 0))
	require.NoError(t, err)
	_, err = body.Placed(shift)
	require.NoError(t, err)

	s, p := plateSketch(t)
	_, err = doc.Extrude(s, p, decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	require.NoError(t, err)

	// Two proven solids whose boxes are far apart: proven disjoint, and the
	// silence is one a Sound report may rest on.
	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.Len(t, report.Bodies, 2)
	require.Equal(t, decad.Sound, report.Status)
	require.True(t, report.Trustworthy())
}

func TestVerifyTouchingBoxesAreDisjoint(t *testing.T) {
	// Boxes sharing only a face have disjoint interiors, and a body's
	// interior lies within its box's interior: proven disjoint.
	doc, body := extrudePlate(t)
	shift, err := r3.Translation(r3.NewVec(100, 0, 0))
	require.NoError(t, err)
	_, err = body.Placed(shift)
	require.NoError(t, err)

	s, p := plateSketch(t)
	_, err = doc.Extrude(s, p, decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	require.NoError(t, err)

	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.Equal(t, decad.Sound, report.Status)
}

func TestVerifyCoincidentPairIsInterfering(t *testing.T) {
	doc, _ := extrudePlate(t)
	s, p := plateSketch(t)
	_, err := doc.Extrude(s, p, decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	require.NoError(t, err)

	// Two coincident analytic plates have an exact set-identity certificate,
	// so Verify reuses the first body's volume without calling the mesh
	// boolean or consuming either operand.
	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.Equal(t, decad.Interfering, report.Status)
	require.False(t, report.Trustworthy())
	require.Len(t, report.Interferences, 1)
	require.Equal(t, 60000.0, report.Interferences[0].Volume.Value.Base())
	for _, br := range report.Bodies {
		require.Equal(t, decad.Sound, br.Status, `the bodies themselves remain sound`)
	}
}

func TestVerifyAskedSurveysAreAnswered(t *testing.T) {
	// The analytic surveys answer each asked question outright on this
	// evaluator's bodies: the plate's wall is its 10 mm slab, no face
	// opposes a +z pull, and no concave feature exists — every answer
	// proven, so the report is Sound, not the old asked-but-unanswered
	// Suspect.
	testcases := []struct {
		Name   string
		Option decad.VerifyOption
		Check  func(t *testing.T, br *decad.BodyReport)
	}{
		{
			Name:   "min wall thickness",
			Option: decad.WithMinWallThickness(units.Millimeters(1)),
			Check: func(t *testing.T, br *decad.BodyReport) {
				require.NotNil(t, br.MinWallThickness)
				require.True(t, br.MinWallThickness.Value.Equal(units.Millimeters(10), 1e-9))
			},
		},
		{
			Name:   "pull direction",
			Option: decad.WithPullDirection(r3.NewVec(0, 0, 1)),
			Check: func(t *testing.T, br *decad.BodyReport) {
				require.NotNil(t, br.Undercuts)
				require.Empty(t, br.Undercuts)
			},
		},
		{
			Name:   "min radius",
			Option: decad.WithMinRadius(),
			Check: func(t *testing.T, br *decad.BodyReport) {
				require.Nil(t, br.MinRadius, `an all-convex plate has no concave feature`)
			},
		},
	}
	for _, tc := range testcases {
		t.Run(tc.Name, func(t *testing.T) {
			doc, _ := extrudePlate(t)
			report, err := doc.Verify(t.Context(), tc.Option)
			require.NoError(t, err)
			require.Equal(t, decad.Sound, report.Status)
			require.True(t, report.Trustworthy())
			br := report.Bodies[0]
			require.Equal(t, decad.Sound, br.Status)
			require.True(t, br.Solid, `validity is decided before any survey is read`)
			tc.Check(t, br)
		})
	}
}

func TestVerifyClearancesMeasureBoxProvenPair(t *testing.T) {
	doc, body := extrudePlate(t)
	shift, err := r3.Translation(r3.NewVec(500, 0, 0))
	require.NoError(t, err)
	_, err = body.Placed(shift)
	require.NoError(t, err)
	s, p := plateSketch(t)
	_, err = doc.Extrude(s, p, decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	require.NoError(t, err)

	// A box-proven pair is already partition-decided, but its row still
	// needs the kernel: the box distance is a lower bound, not a minimum
	// (clearance design §7). The facing side faces sit 400 mm apart.
	report, err := doc.Verify(t.Context(), decad.WithClearances())
	require.NoError(t, err)
	require.Equal(t, decad.Sound, report.Status)
	require.True(t, report.Trustworthy())
	require.Len(t, report.Clearances, 1)
	row := report.Clearances[0]
	require.Equal(t, decad.Exact, row.Gap.Exactness)
	require.True(t, row.Gap.Value.Equal(units.Millimeters(400), 1e-9), `got %s`, row.Gap.Value)
	require.Equal(t, 0.0, row.Gap.Bound.Mag())
}

func TestVerifyClearancesAskedWithNoPairIsSound(t *testing.T) {
	// One body poses no pair, so the asked clearance list is answered in
	// full by the empty list.
	doc, _ := extrudePlate(t)
	report, err := doc.Verify(t.Context(), decad.WithClearances())
	require.NoError(t, err)
	require.Equal(t, decad.Sound, report.Status)
	require.NotNil(t, report.Clearances)
	require.Empty(t, report.Clearances)
}

func TestVerifyOptionValidation(t *testing.T) {
	doc, _ := extrudePlate(t)
	testcases := []struct {
		Name   string
		Option decad.VerifyOption
		Err    error
	}{
		{Name: "tolerance wrong kind", Option: decad.WithTolerance(units.Millimeters(1)), Err: decad.ErrUnitKind},
		{Name: "tolerance negative", Option: decad.WithTolerance(units.Scalar(-1e-3)), Err: decad.ErrNegativeMagnitude},
		{Name: "tolerance NaN", Option: decad.WithTolerance(units.Scalar(math.NaN())), Err: decad.ErrNotFinite},
		{Name: "tolerance infinite", Option: decad.WithTolerance(units.Scalar(math.Inf(1))), Err: decad.ErrNotFinite},
		{Name: "zero wall tool", Option: decad.WithMinWallThickness(units.Millimeters(0)), Err: decad.ErrDegenerate},
		{Name: "wall tool wrong kind", Option: decad.WithMinWallThickness(units.Degrees(1)), Err: decad.ErrUnitKind},
		{Name: "wall tool negative", Option: decad.WithMinWallThickness(units.Millimeters(-1)), Err: decad.ErrNegativeMagnitude},
		{Name: "allowance at 90 degrees", Option: decad.WithMinWallThickness(units.Millimeters(1), decad.WithDraftAllowance(units.Degrees(90))), Err: decad.ErrDegenerate},
		{Name: "allowance wrong kind", Option: decad.WithMinWallThickness(units.Millimeters(1), decad.WithDraftAllowance(units.Millimeters(5))), Err: decad.ErrUnitKind},
		{Name: "allowance negative", Option: decad.WithMinWallThickness(units.Millimeters(1), decad.WithDraftAllowance(units.Degrees(-1))), Err: decad.ErrNegativeMagnitude},
		{Name: "nil wall option", Option: decad.WithMinWallThickness(units.Millimeters(1), nil), Err: decad.ErrDegenerate},
		{Name: "zero pull direction", Option: decad.WithPullDirection(r3.NewVec(0, 0, 0)), Err: decad.ErrDegenerate},
		{Name: "non-finite pull direction", Option: decad.WithPullDirection(r3.NewVec(math.NaN(), 0, 1)), Err: decad.ErrNotFinite},
		{Name: "nil option", Option: nil, Err: decad.ErrDegenerate},
	}
	for _, tc := range testcases {
		t.Run(tc.Name, func(t *testing.T) {
			_, err := doc.Verify(t.Context(), tc.Option)
			require.ErrorIs(t, err, tc.Err)
		})
	}
}

func TestVerifyZeroAllowanceIsLegal(t *testing.T) {
	// A zero allowance is the strictest legal reading (verification §2):
	// exact opposition only — which the plate's parallel skins are, so the
	// 10 mm reading still stands and meets the 1 mm tool.
	doc, _ := extrudePlate(t)
	report, err := doc.Verify(t.Context(), decad.WithMinWallThickness(units.Millimeters(1), decad.WithDraftAllowance(units.Degrees(0))))
	require.NoError(t, err)
	require.Equal(t, decad.Sound, report.Status)
	br := report.Bodies[0]
	require.NotNil(t, br.MinWallThickness)
	require.True(t, br.MinWallThickness.Value.Equal(units.Millimeters(10), 1e-9))
}

func TestVerifyIsNonMutating(t *testing.T) {
	doc, _ := extrudePlate(t)
	before := doc.Recipe()
	_, err := doc.Verify(t.Context(), decad.WithMinRadius(), decad.WithClearances())
	require.NoError(t, err)
	require.Equal(t, before, doc.Recipe())
	require.Len(t, doc.Bodies(), 1)
}

func TestVerifyCoversLiveBodiesOnly(t *testing.T) {
	doc, body := extrudePlate(t)
	shift, err := r3.Translation(r3.NewVec(500, 0, 0))
	require.NoError(t, err)
	placed, err := body.Placed(shift)
	require.NoError(t, err)

	// The original was retired by the placement: it stays readable but it
	// is no longer part of the model, so the report does not cover it.
	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.Len(t, report.Bodies, 1)
	require.Same(t, placed, report.Bodies[0].Body)
}

func TestVerifyContextCancellation(t *testing.T) {
	doc, _ := extrudePlate(t)
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	report, err := doc.Verify(ctx)
	require.ErrorIs(t, err, context.Canceled)
	require.Nil(t, report)
}

func TestVerifyContextCancellationOnEmptyDocument(t *testing.T) {
	doc := decad.New()
	ctx, cancel := context.WithCancel(t.Context())
	cancel()

	report, err := doc.Verify(ctx)
	require.NoError(t, err)
	require.NotNil(t, report)
	require.Equal(t, decad.Sound, report.Status)
	require.True(t, report.Trustworthy())
	require.Empty(t, report.Bodies)
}

func TestVerifyNilDocument(t *testing.T) {
	var doc *decad.Document
	_, err := doc.Verify(t.Context())
	require.ErrorIs(t, err, decad.ErrDegenerate)
}

func TestVerifyPlateWithHole(t *testing.T) {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 0, 100, 60)
	s.Fix(rect.A)
	s.CreateCircle(s.CreatePoint(70, 30), 10)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	var prof *sketch.Profile
	for _, p := range s.Profiles() {
		if len(p.Holes) == 1 {
			prof = p
		}
	}
	require.NotNil(t, prof)

	doc := decad.New()
	_, err = doc.Extrude(s, prof, decad.Distance{D: units.Millimeters(8), Dir: decad.Along})
	require.NoError(t, err)

	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.Equal(t, decad.Suspect, report.Status)
	br := report.Bodies[0]
	require.True(t, br.Solid)
	require.Equal(t, 1, br.Lumps)
	require.Equal(t, 0, br.Voids, `a through hole opens to the outside; it walls off no cavity`)
}

func TestStatusString(t *testing.T) {
	require.Equal(t, "Sound", decad.Sound.String())
	require.Equal(t, "Suspect", decad.Suspect.String())
	require.Equal(t, "Violating", decad.Violating.String())
	require.Equal(t, "Interfering", decad.Interfering.String())
	require.Equal(t, "Unsound", decad.Unsound.String())
	require.Equal(t, "Status(99)", decad.Status(99).String())
}
