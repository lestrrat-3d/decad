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
	t.Parallel()
	var report decad.Report
	require.Equal(t, decad.Unverified, report.Status)
	require.Equal(t, "Unverified", report.Status.String())
	require.False(t, report.Trustworthy())

	var bodyReport decad.BodyReport
	require.Equal(t, decad.Unverified, bodyReport.Status)
}

func TestVerifySoundPlate(t *testing.T) {
	t.Parallel()
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
	t.Parallel()
	doc := decad.New()
	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.NotEqual(t, decad.Unverified, report.Status)
	require.Equal(t, decad.Sound, report.Status)
	require.True(t, report.Trustworthy())
	require.Empty(t, report.Bodies)
}

func TestVerifyDisjointPairIsSound(t *testing.T) {
	t.Parallel()
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
	t.Parallel()
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
	t.Parallel()
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
	t.Parallel()
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
	t.Parallel()
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
	t.Parallel()
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
	t.Parallel()
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
	t.Parallel()
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
	t.Parallel()
	doc, _ := extrudePlate(t)
	before := doc.Recipe()
	_, err := doc.Verify(t.Context(), decad.WithMinRadius(), decad.WithClearances())
	require.NoError(t, err)
	require.Equal(t, before, doc.Recipe())
	require.Len(t, doc.Bodies(), 1)
}

func TestVerifyCoversLiveBodiesOnly(t *testing.T) {
	t.Parallel()
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
	t.Parallel()
	doc, _ := extrudePlate(t)
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	report, err := doc.Verify(ctx)
	require.ErrorIs(t, err, context.Canceled)
	require.Nil(t, report)
}

func TestVerifyContextCancellationOnEmptyDocument(t *testing.T) {
	t.Parallel()
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
	t.Parallel()
	var doc *decad.Document
	_, err := doc.Verify(t.Context())
	require.ErrorIs(t, err, decad.ErrDegenerate)
}

func TestVerifyPlateWithHole(t *testing.T) {
	t.Parallel()
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
	require.Equal(t, decad.Sound, report.Status)
	br := report.Bodies[0]
	require.True(t, br.Solid)
	require.Equal(t, 1, br.Lumps)
	require.Equal(t, 0, br.Voids, `a through hole opens to the outside; it walls off no cavity`)
}

func TestStatusString(t *testing.T) {
	t.Parallel()
	require.Equal(t, "Sound", decad.Sound.String())
	require.Equal(t, "Suspect", decad.Suspect.String())
	require.Equal(t, "Violating", decad.Violating.String())
	require.Equal(t, "Interfering", decad.Interfering.String())
	require.Equal(t, "Unsound", decad.Unsound.String())
	require.Equal(t, "Status(99)", decad.Status(99).String())
}

// worstDiagnosticStatus is verification §1.1's aggregate: the worst
// Diagnostic.Status in the slice, Sound when it is empty.
func worstDiagnosticStatus(diags []decad.Diagnostic) decad.Status {
	worst := decad.Sound
	for _, d := range diags {
		if d.Status > worst {
			worst = d.Status
		}
	}
	return worst
}

// requireDiagnosticInvariants asserts verification §1.1's contract holds for a
// whole report: the slice is empty EXACTLY when Sound, Status is the worst
// diagnostic status, and every entry keys exactly one Observed* form off its
// Reading with a body-xor-pair owner.
func requireDiagnosticInvariants(t *testing.T, report *decad.Report) {
	t.Helper()

	if report.Status == decad.Sound {
		require.Empty(t, report.Diagnostics, `a Sound report carries no diagnostics`)
	} else {
		require.NotEmpty(t, report.Diagnostics, `a non-Sound report itemizes its reasons`)
	}
	require.Equal(t, report.Status, worstDiagnosticStatus(report.Diagnostics),
		`Status is the worst diagnostic status`)

	for _, d := range report.Diagnostics {
		// A diagnostic concerns a body or a pair, never both and never neither.
		require.True(t, (d.Body != nil) != (d.Pair != nil),
			`%s names exactly one of Body / Pair`, d.Code)

		// Exactly one Observed* form is set, keyed by Reading (invariant #2).
		switch d.Reading {
		case decad.ReadingNone:
			require.Nil(t, d.Observed, `%s: ReadingNone carries no scalar`, d.Code)
			require.Nil(t, d.ObservedVec, `%s: ReadingNone carries no vector`, d.Code)
			require.Nil(t, d.ObservedBox, `%s: ReadingNone carries no box`, d.Code)
		case decad.ReadingCentroid:
			require.NotNil(t, d.ObservedVec, `%s: a centroid rides ObservedVec`, d.Code)
			require.Nil(t, d.Observed)
			require.Nil(t, d.ObservedBox)
		case decad.ReadingBounds:
			require.NotNil(t, d.ObservedBox, `%s: a bounds box rides ObservedBox`, d.Code)
			require.Nil(t, d.Observed)
			require.Nil(t, d.ObservedVec)
		default:
			require.NotNil(t, d.Observed, `%s: a scalar rides Observed`, d.Code)
			require.Nil(t, d.ObservedVec)
			require.Nil(t, d.ObservedBox)
		}

		require.NotEmpty(t, d.Message, `%s carries a human-readable message`, d.Code)
		require.NotEqual(t, d.Code.String(), d.Message, `the message is never the branch key`)

		// Every shipped payload class forms its tolerance reference
		// (verification design §3), so a DiagMeasurementBeyondTolerance carries
		// the threshold the reading was judged against. What is left without one
		// is not a payload class but a degenerate reading — a witness set with
		// no usable maximum, or a displacement wide enough to shrink that
		// maximum to nothing — and no body reaching this helper is either.
		if d.Code == decad.DiagMeasurementBeyondTolerance {
			require.NotNil(t, d.Required, `%s: this body's readings are all judged against a reference`, d.Code)
		}
	}
}

func TestDiagnosticReadingKindTokens(t *testing.T) {
	t.Parallel()
	// The pinned lower-snake tokens (verification §1.1) — the identity a caller
	// branches on, never the iota value.
	require.Equal(t, "none", decad.ReadingNone.String())
	require.Equal(t, "area", decad.ReadingArea.String())
	require.Equal(t, "bounds", decad.ReadingBounds.String())
	require.Equal(t, "volume", decad.ReadingVolume.String())
	require.Equal(t, "centroid", decad.ReadingCentroid.String())
	require.Equal(t, "wall", decad.ReadingWall.String())
	require.Equal(t, "min_radius", decad.ReadingMinRadius.String())
	require.Equal(t, "overlap_volume", decad.ReadingOverlapVolume.String())
	require.Equal(t, "gap", decad.ReadingGap.String())
	// An out-of-range value renders reading(<n>), never a panic.
	require.Equal(t, "reading(42)", decad.ReadingKind(42).String())
}

func TestDiagnosticCodeTokens(t *testing.T) {
	t.Parallel()
	require.Equal(t, "measurement_beyond_tolerance", decad.DiagMeasurementBeyondTolerance.String())
	require.Equal(t, "undecided_validity", decad.DiagUndecidedValidity.String())
	require.Equal(t, "invalid_body", decad.DiagInvalidBody.String())
	require.Equal(t, "wall_too_thin", decad.DiagWallTooThin.String())
	require.Equal(t, "undercut", decad.DiagUndercut.String())
	require.Equal(t, "undecided_wall", decad.DiagUndecidedWall.String())
	require.Equal(t, "undecided_undercut", decad.DiagUndecidedUndercut.String())
	require.Equal(t, "undecided_min_radius", decad.DiagUndecidedMinRadius.String())
	require.Equal(t, "interference", decad.DiagInterference.String())
	require.Equal(t, "undecided_pair", decad.DiagUndecidedPair.String())
	require.Equal(t, "unsupported_pair", decad.DiagUnsupportedPair.String())
	require.Equal(t, "undecided_clearance", decad.DiagUndecidedClearance.String())
	require.Equal(t, "undecided_interference", decad.DiagUndecidedInterference.String())
	require.Equal(t, "unsupported_pair_payload", decad.DiagUnsupportedPairPayload.String())
	require.Equal(t, "unsupported_pair_contact", decad.DiagUnsupportedPairContact.String())
	require.Equal(t, "unsupported_pair_pipeline", decad.DiagUnsupportedPairPipeline.String())
	require.Equal(t, "unsupported_survey_payload", decad.DiagUnsupportedSurveyPayload.String())
	// An out-of-range value renders diagnostic(<n>), never a panic.
	require.Equal(t, "diagnostic(99)", decad.DiagnosticCode(99).String())
}

func TestVerifyDiagnosticsEmptyWhenSound(t *testing.T) {
	t.Parallel()
	doc, _ := extrudePlate(t)
	report, err := doc.Verify(t.Context())
	require.NoError(t, err)

	require.Equal(t, decad.Sound, report.Status)
	require.Empty(t, report.Diagnostics, `a Sound report is empty exactly`)
	requireDiagnosticInvariants(t, report)
}

// findDiagnostic returns the first diagnostic with the given code.
func findDiagnostic(diags []decad.Diagnostic, code decad.DiagnosticCode) (decad.Diagnostic, bool) {
	for _, d := range diags {
		if d.Code == code {
			return d, true
		}
	}
	return decad.Diagnostic{}, false
}

func TestVerifyDiagnosticsWallTooThin(t *testing.T) {
	t.Parallel()
	// The 10×10×0.5 mm plate against a 1 mm tool: a proven-thin wall.
	doc := rectPrism(t, 10, 10, 0.5)
	tool := units.Millimeters(1)
	report, err := doc.Verify(t.Context(), decad.WithMinWallThickness(tool))
	require.NoError(t, err)

	require.Equal(t, decad.Violating, report.Status)
	requireDiagnosticInvariants(t, report)

	d, ok := findDiagnostic(report.Diagnostics, decad.DiagWallTooThin)
	require.True(t, ok, `a thin wall emits DiagWallTooThin`)
	require.Equal(t, decad.Violating, d.Status)
	require.Equal(t, decad.ReadingWall, d.Reading)
	require.Same(t, report.Bodies[0].Body, d.Body)
	require.Nil(t, d.Pair)
	require.NotNil(t, d.Observed, `the wall reading rides Observed`)
	require.InDelta(t, 0.5, d.Observed.Value.Base(), 1e-9)
	require.NotNil(t, d.Required, `the tool rides Required`)
	require.True(t, d.Required.Equal(tool, 1e-9), `Required is the tool`)
}

func TestVerifyDiagnosticsUndercut(t *testing.T) {
	t.Parallel()
	// A prism under a tilted pull: two proven undercut faces, one DiagUndercut.
	doc := rectPrism(t, 100, 60, 10)
	report, err := doc.Verify(t.Context(), decad.WithPullDirection(r3.NewVec(1, 0, 1)))
	require.NoError(t, err)

	require.Equal(t, decad.Violating, report.Status)
	requireDiagnosticInvariants(t, report)

	d, ok := findDiagnostic(report.Diagnostics, decad.DiagUndercut)
	require.True(t, ok, `a proven undercut emits DiagUndercut`)
	require.Equal(t, decad.Violating, d.Status)
	require.Equal(t, decad.ReadingNone, d.Reading, `an undercut is a predicate, not a scalar`)
	require.Same(t, report.Bodies[0].Body, d.Body)
	require.NotEmpty(t, report.Bodies[0].Undercuts, `the faces are listed on the BodyReport`)
}

func TestVerifyDiagnosticsUnsupportedFacetedSurveys(t *testing.T) {
	t.Parallel()
	doc, body := allPlanarBoolean(t, 1)
	report, err := doc.Verify(t.Context(),
		decad.WithMinWallThickness(units.Millimeters(1)),
		decad.WithPullDirection(r3.NewVec(0, 0, 1)),
		decad.WithMinRadius())
	require.NoError(t, err)

	require.Equal(t, decad.Suspect, report.Status)
	requireDiagnosticInvariants(t, report)
	require.Len(t, report.Diagnostics, 3, `each staged faceted survey names its own refusal`)
	for _, diagnostic := range report.Diagnostics {
		require.Equal(t, decad.DiagUnsupportedSurveyPayload, diagnostic.Code)
		require.Equal(t, decad.Suspect, diagnostic.Status)
		require.Equal(t, decad.ReadingNone, diagnostic.Reading)
		require.Same(t, body, diagnostic.Body)
		require.Contains(t, diagnostic.Message, "facetedPayload")
		require.Contains(t, diagnostic.Message, "use an analytic body")
	}
	require.Contains(t, report.Diagnostics[0].Message, "wall survey")
	require.Contains(t, report.Diagnostics[1].Message, "pull survey")
	require.Contains(t, report.Diagnostics[2].Message, "concave-radius survey")
}

func TestVerifyDiagnosticsInterference(t *testing.T) {
	t.Parallel()
	// Two coincident analytic plates overlap by an exact set-identity volume.
	doc, _ := extrudePlate(t)
	s, p := plateSketch(t)
	_, err := doc.Extrude(s, p, decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	require.NoError(t, err)

	report, err := doc.Verify(t.Context())
	require.NoError(t, err)

	require.Equal(t, decad.Interfering, report.Status)
	requireDiagnosticInvariants(t, report)

	d, ok := findDiagnostic(report.Diagnostics, decad.DiagInterference)
	require.True(t, ok, `a proven overlap emits DiagInterference`)
	require.Equal(t, decad.Interfering, d.Status)
	require.Equal(t, decad.ReadingOverlapVolume, d.Reading)
	require.Nil(t, d.Body)
	require.NotNil(t, d.Pair, `an interference names its pair`)
	require.NotNil(t, d.Observed, `the overlap volume rides Observed`)
	require.InDelta(t, 60000.0, d.Observed.Value.Base(), 1e-6)
	require.Len(t, report.Interferences, 1, `the diagnostic mirrors the Interference row`)
}

func TestVerifyDiagnosticsUnsupportedPairStagedContact(t *testing.T) {
	t.Parallel()
	// Two 10×10×10 boxes, the second translated to (0,5,5): they overlap with
	// positive volume while sharing coplanar side faces. The read-only intersect
	// stages the face-on-face contact (booleanExpectedContact). Per verification
	// §1.1 a staged boolean contact is a DiagUnsupportedPairContact, not a
	// payload, pipeline, or undecided-partition diagnostic; the Status stays
	// Suspect.
	ws := sketch.NewWorld()
	s, err := ws.CreateSketch(ws.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 0, 10, 10)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	p := s.Profiles()[0]

	doc := decad.New()
	_, err = doc.Extrude(s, p, decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	require.NoError(t, err)
	box2, err := doc.Extrude(s, p, decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	require.NoError(t, err)

	shift, err := r3.Translation(r3.NewVec(0, 5, 5))
	require.NoError(t, err)
	_, err = box2.Placed(shift)
	require.NoError(t, err)

	report, err := doc.Verify(t.Context())
	require.NoError(t, err)

	require.Equal(t, decad.Suspect, report.Status)
	requireDiagnosticInvariants(t, report)

	d, ok := findDiagnostic(report.Diagnostics, decad.DiagUnsupportedPairContact)
	require.True(t, ok, `a staged boolean contact emits its cause-specific code`)
	require.Equal(t, decad.Suspect, d.Status)
	require.Equal(t, decad.ReadingNone, d.Reading, `an unsupported pair names no reading`)
	require.Nil(t, d.Body)
	require.NotNil(t, d.Pair, `an unsupported pair names its pair`)
	require.Equal(t,
		`the pair reaches a contact or near-contact that the read-only intersection cannot classify, and the operands share a face plane; offset the operands so no face plane is shared, or adjust the geometry to create clear separation or deeper overlap, or wait for contact support`,
		d.Message,
		`the contact diagnostic names contact and near-contact cases, names the shared-plane cause, and gives accurate corrective guidance`)

	_, undecided := findDiagnostic(report.Diagnostics, decad.DiagUndecidedPair)
	require.False(t, undecided, `a staged contact is not an undecided partition`)
	legacy, broad := findDiagnostic(report.Diagnostics, decad.DiagUnsupportedPair)
	require.True(t, broad, `a staged contact preserves the broad compatibility code`)
	require.Equal(t, decad.Suspect, legacy.Status)
	require.Equal(t, decad.ReadingNone, legacy.Reading)
	require.NotNil(t, legacy.Pair)
	_, payload := findDiagnostic(report.Diagnostics, decad.DiagUnsupportedPairPayload)
	require.False(t, payload, `a contact refusal is not a payload capability limit`)
	_, pipeline := findDiagnostic(report.Diagnostics, decad.DiagUnsupportedPairPipeline)
	require.False(t, pipeline, `a contact refusal is not an in-pipeline reach`)
}

// TestVerifyDiagnosticsAdmittedCoplanarPrismPairHasNoContactDiagnostic is the
// diagnostic-side pin for
// TestVerifyAdmittedCoplanarPrismPairResolvesAnalytically's fixture
// (interference_test.go): a container box and a taller, footprint-nested box
// sharing the container's own coplanar base plane. Before
// docs/prism-boolean-design.md §14 PR4 this exact pair staged a boolean
// contact (DiagUnsupportedPairContact / DiagUnsupportedPair, Suspect) because
// measuredInterference never reached the analytic OpIntersect dispatch. Now
// it resolves analytically, so neither contact diagnostic fires and the
// report reads Interfering with a DiagInterference row instead.
func TestVerifyDiagnosticsAdmittedCoplanarPrismPairHasNoContactDiagnostic(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	boxBody(t, doc, 0, 0, 20, 20, 10)
	boxBody(t, doc, 5, 5, 9, 10, 15)

	report, err := doc.Verify(t.Context())
	require.NoError(t, err)

	require.Equal(t, decad.Interfering, report.Status)
	requireDiagnosticInvariants(t, report)

	_, contact := findDiagnostic(report.Diagnostics, decad.DiagUnsupportedPairContact)
	require.False(t, contact, `an admitted coplanar pair no longer stages a boolean contact`)
	_, broad := findDiagnostic(report.Diagnostics, decad.DiagUnsupportedPair)
	require.False(t, broad, `an admitted coplanar pair no longer trips the broad compatibility code either`)

	d, ok := findDiagnostic(report.Diagnostics, decad.DiagInterference)
	require.True(t, ok, `the analytic path proves a positive overlap`)
	require.Equal(t, decad.Interfering, d.Status)
	require.Equal(t, decad.ReadingOverlapVolume, d.Reading)
	require.NotNil(t, d.Observed)
	require.InDelta(t, 200.0, d.Observed.Value.Base(), 1e-9)
}

func TestVerifyDiagnosticsProximityRefusalIsContact(t *testing.T) {
	t.Parallel()
	// The cylinder's analytic surface overlaps the plate's x = 20 face by only
	// 0.0005 mm. The pre-tessellation proximity gate refuses the undecidable
	// chord result, so Verify must retain the contact cause for its diagnostic.
	doc := decad.New()
	boxBody(t, doc, 0, 0, 20, 20, 8)
	translated(t, diskBody(t, doc, 29.9995, 10, 10), 0, 0, -6)

	report, err := doc.Verify(t.Context())
	require.NoError(t, err)

	require.Equal(t, decad.Suspect, report.Status)
	requireDiagnosticInvariants(t, report)

	d, ok := findDiagnostic(report.Diagnostics, decad.DiagUnsupportedPairContact)
	require.True(t, ok, `a proximity refusal emits the contact diagnostic`)
	require.Equal(t, decad.Suspect, d.Status)
	require.Equal(t, decad.ReadingNone, d.Reading)
	require.Nil(t, d.Body)
	require.NotNil(t, d.Pair)
	require.Equal(t,
		`the pair reaches a contact or near-contact that the read-only intersection cannot classify; adjust the geometry to create clear separation or deeper overlap, or wait for contact support`,
		d.Message,
		`the proximity diagnostic does not describe a near-contact as actual contact`)
	require.NotContains(t, d.Message, "share a face plane",
		`the plate and disk share no face plane, so the message must not claim one`)

	_, pipeline := findDiagnostic(report.Diagnostics, decad.DiagUnsupportedPairPipeline)
	require.False(t, pipeline, `a proximity refusal is not an in-pipeline reach`)
}

func TestVerifyDiagnosticsBeyondTolerance(t *testing.T) {
	t.Parallel()
	// A faceted union carries an Approximate reading; a tolerance just below the
	// proven boundary pushes its bound past the gate.
	doc, body := allPlanarBoolean(t, 0.3)
	required := requiredBodyTolerance(t, body)
	require.Positive(t, required)
	report, err := doc.Verify(t.Context(), decad.WithTolerance(units.Scalar(math.Nextafter(required, 0))))
	require.NoError(t, err)

	require.Equal(t, decad.Suspect, report.Status)
	requireDiagnosticInvariants(t, report)

	d, ok := findDiagnostic(report.Diagnostics, decad.DiagMeasurementBeyondTolerance)
	require.True(t, ok, `a coarse reading emits DiagMeasurementBeyondTolerance`)
	require.Equal(t, decad.Suspect, d.Status)
	require.NotNil(t, d.Body, `a body reading names its body`)
	require.Nil(t, d.Pair)
	require.NotNil(t, d.Observed)
	require.NotNil(t, d.Required, `Required is rel*Ref, the largest passing bound`)
	require.Equal(t, d.Observed.Value.Kind(), d.Required.Kind(),
		`Required shares the reading's Kind`)
	require.Positive(t, d.Required.Base())
}

func TestVerifyDiagnosticsUndecidedClearance(t *testing.T) {
	t.Parallel()
	// Two box-disjoint cups: the box test proves the pair apart, but the
	// clearance kernel stages the cup payload, so the requested gap is
	// unmeasured — DiagUndecidedClearance, never an unsupported-pair code.
	doc, box1 := shellBox(t)
	cup1, err := box1.Shell(topCap(box1), units.Millimeters(5))
	require.NoError(t, err)
	far, err := r3.Translation(r3.NewVec(500, 0, 0))
	require.NoError(t, err)
	_, err = cup1.Placed(far)
	require.NoError(t, err)
	s2, p2 := plateSketch(t)
	box2, err := doc.Extrude(s2, p2, decad.Distance{D: units.Millimeters(shellBoxHeight), Dir: decad.Along})
	require.NoError(t, err)
	_, err = box2.Shell(topCap(box2), units.Millimeters(5))
	require.NoError(t, err)

	// No clearance asked: the box test alone proves the pair apart, so neither
	// cup's own bounded mass results reach the kernel and both verify Sound.
	quiet, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.Equal(t, decad.Sound, quiet.Status)
	requireDiagnosticInvariants(t, quiet)

	// WithClearances invokes the staged kernel: the gap is unmeasured.
	report, err := doc.Verify(t.Context(), decad.WithClearances())
	require.NoError(t, err)
	require.Equal(t, decad.Suspect, report.Status)
	requireDiagnosticInvariants(t, report)

	d, ok := findDiagnostic(report.Diagnostics, decad.DiagUndecidedClearance)
	require.True(t, ok, `a box-disjoint pair with an unmeasured gap emits DiagUndecidedClearance`)
	require.Equal(t, decad.Suspect, d.Status)
	require.Equal(t, decad.ReadingNone, d.Reading)
	require.NotNil(t, d.Pair)
	require.Nil(t, d.Body)
	require.Empty(t, report.Clearances, `no Clearance row is fabricated`)
}

// TestVerifyDiagnosticsSectionDeltaPrismReadsItsOwnGateDiameter pins
// verification design §3's arm for a prismPayload whose own sectionDelta is
// nonzero (docs/prism-boolean-design.md §7's re-expressed section — here the
// analytic Union of a placed prism pair). The clearance kernel's exact carrier
// model refuses that payload (clearance_geom.go's addPrismFaces), so the
// reference comes from fallbackGateDiameter reading the body's OWN recorded
// section and shrinking the witness maximum by the displacement each witness
// carries. Two things follow, and this test asserts both: bounds this tight
// pass the default gate instead of raising a reference-less Suspect, and a
// reading strict enough to fail is judged against a real threshold.
//
// Operand B sits strictly inside operand A, so the union is operand A itself —
// a 10 mm cube whose true diameter is sqrt(300). The recovered reference is
// that diameter, rounded toward zero by the shared reader and then shrunk by
// twice the section displacement, which is the direction §3 requires:
// understating tightens the gate, overstating loosens it.
func TestVerifyDiagnosticsSectionDeltaPrismReadsItsOwnGateDiameter(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	a := boxBody(t, doc, 0, 0, 10, 10, 10)
	const shift = 1e3
	lo, hi := 2-shift, 8-shift
	b := placedFar(t, boxBody(t, doc, lo, 2, hi, 8, 10), shift)

	got, err := decad.Union(a, b)
	require.NoError(t, err)
	require.False(t, anyFaceIsFaceted(got), "the analytic reduction must own this pair")

	vol, err := got.Volume()
	require.NoError(t, err)
	require.Equal(t, decad.Approximate, vol.Exactness)
	require.Positive(t, vol.Bound.Base(), "a zero bound would pass the gate without consulting a reference")

	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.Equal(t, decad.Sound, report.Status,
		"a displaced section this tightly bounded is judged, not abandoned")
	require.Empty(t, report.Diagnostics)

	// A tolerance no proven bound can meet forces every reading through the
	// gate, so each diagnostic has to name the threshold it missed.
	const rel = 1e-18
	strict, err := doc.Verify(t.Context(), decad.WithTolerance(units.Scalar(rel)))
	require.NoError(t, err)
	require.Equal(t, decad.Suspect, strict.Status)

	var bounds *decad.Diagnostic
	for i, d := range strict.Diagnostics {
		if d.Code != decad.DiagMeasurementBeyondTolerance {
			continue
		}
		require.Equal(t, decad.Suspect, d.Status)
		require.NotNil(t, d.Body)
		require.Nil(t, d.Pair)
		require.NotNil(t, d.Required, "%s: the reading is judged against a reference", d.Reading)
		if d.Reading == decad.ReadingBounds {
			bounds = &strict.Diagnostics[i]
		}
	}
	require.NotNil(t, bounds, "the bounds reading is judged against the body diameter itself")

	// The bounds reading's reference IS the gate diameter, so Required/rel
	// recovers it.
	diameter := bounds.Required.Base() / rel
	trueDiameter := math.Sqrt(300)
	require.Positive(t, diameter)
	require.LessOrEqual(t, diameter, trueDiameter,
		"the reference never overstates the body's own diameter")
	require.InDelta(t, trueDiameter, diameter, 1e-6,
		"the shrink is the section displacement, not a coarse envelope")
}

func allPlanarBoolean(t *testing.T, scale float64) (*decad.Document, *decad.Body) {
	t.Helper()
	doc := decad.New()
	a := boxBody(t, doc, 0, 0, 10*scale, 10*scale, 10*scale)
	b := translated(t, boxBody(t, doc, 0, 0, 10*scale, 10*scale, 10*scale), 5*scale, 5*scale, 5*scale)
	body, err := decad.Union(a, b)
	require.NoError(t, err)
	return doc, body
}

func TestVerifyAllPlanarBooleanGatesApproximateArea(t *testing.T) {
	t.Parallel()
	doc, body := allPlanarBoolean(t, 1)

	volume, err := body.Volume()
	require.NoError(t, err)
	require.Equal(t, decad.Exact, volume.Exactness)
	area, err := body.Area()
	require.NoError(t, err)
	require.Equal(t, decad.Approximate, area.Exactness)
	require.Positive(t, area.Bound.Base())

	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.Equal(t, decad.Approximate, report.Bodies[0].Exactness)
	require.Equal(t, decad.Sound, report.Bodies[0].Status)
	require.True(t, report.Trustworthy())

	zero, err := doc.Verify(t.Context(), decad.WithTolerance(units.Scalar(0)))
	require.NoError(t, err)
	require.Equal(t, decad.Suspect, zero.Bodies[0].Status)
	require.False(t, zero.Trustworthy())
}

func TestVerifyToleranceBoundaryIsInclusive(t *testing.T) {
	t.Parallel()
	doc, body := allPlanarBoolean(t, 0.3)
	required := requiredBodyTolerance(t, body)
	require.Positive(t, required)

	atBoundary, err := doc.Verify(t.Context(), decad.WithTolerance(units.Scalar(required)))
	require.NoError(t, err)
	require.Equal(t, decad.Sound, atBoundary.Status)
	require.True(t, atBoundary.Trustworthy())

	below := math.Nextafter(required, 0)
	tooStrict, err := doc.Verify(t.Context(), decad.WithTolerance(units.Scalar(below)))
	require.NoError(t, err)
	require.Equal(t, decad.Suspect, tooStrict.Status)
	require.False(t, tooStrict.Trustworthy())
}

func TestVerifyToleranceGateTracksScaleAndPlacement(t *testing.T) {
	t.Parallel()
	for _, scale := range []float64{1, 1000} {
		t.Run(units.Scalar(scale).String(), func(t *testing.T) {
			doc, body := allPlanarBoolean(t, scale)
			rotation, err := r3.Rotation(r3.NewVec(1, 2, 3), units.Degrees(37))
			require.NoError(t, err)
			translation, err := r3.Translation(r3.NewVec(1000*scale, -250*scale, 80*scale))
			require.NoError(t, err)
			placement, err := rotation.Then(translation)
			require.NoError(t, err)
			placed, err := body.Placed(placement)
			require.NoError(t, err)

			report, err := doc.Verify(t.Context())
			require.NoError(t, err)
			require.Len(t, report.Bodies, 1)
			require.Same(t, placed, report.Bodies[0].Body)
			require.Equal(t, decad.Approximate, report.Bodies[0].Exactness)
			require.Equal(t, decad.Sound, report.Status)
			require.True(t, report.Trustworthy())
		})
	}
}

func TestVerifyExactBodyPassesZeroTolerance(t *testing.T) {
	t.Parallel()
	doc, _ := extrudePlate(t)
	report, err := doc.Verify(t.Context(), decad.WithTolerance(units.Scalar(0)))
	require.NoError(t, err)
	require.Equal(t, decad.Exact, report.Bodies[0].Exactness)
	require.Equal(t, decad.Sound, report.Status)
	require.True(t, report.Trustworthy())
}

// computedToFacePin builds a short, offset-plane fixture where resolving a
// ToFace level loses precision relative to the body's own diameter. The large
// world coordinates make the inherited axial displacement visible at this
// scale without making the body itself large.
func computedToFacePin(t *testing.T) (*decad.Document, *decad.Body, float64) {
	t.Helper()
	w := sketch.NewWorld()
	frame, err := r3.NewFrame(
		r3.NewVec(1e12, 1e12, 0),
		r3.NewVec(0, 0, 1),
		r3.NewVec(0.6, 0.8, 0),
	)
	require.NoError(t, err)
	plane, err := w.CreatePlaneFromFrame(frame)
	require.NoError(t, err)
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	plateRect := s.CreateRectangle(0, 0, 100, 60)
	s.Fix(plateRect.A)
	s.CreateRectangle(120, 0, 140, 20)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)

	var plateProfile, pinProfile *sketch.Profile
	for _, profile := range s.Profiles() {
		if profile.Area > 1000 {
			plateProfile = profile
			continue
		}
		pinProfile = profile
	}
	require.NotNil(t, plateProfile)
	require.NotNil(t, pinProfile)

	doc := decad.New()
	plate, err := doc.Extrude(s, plateProfile, decad.Distance{D: units.Millimeters(1e6), Dir: decad.Along})
	require.NoError(t, err)
	pin, err := doc.Extrude(s, pinProfile, decad.ToFace{
		Body:   plate,
		Face:   capEndFace(plate),
		Offset: units.Millimeters(-0.001),
	})
	require.NoError(t, err)

	points := make([]r3.Vec, 0, len(pin.Vertices()))
	for _, vertex := range pin.Vertices() {
		points = append(points, vertex.Position().Value)
	}
	diameter := diameterOf(points)
	require.Positive(t, diameter)
	return doc, pin, diameter
}

// requireComputedToFaceDiameterGate fixes a reading's threshold to the held
// prism diameter. A computed axial level means that value can overstate the
// true body's diameter, so Verify must reject the reading.
func requireComputedToFaceDiameterGate(t *testing.T, doc *decad.Document, body *decad.Body, heldDiameter float64, reading decad.ReadingKind, bound float64) {
	t.Helper()
	require.Positive(t, bound)
	rel := bound / heldDiameter
	require.Positive(t, rel)
	report, err := doc.Verify(t.Context(), decad.WithTolerance(units.Scalar(rel)))
	require.NoError(t, err)

	var bodyReport *decad.BodyReport
	for _, candidate := range report.Bodies {
		if candidate.Body == body {
			bodyReport = candidate
			break
		}
	}
	require.NotNil(t, bodyReport)
	require.Equal(t, decad.Suspect, bodyReport.Status)
	require.False(t, report.Trustworthy())

	for _, diagnostic := range report.Diagnostics {
		if diagnostic.Body == body && diagnostic.Reading == reading {
			require.NotNil(t, diagnostic.Required)
			require.Less(t, diagnostic.Required.Base(), bound)
			return
		}
	}
	t.Fatalf("Verify did not report the computed ToFace %s threshold", reading)
}

func requireComputedToFaceDiameterThresholds(t *testing.T, doc *decad.Document, body *decad.Body, heldDiameter float64) {
	t.Helper()
	bounds, err := body.Bounds()
	require.NoError(t, err)
	centroid, err := body.Centroid()
	require.NoError(t, err)

	for _, tc := range []struct {
		reading decad.ReadingKind
		bound   float64
	}{
		{reading: decad.ReadingBounds, bound: bounds.Bound.Base()},
		{reading: decad.ReadingCentroid, bound: centroid.Bound.Base()},
	} {
		t.Run(tc.reading.String(), func(t *testing.T) {
			requireComputedToFaceDiameterGate(t, doc, body, heldDiameter, tc.reading, tc.bound)
		})
	}
}

func TestVerifyComputedToFaceDiameterThreshold(t *testing.T) {
	t.Parallel()
	t.Run("prism", func(t *testing.T) {
		doc, pin, heldDiameter := computedToFacePin(t)
		requireComputedToFaceDiameterThresholds(t, doc, pin, heldDiameter)
	})

	t.Run("cup", func(t *testing.T) {
		doc, pin, heldDiameter := computedToFacePin(t)
		cup, err := pin.Shell(topCap(pin), units.Millimeters(1))
		require.NoError(t, err)
		requireComputedToFaceDiameterThresholds(t, doc, cup, heldDiameter)
	})

	t.Run("cap blend", func(t *testing.T) {
		doc, pin, heldDiameter := computedToFacePin(t)
		chamfered, err := pin.Chamfer(capLoopEdges(pin), units.Millimeters(1))
		require.NoError(t, err)
		requireComputedToFaceDiameterThresholds(t, doc, chamfered, heldDiameter)
	})
}

// TestVerifyCupWithinToleranceIsSound is the cup coverage the tolerance gate
// was missing: bodyGateDiameter had no fallback for cupPayload, so its
// nonzero (if minuscule) centroid Bound failed the gate closed with no
// reference to judge it against, however far inside tolerance the true
// figure sat. A cup whose readings are genuinely within tolerance must
// verify Sound.
func TestVerifyCupWithinToleranceIsSound(t *testing.T) {
	t.Parallel()
	doc, box := shellBox(t)
	cup, err := box.Shell(topCap(box), units.Millimeters(5))
	require.NoError(t, err)

	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.Len(t, report.Bodies, 1)
	require.Same(t, cup, report.Bodies[0].Body)
	requireDiagnosticInvariants(t, report)

	// The mass-subtracted centroid leaves a genuine nonzero (Approximate)
	// bound, unlike the cup's Exact area and volume, so this body exercises
	// bodyGateDiameter's cup fallback rather than passing on a zero Bound
	// that needs no reference at all.
	require.NotNil(t, report.Bodies[0].Centroid)
	require.Equal(t, decad.Approximate, report.Bodies[0].Centroid.Exactness)
	require.Positive(t, report.Bodies[0].Centroid.Bound.Base())

	require.Equal(t, decad.Sound, report.Bodies[0].Status)
	require.Equal(t, decad.Sound, report.Status)
	require.True(t, report.Trustworthy())
}

// TestVerifyCapBlendChamferAreaVolumeCentroidAllPass is the cap-blend
// coverage the tolerance gate was missing, on the exact body
// docs/modify-reach-design.md's own example builds (a 100x60x20 plate with a
// complete end-cap chamfer at d=5mm): bodyGateDiameter had no fallback for
// capBlendPayload either, so its area, volume AND centroid readings all
// failed the gate closed with no reference, regardless of how each reading's
// own bound compared to the caller's tolerance. With the envelope-prism
// fallback in place, area and volume pass with bounds many decades below any
// reasonable gate. The centroid used to be a known separate defect (an
// area-weighted face-vertex estimate, no real first moment behind it,
// capblend_moments.go) that kept this same body Suspect; docs/modify-reach-design.md
// §8.4's closed-form first moments (capblend_centroid.go) fixed it — an
// all-Plane band's centroid is exact rational end to end here — so all three
// readings now pass and the body reads Sound.
func TestVerifyCapBlendChamferAreaVolumeCentroidAllPass(t *testing.T) {
	t.Parallel()
	doc, box := capBlendBox(t)
	_, err := box.Chamfer(capLoopEdges(box), units.Millimeters(5))
	require.NoError(t, err)

	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.Len(t, report.Bodies, 1)
	requireDiagnosticInvariants(t, report)

	br := report.Bodies[0]
	require.Equal(t, decad.Sound, br.Status)
	require.Equal(t, decad.Sound, report.Status)
	require.True(t, report.Trustworthy())
	require.Empty(t, report.Diagnostics)
}

func requiredBodyTolerance(t *testing.T, body *decad.Body) float64 {
	t.Helper()
	mesh, err := body.Tessellate(units.Millimeters(1000))
	require.NoError(t, err)
	diameter := diameterOf(mesh.Vertices())
	require.Positive(t, diameter)

	area, err := body.Area()
	require.NoError(t, err)
	edgeLength := 0.0
	for _, edge := range body.Edges() {
		length, err := edge.Length()
		require.NoError(t, err)
		edgeLength += length.Value.Base()
	}
	areaRef := math.Max(math.Abs(area.Value.Base()), 1e-9*diameter*edgeLength)
	required := area.Bound.Base() / areaRef

	bounds, err := body.Bounds()
	require.NoError(t, err)
	required = math.Max(required, bounds.Bound.Base()/diameter)

	volume, err := body.Volume()
	require.NoError(t, err)
	volumeRef := math.Max(math.Abs(volume.Value.Base()), 1e-9*diameter*math.Abs(area.Value.Base()))
	required = math.Max(required, volume.Bound.Base()/volumeRef)

	centroid, err := body.Centroid()
	require.NoError(t, err)
	required = math.Max(required, centroid.Bound.Base()/diameter)
	return required
}

func diameterOf(points []r3.Vec) float64 {
	best := 0.0
	for i := range points {
		for j := i + 1; j < len(points); j++ {
			best = math.Max(best, points[i].Sub(points[j]).Len())
		}
	}
	return best
}
