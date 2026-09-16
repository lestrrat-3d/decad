package decadtest_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// soundPlateWithSurveysReport builds the "Sound plate with surveys" fixture:
// the 100x60x10 mm plate, verified with every survey option Verify accepts.
func soundPlateWithSurveysReport(t *testing.T) (*decad.Report, *decad.Body) {
	t.Helper()

	doc, plate := soundPlateDoc(t)
	report := decadtest.Verify(t, doc,
		decad.WithMinWallThickness(units.Millimeters(1)),
		decad.WithPullDirection(r3.NewVec(0, 0, 1)),
		decad.WithConcaveRadius())
	return report, plate
}

// holePlateReport builds a 100x60x8 mm plate with a Ø20 through hole
// centred at (70, 30), verified with decad.WithConcaveRadius(). The hole
// wall is the plate's one concave face: the survey reads a measured 10 mm
// radius, the ConcaveRadius fixture no supported straight prism produces
// (TestMinRadiusHolePlate, survey_test.go:580, is the same shape built in
// the root package's own tests; this rebuilds it here rather than importing
// an unexported test helper from that package).
func holePlateReport(t *testing.T) (*decad.Report, *decad.Body) {
	t.Helper()

	ws := sketch.NewWorld()
	s, err := ws.CreateSketch(ws.XY())
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
	require.NotNil(t, prof, "the sketch must solve one profile with one hole")

	doc := decad.New()
	body, err := doc.Extrude(s, prof, decad.Distance{D: units.Millimeters(8), Dir: decad.Along})
	require.NoError(t, err)

	report := decadtest.Verify(t, doc, decad.WithConcaveRadius())
	return report, body
}

func TestWallMinimumMeasuresThePlate(t *testing.T) {
	t.Parallel()

	report, plate := soundPlateWithSurveysReport(t)
	br := decadtest.BodyReport(t, report, plate)
	decadtest.WallMinimum(t, br, units.Millimeters(10), decadtest.Exactly())
}

func TestUndercutFacesAcceptsAPlateWithNoUndercut(t *testing.T) {
	t.Parallel()

	report, plate := soundPlateWithSurveysReport(t)
	br := decadtest.BodyReport(t, report, plate)
	faces := decadtest.UndercutFaces(t, br, 0)
	require.Empty(t, faces)
}

func TestConcaveRadiusMeasuresAConcaveBody(t *testing.T) {
	t.Parallel()

	report, body := holePlateReport(t)
	br := decadtest.BodyReport(t, report, body)
	decadtest.ConcaveRadius(t, br, units.Millimeters(10), decadtest.Exactly())
}

func TestWallMinimumRejectsANilBodyReport(t *testing.T) {
	t.Parallel()

	out := captureFailure(t, func(tb testing.TB) {
		decadtest.WallMinimum(tb, nil, units.Millimeters(1))
	})
	require.Contains(t, out, "br must not be nil")
}

func TestWallMinimumReportsAnUnrequestedSurvey(t *testing.T) {
	t.Parallel()

	doc, plate := soundPlateDoc(t)
	report := decadtest.Verify(t, doc) // no decad.WithMinWallThickness
	br := decadtest.BodyReport(t, report, plate)

	out := captureFailure(t, func(tb testing.TB) {
		decadtest.WallMinimum(tb, br, units.Millimeters(1))
	})
	require.Contains(t, out, "want measured")
}

func TestWallMinimumReportsAMiss(t *testing.T) {
	t.Parallel()

	report, plate := soundPlateWithSurveysReport(t)
	br := decadtest.BodyReport(t, report, plate)

	out := captureFailure(t, func(tb testing.TB) {
		decadtest.WallMinimum(tb, br, units.Millimeters(5))
	})
	require.Contains(t, out, "does not enclose")
}

func TestConcaveRadiusReportsAnAbsentSurvey(t *testing.T) {
	t.Parallel()

	report, plate := soundPlateWithSurveysReport(t)
	br := decadtest.BodyReport(t, report, plate)

	out := captureFailure(t, func(tb testing.TB) {
		decadtest.ConcaveRadius(tb, br, units.Millimeters(1))
	})
	require.Contains(t, out, "want measured")
}

func TestUndercutFacesRejectsANilBodyReport(t *testing.T) {
	t.Parallel()

	out := captureFailure(t, func(tb testing.TB) {
		decadtest.UndercutFaces(tb, nil, 0)
	})
	require.Contains(t, out, "br must not be nil")
}

func TestUndercutFacesReportsAnUnrequestedSurvey(t *testing.T) {
	t.Parallel()

	doc, plate := soundPlateDoc(t)
	report := decadtest.Verify(t, doc) // no decad.WithPullDirection
	br := decadtest.BodyReport(t, report, plate)

	out := captureFailure(t, func(tb testing.TB) {
		decadtest.UndercutFaces(tb, br, 0)
	})
	require.Contains(t, out, "want complete")
}

func TestUndercutFacesReportsAWrongCount(t *testing.T) {
	t.Parallel()

	report, plate := soundPlateWithSurveysReport(t)
	br := decadtest.BodyReport(t, report, plate)

	out := captureFailure(t, func(tb testing.TB) {
		decadtest.UndercutFaces(tb, br, 2)
	})
	require.Contains(t, out, "opposing face(s), want 2")
}
