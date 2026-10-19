package apitest_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestVerifyCoilClearanceAgainstPrism(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	s, profile := clearanceCoilSquare(t)
	spring, err := decad.Coil(t.Context(), doc, s, profile, clearanceCoilAxis, units.Millimeters(1.5), units.Scalar(2))
	require.NoError(t, err)
	core := clearanceCoilBox(t, doc)

	report, err := doc.Verify(t.Context(), decad.WithClearances())
	require.NoError(t, err)
	require.Empty(t, report.Interferences)
	require.Len(t, report.Clearances, 1)
	row := report.Clearances[0]
	require.ElementsMatch(t, []*decad.Body{spring, core}, []*decad.Body{row.A, row.B})
	gap, err := row.Gap.Value.In(units.Millimeter)
	require.NoError(t, err)
	require.InDelta(t, 1, gap, row.Gap.Bound.Base())
}

func TestVerifyCoilClearanceAgainstUnsupportedRevolveIsSuspect(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	s, profile := clearanceCoilSquare(t)
	_, err := decad.Coil(t.Context(), doc, s, profile, clearanceCoilAxis, units.Millimeters(1.5), units.Scalar(2))
	require.NoError(t, err)
	ringSketch, ringProfile := clearanceCoilLoops(t, [][2]float64{{10, 0}, {11, 0}, {11, 1}, {10, 1}})
	_, err = decad.Revolve(t.Context(), doc, ringSketch, ringProfile, clearanceCoilAxis, decad.FullRevolution{})
	require.NoError(t, err)

	report, err := doc.Verify(t.Context(), decad.WithClearances())
	require.NoError(t, err)
	require.Empty(t, report.Clearances)
	require.Equal(t, decad.Suspect, report.Status)
	diagnostic, ok := findDiagnostic(report.Diagnostics, decad.DiagUndecidedPair)
	require.True(t, ok)
	require.Equal(t, decad.Suspect, diagnostic.Status)
}

var clearanceCoilAxis = decad.SketchLine{
	Start: decad.Point2{U: 0, V: 0},
	End:   decad.Point2{U: 0, V: 1},
}

func clearanceCoilSquare(t *testing.T) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	return clearanceCoilLoops(t, [][2]float64{{2, 0}, {3, 0}, {3, 1}, {2, 1}})
}

func clearanceCoilLoops(t *testing.T, loops ...[][2]float64) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	for _, loop := range loops {
		points := make([]*sketch.Point, len(loop))
		for i, point := range loop {
			points[i] = s.CreatePoint(point[0], point[1])
			s.Fix(points[i])
		}
		for i := range points {
			s.CreateLine(points[i], points[(i+1)%len(points)])
		}
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	profiles := s.Profiles()
	require.Len(t, profiles, 1)
	return s, profiles[0]
}

func clearanceCoilBox(t *testing.T, doc *decad.Document) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(4, 0, 5, 4)
	for _, point := range []*sketch.Point{rect.A, rect.B, rect.C, rect.D} {
		s.Fix(point)
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	box, err := decad.Extrude(t.Context(), doc, s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(2), Dir: decad.Along})
	require.NoError(t, err)
	return box
}
