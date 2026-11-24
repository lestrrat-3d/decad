package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is agent-docs/cut-profile-design.md's public-surface tests for
// CutExtrude and CutRevolve, on a 40 x 40 x 5 plate spanning z = 0 to 5.

func cutProfileSketch(t *testing.T, rects ...[4]float64) *sketch.Sketch {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	for _, r := range rects {
		rect := s.CreateRectangle(r[0], r[1], r[2], r[3])
		s.Fix(rect.A)
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	return s
}

func TestCutExtrudeSeveralProfilesOneFeature(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	plate := boxBody(t, doc, -20, -20, 20, 20, 5)
	s := cutProfileSketch(t, [4]float64{-10, -10, -6, -6}, [4]float64{6, 6, 10, 10})
	require.Len(t, s.Profiles(), 2)
	out, err := decad.CutExtrude(t.Context(), plate, s, s.Profiles(),
		decad.Distance{D: units.Millimeters(3), Dir: decad.Along})
	require.NoError(t, err)
	require.Equal(t, []*decad.Body{out}, doc.Bodies(), "neither the plate nor a tool stays live")
	ops := doc.OperationHistory().Operations()
	require.Equal(t, decad.OperationCutExtrude, ops[len(ops)-1].Kind())
	decadtest.MeasuresVolume(t, out, units.CubicMillimeters(40*40*5-2*4*4*3))
	decadtest.MeasuresBounds(t, out, r3.Vec{X: -20, Y: -20}, r3.Vec{X: 20, Y: 20, Z: 5})
}

func TestCutExtrudeOverlappingProfilesRemoveOneVolume(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	plate := boxBody(t, doc, -20, -20, 20, 20, 5)
	s := cutProfileSketch(t, [4]float64{-10, -10, 0, 0}, [4]float64{-5, -5, 5, 5})
	profiles := s.Profiles()
	require.NotEmpty(t, profiles)
	out, err := decad.CutExtrude(t.Context(), plate, s, profiles,
		decad.Distance{D: units.Millimeters(5), Dir: decad.Along})
	require.NoError(t, err)
	// Two 10 x 10 squares overlapping in a 5 x 5 square leave 175 mm2 of footprint.
	decadtest.MeasuresVolume(t, out, units.CubicMillimeters(40*40*5-175*5))
}

func TestCutExtrudeRefusalLeavesTargetLive(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	plate := boxBody(t, doc, -20, -20, 20, 20, 5)
	s := cutProfileSketch(t, [4]float64{-10, -10, -6, -6})
	ext := decad.Distance{D: units.Millimeters(3), Dir: decad.Along}

	_, err := decad.CutExtrude(t.Context(), plate, s, nil, ext)
	require.ErrorIs(t, err, decad.ErrDegenerate)
	_, err = decad.CutExtrude(t.Context(), plate, s, []*sketch.Profile{nil}, ext)
	require.ErrorIs(t, err, decad.ErrDegenerate)
	_, err = decad.CutExtrude(t.Context(), nil, s, s.Profiles(), ext)
	require.ErrorIs(t, err, decad.ErrDegenerate)
	// A tool that covers the whole plate removes everything.
	big := cutProfileSketch(t, [4]float64{-30, -30, 30, 30})
	_, err = decad.CutExtrude(t.Context(), plate, big, big.Profiles(),
		decad.Distance{D: units.Millimeters(6), Dir: decad.Along})
	require.ErrorIs(t, err, decad.ErrBooleanFailed)
	require.Equal(t, []*decad.Body{plate}, doc.Bodies(), "a refusal leaves the plate live and no tool behind")
}

func TestCutRevolveGroove(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	plate := boxBody(t, doc, -20, -20, 20, 20, 5)
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(3, 1, 5, 3)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	axis := decad.SketchLine{Start: decad.Point2{}, End: decad.Point2{V: 1}}
	out, err := decad.CutRevolve(t.Context(), plate, s, s.Profiles(), axis, decad.FullRevolution{})
	require.NoError(t, err)
	require.Equal(t, []*decad.Body{out}, doc.Bodies())
	ops := doc.OperationHistory().Operations()
	require.Equal(t, decad.OperationCutRevolve, ops[len(ops)-1].Kind())
	// The ring spans z = -5 to 5 about the Y axis; the plate holds its upper half.
	decadtest.MeasuresVolume(t, out, units.CubicMillimeters(40*40*5-16*math.Pi))
}
