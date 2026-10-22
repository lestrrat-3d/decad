package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"

	"github.com/lestrrat-3d/decad"
)

// fitToothSection draws one tooth-like section on plane: two fit-spline flanks
// of fit points each, joined by a tip arc and a root arc about the origin.
func fitToothSection(t *testing.T, w *sketch.World, plane *sketch.Plane, fit int) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	pt := func(rad, a float64) *sketch.Point {
		p := s.CreatePoint(rad*math.Cos(a), rad*math.Sin(a))
		s.Fix(p)
		return p
	}
	right, left := make([]*sketch.Point, fit), make([]*sketch.Point, fit)
	for i := range fit {
		f := float64(i) / float64(fit-1)
		rad, a := 9.5+1.9*f, 0.2*(1-0.4*f)
		right[i], left[i] = pt(rad, -a), pt(rad, a)
	}
	_, err = s.CreateFitSpline(right...)
	require.NoError(t, err)
	s.CreateArc(s.Origin(), right[fit-1], left[fit-1])
	_, err = s.CreateFitSpline(left...)
	require.NoError(t, err)
	s.CreateArc(s.Origin(), right[0], left[0])
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Profiles(), 1)
	return s, s.Profiles()[0]
}

func TestExtrudeFitSplineSectionScalesWorkWithPieces(t *testing.T) {
	const height = 3.0
	for _, fit := range []int{9, 15} {
		w := sketch.NewWorld()
		doc := decad.New()
		s, p := fitToothSection(t, w, w.XY(), fit)
		b, err := decad.Extrude(t.Context(), doc, s, p, decad.Distance{D: units.Millimeters(height), Dir: decad.Along})
		require.NoError(t, err, "fit %d", fit)
		ext, err := b.Volume()
		require.NoError(t, err)

		// The same section lofted to itself is the same prism.
		w2 := sketch.NewWorld()
		doc2 := decad.New()
		s0, p0 := fitToothSection(t, w2, w2.XY(), fit)
		s1, p1 := fitToothSection(t, w2, splitTopPlane(t, w2, height), fit)
		l, err := decad.Loft(t.Context(), doc2, s0, p0, s1, p1)
		require.NoError(t, err)
		lv, err := l.Volume()
		require.NoError(t, err)

		require.Greater(t, ext.Value.Base(), 0.0)
		require.LessOrEqual(t, math.Abs(ext.Value.Base()-lv.Value.Base()), ext.Bound.Base()+lv.Bound.Base(),
			"fit %d: extrude %v vs loft %v", fit, ext.Value, lv.Value)
	}
}
