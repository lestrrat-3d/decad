package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"

	"github.com/lestrrat-3d/decad"
)

// fitToothSection draws one tooth-like section on plane: two fit-spline flanks
// of fit points each, joined by a tip arc and a root arc about the origin.
func fitToothSection(t *testing.T, w *sketch.World, plane *sketch.Plane, fit int, half float64) (*sketch.Sketch, *sketch.Profile) {
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
		rad, a := 9.8+1.6*f, half*(1-0.4*f)
		right[i], left[i] = pt(rad, -a), pt(rad, a)
	}
	rootRight, rootLeft := pt(9.5, -half), pt(9.5, half)
	s.CreateLine(rootRight, right[0])
	_, err = s.CreateFitSpline(right...)
	require.NoError(t, err)
	s.CreateArc(s.Origin(), right[fit-1], left[fit-1])
	_, err = s.CreateFitSpline(left...)
	require.NoError(t, err)
	s.CreateLine(left[0], rootLeft)
	s.CreateArc(s.Origin(), rootRight, rootLeft)
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
		s, p := fitToothSection(t, w, w.XY(), fit, 0.2)
		b, err := decad.Extrude(t.Context(), doc, s, p, decad.Distance{D: units.Millimeters(height), Dir: decad.Along})
		require.NoError(t, err, "fit %d", fit)
		ext, err := b.Volume()
		require.NoError(t, err)

		// The same section lofted to itself is the same prism.
		w2 := sketch.NewWorld()
		doc2 := decad.New()
		s0, p0 := fitToothSection(t, w2, w2.XY(), fit, 0.2)
		s1, p1 := fitToothSection(t, w2, splitTopPlane(t, w2, height), fit, 0.2)
		l, err := decad.Loft(t.Context(), doc2, s0, p0, s1, p1)
		require.NoError(t, err)
		lv, err := l.Volume()
		require.NoError(t, err)

		require.Greater(t, ext.Value.Base(), 0.0)
		require.LessOrEqual(t, math.Abs(ext.Value.Base()-lv.Value.Base()), ext.Bound.Base()+lv.Bound.Base(),
			"fit %d: extrude %v vs loft %v", fit, ext.Value, lv.Value)
	}
}

// TestExtrudedToothPatternJoinsADisc builds a gear of extruded fit-spline teeth
// the way a CAD user does, one tooth and a circular pattern joined to the disc.
// The teeth touch the disc only along their root arcs, so the joined volume is
// the disc plus every tooth.
func TestExtrudedToothPatternJoinsADisc(t *testing.T) {
	const height = 3.0
	for _, tc := range []struct {
		teeth, fit int
		half       float64
	}{{12, 9, 0.12}, {40, 6, 0.03}} {
		doc := decad.New()
		w := sketch.NewWorld()
		s, p := fitToothSection(t, w, w.XY(), tc.fit, tc.half)
		tooth, err := decad.Extrude(t.Context(), doc, s, p, decad.Distance{D: units.Millimeters(height), Dir: decad.Along})
		require.NoError(t, err, "teeth %d", tc.teeth)
		toothVol, err := tooth.Volume()
		require.NoError(t, err)
		disc := circleBodyAtZ(t, doc, 0, 9.5, 0, height)
		discVol, err := disc.Volume()
		require.NoError(t, err)

		copies, err := decad.PatternCopies(t.Context(), tooth, decad.CircularPattern{Axis: r3.Vec{Z: 1}, Count: tc.teeth})
		require.NoError(t, err)
		joined, err := decad.Union(t.Context(), disc, append([]*decad.Body{tooth}, copies...)...)
		require.NoError(t, err, "teeth %d", tc.teeth)
		require.True(t, joined.IsSolid())
		got, err := joined.Volume()
		require.NoError(t, err)

		want := discVol.Value.Base() + float64(tc.teeth)*toothVol.Value.Base()
		slack := got.Bound.Base() + discVol.Bound.Base() + float64(tc.teeth)*toothVol.Bound.Base()
		require.LessOrEqual(t, math.Abs(got.Value.Base()-want), slack,
			"teeth %d: joined %v want %v", tc.teeth, got.Value, want)
	}
}
