package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is agent-docs/text-design.md §6's public-surface tests for
// TextSpec.Arc, on the glyph boxes of text_test.go.

// localBox is the extent of a profile's outer boundary in a letter's own frame:
// x along the baseline tangent and y up, measured from the point on the arc where
// the letter stands.
func localBox(p *sketch.Profile, ox, oy, tx, ty, ux, uy float64) box {
	b := box{math.Inf(1), math.Inf(1), math.Inf(-1), math.Inf(-1)}
	for _, e := range p.Outer {
		for _, pt := range e.Polyline {
			dx, dy := pt[0]-ox, pt[1]-oy
			x, y := dx*tx+dy*ty, dx*ux+dy*uy
			b.x0, b.x1 = math.Min(b.x0, x), math.Max(b.x1, x)
			b.y0, b.y1 = math.Min(b.y0, y), math.Max(b.y1, y)
		}
	}
	return b
}

func TestTextProfilesAlongAnArcStandEachLetterOnTheCircle(t *testing.T) {
	t.Parallel()
	const radius, height = 50.0, 10.0
	for _, clockwise := range []bool{true, false} {
		w := sketch.NewWorld()
		start := 90.0
		if !clockwise {
			start = 270
		}
		spec := textSpec("HI", height)
		spec.Arc = &decad.TextArc{
			Center: decad.Point2{U: 3, V: -2}, Radius: units.Millimeters(radius),
			Start: units.Degrees(start), Clockwise: clockwise,
		}
		_, profiles, err := decad.TextProfiles(t.Context(), w, w.XY(), spec)
		require.NoError(t, err)
		require.Len(t, profiles, 2)
		_, upm := goFont(t)
		scale := height / upm
		pen := 0.0
		for i, r := range []rune("HI") {
			glyph, advance := glyphBox(t, r)
			mid := pen + advance*scale/2
			sign := 1.0
			if clockwise {
				sign = -1
			}
			theta := start*math.Pi/180 + sign*mid/radius
			sin, cos := math.Sincos(theta)
			// The tangent of the text and its way up: outward for clockwise text, inward for the other.
			tx, ty := -sign*sin, sign*cos
			ux, uy := cos, sin
			if !clockwise {
				ux, uy = -cos, -sin
			}
			got := localBox(profiles[i], 3+radius*cos, -2+radius*sin, tx, ty, ux, uy)
			require.InDelta(t, glyph.x0*scale-advance*scale/2, got.x0, 1e-9, "%c, clockwise %v", r, clockwise)
			require.InDelta(t, glyph.x1*scale-advance*scale/2, got.x1, 1e-9)
			require.InDelta(t, glyph.y0*scale, got.y0, 1e-9)
			require.InDelta(t, glyph.y1*scale, got.y1, 1e-9)
			pen += advance * scale
		}
	}
}

func TestTextProfilesAlongAnArcKeepEachLettersArea(t *testing.T) {
	t.Parallel()
	w := sketch.NewWorld()
	flat, err := func() ([]float64, error) {
		_, profiles, err := decad.TextProfiles(t.Context(), w, w.XY(), textSpec("HI", 10))
		var out []float64
		for _, p := range profiles {
			out = append(out, p.Area)
		}
		return out, err
	}()
	require.NoError(t, err)
	spec := textSpec("HI", 10)
	spec.Arc = &decad.TextArc{Radius: units.Millimeters(30), Start: units.Degrees(0), Clockwise: true}
	_, profiles, err := decad.TextProfiles(t.Context(), w, w.XY(), spec)
	require.NoError(t, err)
	for i, p := range profiles {
		require.InDelta(t, flat[i], p.Area, 1e-6, "a letter is turned, not bent: its area is the same")
	}
}

func TestTextProfilesAlongAnArcRefusals(t *testing.T) {
	t.Parallel()
	w := sketch.NewWorld()
	with := func(text string, edit func(*decad.TextArc)) error {
		spec := textSpec(text, 10)
		spec.Arc = &decad.TextArc{Radius: units.Millimeters(30), Start: units.Degrees(0)}
		edit(spec.Arc)
		_, _, err := decad.TextProfiles(t.Context(), w, w.XY(), spec)
		return err
	}
	require.ErrorIs(t, with("A\nB", func(*decad.TextArc) {}), decad.ErrDegenerate)
	require.ErrorIs(t, with("A", func(a *decad.TextArc) { a.Radius = units.Millimeters(0) }), decad.ErrDegenerate)
	require.ErrorIs(t, with("A", func(a *decad.TextArc) { a.Radius = units.Millimeters(-5) }), decad.ErrNegativeMagnitude)
	require.ErrorIs(t, with("A", func(a *decad.TextArc) { a.Radius = units.Degrees(5) }), decad.ErrUnitKind)
	require.ErrorIs(t, with("A", func(a *decad.TextArc) { a.Start = units.Millimeters(5) }), decad.ErrUnitKind)
}
