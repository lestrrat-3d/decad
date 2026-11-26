package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestCutRegisteredPrismArc(t *testing.T) {
	t.Parallel()
	ctx := t.Context()
	d := decad.New()
	frame := func(o, u, v r3.Vec) *sketch.Sketch {
		f, e := r3.NewFrame(o, u, v)
		require.NoError(t, e)
		w := sketch.NewWorld()
		p, e := w.CreatePlaneFromFrame(f)
		require.NoError(t, e)
		s, e := w.CreateSketch(p)
		require.NoError(t, e)
		return s
	}
	x, z := r3.Vec{X: 1}, r3.Vec{Z: 1}
	profile := func(s *sketch.Sketch) *sketch.Profile {
		_, e := s.Solve(ctx)
		require.NoError(t, e)
		for _, p := range s.Profiles() {
			if p.Valid {
				return p
			}
		}
		t.Fatal("invalid profile")
		return nil
	}
	extrude := func(s *sketch.Sketch, h float64, dir decad.Direction) *decad.Body {
		b, e := decad.Extrude(ctx, d, s, profile(s), decad.Distance{D: units.Millimeters(h), Dir: dir})
		require.NoError(t, e)
		return b
	}
	point := func(s *sketch.Sketch, a, b float64) *sketch.Point {
		p := s.CreatePoint(a, b)
		s.Fix(p)
		return p
	}
	stock := frame(r3.Vec{X: 8, Y: -1}, x, z)
	const width = 8.0
	cx, cz := -2.5*math.Cos(1.43), 2.5*math.Sin(1.43)
	a, b := point(stock, cx, cz), point(stock, cx, -cz)
	stock.CreateArc(stock.Origin(), a, b)
	c := point(stock, width+3, -(6.25-cx*(width+3))/cz)
	f := point(stock, width, 3)
	mid := point(stock, width+3, 0)
	farCenter := point(stock, width, 0)
	stock.CreateLine(b, c)
	stock.CreateLine(c, mid)
	stock.CreateArc(farCenter, mid, f)
	top := point(stock, width-2, (6.25-cx*(width-2))/cz)
	shoulder := point(stock, width-1, 3)
	stock.CreateLine(f, shoulder)
	stock.CreateLine(shoulder, top)
	stock.CreateLine(top, a)
	plate := extrude(stock, 1, decad.Against)
	padSketch := frame(r3.Vec{X: 8, Y: -.5}, x, z)
	padSketch.CreateCircle(padSketch.Origin(), 2.5)
	pad := extrude(padSketch, .75, decad.Against)
	joined, e := decad.Union(plate, pad).Do(ctx)
	require.NoError(t, e)
	joined, e = decad.Chamfer(ctx, joined, decad.Edges(decad.OuterLoopOf(decad.CapStart(pad))).Exactly(1), units.Millimeters(.25))
	require.NoError(t, e)
	section := func(y, r float64) (*sketch.Sketch, *sketch.Profile) {
		s := frame(r3.Vec{X: 8 + width, Y: y}, x, z)

		a := point(s, .8*r, .6*r)
		b := point(s, .6*r, .8*r)
		s.CreateArc(s.Origin(), a, b)
		c := point(s, 2.4, 3.2)
		f := point(s, 3.2, 2.4)
		s.CreateLine(b, c)
		s.CreateLine(c, f)
		s.CreateLine(f, a)
		return s, profile(s)
	}
	fl, fp := section(-1.06, 2.74)
	fh, fq := section(-.78, 3.02)
	first, err := decad.Loft(ctx, d, fl, fp, fh, fq)
	require.NoError(t, err)
	joined, err = decad.Cut(joined, first).Do(ctx)
	require.NoError(t, err)
	l, lp := section(-.63, 3.02)
	h, hp := section(.06, 2.74)
	tool, e := decad.Loft(ctx, d, l, lp, h, hp)
	require.NoError(t, e)
	before, e := joined.Tessellate(ctx, units.Millimeters(.5), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, e)
	cut, e := decad.Cut(joined, tool).Do(ctx)
	require.NoError(t, e)
	m, e := cut.Tessellate(ctx, units.Millimeters(.5), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, e)
	require.True(t, m.BoundaryVerified() && m.VolumeVerified())
	// At 45 degrees the cutter wall reaches radius 2.90 at this height.
	// The same line reached the upper cap before the second subtraction.
	const radius = 2.90
	px, pz := 8+width+radius/math.Sqrt2, radius/math.Sqrt2
	require.InDelta(t, 0, highestMeshYAt(before, px, pz), 1e-8)
	require.InDelta(t, -.63+.69*(3.02-radius)/.28, highestMeshYAt(m, px, pz), .01)
}
