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

func TestAlignedPadRimOnTrimmedFlange(t *testing.T) {
	for _, bore := range []bool{false, true} {
		name := "whole cap"
		if bore {
			name = "pierced cap"
		}
		t.Run(name, func(t *testing.T) { alignedTrimmedPad(t, bore) })
	}
}
func alignedTrimmedPad(t *testing.T, bore bool) {
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
	x, y, z := r3.Vec{X: 1}, r3.Vec{Y: 1}, r3.Vec{Z: 1}
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
	const theta, width = 1.43, 6
	cx, cz := -2.5*math.Cos(theta), 2.5*math.Sin(theta)
	a, b := point(stock, cx, cz), point(stock, cx, -cz)
	stock.CreateArc(stock.Origin(), a, b)
	reach := (6.25 - cx*width) / cz
	c, f := point(stock, width, -reach), point(stock, width, reach)
	stock.CreateLine(b, c)
	stock.CreateLine(c, f)
	stock.CreateLine(f, a)
	plate := extrude(stock, 1, decad.Against)
	padSketch := frame(r3.Vec{X: 8, Y: -.5}, x, z)
	padSketch.CreateCircle(padSketch.Origin(), 2.5)
	pad := extrude(padSketch, .75, decad.Against)
	joined, e := decad.Union(ctx, plate, pad)
	require.NoError(t, e)
	require.Empty(t, joined.OperationDiagnostics())
	drill := frame(r3.Vec{X: 8 + width - .7, Y: -.5, Z: -5}, x, y)
	drill.CreateCircle(drill.Origin(), .2)
	joined, e = decad.Cut(ctx, joined, extrude(drill, 10, decad.Along))
	require.NoError(t, e)
	if bore {
		s := frame(r3.Vec{X: 8, Y: .5}, x, z)
		s.CreateCircle(s.Origin(), .2)
		joined, e = decad.Cut(ctx, joined, extrude(s, 2, decad.Along))
		require.NoError(t, e)
	}
	res, e := decad.Chamfer(ctx, joined, decad.Edges(decad.OuterLoopOf(decad.CapStart(pad))).Exactly(1), units.Millimeters(.25))
	if bore {
		require.ErrorIs(t, e, decad.ErrUnsupported)
		return
	}
	require.NoError(t, e)
	mesh, e := res.Tessellate(ctx, units.Millimeters(1), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, e)
	require.True(t, mesh.VolumeVerified() && mesh.BoundaryVerified())
	var ring int
	for _, v := range mesh.Vertices() {
		if v.Y == .25 {
			require.InDelta(t, 2.25, math.Hypot(v.X-8, v.Z), 1e-8)
			ring++
		}
	}
	require.Greater(t, ring, 8)
}
