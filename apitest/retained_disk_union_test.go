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

func TestRetainedArcDiskUnion(t *testing.T) {
	t.Parallel()
	for _, diskFirst := range []bool{false, true} {
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
		joined, e := decad.Union(ctx, plate, pad)
		require.NoError(t, e)
		far := frame(r3.Vec{X: 8 + width, Y: -2}, x, z)
		far.CreateCircle(far.Origin(), 3)
		boss := extrude(far, 1.5, decad.Against)
		if diskFirst {
			joined, boss = boss, joined
		}
		result, err := decad.Union(ctx, joined, boss)
		require.NoError(t, err)
		m, err := result.Tessellate(ctx, units.Millimeters(.5), decad.WithVerification(decad.VerifyAll))
		require.NoError(t, err)
		require.True(t, m.BoundaryVerified() && m.VolumeVerified())
		require.Less(t, m.Bound().Base(), .001)
		require.Empty(t, result.OperationDiagnostics())
	}
}
