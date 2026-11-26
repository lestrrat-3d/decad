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

func TestConcentricPrismStations(t *testing.T) {
	t.Parallel()
	for _, reversed := range []bool{false, true} {
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
			b, e := decad.Extrude(d, s, profile(s), decad.Distance{D: units.Millimeters(h), Dir: dir}).Do(ctx)
			require.NoError(t, e)
			return b
		}
		point := func(s *sketch.Sketch, a, b float64) *sketch.Point {
			p := s.CreatePoint(a, b)
			s.Fix(p)
			return p
		}
		stock := frame(r3.Vec{X: 8, Y: -1}, x, z)
		const width = 6
		cx, cz := math.Nextafter(-1.5, 0), 2.0
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
		if reversed {
			plate, pad = pad, plate
		}
		joined, e := decad.Union(plate, pad).Do(ctx)
		require.NoError(t, e)
		require.Empty(t, joined.OperationDiagnostics())
		m, e := joined.Tessellate(ctx, units.Millimeters(.5), decad.WithVerification(decad.VerifyAll))
		require.NoError(t, e)
		require.True(t, m.BoundaryVerified())
		require.True(t, m.VolumeVerified())
		t.Logf("Union has %d facets and bound %.9g mm", len(m.Triangles()), m.Bound().Base())
		require.Less(t, m.Bound().Base(), 0.001)
	}
}
