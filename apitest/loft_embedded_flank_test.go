package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/stretchr/testify/require"
)

// The gallery's smallest embedded-flank tooth uses the original interpolating
// curves on both sections. Sketch certifies their root-circle trims, and Loft
// records those same curves to build one solid.
func TestLoftEmbeddedFitFlanksKeepOriginalCurve(t *testing.T) {
	t.Parallel()
	world := sketch.NewWorld()
	frame, err := r3.NewFrame(r3.NewVec(0, 0, 2), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	upper, err := world.CreatePlaneFromFrame(frame)
	require.NoError(t, err)
	makeProfile := func(plane *sketch.Plane) (*sketch.Sketch, *sketch.Profile) {
		const teeth, samples = 30, 5
		const module, pressure = 0.5, math.Pi / 6
		pitch := module * teeth / 2
		base, root, tip := pitch*math.Cos(pressure), (module*teeth-2.5*module)/2, (module*teeth+2*module)/2
		flank := func(radius float64) (float64, float64) {
			a := math.Acos(base / radius)
			v := math.Tan(a)
			return base * (math.Cos(v) + v*math.Sin(v)), base * (math.Sin(v) - v*math.Cos(v))
		}
		px, py := flank(pitch)
		rotation := math.Pi/(2*teeth) - math.Atan2(-py, px)
		s, err := world.CreateSketch(plane)
		require.NoError(t, err)
		center := s.CreatePoint(0, 0)
		s.Fix(center)
		left, right := make([]*sketch.Point, samples), make([]*sketch.Point, samples)
		for i := range left {
			radius := base + (tip-base)*float64(i)/float64(samples-1)
			x, y := flank(radius)
			xl, yl := x*math.Cos(rotation)+y*math.Sin(rotation), x*math.Sin(rotation)-y*math.Cos(rotation)
			left[i], right[i] = s.CreatePoint(xl, yl), s.CreatePoint(xl, -yl)
			s.Fix(left[i])
			s.Fix(right[i])
		}
		_, err = s.CreateFitSpline(right...)
		require.NoError(t, err)
		s.CreateArc(center, right[samples-1], left[samples-1])
		_, err = s.CreateFitSpline(left...)
		require.NoError(t, err)
		s.CreateCircle(center, root)
		_, err = s.Solve(t.Context())
		require.NoError(t, err)
		for _, profile := range s.Profiles() {
			var flanks int
			for _, edge := range profile.Outer {
				fit, ok := edge.Entity.(*sketch.FitSpline)
				if !ok || !edge.Partial {
					continue
				}
				flanks++
				require.True(t, edge.TExact)
				require.Greater(t, edge.TStart, 0.0)
				x, y := fit.Eval(edge.TStart)
				require.InDelta(t, root, math.Hypot(x, y), 1e-12)
			}
			if flanks == 2 {
				require.True(t, profile.Valid)
				return s, profile
			}
		}
		t.Fatal("no valid tooth profile with two trimmed flanks")
		return nil, nil
	}
	s0, p0 := makeProfile(world.XY())
	s1, p1 := makeProfile(upper)
	body, err := decad.New().Loft(t.Context(), s0, p0, s1, p1)
	require.NoError(t, err)
	require.Equal(t, decad.BodySolid, body.Kind())
	require.True(t, body.IsSolid())
	require.Len(t, body.Lumps(), 1)
	volume, err := body.Volume()
	require.NoError(t, err)
	require.Greater(t, volume.Value.Base(), 0.0)
}
