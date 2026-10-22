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

// betweenHalf is the half width of the squares these tests loft.
const betweenHalf = 2.0

// betweenSquare draws a square of half width betweenHalf on the XY plane.
func betweenSquare(t *testing.T, w *sketch.World) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	half := betweenHalf
	return splitPolygon(t, w, w.XY(), [][2]float64{{-half, -half}, {half, -half}, {half, half}, {-half, half}})
}

func betweenDisc(t *testing.T, w *sketch.World, radius float64) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	c := s.CreatePoint(0, 0)
	s.Fix(c)
	s.CreateCircle(c, radius)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Profiles(), 1)
	return s, s.Profiles()[0]
}

// TestLoftBetweenProfileToProfileIsLoft pins that two profile ends are Loft.
func TestLoftBetweenProfileToProfileIsLoft(t *testing.T) {
	t.Parallel()
	vol := func(between bool) float64 {
		w := sketch.NewWorld()
		s0, p0 := betweenSquare(t, w)
		s1, p1 := splitPolygon(t, w, splitTopPlane(t, w, 3), [][2]float64{{-1, -1}, {1, -1}, {1, 1}, {-1, 1}})
		var b *decad.Body
		var err error
		if between {
			b, err = decad.LoftBetween(t.Context(), decad.New(), decad.ProfileEnd(s0, p0), decad.ProfileEnd(s1, p1))
		} else {
			b, err = decad.Loft(t.Context(), decad.New(), s0, p0, s1, p1)
		}
		require.NoError(t, err)
		v, err := b.Volume()
		require.NoError(t, err)
		return v.Value.Base()
	}
	require.Equal(t, vol(false), vol(true))
}

// TestLoftBetweenPointAndProfile lofts a square to a point from both orders. The
// solid is a pyramid, whose volume is base area times height over three.
func TestLoftBetweenPointAndProfile(t *testing.T) {
	t.Parallel()
	const height = 3.0
	want := 4 * betweenHalf * betweenHalf * height / 3
	apex := r3.NewVec(0, 0, height)
	for name, order := range map[string]bool{"point first": true, "profile first": false} {
		t.Run(name, func(t *testing.T) {
			t.Parallel()
			w := sketch.NewWorld()
			s, p := betweenSquare(t, w)
			from, to := decad.PointEnd(apex), decad.ProfileEnd(s, p)
			if !order {
				from, to = to, from
			}
			b, err := decad.LoftBetween(t.Context(), decad.New(), from, to)
			require.NoError(t, err)
			require.True(t, b.IsSolid())
			v, err := b.Volume()
			require.NoError(t, err)
			require.InDelta(t, want, v.Value.Base(), v.Bound.Base()+1e-9)
			require.Less(t, v.Bound.Base(), 1e-6)
			// The same solid as LoftFromPoint's.
			w2 := sketch.NewWorld()
			s2, p2 := betweenSquare(t, w2)
			ref, err := decad.LoftFromPoint(t.Context(), decad.New(), apex, s2, p2)
			require.NoError(t, err)
			rv, err := ref.Volume()
			require.NoError(t, err)
			require.Equal(t, rv.Value.Base(), v.Value.Base())
		})
	}
}

// TestLoftBetweenConeOverACircle checks a curved profile against the cone volume.
func TestLoftBetweenConeOverACircle(t *testing.T) {
	t.Parallel()
	const radius, height = 3.0, 4.0
	w := sketch.NewWorld()
	s, p := betweenDisc(t, w, radius)
	b, err := decad.LoftBetween(t.Context(), decad.New(), decad.ProfileEnd(s, p), decad.PointEnd(r3.NewVec(0, 0, height)))
	require.NoError(t, err)
	v, err := b.Volume()
	require.NoError(t, err)
	want := math.Pi * radius * radius * height / 3
	require.InDelta(t, want, v.Value.Base(), v.Bound.Base()+1e-9)
	mesh, err := b.Tessellate(t.Context(), units.Millimeters(0.05), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
}

func TestLoftBetweenRefusals(t *testing.T) {
	t.Parallel()
	apex := r3.NewVec(0, 0, 3)
	build := func(from, to func(*sketch.World) decad.LoftEnd, opts ...decad.LoftOption) error {
		w := sketch.NewWorld()
		_, err := decad.LoftBetween(t.Context(), decad.New(), from(w), to(w), opts...)
		return err
	}
	square := func(w *sketch.World) decad.LoftEnd {
		s, p := betweenSquare(t, w)
		return decad.ProfileEnd(s, p)
	}
	point := func(*sketch.World) decad.LoftEnd { return decad.PointEnd(apex) }
	t.Run("two points", func(t *testing.T) {
		t.Parallel()
		require.ErrorIs(t, build(point, point), decad.ErrDegenerate)
	})
	t.Run("an end built by neither constructor", func(t *testing.T) {
		t.Parallel()
		zero := func(*sketch.World) decad.LoftEnd { return decad.LoftEnd{} }
		require.ErrorIs(t, build(zero, square), decad.ErrDegenerate)
		require.ErrorIs(t, build(point, zero), decad.ErrDegenerate)
	})
	t.Run("a nil sketch", func(t *testing.T) {
		t.Parallel()
		nilEnd := func(*sketch.World) decad.LoftEnd { return decad.ProfileEnd(nil, nil) }
		require.ErrorIs(t, build(nilEnd, point), decad.ErrDegenerate)
	})
	t.Run("alignment and splits name no pairing for a point end", func(t *testing.T) {
		t.Parallel()
		require.ErrorIs(t, build(point, square, decad.WithLoftAlignment(0)), decad.ErrDegenerate)
		splits := []decad.LoftSplit{{Loop: 0, Segment: 0, At: []float64{0.5}}}
		require.ErrorIs(t, build(square, point, decad.WithLoftSplits(splits, nil)), decad.ErrDegenerate)
	})
	t.Run("a surface result", func(t *testing.T) {
		t.Parallel()
		require.ErrorIs(t, build(point, square, decad.WithSurfaceResult()), decad.ErrUnsupported)
	})
	t.Run("the point in the profile plane", func(t *testing.T) {
		t.Parallel()
		inPlane := func(*sketch.World) decad.LoftEnd { return decad.PointEnd(r3.NewVec(0, 0, 0)) }
		require.ErrorIs(t, build(inPlane, square), decad.ErrDegenerate)
	})
	t.Run("a profile with a hole", func(t *testing.T) {
		t.Parallel()
		ring := func(w *sketch.World) decad.LoftEnd {
			s, err := w.CreateSketch(w.XY())
			require.NoError(t, err)
			c := s.CreatePoint(0, 0)
			s.Fix(c)
			s.CreateCircle(c, 3)
			s.CreateCircle(c, 1)
			_, err = s.Solve(t.Context())
			require.NoError(t, err)
			for _, p := range s.Profiles() {
				if len(p.Holes) == 1 {
					return decad.ProfileEnd(s, p)
				}
			}
			t.Fatal("no ring profile")
			return decad.LoftEnd{}
		}
		require.ErrorIs(t, build(point, ring), decad.ErrUnsupported)
	})
}
