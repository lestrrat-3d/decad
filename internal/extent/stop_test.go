package extent_test

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/extent"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestExactlyParallel(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name string
		a, b r3.Vec
		want bool
	}{
		{name: "same", a: r3.NewVec(0, 0, 1), b: r3.NewVec(0, 0, 1), want: true},
		{name: "opposite", a: r3.NewVec(0, 0, 1), b: r3.NewVec(0, 0, -1), want: true},
		{name: "scaled", a: r3.NewVec(1, 2, 3), b: r3.NewVec(-2, -4, -6), want: true},
		{name: "one ulp tilt", a: r3.NewVec(0, 0, 1), b: r3.NewVec(math.SmallestNonzeroFloat64, 0, 1)},
		{name: "rounded flip", a: r3.NewVec(0, 0, 1), b: r3.NewVec(0, -1.2246467991473532e-16, -1)},
		{name: "zero", a: r3.NewVec(0, 0, 0), b: r3.NewVec(0, 0, 1)},
		{name: "non finite", a: r3.NewVec(math.Inf(1), 0, 0), b: r3.NewVec(1, 0, 0)},
		{name: "perpendicular", a: r3.NewVec(1, 0, 0), b: r3.NewVec(0, 0, 1)},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			require.Equal(t, tc.want, extent.ExactlyParallel(tc.a, tc.b))
		})
	}
}

func ratOf(f float64) *big.Rat { return new(big.Rat).SetFloat64(f) }

func mustFrame(t *testing.T, o, u, v r3.Vec) r3.Frame {
	t.Helper()
	f, err := r3.NewFrame(o, u, v)
	require.NoError(t, err)
	return f
}

// TestPlaneStop pins the exact stop level a cap record denotes against a sweep
// frame, and its refusals.
func TestPlaneStop(t *testing.T) {
	t.Parallel()
	xy := mustFrame(t, r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	generic := mustFrame(t, r3.Vec{}, r3.NewVec(1, 1, 0), r3.NewVec(-1, 1, 1))

	t.Run("axis frame at an integer level", func(t *testing.T) {
		t.Parallel()
		c, k, ok := extent.PlaneStop(xy, r3.Identity(), 10, xy)
		require.True(t, ok)
		require.Equal(t, 0, c.Cmp(big.NewRat(10, 1)))
		require.Equal(t, 1.0, k)
	})

	t.Run("origin 0.1 is read exactly", func(t *testing.T) {
		t.Parallel()
		f := mustFrame(t, r3.NewVec(0, 0, 0.1), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
		c, k, ok := extent.PlaneStop(f, r3.Identity(), 10, xy)
		require.True(t, ok)
		require.Equal(t, 0, c.Cmp(new(big.Rat).Add(ratOf(0.1), big.NewRat(10, 1))))
		require.Equal(t, 1.0, k)
		// The held level float64(10.1) is not the exact level.
		require.NotEqual(t, 0, c.Cmp(ratOf(10.1)))
		require.Positive(t, extent.StopLevelError(c, 0, 0, 10.1))
	})

	t.Run("generic frame against itself", func(t *testing.T) {
		t.Parallel()
		const z = 1000.0
		c, k, ok := extent.PlaneStop(generic, r3.Identity(), z, generic)
		require.True(t, ok)
		require.Equal(t, 0, c.Cmp(ratOf(z)), "c is exactly the record's level")
		require.GreaterOrEqual(t, k, 1.0)
		// k covers |m|/|m·N'| exactly: k²·(m·N')² ≥ m·m, here m·N' and m·m
		// read from the same frame.
		u, v, n := generic.U(), generic.V(), generic.N()
		var mr, nr [3]*big.Rat
		ur, vr := [3]*big.Rat{ratOf(u.X), ratOf(u.Y), ratOf(u.Z)}, [3]*big.Rat{ratOf(v.X), ratOf(v.Y), ratOf(v.Z)}
		nr = [3]*big.Rat{ratOf(n.X), ratOf(n.Y), ratOf(n.Z)}
		for i := range mr {
			j, l := (i+1)%3, (i+2)%3
			mr[i] = new(big.Rat).Sub(new(big.Rat).Mul(ur[j], vr[l]), new(big.Rat).Mul(ur[l], vr[j]))
		}
		dot := func(a, b [3]*big.Rat) *big.Rat {
			sum := new(big.Rat)
			for i := range a {
				sum.Add(sum, new(big.Rat).Mul(a[i], b[i]))
			}
			return sum
		}
		mn := dot(mr, nr)
		lhs := new(big.Rat).Mul(ratOf(k*k), new(big.Rat).Mul(mn, mn))
		require.GreaterOrEqual(t, lhs.Cmp(dot(mr, mr)), 0)
	})

	t.Run("quarter turn placement", func(t *testing.T) {
		t.Parallel()
		turn, err := r3.Rotation(r3.NewVec(0, 0, 1), units.Degrees(90))
		require.NoError(t, err)
		c, k, ok := extent.PlaneStop(xy, turn, 10, xy)
		require.True(t, ok)
		require.Equal(t, 0, c.Cmp(big.NewRat(10, 1)))
		require.Equal(t, 1.0, k)
	})

	t.Run("tilted sweep", func(t *testing.T) {
		t.Parallel()
		tilted := mustFrame(t, r3.Vec{}, r3.NewVec(1, 0, -1e-3), r3.NewVec(0, 1, 0))
		_, _, ok := extent.PlaneStop(xy, r3.Identity(), 10, tilted)
		require.False(t, ok)
	})

	t.Run("sweep a rounding step off the plane", func(t *testing.T) {
		t.Parallel()
		skewed := mustFrame(t, generic.Origin(), generic.U().Add(generic.V()), generic.V().Sub(generic.U()))
		_, _, ok := extent.PlaneStop(generic, r3.Identity(), 10, skewed)
		require.False(t, ok, "m·U' is nonzero in rationals")
	})

	t.Run("sweep perpendicular to the plane", func(t *testing.T) {
		t.Parallel()
		side := mustFrame(t, r3.Vec{}, r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1))
		_, _, ok := extent.PlaneStop(xy, r3.Identity(), 10, side)
		require.False(t, ok)
	})

	for _, tc := range []struct {
		name  string
		frame r3.Frame
		z     float64
	}{
		{name: "zero frame", frame: r3.Frame{}, z: 10},
		{name: "NaN level", frame: xy, z: math.NaN()},
		{name: "infinite level", frame: xy, z: math.Inf(1)},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			_, _, ok := extent.PlaneStop(tc.frame, r3.Identity(), tc.z, xy)
			require.False(t, ok)
		})
	}
}

func TestStopLevelError(t *testing.T) {
	t.Parallel()
	c := big.NewRat(10, 1)
	require.Zero(t, extent.StopLevelError(c, 1, 0, 10))
	require.Zero(t, extent.StopLevelError(c, 1, 0.5, 10.5))
	require.Positive(t, extent.StopLevelError(c, 1, 0.1, 10.1), "float64(0.1) is not 1/10")
	require.True(t, math.IsInf(extent.StopLevelError(c, math.NaN(), 0, 10), 1))
	require.True(t, math.IsInf(extent.StopLevelError(nil, 1, 0, 10), 1))
}
