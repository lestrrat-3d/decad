package apitest_test

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file checks a placed loft's cap and body areas against the area of the
// solid its record denotes: the plane-local loft carried through the held
// placement basis read as exact rationals (agent-docs/loft-design.md §8).
// r3.Transform.Then does not re-orthonormalize, so a long chain of float
// rotations holds a basis whose area factor departs from 1 by far more than
// the caps' own rounding and placement allowance.

// loftCapMapRat reads a float exactly.
func loftCapMapRat(x float64) *big.Rat { return new(big.Rat).SetFloat64(x) }

// loftCapMapBasis reads a placement's held basis columns as exact rationals.
func loftCapMapBasis(pose r3.Transform) [3][3]*big.Rat {
	b := pose.Basis()
	var out [3][3]*big.Rat
	for k, col := range []r3.Vec{b.EX, b.EY, b.EZ} {
		out[k] = [3]*big.Rat{loftCapMapRat(col.X), loftCapMapRat(col.Y), loftCapMapRat(col.Z)}
	}
	return out
}

// loftCapMapApply is B·p over exact rationals, B given by its columns.
func loftCapMapApply(cols [3][3]*big.Rat, p [3]*big.Rat) [3]*big.Rat {
	var out [3]*big.Rat
	for i := range 3 {
		out[i] = new(big.Rat)
		for k := range 3 {
			out[i].Add(out[i], new(big.Rat).Mul(cols[k][i], p[k]))
		}
	}
	return out
}

// loftCapMapCrossNorm2 is |a × b|² over exact rationals.
func loftCapMapCrossNorm2(a, b [3]*big.Rat) *big.Rat {
	sum := new(big.Rat)
	for i := range 3 {
		j, k := (i+1)%3, (i+2)%3
		c := new(big.Rat).Sub(new(big.Rat).Mul(a[j], b[k]), new(big.Rat).Mul(a[k], b[j]))
		sum.Add(sum, c.Mul(c, c))
	}
	return sum
}

// loftCapMapVec lifts plane-local integers to an exact rational vector.
func loftCapMapVec(x, y, z int64) [3]*big.Rat {
	return [3]*big.Rat{big.NewRat(x, 1), big.NewRat(y, 1), big.NewRat(z, 1)}
}

// loftCapMapDriftedPose composes n float rotations. Each Then rounds the
// basis again, so the held basis drifts from orthonormal.
func loftCapMapDriftedPose(t *testing.T, n int) r3.Transform {
	t.Helper()
	step, err := r3.Rotation(r3.NewVec(1, 2, 3), units.Radians(0.1))
	require.NoError(t, err)
	pose := r3.Identity()
	for range n {
		pose, err = pose.Then(step)
		require.NoError(t, err)
	}
	return pose
}

// requireAreaCovers asserts that the true area, whose exact square is
// trueSquared, lies within the published reading: (v − b)² ≤ trueSquared ≤
// (v + b)², every term an exact rational.
func requireAreaCovers(t *testing.T, what string, got decad.Measurement, trueSquared *big.Rat) {
	t.Helper()
	value, bound := got.Value.Base(), got.Bound.Base()
	lo := new(big.Rat).Sub(loftCapMapRat(value), loftCapMapRat(bound))
	hi := new(big.Rat).Add(loftCapMapRat(value), loftCapMapRat(bound))
	require.True(t, lo.Sign() <= 0 || new(big.Rat).Mul(lo, lo).Cmp(trueSquared) <= 0,
		"%s: true area lies below %v - %v", what, value, bound)
	require.True(t, trueSquared.Cmp(new(big.Rat).Mul(hi, hi)) <= 0,
		"%s: true area lies above %v + %v", what, value, bound)
}

// loftCapMapFace finds the one planar face whose held area is near want.
func loftCapMapFace(t *testing.T, body *decad.Body, want float64) decad.Measurement {
	t.Helper()
	var found []decad.Measurement
	for _, f := range body.Faces() {
		a, err := f.Area()
		require.NoError(t, err)
		if math.Abs(a.Value.Base()-want) < 1e-3 {
			found = append(found, a)
		}
	}
	require.Len(t, found, 1)
	return found[0]
}

func TestLoftCapAreaChargesPlacementStretch(t *testing.T) {
	t.Parallel()
	// A 100×100 square at z = 0 lofts to a 60×60 square at z = 2, both on
	// world-XY frames through the origin, so the plane-local lift is exact and
	// the denoted map is the placement basis B alone.
	s0, p0, s1, p1 := loftSquaresAt(t, r3.NewVec(0, 0, 0), 50, 30, 2)
	body, err := decad.Loft(t.Context(), decad.New(), s0, p0, s1, p1)
	require.NoError(t, err)
	pose := loftCapMapDriftedPose(t, 5000)
	placed, err := decad.Transform(t.Context(), body, pose)
	require.NoError(t, err)

	b := loftCapMapBasis(pose)
	// Each cap lies in a plane spanned by X and Y, so its true area is the
	// plane-local area times |B·X × B·Y|.
	capFactor2 := loftCapMapCrossNorm2(b[0], b[1])
	t.Run("caps", func(t *testing.T) {
		for _, c := range []struct {
			name string
			area int64
		}{{"start", 10000}, {"end", 3600}} {
			got := loftCapMapFace(t, placed, float64(c.area))
			want := new(big.Rat).Mul(capFactor2, big.NewRat(c.area*c.area, 1))
			requireAreaCovers(t, c.name, got, want)
		}
	})
	t.Run("body", func(t *testing.T) {
		// The true body area is both caps plus four planar trapezoid walls.
		// Each trapezoid is two triangles with exact plane-local corners; its
		// area is a sum of square roots, so the walls are read at 256 bits.
		corners := func(h int64) [4][3]*big.Rat {
			z := int64(0)
			if h == 30 {
				z = 2
			}
			return [4][3]*big.Rat{
				loftCapMapVec(-h, -h, z), loftCapMapVec(h, -h, z),
				loftCapMapVec(h, h, z), loftCapMapVec(-h, h, z),
			}
		}
		bottom, top := corners(50), corners(30)
		triangle := func(a, c, d [3]*big.Rat) *big.Float {
			ab := [3]*big.Rat{new(big.Rat).Sub(c[0], a[0]), new(big.Rat).Sub(c[1], a[1]), new(big.Rat).Sub(c[2], a[2])}
			ad := [3]*big.Rat{new(big.Rat).Sub(d[0], a[0]), new(big.Rat).Sub(d[1], a[1]), new(big.Rat).Sub(d[2], a[2])}
			n2 := loftCapMapCrossNorm2(loftCapMapApply(b, ab), loftCapMapApply(b, ad))
			f := new(big.Float).SetPrec(256).SetRat(n2)
			return f.Sqrt(f).Quo(f, big.NewFloat(2))
		}
		total := new(big.Float).SetPrec(256)
		for j := range 4 {
			jn := (j + 1) % 4
			total.Add(total, triangle(bottom[j], bottom[jn], top[jn]))
			total.Add(total, triangle(bottom[j], top[jn], top[j]))
		}
		capRoot := new(big.Float).SetPrec(256).SetRat(capFactor2)
		capRoot.Sqrt(capRoot)
		total.Add(total, new(big.Float).Mul(capRoot, big.NewFloat(13600)))
		got, err := placed.Area()
		require.NoError(t, err)
		gap := new(big.Float).Sub(total, big.NewFloat(got.Value.Base()))
		gap.Abs(gap)
		require.True(t, gap.Cmp(big.NewFloat(got.Bound.Base())) <= 0,
			"true body area %s lies outside %v ± %v", total.Text('g', 20), got.Value.Base(), got.Bound.Base())
	})
}

func TestLoftCapAreaExactMapKeepsBound(t *testing.T) {
	t.Parallel()
	// World-XY frames through the origin and no placement denote through the
	// identity, so the cap charge is zero and each cap area stays Exact.
	s0, p0, s1, p1 := loftSquaresAt(t, r3.NewVec(0, 0, 0), 50, 30, 2)
	body, err := decad.Loft(t.Context(), decad.New(), s0, p0, s1, p1)
	require.NoError(t, err)
	for _, want := range []float64{10000, 3600} {
		got := loftCapMapFace(t, body, want)
		require.Equal(t, want, got.Value.Base())
		require.Zero(t, got.Bound.Base())
		require.Equal(t, decad.Exact, got.Exactness)
	}
}
