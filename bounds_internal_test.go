package decad

import (
	"math"
	"math/big"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file tests bounds.go's perturbedTriangleAreaAllow, the one new
// helper docs/loft-design.md §12 PR 2a extracts from
// perturbedAreaUpperWithBudget's own per-facet term, and rigidRoundAllow's
// own saturated-scale answer, which that PR's placement path is the first
// consumer to reach at an extreme coordinate.

// TestRigidRoundAllowIsAlwaysAFiniteBound pins bounds.go's rigidRoundAllow
// answering a finite, positive bound at every magnitude, including the ones
// whose 2·maxInputAbs + maxTransAbs scale leaves the finite float64 range.
// NaN is the failure this guards: it is not a large bound but the absence of
// one, and a consumer's own `delta > 0` widening skips it silently, so a
// placement's volume term would vanish from the very measurements it is
// there to widen.
func TestRigidRoundAllowIsAlwaysAFiniteBound(t *testing.T) {
	// The largest scale whose ulp is still readable: 2·maxInputAbs +
	// maxTransAbs sits one binade below MaxFloat64, so nothing saturates and
	// the answer is the plain 16-ulp charge.
	unsaturated := rigidRoundAllow(math.Ldexp(1, 1021), 0)
	require.Equal(t, radius3D(16*math.Ldexp(1, 1022-52)), unsaturated)

	for _, tc := range []struct {
		name       string
		input      float64
		trans      float64
		mustExceed float64
	}{
		{"ordinary magnitudes", 100, 10, 0},
		// 2*x overflows for any coordinate above MaxFloat64/2 — the
		// reported reproduction's own magnitude.
		{"input scale overflows", 0.75 * math.MaxFloat64, 1, unsaturated},
		{"both at the ceiling", math.MaxFloat64, math.MaxFloat64, unsaturated},
		// ulpOf itself answers +Inf exactly at MaxFloat64, because its own
		// math.Nextafter step leaves the finite range.
		{"scale lands exactly on MaxFloat64", math.MaxFloat64 / 2, 0, unsaturated},
	} {
		t.Run(tc.name, func(t *testing.T) {
			got := rigidRoundAllow(tc.input, tc.trans)
			require.False(t, math.IsNaN(got), "a NaN bound is no bound at all")
			require.False(t, math.IsInf(got, 0), "the bound must stay finite")
			require.Greater(t, got, 0.0)
			require.GreaterOrEqual(t, got, tc.mustExceed,
				"a saturated scale must charge at least what the largest readable scale does")
		})
	}
}

// TestRigidRoundAllowEnclosesAPlacedCoordinate proves the saturated answer is
// a real bound and not merely a finite number: at a coordinate whose scale
// saturates, the helper's answer must still enclose the displacement an
// actual rigid motion commits on that coordinate.
func TestRigidRoundAllowEnclosesAPlacedCoordinate(t *testing.T) {
	x := 0.75 * math.MaxFloat64
	allow := rigidRoundAllow(x, 1)

	// A rotation about Z by 37 degrees, then a 1 mm translation: the placed
	// coordinates round inside their own products and sums, which is exactly
	// what allow speaks for.
	rot, err := r3.Rotation(r3.NewVec(0, 0, 1), units.Degrees(37))
	require.NoError(t, err)
	trans, err := r3.Translation(r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	move, err := rot.Then(trans)
	require.NoError(t, err)

	for _, p := range []r3.Vec{
		r3.NewVec(x, 0, 0),
		r3.NewVec(x/2, x/4, 0),
		r3.NewVec(-x/2, 0, x/8),
	} {
		got := move.Apply(p)
		require.True(t, finiteVec(got), "allow speaks only for a placed point the caller has proven finite")

		want := exactApply(move, p)
		dist := new(big.Float).SetPrec(400)
		for i, held := range []float64{got.X, got.Y, got.Z} {
			d := new(big.Float).SetPrec(400).Sub(big.NewFloat(held).SetPrec(400), want[i])
			dist.Add(dist, d.Mul(d, d))
		}
		require.Positive(t, dist.Sign(), "the fixture must actually commit rounding, or it proves nothing")
		require.LessOrEqual(t, dist.Cmp(new(big.Float).SetPrec(400).Mul(big.NewFloat(allow), big.NewFloat(allow))), 0,
			"the placed point %v sits further from its exact image than the %.17g bound allows", p, allow)
	}
}

// exactApply is move.Apply in arbitrary precision: the same three products,
// two sums and translation, with no rounding of its own.
func exactApply(move r3.Transform, p r3.Vec) [3]*big.Float {
	b := move.Basis()
	tr := move.Translation()

	term := func(basis, coord float64) *big.Float {
		return new(big.Float).SetPrec(400).Mul(big.NewFloat(basis).SetPrec(400), big.NewFloat(coord).SetPrec(400))
	}
	axis := func(bx, by, bz, t float64) *big.Float {
		sum := new(big.Float).SetPrec(400).Add(term(bx, p.X), term(by, p.Y))
		sum.Add(sum, term(bz, p.Z))
		return sum.Add(sum, big.NewFloat(t).SetPrec(400))
	}
	return [3]*big.Float{
		axis(b.EX.X, b.EY.X, b.EZ.X, tr.X),
		axis(b.EX.Y, b.EY.Y, b.EZ.Y, tr.Y),
		axis(b.EX.Z, b.EY.Z, b.EZ.Z, tr.Z),
	}
}

// TestWalkEndpointAllow pins docs/prism-boolean-design.md §7's δ_walk
// mechanism (task fu143): a positive OPERAND envelope — the magnitude of the
// values the walk's own arithmetic touches, which is what this helper's
// contract requires and never the endpoint that arithmetic produced — answers
// a bound at least the worst-case lerp2 rounding its own doc comment derives
// at that magnitude, a zero envelope answers zero, and a non-finite envelope
// answers +Inf rather than a number a caller's own `> 0` widening could skip.
//
// That the CALLER hands over the right envelope is walkChargeOf's own claim,
// proven against exact rational residuals over cancelling carriers in
// prism_boolean_internal_test.go's TestWalkChargeOfCoversLerpCancellation.
func TestWalkEndpointAllow(t *testing.T) {
	t.Run("zero envelope gives zero", func(t *testing.T) {
		require.Equal(t, 0.0, walkEndpointAllow(0))
	})

	t.Run("positive envelope exceeds the derived lerp2 rounding at that magnitude", func(t *testing.T) {
		for _, envelope := range []float64{12, 1, 1e-3, 1e6, 1e12} {
			got := walkEndpointAllow(envelope)
			require.Positive(t, got)
			require.False(t, math.IsInf(got, 0))
			// The helper's own derivation: lerp2's general arm rounds three
			// times — the difference, the product, the sum — for at most
			// 5·u·E per coordinate at operand magnitude E. The published
			// bound is a 3D radius over 16 ulps of 2E and must clear it.
			require.Greaterf(t, got, 5*unitRoundoff*envelope,
				"the bound at envelope %g must contain the derived per-coordinate worst case", envelope)
			require.Greaterf(t, got, ulpOf(envelope), "the bound at envelope %g must exceed one ulp there", envelope)
		}
	})

	t.Run("larger envelope gives a larger bound", func(t *testing.T) {
		require.Greater(t, walkEndpointAllow(1e6), walkEndpointAllow(12))
	})

	for _, tc := range []struct {
		name  string
		input float64
	}{
		{"+Inf", math.Inf(1)},
		{"NaN", math.NaN()},
	} {
		t.Run(tc.name+" envelope gives +Inf, never 0", func(t *testing.T) {
			got := walkEndpointAllow(tc.input)
			require.True(t, math.IsInf(got, 1), "an absent bound must never read as a small one")
		})
	}
}

// TestPerturbedTriangleAreaAllowEnclosesBruteForceSweep proves the
// extracted helper against a brute-force sweep of perturbed vertices
// (required test, §13): for every one of many random perturbations of up
// to delta per vertex, the resulting triangle's own area must lie within
// [heldArea - allow, heldArea + allow], at aspect ratios 1, 1e-3 and 1e-6 —
// the same slivers loft_moments_internal_test.go's area-bound tests use,
// since a thin triangle is exactly where a naive bound scaled off the held
// total stops enclosing anything.
func TestPerturbedTriangleAreaAllowEnclosesBruteForceSweep(t *testing.T) {
	triArea := func(a, b, c r3.Vec) float64 {
		return b.Sub(a).Cross(c.Sub(a)).Len() / 2
	}

	// Two orthonormal, non-axis-aligned directions, matching
	// loft_moments_internal_test.go's own sliver construction.
	dir := r3.NewVec(0.6, 0.8, 0)
	perp := r3.NewVec(-0.48, 0.36, 0.8)
	require.InDelta(t, 0, dir.Dot(perp), 1e-15)

	sliver := func(base, aspect float64) (r3.Vec, r3.Vec, r3.Vec) {
		origin := r3.NewVec(3, -5, 7)
		a := origin
		b := origin.Add(dir.Scale(base))
		c := origin.Add(dir.Scale(base * 0.37)).Add(perp.Scale(base * aspect))
		return a, b, c
	}

	rng := rand.New(rand.NewPCG(1, 2))
	randomUnit := func() r3.Vec {
		for {
			v := r3.NewVec(rng.Float64()*2-1, rng.Float64()*2-1, rng.Float64()*2-1)
			if l := v.Len(); l > 1e-9 {
				return v.Scale(1 / l)
			}
		}
	}

	for _, tc := range []struct {
		name   string
		aspect float64
		delta  float64
	}{
		{"aspect 1", 1, 1e-3},
		{"aspect 1e-3", 1e-3, 1e-3},
		{"aspect 1e-6", 1e-6, 1e-9},
	} {
		t.Run(tc.name, func(t *testing.T) {
			a, b, c := sliver(10, tc.aspect)
			held := triArea(a, b, c)
			allow := perturbedTriangleAreaAllow(a, b, c, tc.delta)
			require.Greater(t, allow, 0.0)

			for range 20000 {
				pa := a.Add(randomUnit().Scale(rng.Float64() * tc.delta))
				pb := b.Add(randomUnit().Scale(rng.Float64() * tc.delta))
				pc := c.Add(randomUnit().Scale(rng.Float64() * tc.delta))
				got := triArea(pa, pb, pc)
				require.LessOrEqualf(t, math.Abs(got-held), allow,
					"perturbed area %.17g must stay within %.17g of held %.17g", got, allow, held)
			}
		})
	}
}
