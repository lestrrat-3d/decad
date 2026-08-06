package decad

import (
	"math"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// This file tests bounds.go's perturbedTriangleAreaAllow, the one new
// helper docs/loft-design.md §12 PR 2a extracts from
// perturbedAreaUpperWithBudget's own per-facet term.

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
