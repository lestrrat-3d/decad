package proofbound_test

import (
	"fmt"
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// TestSumSlopNeverZeroForSubnormalScale pins SumSlop's documented invariant
// down to the subnormal range: a positive finite absSum never yields a zero
// bound. Both of SumSlop's relative charges are at least 2⁻⁵¹·absSum, so they
// round to +0 once absSum falls below about 2⁻¹⁰²⁴, and only the absolute
// per-term underflow charge keeps the bound positive there. The rows near
// 2⁻¹⁰²² and at 1 sit on the other side of that edge.
//
// Shown to fail first: with the underflow charge removed (SumSlop returning
// UpRound(loop + terms)), the 2⁻¹⁰⁷⁴, 2⁻¹⁰⁷³ and 1e-320 rows return 0 and go
// red at every n, while the rows from 2⁻¹⁰²³ up stay green.
func TestSumSlopNeverZeroForSubnormalScale(t *testing.T) {
	t.Parallel()
	scales := []float64{
		math.SmallestNonzeroFloat64,
		math.Ldexp(1, -1073),
		1e-320,
		math.Ldexp(1, -1023),
		math.Nextafter(math.Ldexp(1, -1022), 0),
		math.Ldexp(1, -1022),
		1,
	}
	for _, n := range []int{1, 2, 8, 1_000_000} {
		for _, s := range scales {
			t.Run(fmt.Sprintf("n=%d/absSum=%g", n, s), func(t *testing.T) {
				t.Parallel()
				require.Positive(t, proofbound.SumSlop(n, s))
			})
		}
	}
}

// TestSumSlopEnclosesSubnormalFacetAreaError measures the error SumSlop speaks
// for directly: facet areas summed the way boolean_body.go and
// facetproof.MeshAreaUpper sum them, b.Sub(a).Cross(c.Sub(a)).Len()/2 in a
// naive loop, against the exact area sum. Every facet lies in the z = 0
// plane, so its exact area is |u_x·v_y − u_y·v_x|/2, a rational computed here
// exactly over big.Rat.
//
// The facets are scaled so their cross products land in the subnormal range,
// where a product's rounding is absolute rather than relative: the first is
// held at 2⁻¹⁰⁷⁴ against an exact 1.25·2⁻¹⁰⁷⁴, the second at 5·2⁻¹⁰⁷⁴
// against an exact 5.25·2⁻¹⁰⁷⁴. Both relative charges round to +0 at those
// scales, so the absolute per-term underflow charge is the whole bound.
//
// Shown to fail first: with the underflow charge removed (SumSlop returning
// UpRound(loop + terms)), SumSlop returns 0 for every row and every row goes
// red against its positive exact error.
func TestSumSlopEnclosesSubnormalFacetAreaError(t *testing.T) {
	t.Parallel()
	origin := r3.NewVec(0, 0, 0)
	single := [3]r3.Vec{origin, r3.NewVec(math.Ldexp(1, -520), 0, 0), r3.NewVec(0, math.Ldexp(5, -555), 0)}
	wider := [3]r3.Vec{origin, r3.NewVec(math.Ldexp(3, -520), 0, 0), r3.NewVec(0, math.Ldexp(7, -555), 0)}

	for _, tc := range []struct {
		name   string
		facets [][3]r3.Vec
	}{
		{name: "one facet", facets: [][3]r3.Vec{single}},
		{name: "four equal facets", facets: [][3]r3.Vec{single, single, single, single}},
		{name: "mixed facets", facets: [][3]r3.Vec{single, wider, single, wider, wider}},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			held := 0.0
			exact := new(big.Rat)
			for _, f := range tc.facets {
				a, b, c := f[0], f[1], f[2]
				held += b.Sub(a).Cross(c.Sub(a)).Len() / 2
				exact.Add(exact, exactPlanarFacetArea(a, b, c))
			}
			gap := new(big.Rat).Sub(exact, new(big.Rat).SetFloat64(held))
			gap.Abs(gap)
			require.Positive(t, gap.Sign(), "the fixture must actually round")

			bound := proofbound.SumSlop(len(tc.facets), held)
			// Log in units of the smallest subnormal, 2⁻¹⁰⁷⁴, where the
			// fixture's arithmetic lives.
			grid := new(big.Rat).SetFloat64(math.SmallestNonzeroFloat64)
			inGrid := func(x *big.Rat) string { return new(big.Rat).Quo(x, grid).FloatString(2) }
			t.Logf("held=%s exact=%s gap=%s bound=%s (units of 2^-1074)",
				inGrid(new(big.Rat).SetFloat64(held)), inGrid(exact), inGrid(gap), inGrid(new(big.Rat).SetFloat64(bound)))
			require.GreaterOrEqual(t, new(big.Rat).SetFloat64(bound).Cmp(gap), 0,
				"SumSlop must enclose the subnormal facet areas' own evaluation error")
		})
	}
}

// exactPlanarFacetArea is the exact area of a facet lying in a z = const
// plane: half the absolute z component of (b − a) × (c − a), over big.Rat.
func exactPlanarFacetArea(a, b, c r3.Vec) *big.Rat {
	rat := func(x float64) *big.Rat { return new(big.Rat).SetFloat64(x) }
	ux := new(big.Rat).Sub(rat(b.X), rat(a.X))
	uy := new(big.Rat).Sub(rat(b.Y), rat(a.Y))
	vx := new(big.Rat).Sub(rat(c.X), rat(a.X))
	vy := new(big.Rat).Sub(rat(c.Y), rat(a.Y))
	z := new(big.Rat).Sub(new(big.Rat).Mul(ux, vy), new(big.Rat).Mul(uy, vx))
	z.Abs(z)
	return z.Quo(z, big.NewRat(2, 1))
}
