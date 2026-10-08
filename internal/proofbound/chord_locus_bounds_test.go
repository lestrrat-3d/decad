package proofbound_test

import (
	"math"
	"math/big"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/stretchr/testify/require"
)

// chordLocusVolumeLower is ChordLocusVolumeAllow's own expression evaluated
// as an exact LOWER bound: the flux difference and the two bounds exactly,
// √(R0·R1) rounded down, sin(Φ/2) at the low end of its certified enclosure,
// and every product and sum over rationals.
func chordLocusVolumeLower(wide, wideBound, narrow, narrowBound, r0, r1, skew, area float64) *big.Rat {
	rat := func(x float64) *big.Rat { return new(big.Rat).SetFloat64(x) }
	sum := new(big.Rat).Abs(new(big.Rat).Sub(rat(wide), rat(narrow)))
	sum.Add(sum, rat(wideBound)).Add(sum, rat(narrowBound))
	sin, _, ok := proofbound.RadSinCosInterval(new(big.Rat).Mul(rat(skew), big.NewRat(1, 2)))
	if !ok {
		return nil
	}
	radial := new(big.Rat).Mul(rat(proofbound.RatSqrtDown(new(big.Rat).Mul(rat(r0), rat(r1)))), sin.Lo)
	deviation := new(big.Rat).Add(radial, new(big.Rat).Mul(rat(math.Max(r0, r1)), rat(skew)))
	swept := new(big.Rat).Mul(deviation, rat(area))
	return sum.Add(sum, swept.Mul(swept, big.NewRat(3, 1)))
}

// TestChordLocusVolumeAllowRoundsOutward checks the chord-versus-locus term
// is never below its own expression evaluated exactly, over a randomized sweep
// of radii, skews below a quarter turn, areas and flux pairs, and that a skew
// it cannot read refuses rather than reading zero.
//
// Shown to fail: the non-finite rows fail against the previous form, which
// answered 0 for a non-finite skew. No row of the sweep went below the exact
// expression with the previous float evaluation either (200000 draws: its
// final upward rounding of the sum covered the uncharged square root, sine
// and product); the sweep pins that the outward-rounded form keeps it so.
func TestChordLocusVolumeAllowRoundsOutward(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(7, 11))
	for range 2000 {
		r0 := 0.5 + rng.Float64()*100
		r1 := r0 * (0.25 + rng.Float64()*1.5)
		skew := rng.Float64() * 1.5
		area := rng.Float64() * 1e4
		wide := (rng.Float64() - 0.5) * 1e6
		narrow := wide * (1 - rng.Float64()*1e-3)
		wideBound, narrowBound := rng.Float64()*1e-6, rng.Float64()*1e-6
		got := proofbound.ChordLocusVolumeAllow(wide, wideBound, narrow, narrowBound, r0, r1, skew, area)
		lower := chordLocusVolumeLower(wide, wideBound, narrow, narrowBound, r0, r1, skew, area)
		require.NotNil(t, lower)
		require.GreaterOrEqual(t, new(big.Rat).SetFloat64(got).Cmp(lower), 0,
			`r0=%v r1=%v skew=%v area=%v: %v is below the exact expression`, r0, r1, skew, area, got)
	}

	require.Zero(t, proofbound.ChordLocusVolumeAllow(1, 0, 1, 0, 10, 9, 0, 100), `a zero skew charges nothing`)
	for _, skew := range []float64{math.Inf(1), math.NaN()} {
		require.True(t, math.IsInf(proofbound.ChordLocusVolumeAllow(1, 0, 1, 0, 10, 9, skew, 100), 1),
			`a skew of %v states no bound`, skew)
	}
}
