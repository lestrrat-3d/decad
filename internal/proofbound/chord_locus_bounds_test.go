package proofbound_test

import (
	"math"
	"math/big"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/stretchr/testify/require"
)

func ratOf(x float64) *big.Rat { return new(big.Rat).SetFloat64(x) }

// TestChordLocusVolumeAllowRoundsOutward checks the chord-versus-locus flux
// term is the farther end of the denoted enclosure from the built enclosure
// plus the corner flux: never below max(0, hi − (b − bb), (b + bb) − lo) +
// corner evaluated exactly and within a few ulps above it, over a randomized
// sweep that draws the built flux inside the enclosure and past either end.
// Rows with the built flux on one side of the enclosure must charge the
// far end, and an input it cannot read, or an empty enclosure, must refuse.
//
// Shown to fail on 2026-10-09: with the term reading only how far the built
// flux sits above the enclosure's bottom, the sweep's draws above the middle
// and the row below the enclosure fall below the exact expression; with the
// corner flux left out, every draw does.
func TestChordLocusVolumeAllowRoundsOutward(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(7, 11))
	for range 2000 {
		lo := (rng.Float64() - 0.5) * 1e6
		hi := lo + rng.Float64()*1e3
		built := lo + (hi-lo)*(rng.Float64()*3-1)
		builtBound := rng.Float64() * 1e-6
		corner := rng.Float64() * 1e3
		got := proofbound.ChordLocusVolumeAllow(lo, hi, built, builtBound, corner)
		above := new(big.Rat).Sub(ratOf(hi), new(big.Rat).Sub(ratOf(built), ratOf(builtBound)))
		below := new(big.Rat).Sub(new(big.Rat).Add(ratOf(built), ratOf(builtBound)), ratOf(lo))
		exact := new(big.Rat)
		if above.Cmp(exact) > 0 {
			exact = above
		}
		if below.Cmp(exact) > 0 {
			exact = below
		}
		exact.Add(exact, ratOf(corner))
		require.GreaterOrEqual(t, ratOf(got).Cmp(exact), 0,
			`lo=%v hi=%v built=%v: %v is below the exact expression`, lo, hi, built, got)
		ceiling, _ := new(big.Rat).Mul(exact, big.NewRat(1000000000001, 1000000000000)).Float64()
		require.LessOrEqual(t, got, ceiling, `lo=%v hi=%v built=%v: %v is past the exact expression`, lo, hi, built, got)
	}
	for _, tc := range []struct {
		name          string
		lo, hi, built float64
		want          float64
	}{
		{name: `inside, nearer the top`, lo: 6, hi: 10, built: 9, want: 3},
		{name: `inside, nearer the bottom`, lo: 6, hi: 10, built: 7, want: 3},
		{name: `above`, lo: 6, hi: 10, built: 11, want: 5},
		{name: `below`, lo: 6, hi: 10, built: 4, want: 6},
	} {
		require.Equal(t, tc.want, proofbound.ChordLocusVolumeAllow(tc.lo, tc.hi, tc.built, 0, 0), tc.name)
	}
	require.Zero(t, proofbound.ChordLocusVolumeAllow(0, 0, 0, 0, 0), `a point enclosure at the built flux charges nothing`)
	require.Equal(t, 0.5, proofbound.ChordLocusVolumeAllow(0, 0, 0, 0, 0.5), `the corner flux is charged alone`)
	for _, bad := range []float64{math.Inf(1), math.NaN(), -1} {
		require.True(t, math.IsInf(proofbound.ChordLocusVolumeAllow(1, 2, 1.5, 0, bad), 1), `a corner flux of %v states no bound`, bad)
		require.True(t, math.IsInf(proofbound.ChordLocusVolumeAllow(1, 2, 1.5, bad, 0), 1), `a built bound of %v states no bound`, bad)
	}
	require.True(t, math.IsInf(proofbound.ChordLocusVolumeAllow(2, 1, 1.5, 0, 0), 1), `an empty enclosure states no bound`)
	require.True(t, math.IsInf(proofbound.ChordLocusVolumeAllow(math.NaN(), 1, 1, 0, 0), 1), `an enclosure that does not lift states no bound`)
}

// shellExact is ChordLocusShellUpper's expression over rationals:
// height·radius·(window·radius·Φ²/4 + (s0 + s1)·(3/8)·radius·Φ²), Φ the larger
// skew.
func shellExact(radius, window, s0, s1, height float64) *big.Rat {
	phi := ratOf(math.Max(s0, s1))
	phi2 := new(big.Rat).Mul(phi, phi)
	built := new(big.Rat).Mul(new(big.Rat).Mul(ratOf(radius), big.NewRat(1, 4)), phi2)
	corner := new(big.Rat).Mul(new(big.Rat).Mul(ratOf(radius), big.NewRat(3, 8)), phi2)
	sum := new(big.Rat).Mul(ratOf(window), built)
	sum.Add(sum, new(big.Rat).Mul(new(big.Rat).Add(ratOf(s0), ratOf(s1)), corner))
	return sum.Mul(sum, new(big.Rat).Mul(ratOf(height), ratOf(radius)))
}

// TestChordLocusRegionAllowRoundsOutward checks the region term is three times
// the shell, never below that expression evaluated exactly and within a few
// ulps above it, over a randomized sweep; that zero skews charge nothing; and
// that an input it cannot read refuses.
//
// Shown to fail on 2026-10-09: with the shell composed without its factor 3,
// or with either deficit's factor cut (1/4 to 1/8, 3/8 to 1/4), the sweep
// falls below the exact expression, and with a swept-volume term still added,
// it rises past the ceiling.
func TestChordLocusRegionAllowRoundsOutward(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(19, 23))
	for range 2000 {
		radius := 0.5 + rng.Float64()*100
		window := rng.Float64() * 6.3
		s0, s1 := rng.Float64()*1.5, rng.Float64()*1.5
		height := rng.Float64() * 1e3
		got := proofbound.ChordLocusRegionAllow(radius, window, s0, s1, height)
		exact := shellExact(radius, window, s0, s1, height)
		exact.Mul(exact, big.NewRat(3, 1))
		require.GreaterOrEqual(t, ratOf(got).Cmp(exact), 0,
			`radius=%v window=%v skews=%v,%v height=%v: %v is below the exact expression`, radius, window, s0, s1, height, got)
		ceiling, _ := new(big.Rat).Mul(exact, big.NewRat(1000000000001, 1000000000000)).Float64()
		require.LessOrEqual(t, got, ceiling,
			`radius=%v window=%v skews=%v,%v height=%v: %v carries more than the shell`, radius, window, s0, s1, height, got)
	}
	require.Zero(t, proofbound.ChordLocusRegionAllow(10, 1, 0, 0, 2), `zero skews charge nothing`)
	for _, bad := range []float64{math.Inf(1), math.NaN(), -1} {
		require.True(t, math.IsInf(proofbound.ChordLocusRegionAllow(10, 1, bad, 0.1, 2), 1),
			`a skew of %v states no bound`, bad)
		require.True(t, math.IsInf(proofbound.ChordLocusRegionAllow(10, bad, 0.1, 0.1, 2), 1),
			`a window of %v states no bound`, bad)
		require.True(t, math.IsInf(proofbound.ChordLocusShellUpper(bad, 1, 0.1, 0.1, 2), 1),
			`a radius of %v states no shell`, bad)
		require.True(t, math.IsInf(proofbound.ChordLocusBuiltDeficitUpper(bad, 0.1), 1),
			`a radius of %v states no built deficit`, bad)
		require.True(t, math.IsInf(proofbound.ChordLocusCornerDeficitUpper(bad, 0.1), 1),
			`a radius of %v states no corner deficit`, bad)
	}
}

// TestChordLocusCornerFluxRoundsOutward checks one corner's share
// (ds/dc)·moment is never below its exact rational value over a randomized
// sweep, that a zero moment charges nothing, and that an input it cannot
// read refuses.
//
// Shown to fail on 2026-10-09: with the quotient and product taken in plain
// float arithmetic, the sweep finds draws below the exact value.
func TestChordLocusCornerFluxRoundsOutward(t *testing.T) {
	t.Parallel()
	rat := func(x float64) *big.Rat { return new(big.Rat).SetFloat64(x) }
	rng := rand.New(rand.NewPCG(13, 17))
	for range 20000 {
		ds := rng.Float64() * 100
		dc := 1e-3 + rng.Float64()*100
		moment := rng.Float64() * 1e4
		got := proofbound.ChordLocusCornerFlux(ds, dc, moment)
		exact := new(big.Rat).Quo(new(big.Rat).Mul(rat(ds), rat(moment)), rat(dc))
		require.GreaterOrEqual(t, rat(got).Cmp(exact), 0,
			`ds=%v dc=%v moment=%v: %v is below the exact share`, ds, dc, moment, got)
	}
	require.Zero(t, proofbound.ChordLocusCornerFlux(1, 1, 0), `a straight locus charges nothing`)
	for _, in := range [][3]float64{{math.NaN(), 1, 1}, {1, 0, 1}, {1, 1, math.Inf(1)}, {1, 1, -1}, {-1, 1, 1}} {
		require.True(t, math.IsInf(proofbound.ChordLocusCornerFlux(in[0], in[1], in[2]), 1),
			`inputs %v state no bound`, in)
	}
}
