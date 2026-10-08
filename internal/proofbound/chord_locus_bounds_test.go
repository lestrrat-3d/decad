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

// chordLocusExcursionExact is ChordLocusBuiltExcursion's expression over
// rationals: how far the built enclosure reaches past the interval the wide
// and narrow enclosures span.
func chordLocusExcursionExact(wide, wideBound, narrow, narrowBound, built, builtBound float64) *big.Rat {
	w, wb, n, nb, b, bb := ratOf(wide), ratOf(wideBound), ratOf(narrow), ratOf(narrowBound), ratOf(built), ratOf(builtBound)
	top := new(big.Rat).Sub(w, wb)
	if alt := new(big.Rat).Sub(n, nb); alt.Cmp(top) > 0 {
		top = alt
	}
	bottom := new(big.Rat).Add(w, wb)
	if alt := new(big.Rat).Add(n, nb); alt.Cmp(bottom) < 0 {
		bottom = alt
	}
	above := new(big.Rat).Sub(new(big.Rat).Add(b, bb), top)
	below := new(big.Rat).Sub(bottom, new(big.Rat).Sub(b, bb))
	out := new(big.Rat)
	if above.Cmp(out) > 0 {
		out = above
	}
	if below.Cmp(out) > 0 {
		out = below
	}
	return out
}

// envelopeSlackExact is |wide − narrow| + wideBound + narrowBound over
// rationals.
func envelopeSlackExact(wide, wideBound, narrow, narrowBound float64) *big.Rat {
	sum := new(big.Rat).Abs(new(big.Rat).Sub(ratOf(wide), ratOf(narrow)))
	return sum.Add(sum, ratOf(wideBound)).Add(sum, ratOf(narrowBound))
}

// TestChordLocusVolumeAllowRoundsOutward checks the chord-versus-locus flux
// term is never below its own expression evaluated exactly, |W − N| plus both
// reference bounds plus the built excursion ε plus the corner flux, over a
// randomized sweep that draws the built flux inside the reference interval,
// past either end of it, and with the two references in either order (a
// clockwise-walked patch negates all three). Rows with the built flux past
// an end must charge that excursion on top of the envelope, and an input it
// cannot read must refuse rather than read zero.
//
// Shown to fail on 2026-10-09: with ε left out of the sum, the excursion rows
// and the sweep's outside draws fall below the exact expression; with the
// excursion taken against [N, W] in that order only, the negated inside row
// charges an excursion of 2 it does not have.
func TestChordLocusVolumeAllowRoundsOutward(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(7, 11))
	for range 2000 {
		wide := (rng.Float64() - 0.5) * 1e6
		narrow := wide * (1 - rng.Float64()*1e-3)
		// built ranges from well below the interval to well above it.
		built := narrow + (wide-narrow)*(rng.Float64()*3-1)
		if rng.IntN(2) == 0 {
			wide, narrow, built = -wide, -narrow, -built
		}
		wideBound, narrowBound, builtBound := rng.Float64()*1e-6, rng.Float64()*1e-6, rng.Float64()*1e-6
		corner := rng.Float64() * 1e3
		got := proofbound.ChordLocusVolumeAllow(wide, wideBound, narrow, narrowBound, built, builtBound, corner)
		lower := envelopeSlackExact(wide, wideBound, narrow, narrowBound)
		lower.Add(lower, chordLocusExcursionExact(wide, wideBound, narrow, narrowBound, built, builtBound))
		lower.Add(lower, ratOf(corner))
		require.GreaterOrEqual(t, ratOf(got).Cmp(lower), 0,
			`wide=%v narrow=%v built=%v: %v is below the exact expression`, wide, narrow, built, got)
	}

	for _, tc := range []struct {
		name                string
		wide, narrow, built float64
		excursion           float64
	}{
		{name: `inside`, wide: 10, narrow: 6, built: 8},
		{name: `above the wide flux`, wide: 10, narrow: 6, built: 11, excursion: 1},
		{name: `below the narrow flux`, wide: 10, narrow: 6, built: 4, excursion: 2},
		{name: `negated, inside`, wide: -10, narrow: -6, built: -8},
		{name: `negated, below`, wide: -10, narrow: -6, built: -11, excursion: 1},
	} {
		eps := proofbound.ChordLocusBuiltExcursion(tc.wide, 0, tc.narrow, 0, tc.built, 0)
		require.Equal(t, tc.excursion, eps, `%s: the excursion`, tc.name)
		require.Equal(t, math.Abs(tc.wide-tc.narrow)+tc.excursion,
			proofbound.ChordLocusVolumeAllow(tc.wide, 0, tc.narrow, 0, tc.built, 0, 0), `%s: the term`, tc.name)
	}

	require.Zero(t, proofbound.ChordLocusVolumeAllow(0, 0, 0, 0, 0, 0, 0), `three equal fluxes charge nothing`)
	require.Equal(t, 0.5, proofbound.ChordLocusVolumeAllow(0, 0, 0, 0, 0, 0, 0.5), `the corner flux is charged alone`)
	for _, corner := range []float64{math.Inf(1), math.NaN(), -1} {
		require.True(t, math.IsInf(proofbound.ChordLocusVolumeAllow(1, 0, 1, 0, 1, 0, corner), 1),
			`a corner flux of %v states no bound`, corner)
	}
	for _, bad := range []float64{math.Inf(1), math.NaN(), -1} {
		require.True(t, math.IsInf(proofbound.ChordLocusVolumeAllow(1, 0, 1, 0, 1, bad, 0), 1),
			`a built bound of %v states no bound`, bad)
		require.True(t, math.IsInf(proofbound.ChordLocusBuiltExcursion(1, bad, 1, 0, 1, 0), 1),
			`a wide bound of %v states no excursion`, bad)
	}
	require.True(t, math.IsInf(proofbound.ChordLocusVolumeAllow(math.NaN(), 0, 1, 0, 1, 0, 0), 1),
		`a flux that does not lift states no bound`)
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
