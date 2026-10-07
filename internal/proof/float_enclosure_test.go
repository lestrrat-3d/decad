package proof_test

import (
	"math"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/stretchr/testify/require"
)

// enclosureDyadic draws a dyadic whose mantissa usually needs more than 53
// bits, a product of up to three random floats, so its float conversion
// rounds; one draw in sixteen is zero.
func enclosureDyadic(rng *rand.Rand) proof.Dyadic {
	if rng.IntN(16) == 0 {
		return proof.DyZero()
	}
	d := proof.MustDyOf(rng.Float64()*16 - 8)
	for range rng.IntN(3) {
		d = proof.DyMul(d, proof.MustDyOf(rng.Float64()*4-2))
	}
	return d
}

// floatAtMost reports whether the float f is at or below the exact d.
func floatAtMost(f float64, d proof.Dyadic) bool {
	if math.IsInf(f, 0) {
		return f < 0
	}
	return proof.DyCmp(proof.MustDyOf(f), d) <= 0
}

// TestFloatBoundsBracketDyadic holds FloatBounds to lo ≤ d ≤ hi.
//
// Leg shown to fail: either Nextafter step removed, the nearest float lies on
// the wrong side of d for about half the inexact draws.
func TestFloatBoundsBracketDyadic(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(83, 89))
	inexact := 0
	for range 20000 {
		d := enclosureDyadic(rng)
		lo, hi := proof.FloatBounds(d)
		require.True(t, floatAtMost(lo, d), "lo %v above %v", lo, d.Rat())
		require.True(t, floatAtMost(-hi, proof.DyNeg(d)), "hi %v below %v", hi, d.Rat())
		if lo != hi {
			inexact++
		}
	}
	require.Positive(t, inexact, "premise: some conversions round")
}

// TestDotSubEnclosureBracketsExact holds DotSubEnclosure to lo ≤ x·y − c ≤ hi
// for exact x, y and c drawn with long mantissas, read through their float
// bounds, including draws where x·y and c nearly cancel.
//
// Legs shown to fail: dropping the outward step of the products, of the sums,
// or of the final subtraction each puts the exact value outside the
// enclosure on some draws.
func TestDotSubEnclosureBracketsExact(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(97, 101))
	tight := 0
	for range 40000 {
		var x, y proof.DyV3
		for k := range 3 {
			x[k], y[k] = enclosureDyadic(rng), enclosureDyadic(rng)
		}
		dot := proof.DvDot(x, y)
		c := enclosureDyadic(rng)
		if rng.IntN(2) == 0 {
			// c within a few ulps of x·y, where the enclosure is tightest.
			f, _ := dot.Float64()
			for range rng.IntN(4) {
				f = math.Nextafter(f, math.Inf(1-2*rng.IntN(2)))
			}
			c = proof.MustDyOf(f)
		}
		cLo, cHi := proof.FloatBounds(c)
		lo, hi := proof.DotSubEnclosure(proof.DvFloatBox(x), proof.DvFloatBox(y), cLo, cHi)
		exact := proof.DySubScalar(dot, c)
		require.True(t, floatAtMost(lo, exact), "lo %v above %v", lo, exact.Rat())
		require.True(t, floatAtMost(-hi, proof.DyNeg(exact)), "hi %v below %v", hi, exact.Rat())
		if lo <= 0 && hi >= 0 && exact.Sign() != 0 {
			tight++
		}
	}
	require.Positive(t, tight, "premise: some enclosures straddle zero around a nonzero value")
}

// TestDotSubEnclosureUndecidedOnNaN holds an infinity times zero to a NaN
// enclosure, which no ordered comparison reads as a decided sign.
func TestDotSubEnclosureUndecidedOnNaN(t *testing.T) {
	t.Parallel()
	x := proof.FloatBox3{Lo: [3]float64{math.Inf(-1), 0, 0}, Hi: [3]float64{1, 0, 0}}
	y := proof.FloatBox3{Lo: [3]float64{0, 0, 0}, Hi: [3]float64{0, 0, 0}}
	lo, hi := proof.DotSubEnclosure(x, y, 0, 0)
	require.True(t, math.IsNaN(lo))
	require.True(t, math.IsNaN(hi))
}
