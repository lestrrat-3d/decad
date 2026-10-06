package proof_test

import (
	"bytes"
	"math"
	"math/big"
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

// TestDvPrimitiveNamesDirection holds DvPrimitive to its claim: the result
// is an integer vector with coprime components, a positive multiple of the
// input, and equal for two inputs exactly when one is a positive multiple of
// the other.
//
// Legs shown to fail: skipping the division by the gcd leaves multiples of a
// direction with different results; an AppendKey that drops the sign gives a
// direction and its opposite one key.
func TestDvPrimitiveNamesDirection(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(103, 107))
	component := func() proof.Dyadic {
		if rng.IntN(4) == 0 {
			return proof.DyZero()
		}
		return proof.DyShift(proof.DyInt(int64(rng.IntN(41)-20)), rng.IntN(40)-20)
	}
	for range 4000 {
		v := proof.DyV3{component(), component(), component()}
		if proof.DvIsZero(v) {
			require.True(t, proof.DvIsZero(proof.DvPrimitive(v)))
			continue
		}
		p := proof.DvPrimitive(v)
		gcd := new(big.Int)
		for k := range 3 {
			r := p[k].Rat()
			require.True(t, r.IsInt(), "component %v is not an integer", r)
			gcd.GCD(nil, nil, gcd, new(big.Int).Abs(r.Num()))
		}
		require.Zero(t, gcd.Cmp(big.NewInt(1)), "components share a factor")
		// p is a positive multiple of v: p×v = 0 and p·v > 0.
		require.True(t, proof.DvIsZero(proof.DvCross(p, v)))
		require.Positive(t, proof.DvDot(p, v).Sign())

		scale := proof.DyMul(proof.DyShift(proof.DyInt(int64(1+rng.IntN(9))), rng.IntN(30)-15),
			proof.MustDyOf(1+rng.Float64()))
		scaled := proof.DyV3{proof.DyMul(v[0], scale), proof.DyMul(v[1], scale), proof.DyMul(v[2], scale)}
		require.Equal(t, keyOf(p), keyOf(proof.DvPrimitive(scaled)))
		negated := proof.DyV3{proof.DyNeg(v[0]), proof.DyNeg(v[1]), proof.DyNeg(v[2])}
		require.NotEqual(t, keyOf(p), keyOf(proof.DvPrimitive(negated)))
		other := proof.DyV3{component(), component(), component()}
		parallel := proof.DvIsZero(proof.DvCross(v, other)) && proof.DvDot(v, other).Sign() > 0
		require.Equal(t, parallel, bytes.Equal(keyOf(p), keyOf(proof.DvPrimitive(other))))
	}
}

func keyOf(v proof.DyV3) []byte {
	var b []byte
	for _, c := range v {
		b = c.AppendKey(b)
	}
	return b
}

// TestAppendKeyMatchesEquality holds AppendKey to its claim: two keys, and
// two keys with further keys appended, are equal exactly when the values are.
//
// Leg shown to fail: dropping the mantissa's length prefix lets the two pairs
// at the end encode to the same bytes.
func TestAppendKeyMatchesEquality(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(109, 113))
	value := func() proof.Dyadic {
		if rng.IntN(8) == 0 {
			return proof.DyZero()
		}
		return proof.DyShift(proof.DyInt(int64(rng.IntN(1<<20)-1<<19)), rng.IntN(20)-10)
	}
	equal := 0
	for range 40000 {
		a0, a1, b0, b1 := value(), value(), value(), value()
		if rng.IntN(4) == 0 {
			// The same value reached another way: a0·2 then halved.
			b0 = proof.DyShift(proof.DyMul(a0, proof.DyInt(2)), -1)
			b1 = a1
		}
		ka := a1.AppendKey(a0.AppendKey(nil))
		kb := b1.AppendKey(b0.AppendKey(nil))
		want := proof.DyCmp(a0, b0) == 0 && proof.DyCmp(a1, b1) == 0
		require.Equal(t, want, bytes.Equal(ka, kb))
		if want {
			equal++
		}
	}
	require.Positive(t, equal, "premise: some pairs are equal")

	// Without the length prefix, (1, 259·2) and (257, −3·2⁻¹) both encode as
	// sign 1, exponent 0, then bytes 01 01 02 01 03.
	ka := proof.DyShift(proof.DyInt(259), 1).AppendKey(proof.DyInt(1).AppendKey(nil))
	kb := proof.DyShift(proof.DyInt(-3), -1).AppendKey(proof.DyInt(257).AppendKey(nil))
	require.NotEqual(t, ka, kb)
}
