package proof_test

import (
	"bytes"
	"math/big"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/stretchr/testify/require"
)

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
