package proof_test

import (
	"bytes"
	"encoding/binary"
	"fmt"
	"math"
	"math/big"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/stretchr/testify/require"
)

// TestDyadicFastPathsRandomizedMatchesBigRat exercises arbitrary finite
// float64 bit patterns, including subnormals and values near the largest
// finite exponent. The reference deliberately stays in math/big so the test
// does not repeat the proof.Dyadic implementation's alignment logic.
func TestDyadicFastPathsRandomizedMatchesBigRat(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(0x243F6A8885A308D3, 0x13198A2E03707344))
	randomFinite := func() float64 {
		for {
			bits := rng.Uint64() &^ (uint64(0x7ff) << 52)
			bits |= (rng.Uint64() & 0x7fe) << 52
			f := math.Float64frombits(bits)
			if math.IsNaN(f) || math.IsInf(f, 0) {
				continue
			}
			return f
		}
	}
	for range 1000 {
		a, b := randomFinite(), randomFinite()
		da, db := proof.MustDyOf(a), proof.MustDyOf(b)
		ra, rb := proof.FloatRat(a), proof.FloatRat(b)
		require.Zero(t, independentDyadicRat(proof.DyAdd(da, db)).Cmp(new(big.Rat).Add(ra, rb)), "%v + %v", a, b)
		require.Zero(t, independentDyadicRat(proof.DySubScalar(da, db)).Cmp(new(big.Rat).Sub(ra, rb)), "%v - %v", a, b)
		require.Equal(t, ra.Cmp(rb), proof.DyCmp(da, db), "cmp(%v, %v)", a, b)
	}

	// These operands force the widest exponent gap representable by finite
	// float64 values while keeping the reference rational practical to build.
	large := proof.DyShift(proof.MustDyOf(1), 1023)
	small := proof.DyShift(proof.MustDyOf(1), -1074)
	largeRat, smallRat := independentDyadicRat(large), independentDyadicRat(small)
	require.Equal(t, largeRat.Cmp(smallRat), proof.DyCmp(large, small))
	require.Zero(t, independentDyadicRat(proof.DyAdd(large, small)).Cmp(new(big.Rat).Add(largeRat, smallRat)))
	require.Zero(t, independentDyadicRat(proof.DySubScalar(large, small)).Cmp(new(big.Rat).Sub(largeRat, smallRat)))
}

// TestDyadicCompareAcrossExponents compares every pair of a grid of values
// whose mantissas and exponents differ, so their leading bits fall at the same
// binary position as often as at different ones, against big.Rat.
func TestDyadicCompareAcrossExponents(t *testing.T) {
	t.Parallel()
	var values []proof.Dyadic
	for _, mant := range []int64{1, 3, 5, 7, 11, 13, 255, 257, -1, -3, -7, -11, -255} {
		for exp := -4; exp <= 4; exp++ {
			values = append(values, proof.DyShift(proof.DyInt(mant), exp))
		}
	}
	values = append(values, proof.DyZero())
	for _, a := range values {
		for _, b := range values {
			want := independentDyadicRat(a).Cmp(independentDyadicRat(b))
			require.Equal(t, want, proof.DyCmp(a, b), "cmp(%v, %v)", independentDyadicRat(a), independentDyadicRat(b))
		}
	}
}

// TestDyadicFastPathsPreserveInputs ensures each result owns the only mutable
// big.Int touched by the operation.
func TestDyadicFastPathsPreserveInputs(t *testing.T) {
	t.Parallel()
	values := []proof.Dyadic{
		proof.MustDyOf(-math.MaxFloat64),
		proof.MustDyOf(-0.1),
		proof.DyZero(),
		proof.MustDyOf(math.SmallestNonzeroFloat64),
		proof.MustDyOf(math.MaxFloat64),
	}
	for _, a := range values {
		for _, b := range values {
			aExp, bExp := a.Exp(), b.Exp()
			am, bm := new(big.Int), new(big.Int)
			if a.Mant() != nil {
				am.Set(a.Mant())
			}
			if b.Mant() != nil {
				bm.Set(b.Mant())
			}
			proof.DyAdd(a, b)
			proof.DySubScalar(a, b)
			proof.DyCmp(a, b)
			if a.Mant() != nil {
				require.Equal(t, 0, a.Mant().Cmp(am))
			}
			if b.Mant() != nil {
				require.Equal(t, 0, b.Mant().Cmp(bm))
			}
			require.Equal(t, aExp, a.Exp())
			require.Equal(t, bExp, b.Exp())
		}
	}
}

func independentDyadicRat(d proof.Dyadic) *big.Rat {
	if d.Mant() == nil {
		return new(big.Rat)
	}
	out := new(big.Rat).SetInt(d.Mant())
	if d.Exp() >= 0 {
		return out.Mul(out, new(big.Rat).SetInt(new(big.Int).Lsh(big.NewInt(1), uint(d.Exp()))))
	}
	return out.Quo(out, new(big.Rat).SetInt(new(big.Int).Lsh(big.NewInt(1), uint(-d.Exp()))))
}

// boundaryBitLens are mantissa widths on both sides of every 64-bit word
// boundary up to and past the inline width (proof.InlineBits), where a carry,
// a product word or an aligning shift moves a value between the inline words
// and big.Int.
var boundaryBitLens = []int{1, 2, 52, 53, 54, 62, 63, 64, 65, 66, 126, 127, 128, 129, 190, 191, 192, 193,
	194, 254, 255, 256, 257, 258, 300, 383, 384, 385, 511, 512, 513}

// boundaryMantissa returns a random odd mantissa of exactly n bits. A third
// of them are all ones, which carries out of every word on an add and gives a
// product its full width; a third are 2^(n-1)+1, whose products are one bit
// short of full width.
func boundaryMantissa(rng *rand.Rand, n int) *big.Int {
	one := big.NewInt(1)
	switch rng.IntN(3) {
	case 0:
		m := new(big.Int).Lsh(one, uint(n))
		return m.Sub(m, one)
	case 1:
		if n == 1 {
			return big.NewInt(1)
		}
		m := new(big.Int).Lsh(one, uint(n-1))
		return m.Add(m, one)
	}
	m := new(big.Int)
	for m.BitLen() < n {
		m.Lsh(m, 64)
		m.Or(m, new(big.Int).SetUint64(rng.Uint64()))
	}
	m.Rsh(m, uint(m.BitLen()-n))
	m.SetBit(m, n-1, 1)
	return m.SetBit(m, 0, 1)
}

// dyadicFromParts builds mant × 2^exp through big.Rat and DyOfRat, so the
// operands do not come from the arithmetic under test.
func dyadicFromParts(t *testing.T, mant *big.Int, exp int) proof.Dyadic {
	t.Helper()
	r := new(big.Rat).SetInt(mant)
	scale := new(big.Rat).SetInt(new(big.Int).Lsh(big.NewInt(1), uint(abs(exp))))
	if exp >= 0 {
		r.Mul(r, scale)
	} else {
		r.Quo(r, scale)
	}
	d, ok := proof.DyOfRat(r)
	require.True(t, ok)
	return d
}

// requireDyadicIs checks d against the exact value want through every reading
// the type offers: its mantissa and exponent, Rat, Sign, IsZero, MantInto into
// reused storage, Float64 against big.Rat's own rounding, and the canonical
// form, which a round trip through big.Rat must reproduce field for field.
func requireDyadicIs(t *testing.T, want *big.Rat, d proof.Dyadic, msg string) {
	t.Helper()
	require.Zero(t, independentDyadicRat(d).Cmp(want), msg)
	require.Zero(t, d.Rat().Cmp(want), "%s: Rat", msg)
	require.Equal(t, want.Sign(), d.Sign(), "%s: Sign", msg)
	require.Equal(t, want.Sign() == 0, d.IsZero(), "%s: IsZero", msg)
	if d.Mant() != nil {
		require.Equal(t, 1, int(d.Mant().Bit(0)), "%s: an odd mantissa", msg)
	}
	storage := big.NewInt(12345)
	require.Same(t, storage, d.MantInto(storage))
	if d.Mant() == nil {
		require.Zero(t, storage.Sign(), "%s: MantInto of zero", msg)
	} else {
		require.Zero(t, storage.Cmp(d.Mant()), "%s: MantInto", msg)
	}
	wantF, wantExact := want.Float64()
	if wantF == 0 && want.Sign() != 0 {
		// big.Rat.Float64 reports a non-zero value that underflows to zero as
		// exact. Dyadic.Float64 follows big.Float, which reports it inexact,
		// and that is the claim a directed rounding needs.
		wantExact = false
	}
	gotF, gotExact := d.Float64()
	require.Equal(t, math.Float64bits(wantF), math.Float64bits(gotF), "%s: Float64 %v want %v", msg, gotF, wantF)
	require.Equal(t, wantExact, gotExact, "%s: Float64 exactness", msg)
	back, ok := proof.DyOfRat(want)
	require.True(t, ok)
	require.Equal(t, back, d, "%s: canonical form", msg)
}

// TestDyadicInlineBoundariesMatchBigRat drives every operation across the
// inline mantissa's boundaries — sums that carry out of the top word,
// aligning shifts that reach the width exactly, products one bit either side
// of it, operands already wider than it, both signs and zero — and checks
// each result against big.Rat.
//
// Legs shown to fail, each by breaking one inline path in dyadic.go: ignoring
// a sum's carry into bit 127; dropping that carry when building the wider
// sum; letting shl128 drop bits shifted past the top or mis-shift by 64 or
// more; subtracting the magnitudes in the wrong order; skipping either
// product's high-bit check; flipping the product's sign rule; dropping a
// product cross term; comparing two inline values without the aligning
// shift; dropping Float64's sticky bit; admitting magnitudes up to 2^1025 to
// its inline conversion; and reporting an inline conversion exact up to 54
// bits.
func TestDyadicInlineBoundariesMatchBigRat(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(0xB7E151628AED2A6A, 0xBF7158809CF4F3C7))
	operand := func(n int) (proof.Dyadic, *big.Rat) {
		if n == 0 {
			return proof.DyZero(), new(big.Rat)
		}
		mant := boundaryMantissa(rng, n)
		if rng.IntN(2) == 0 {
			mant.Neg(mant)
		}
		exp := rng.IntN(41) - 20
		if rng.IntN(8) == 0 {
			// Exponents whose float values are subnormal or overflow.
			exp += (rng.IntN(2)*2 - 1) * (1000 + rng.IntN(100))
		}
		d := dyadicFromParts(t, mant, exp)
		return d, independentDyadicRat(d)
	}
	lens := append([]int{0}, boundaryBitLens...)
	for range 30000 {
		a, ra := operand(lens[rng.IntN(len(lens))])
		b, rb := operand(lens[rng.IntN(len(lens))])
		if !a.IsZero() && !b.IsZero() && rng.IntN(2) == 0 {
			// Choose b's exponent so the aligning shift lands on the inline
			// width: a's bit length plus the gap is one bit short of it, on
			// it, or one bit past it.
			gap := proof.InlineBits - a.Mant().BitLen() + rng.IntN(3) - 1
			if gap < 0 {
				gap = -gap
			}
			b = dyadicFromParts(t, b.Mant(), a.Exp()-gap)
			rb = independentDyadicRat(b)
		}
		msg := fmt.Sprintf("a=%s b=%s", ra.String(), rb.String())
		requireDyadicIs(t, ra, a, msg+": a")
		requireDyadicIs(t, new(big.Rat).Add(ra, rb), proof.DyAdd(a, b), msg+": add")
		requireDyadicIs(t, new(big.Rat).Sub(ra, rb), proof.DySubScalar(a, b), msg+": sub")
		requireDyadicIs(t, new(big.Rat).Mul(ra, rb), proof.DyMul(a, b), msg+": mul")
		requireDyadicIs(t, new(big.Rat).Neg(ra), proof.DyNeg(a), msg+": neg")
		requireDyadicIs(t, new(big.Rat).Abs(ra), proof.DyAbs(a), msg+": abs")
		shift := rng.IntN(600) - 300
		scaled := new(big.Rat).SetInt(new(big.Int).Lsh(big.NewInt(1), uint(abs(shift))))
		if shift < 0 {
			scaled.Inv(scaled)
		}
		requireDyadicIs(t, scaled.Mul(scaled, ra), proof.DyShift(a, shift), msg+": shift")
		require.Equal(t, ra.Cmp(rb), proof.DyCmp(a, b), msg+": cmp")
		require.Equal(t, rb.Cmp(ra), proof.DyCmp(b, a), msg+": reverse cmp")
		require.Equal(t, ra.Cmp(rb) == 0, bytes.Equal(a.AppendKey(nil), b.AppendKey(nil)), msg+": key")
	}
}

// TestDyadicInlineKeyMatchesBigIntBytes pins AppendKey's encoding of an inline
// mantissa to the bytes big.Int.Bytes gives for the same magnitude, the
// encoding a mantissa held in a big.Int uses, so a value encodes one way
// wherever it is stored.
func TestDyadicInlineKeyMatchesBigIntBytes(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(0x9E3779B97F4A7C15, 0xF39CC0605CEDC834))
	for _, n := range boundaryBitLens {
		for range 50 {
			mant := boundaryMantissa(rng, n)
			exp := rng.IntN(41) - 20
			d := dyadicFromParts(t, mant, exp)
			want := []byte{1}
			want = binary.AppendVarint(want, int64(exp))
			want = binary.AppendUvarint(want, uint64(len(mant.Bytes())))
			want = append(want, mant.Bytes()...)
			require.Equal(t, want, d.AppendKey(nil), "%d-bit mantissa", n)
			want[0] = 2
			require.Equal(t, want, proof.DyNeg(d).AppendKey(nil), "negative %d-bit mantissa", n)
		}
	}
}

// TestDyadicFloat64AtRangeEdges converts values whose leading bit sits on
// either side of the smallest normal float64 and of the overflow threshold,
// for mantissas narrower and wider than a float's 53 bits, against big.Float
// rounding the whole mantissa once. Below the normal range the result is
// subnormal and holds fewer than 53 bits, so a conversion that rounded to 53
// bits first and scaled second would round twice, and one that reported a
// 53-bit mantissa exact would claim a bit the result dropped.
//
// Legs shown to fail: admitting magnitudes from 2^-1023 to the inline
// conversion, and admitting them up to 2^1025.
func TestDyadicFloat64AtRangeEdges(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(0x3C6EF372FE94F82B, 0xA54FF53A5F1D36F1))
	widths := []int{1, 2, 3, 52, 53, 54, 55, 64, 65, 100, 128, 129, 191, 192, 193, 300}
	tops := []int{}
	for top := -1080; top <= -1015; top++ {
		tops = append(tops, top)
	}
	for top := 1018; top <= 1028; top++ {
		tops = append(tops, top)
	}
	for _, n := range widths {
		for _, top := range tops {
			for range 12 {
				mant := boundaryMantissa(rng, n)
				if rng.IntN(2) == 0 {
					mant.Neg(mant)
				}
				exp := top - n
				d := dyadicFromParts(t, mant, exp)
				ref := new(big.Float).SetPrec(uint(max(n, 53))).SetInt(mant)
				ref.SetMantExp(ref, exp)
				wantF, acc := ref.Float64()
				gotF, gotExact := d.Float64()
				require.Equal(t, math.Float64bits(wantF), math.Float64bits(gotF), "%d-bit mantissa, leading bit 2^%d", n, top-1)
				require.Equal(t, acc == big.Exact, gotExact, "%d-bit mantissa, leading bit 2^%d: exactness", n, top-1)
			}
		}
	}
}
