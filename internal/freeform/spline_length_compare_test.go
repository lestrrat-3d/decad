package freeform_test

import (
	"math"
	"math/big"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/stretchr/testify/require"
)

// randomMagnitude is a non-negative integer of up to maxBits bits, zero about
// one time in eight, and a power of two about one time in eight so a·2^s and
// b can share their leading bits and differ only below them.
func randomMagnitude(r *rand.Rand, maxBits int) *big.Int {
	out := new(big.Int)
	switch r.IntN(8) {
	case 0:
		return out
	case 1:
		return out.Lsh(big.NewInt(1), uint(r.IntN(maxBits)))
	}
	width := 1 + r.IntN(maxBits)
	for out.BitLen() < width {
		out.Lsh(out, 64)
		out.Or(out, new(big.Int).SetUint64(r.Uint64()))
	}
	out.Rsh(out, uint(out.BitLen()-width))
	return out
}

// TestShiftedCmpMatchesShiftedBigIntCmp holds the word-level comparison to
// big.Int's own shift and compare, on zero operands, equal values, values that
// agree in every word but the lowest, and arbitrary ones.
func TestShiftedCmpMatchesShiftedBigIntCmp(t *testing.T) {
	t.Parallel()
	r := rand.New(rand.NewPCG(21, 22))
	cases := 20_000
	if raceDetector {
		cases = 2_000
	}
	check := func(a *big.Int, s uint, b *big.Int) {
		want := new(big.Int).Lsh(a, s).Cmp(b)
		require.Equal(t, want, freeform.ShiftedCmp(a.Bits(), s, b.Bits()), "a %v s %d b %v", a, s, b)
	}
	zero, one := new(big.Int), big.NewInt(1)
	for _, s := range []uint{0, 1, 63, 64, 65, 200} {
		check(zero, s, zero)
		check(zero, s, one)
		check(one, s, zero)
		check(one, s, new(big.Int).Lsh(one, s))
	}
	for range cases {
		a := randomMagnitude(r, 300)
		s := uint(r.IntN(260))
		var b *big.Int
		switch r.IntN(4) {
		case 0:
			b = randomMagnitude(r, 600)
		default:
			// The same value, or one unit either side of it, so only the
			// lowest word of the shifted operand decides.
			b = new(big.Int).Lsh(a, s)
			b.Add(b, big.NewInt(int64(r.IntN(3)-1)))
			if b.Sign() < 0 {
				b.SetInt64(0)
			}
		}
		check(a, s, b)
	}
}

// TestSpanSquareCmpScratchMatchesSpanSquareCmp holds the scratch comparison,
// with its machine-word product and word-level shift, to SpanSquareCmp's
// big.Int form. The squared distances are exact squares of the float under
// test, one unit either side, and arbitrary values, over one-word and wider
// denSq, every exponent sign and zero and negative numerators.
func TestSpanSquareCmpScratchMatchesSpanSquareCmp(t *testing.T) {
	t.Parallel()
	r := rand.New(rand.NewPCG(23, 24))
	cases := 20_000
	if raceDetector {
		cases = 2_000
	}
	floats := []float64{
		0, math.Copysign(0, -1), math.NaN(), math.Inf(1), math.Inf(-1),
		1, -1, 5, math.MaxFloat64, math.SmallestNonzeroFloat64, 0x1p-1022, 0x1.8p-1070,
	}
	denSqs := []*big.Int{
		big.NewInt(1), big.NewInt(9), big.NewInt(81),
		new(big.Int).SetUint64(math.MaxUint64),
		new(big.Int).Lsh(big.NewInt(3), 70),
		new(big.Int).Exp(big.NewInt(3), big.NewInt(100), nil),
	}
	var scratch freeform.LengthDistanceScratch
	check := func(f float64, d freeform.SpanSquaredDistance) {
		want := freeform.SpanSquareCmp(f, d)
		got := freeform.SpanSquareCmpScratch(f, d, &scratch)
		require.Equal(t, want, got, "f %x num %v denSq %v exp %d", math.Float64bits(f), d.Num, d.DenSq, d.Exp)
	}
	for range cases {
		var f float64
		if r.IntN(4) == 0 {
			f = floats[r.IntN(len(floats))]
		} else {
			f = math.Float64frombits(r.Uint64())
		}
		denSq := denSqs[r.IntN(len(denSqs))]
		exp := uint(r.IntN(600))
		var num *big.Int
		switch r.IntN(4) {
		case 0:
			num = randomMagnitude(r, 2200)
			if r.IntN(8) == 0 {
				num.Neg(num)
			}
		default:
			// f²·denSq·2^(2 exp) is an integer when f's exponent allows it,
			// so the comparison lands on equality or one unit off it.
			if math.IsNaN(f) || math.IsInf(f, 0) || f == 0 {
				num = randomMagnitude(r, 64)
				break
			}
			if r.IntN(2) == 0 {
				// An exponent past 1074 makes even a subnormal f's square an
				// integer numerator.
				exp = 1075 + uint(r.IntN(64))
			}
			fr := new(big.Rat).SetFloat64(f)
			fr.Mul(fr, fr)
			fr.Mul(fr, new(big.Rat).SetInt(new(big.Int).Lsh(denSq, 2*exp)))
			if !fr.IsInt() {
				num = new(big.Int).Quo(fr.Num(), fr.Denom())
				break
			}
			num = new(big.Int).Add(fr.Num(), big.NewInt(int64(r.IntN(3)-1)))
		}
		check(f, freeform.SpanSquaredDistance{Num: num, DenSq: denSq, Exp: exp})
	}
}

// BenchmarkSpanSquareCmpScratch measures one leaf comparison of the
// fit-spline shape: a seeded root against a three-word squared numerator over
// a one-word denSq.
func BenchmarkSpanSquareCmpScratch(b *testing.B) {
	r := rand.New(rand.NewPCG(25, 26))
	num := randomMagnitude(r, 380)
	num.SetBit(num, 379, 1)
	d := freeform.SpanSquaredDistance{Num: num, DenSq: big.NewInt(9), Exp: 190}
	f := freeform.SpanSqrtSeed(d)
	var scratch freeform.LengthDistanceScratch
	b.ReportAllocs()
	for b.Loop() {
		if freeform.SpanSquareCmpScratch(f, d, &scratch) == 2 {
			b.Fatal("impossible comparison")
		}
	}
}
