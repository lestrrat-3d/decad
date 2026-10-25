package freeform_test

import (
	"math"
	"math/big"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/stretchr/testify/require"
)

func TestDownRoundMatchesNextafter(t *testing.T) {
	t.Parallel()
	ref := func(x float64) float64 {
		if x <= 0 || math.IsNaN(x) || math.IsInf(x, 0) {
			return x
		}
		return math.Nextafter(x, 0)
	}
	vals := []float64{
		0, math.Copysign(0, -1), math.Inf(1), math.Inf(-1), math.NaN(), math.Float64frombits(0x7ff0000000000001),
		math.MaxFloat64, -math.MaxFloat64, math.Nextafter(math.MaxFloat64, 0),
		math.SmallestNonzeroFloat64, -math.SmallestNonzeroFloat64, 0x1p-1022, 0x1p-969, 1, -1,
	}
	r := rand.New(rand.NewPCG(3, 4))
	for range 100_000 {
		vals = append(vals, math.Float64frombits(r.Uint64()))
	}
	for _, x := range vals {
		require.Equal(t, math.Float64bits(ref(x)), math.Float64bits(freeform.DownRound(x)), "%x", math.Float64bits(x))
	}
}

func TestSpanSqrtSeedsMatchFullDivisionAndKeepExactBounds(t *testing.T) {
	var scratch freeform.LengthDistanceScratch
	for _, width := range []uint{1, 31, 32, 63, 64, 65, 127, 128, 513, 2048} {
		base := new(big.Int).Lsh(big.NewInt(1), width)
		for _, extra := range []int64{1, 1024} {
			num := new(big.Int).Add(base, big.NewInt(extra))
			num.Mul(num, base)
			denSq := new(big.Int).Mul(base, base)
			for _, shift := range []uint{0, 1, 31, 512, 1100, 2200} {
				d := freeform.SpanSquaredDistance{Num: num, DenSq: denSq, Exp: shift}
				mant := new(big.Float).SetPrec(64)
				ratio := new(big.Float).SetPrec(64).Quo(new(big.Float).SetInt(num), new(big.Float).SetInt(denSq))
				exp := ratio.MantExp(mant) - 2*int(shift)
				if exp%2 != 0 {
					exp--
					mant.SetMantExp(mant, 1)
				}
				m, _ := mant.Float64()
				want := math.Float64bits(math.Ldexp(math.Sqrt(m), exp/2))
				require.Equal(t, want, math.Float64bits(freeform.SpanSqrtSeed(d)))
				require.Equal(t, want, math.Float64bits(freeform.SpanSqrtSeedScratch(d, &scratch)))
				lower, upper := freeform.SpanSqrtDownScratch(d, &scratch), freeform.SpanSqrtUpScratch(d, &scratch)
				require.Equal(t, math.Float64bits(freeform.SpanSqrtDown(d)), math.Float64bits(lower))
				require.Equal(t, math.Float64bits(freeform.SpanSqrtUp(d)), math.Float64bits(upper))
				q := new(big.Rat).SetFrac(num, new(big.Int).Lsh(new(big.Int).Set(denSq), 2*shift))
				lo := new(big.Rat).SetFloat64(lower)
				require.LessOrEqual(t, lo.Mul(lo, lo).Cmp(q), 0)
				if !math.IsInf(upper, 1) {
					hi := new(big.Rat).SetFloat64(upper)
					require.GreaterOrEqual(t, hi.Mul(hi, hi).Cmp(q), 0)
				}
			}
		}
	}
}
