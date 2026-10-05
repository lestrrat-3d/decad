package decad

import (
	"math"
	"math/big"
	"math/rand/v2"
	"testing"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/stretchr/testify/require"
)

// TestFloatRatMatchesSetFloat64 pins floatRat and dyadic.rat, which both build
// their big.Rat without the GCD normalisation, against big.Rat.SetFloat64 in
// the reduced form: the same value, numerator, denominator and string. Every
// later big.Rat operation reads those fields, so the value comparing equal is
// not enough on its own.
//
// Shown to fail: deleting floatRat's TrailingZeros64 normalisation turns the
// numerator leg red while the value still compares equal, and deleting
// dyadic.rat's Denom shift turns the widened-dyadic denominator leg red.
// floatRat's value == 0 guard is proven redundant rather than shown to fail:
// without it, Frexp(±0) gives a zero mantissa, TrailingZeros64(0) is 64, and
// Go defines a shift by 64 of an int64 as 0, so the Rat built is 0/1 either
// way. The guard stays so the zero case does not rest on that shift rule.
func TestFloatRatMatchesSetFloat64(t *testing.T) {
	t.Parallel()
	check := func(t *testing.T, f float64) {
		t.Helper()
		want := new(big.Rat)
		if want.SetFloat64(f) == nil {
			require.Nil(t, proofarith.FloatRat(f), "%v has no rational", f)
			return
		}
		got := proofarith.FloatRat(f)
		require.NotNil(t, got, "%v", f)
		require.Equal(t, 0, want.Cmp(got), "value %v", f)
		require.Equal(t, want.Num().String(), got.Num().String(), "numerator %v", f)
		require.Equal(t, want.Denom().String(), got.Denom().String(), "denominator %v", f)
		require.Equal(t, want.String(), got.String(), "%v", f)
		require.Equal(t, want.IsInt(), got.IsInt(), "%v", f)

		d, ok := proofarith.DyOf(f)
		require.True(t, ok, "%v is finite and must lift", f)
		dr := d.Rat()
		require.Equal(t, 0, want.Cmp(dr), "dyadic value %v", f)
		require.Equal(t, want.Num().String(), dr.Num().String(), "dyadic numerator %v", f)
		require.Equal(t, want.Denom().String(), dr.Denom().String(), "dyadic denominator %v", f)

		// A dyadic no float holds, with a mantissa wider than 53 bits, reads
		// back as the test's own independent composition of it.
		wide := proofarith.DyAdd(proofarith.DyMul(d, d), proofarith.DyShift(proofarith.MustDyOf(1), -1080))
		wr, ww := wide.Rat(), ratOfDyadic(t, wide)
		require.Equal(t, ww.Num().String(), wr.Num().String(), "widened numerator %v", f)
		require.Equal(t, ww.Denom().String(), wr.Denom().String(), "widened denominator %v", f)

		// The result is usable as an operand and as a receiver afterwards.
		require.Equal(t, 0, new(big.Rat).Add(want, want).Cmp(new(big.Rat).Add(got, got)), "%v", f)
		got.Add(got, big.NewRat(1, 3))
		require.Equal(t, 0, new(big.Rat).Add(want, big.NewRat(1, 3)).Cmp(got), "%v", f)
		dr.Mul(dr, big.NewRat(2, 7))
		require.Equal(t, 0, new(big.Rat).Mul(want, big.NewRat(2, 7)).Cmp(dr), "%v", f)
	}
	for _, f := range []float64{
		0, math.Copysign(0, -1), 1, -1, 0.5, 0.1, -0.1, 10, 25, 1e300, -1e-300,
		math.MaxFloat64, -math.MaxFloat64, math.SmallestNonzeroFloat64,
		-math.SmallestNonzeroFloat64, 0x1p-1022, -0x1p-1022, 1.5, math.Pi,
		math.NaN(), math.Inf(1), math.Inf(-1), 0x1p52, 0x1p53 + 2, 0x1p53 + 1,
	} {
		check(t, f)
	}
	rng := rand.New(rand.NewPCG(1, 2))
	for range 20000 {
		check(t, math.Float64frombits(rng.Uint64()))
		check(t, rng.NormFloat64()*1e3)
		check(t, float64(rng.IntN(2000)-1000))
	}
}
