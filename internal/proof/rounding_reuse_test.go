package proof_test

import (
	"math"
	"math/big"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/stretchr/testify/require"
)

func TestSetFloatRatMatchesSetFloat64(t *testing.T) {
	t.Parallel()
	var dst big.Rat
	check := func(value float64) {
		t.Helper()
		want := new(big.Rat).SetFloat64(value)
		got := proof.SetFloatRat(&dst, value)
		require.True(t, got == &dst, "receiver differs, bits %x", math.Float64bits(value))
		require.Zero(t, dst.Num().Cmp(want.Num()), "numerator, bits %x", math.Float64bits(value))
		require.Zero(t, dst.Denom().Cmp(want.Denom()), "denominator, bits %x", math.Float64bits(value))
	}
	for _, value := range []float64{0, math.Copysign(0, -1), 1, -1, 0.1, -0.1,
		math.MaxFloat64, -math.MaxFloat64, math.SmallestNonzeroFloat64, -math.SmallestNonzeroFloat64} {
		check(value)
	}
	for exponent := -1074; exponent <= 1023; exponent++ {
		value := math.Ldexp(1, exponent)
		check(value)
		check(math.Nextafter(value, 0))
		check(math.Nextafter(value, math.Inf(1)))
	}
	rng := rand.New(rand.NewPCG(521, 523))
	for range 4096 {
		check(finiteBits(rng))
	}
	dst.SetFrac64(7, 3)
	for _, value := range []float64{math.NaN(), math.Inf(1), math.Inf(-1), math.Float64frombits(0xfff8000000000001)} {
		require.True(t, proof.SetFloatRat(&dst, value) == nil)
		require.Zero(t, dst.Cmp(big.NewRat(7, 3)), "non-finite input changed receiver")
	}
}

func TestSetFloatRatReusesStorage(t *testing.T) {
	values := []float64{math.MaxFloat64, -math.MaxFloat64, math.SmallestNonzeroFloat64,
		-math.SmallestNonzeroFloat64, 0, 0.125, -17.25}
	var dst big.Rat
	for _, value := range values {
		proof.SetFloatRat(&dst, value)
	}
	allocs := testing.AllocsPerRun(100, func() {
		for _, value := range values {
			proof.SetFloatRat(&dst, value)
		}
	})
	require.Zero(t, allocs)
}
