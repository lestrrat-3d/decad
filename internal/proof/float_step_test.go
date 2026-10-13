package proof_test

import (
	"math"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/stretchr/testify/require"
)

// TestFloatStepMatchesMath compares NextUp, NextDown and Abs with
// math.Nextafter and math.Abs by exact bit pattern: signed zeros, infinities,
// NaN payloads, the range ends, every power of two with its neighbours, and
// random bit patterns.
func TestFloatStepMatchesMath(t *testing.T) {
	check := func(x float64) {
		up, wantUp := proof.NextUp(x), math.Nextafter(x, math.Inf(1))
		require.Equal(t, math.Float64bits(wantUp), math.Float64bits(up), "NextUp(%x)", math.Float64bits(x))
		down, wantDown := proof.NextDown(x), math.Nextafter(x, math.Inf(-1))
		require.Equal(t, math.Float64bits(wantDown), math.Float64bits(down), "NextDown(%x)", math.Float64bits(x))
		abs, wantAbs := proof.Abs(x), math.Abs(x)
		require.Equal(t, math.Float64bits(wantAbs), math.Float64bits(abs), "Abs(%x)", math.Float64bits(x))
	}

	specials := []float64{
		0, math.Copysign(0, -1), math.Inf(1), math.Inf(-1), math.NaN(), -math.NaN(),
		math.Float64frombits(0xfff0000000000001),
		math.MaxFloat64, -math.MaxFloat64,
		math.SmallestNonzeroFloat64, -math.SmallestNonzeroFloat64,
		0x1p-1022, -0x1p-1022, 0x1p-969, -0x1p-969, 0x1p1023, 1, -1,
	}
	for _, x := range specials {
		check(x)
		check(math.Nextafter(x, 0))
		check(math.Nextafter(x, math.Inf(1)))
		check(math.Nextafter(x, math.Inf(-1)))
	}
	for e := -1074; e <= 1023; e++ {
		p := math.Ldexp(1, e)
		for _, x := range []float64{p, -p, math.Nextafter(p, 0), math.Nextafter(p, math.Inf(1)), -math.Nextafter(p, 0)} {
			check(x)
		}
	}
	r := rand.New(rand.NewPCG(1, 2))
	for range 200_000 {
		check(math.Float64frombits(r.Uint64()))
	}
}
