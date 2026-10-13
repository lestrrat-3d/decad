package proofbound_test

import (
	"math"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func finiteTestValues() []float64 {
	vals := []float64{
		0, math.Copysign(0, -1), math.Inf(1), math.Inf(-1), math.NaN(), -math.NaN(),
		math.Float64frombits(0x7ff0000000000001), math.Float64frombits(0xfff8000000000123),
		math.MaxFloat64, -math.MaxFloat64,
		math.Nextafter(math.MaxFloat64, 0), -math.Nextafter(math.MaxFloat64, 0),
		math.SmallestNonzeroFloat64, -math.SmallestNonzeroFloat64, 1, -1,
	}
	r := rand.New(rand.NewPCG(5, 9))
	for range 100_000 {
		vals = append(vals, math.Float64frombits(r.Uint64()))
	}
	return vals
}

func TestIsNonFiniteAndFiniteVecMatchMath(t *testing.T) {
	t.Parallel()
	vals := finiteTestValues()
	for _, x := range vals {
		require.Equal(t, math.IsNaN(x) || math.IsInf(x, 0), proofbound.IsNonFinite(x), "%x", math.Float64bits(x))
	}
	for i, x := range vals {
		y, z := vals[(i*7+1)%len(vals)], vals[(i*13+2)%len(vals)]
		want := !math.IsNaN(x) && !math.IsInf(x, 0) && !math.IsNaN(y) && !math.IsInf(y, 0) &&
			!math.IsNaN(z) && !math.IsInf(z, 0)
		require.Equal(t, want, proofbound.FiniteVec(r3.Vec{X: x, Y: y, Z: z}))
		if i < 64 {
			// The nonfinite component in each slot.
			require.Equal(t, !proofbound.IsNonFinite(x), proofbound.FiniteVec(r3.Vec{X: 1, Y: x, Z: 2}))
			require.Equal(t, !proofbound.IsNonFinite(x), proofbound.FiniteVec(r3.Vec{X: x, Y: 1, Z: 2}))
			require.Equal(t, !proofbound.IsNonFinite(x), proofbound.FiniteVec(r3.Vec{X: 1, Y: 2, Z: x}))
		}
	}
}
