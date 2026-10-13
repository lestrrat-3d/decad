package freeform_test

import (
	"math"
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
