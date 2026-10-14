package shellseparation

import (
	"math"
	"math/rand/v2"
	"testing"

	"github.com/stretchr/testify/require"
)

// TestFilterStepsMatchNextafter holds fDown and fUp to the math.Nextafter form
// they replaced, by exact bit pattern, over signed zeros, infinities, NaN, the
// range ends, subnormals, binade boundaries and random bit patterns.
func TestFilterStepsMatchNextafter(t *testing.T) {
	check := func(x float64) {
		require.Equal(t, math.Float64bits(math.Nextafter(x, math.Inf(-1))), math.Float64bits(fDown(x)),
			"down(%x)", math.Float64bits(x))
		require.Equal(t, math.Float64bits(math.Nextafter(x, math.Inf(1))), math.Float64bits(fUp(x)),
			"up(%x)", math.Float64bits(x))
	}
	for _, x := range []float64{
		0, math.Copysign(0, -1), math.Inf(1), math.Inf(-1), math.NaN(), math.MaxFloat64, -math.MaxFloat64,
		math.SmallestNonzeroFloat64, -math.SmallestNonzeroFloat64, 1, -1,
	} {
		check(x)
	}
	for e := -1074; e <= 1023; e++ {
		p := math.Ldexp(1, e)
		for _, x := range []float64{p, -p, math.Nextafter(p, 0), -math.Nextafter(p, 0),
			math.Nextafter(p, math.Inf(1)), -math.Nextafter(p, math.Inf(1))} {
			check(x)
		}
	}
	r := rand.New(rand.NewPCG(7, 8))
	for range 100_000 {
		check(math.Float64frombits(r.Uint64()))
		check(r.NormFloat64() * math.Ldexp(1, r.IntN(60)-30))
	}
}
