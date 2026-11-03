package occupancy

import (
	"math"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// mathProjRange is the fold projRange replaced: math.Min and math.Max from
// +Inf and -Inf.
func mathProjRange(a r3.Vec, vs []r3.Vec) (float64, float64) {
	lo, hi := math.Inf(1), math.Inf(-1)
	for _, v := range vs {
		d := a.Dot(v)
		lo, hi = math.Min(lo, d), math.Max(hi, d)
	}
	return lo, hi
}

// TestProjRangeMatchesMathFold checks projRange against the math.Min and
// math.Max fold bit for bit, over finite, signed-zero, subnormal, huge,
// infinite and NaN inputs, including empty and single-element slices.
func TestProjRangeMatchesMathFold(t *testing.T) {
	rng := rand.New(rand.NewPCG(6, 6))
	special := []float64{
		0, math.Copysign(0, -1), math.Inf(1), math.Inf(-1), math.NaN(),
		math.MaxFloat64, -math.MaxFloat64, 1e308, -1e308, 5e-324, -5e-324,
	}
	coord := func() float64 {
		switch rng.IntN(6) {
		case 0:
			return special[rng.IntN(len(special))]
		case 1:
			return float64(rng.IntN(5) - 2)
		default:
			return (rng.Float64() - 0.5) * math.Pow(10, float64(rng.IntN(8)-3))
		}
	}
	vec := func() r3.Vec { return r3.Vec{X: coord(), Y: coord(), Z: coord()} }
	mismatches := 0
	for range 200_000 {
		a := vec()
		vs := make([]r3.Vec, rng.IntN(10))
		for i := range vs {
			vs[i] = vec()
		}
		olo, ohi := mathProjRange(a, vs)
		nlo, nhi := projRange(a, vs)
		if math.Float64bits(olo) != math.Float64bits(nlo) || math.Float64bits(ohi) != math.Float64bits(nhi) {
			mismatches++
		}
	}
	require.Zero(t, mismatches)
}
