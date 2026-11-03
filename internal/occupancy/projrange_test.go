package occupancy

import (
	"fmt"
	"math"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// mathProjRange is the reference fold projRange must match bit for bit:
// math.Min and math.Max from +Inf and -Inf.
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
	var samples []string
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
			if len(samples) < 3 {
				projections := make([]uint64, len(vs))
				for i, v := range vs {
					projections[i] = math.Float64bits(a.Dot(v))
				}
				samples = append(samples, fmt.Sprintf(
					"projection bits %016x; want (%016x,%016x); got (%016x,%016x)",
					projections, math.Float64bits(olo), math.Float64bits(ohi),
					math.Float64bits(nlo), math.Float64bits(nhi),
				))
			}
		}
	}
	require.Zero(t, mismatches, "mismatch samples: %v", samples)
}

// TestProjRangeSpecialValuesExhaustive checks projRange against the math.Min
// and math.Max fold bit for bit on every sequence of up to five projections
// drawn from NaN, both infinities, both zeros, both units, both smallest
// subnormals and both largest finite values. The axis (1, 0, 0) and vertices
// (d, -0, -0) make each projection exactly d, signed zeros included.
func TestProjRangeSpecialValuesExhaustive(t *testing.T) {
	values := []float64{
		math.NaN(), math.Inf(1), math.Inf(-1), 0, math.Copysign(0, -1), 1, -1,
		5e-324, -5e-324, math.MaxFloat64, -math.MaxFloat64,
	}
	negZero := math.Copysign(0, -1)
	a := r3.Vec{X: 1}
	mismatches, unexact, cases := 0, 0, 0
	idx := make([]int, 0, 5)
	vs := make([]r3.Vec, 0, 5)
	var walk func()
	walk = func() {
		cases++
		olo, ohi := mathProjRange(a, vs)
		nlo, nhi := projRange(a, vs)
		if math.Float64bits(olo) != math.Float64bits(nlo) || math.Float64bits(ohi) != math.Float64bits(nhi) {
			mismatches++
		}
		if len(idx) == cap(idx) {
			return
		}
		for i, d := range values {
			v := r3.Vec{X: d, Y: negZero, Z: negZero}
			if math.Float64bits(a.Dot(v)) != math.Float64bits(d) {
				unexact++
			}
			idx, vs = append(idx, i), append(vs, v)
			walk()
			idx, vs = idx[:len(idx)-1], vs[:len(vs)-1]
		}
	}
	walk()
	require.Zero(t, unexact, "a projection differs from its value")
	require.Zero(t, mismatches)
	require.Equal(t, 177156, cases, "1 + 11 + ... + 11^5 sequences")
}
