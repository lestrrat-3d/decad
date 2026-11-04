package planar

import (
	"math"
	"testing"

	"github.com/stretchr/testify/require"
)

func minmaxOperands() []float64 {
	negZero := math.Copysign(0, -1)
	return []float64{
		math.NaN(), math.Inf(1), math.Inf(-1), 0, negZero, 1, -1,
		math.SmallestNonzeroFloat64, -math.SmallestNonzeroFloat64,
		math.MaxFloat64, -math.MaxFloat64,
	}
}

// TestMinMaxFloatMatchesMath holds minFloat and maxFloat to math.Min and
// math.Max on every pair of NaN, infinity, signed zero and finite operands,
// compared by bit pattern so a zero's sign counts. The builtin min and max
// differ from math.Min(-Inf, NaN) and math.Max(+Inf, NaN), so the helpers
// must not be the builtins alone.
func TestMinMaxFloatMatchesMath(t *testing.T) {
	t.Parallel()
	ops := minmaxOperands()
	builtinDiffers := 0
	for _, a := range ops {
		for _, b := range ops {
			require.Equal(t, math.Float64bits(math.Min(a, b)), math.Float64bits(minFloat(a, b)), "min(%v, %v)", a, b)
			require.Equal(t, math.Float64bits(math.Max(a, b)), math.Float64bits(maxFloat(a, b)), "max(%v, %v)", a, b)
			if math.Float64bits(math.Min(a, b)) != math.Float64bits(min(a, b)) ||
				math.Float64bits(math.Max(a, b)) != math.Float64bits(max(a, b)) {
				builtinDiffers++
			}
		}
	}
	require.Positive(t, builtinDiffers, "the builtins must differ from math on an infinity against NaN")
}

func BenchmarkMulEnclosure(b *testing.B) {
	var lo, hi float64
	for i := range b.N {
		x := float64(i&1023) + 0.5
		lo, hi = mulEnclosure(-x, x+1, -2.5, 3.25)
	}
	_, _ = lo, hi
}
