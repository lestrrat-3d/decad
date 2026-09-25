package decad

import (
	"math"
	"testing"
)

// BenchmarkBoundedRoundError measures ordinary finite Add and Mul operations,
// including their exact rounding-error calculation.
func BenchmarkBoundedRoundError(b *testing.B) {
	a := measuredScalar(1.23456789, 1e-12)
	c := measuredScalar(9.87654321, 2e-12)

	b.Run("Add", func(b *testing.B) {
		b.ReportAllocs()
		for b.Loop() {
			result := boundedAdd(a, c)
			if math.IsNaN(result.value) || math.IsInf(result.value, 0) ||
				math.IsNaN(result.bound) || math.IsInf(result.bound, 0) {
				b.Fatalf("expected finite result and bound, got %+v", result)
			}
		}
	})

	b.Run("Mul", func(b *testing.B) {
		b.ReportAllocs()
		for b.Loop() {
			result := boundedMul(a, c)
			if math.IsNaN(result.value) || math.IsInf(result.value, 0) ||
				math.IsNaN(result.bound) || math.IsInf(result.bound, 0) {
				b.Fatalf("expected finite result and bound, got %+v", result)
			}
		}
	})
}
