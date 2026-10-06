package proofbound

import (
	"math"
	"testing"
)

// BenchmarkBoundedRoundError measures ordinary finite Add and Mul operations,
// including their exact rounding-error calculation.
func BenchmarkBoundedRoundError(b *testing.B) {
	a := MeasuredScalar(1.23456789, 1e-12)
	c := MeasuredScalar(9.87654321, 2e-12)

	b.Run("Add", func(b *testing.B) {
		b.ReportAllocs()
		for b.Loop() {
			result := BoundedAdd(a, c)
			if math.IsNaN(result.Value) || math.IsInf(result.Value, 0) ||
				math.IsNaN(result.Bound) || math.IsInf(result.Bound, 0) {
				b.Fatalf("expected finite result and bound, got %+v", result)
			}
		}
	})

	b.Run("Mul", func(b *testing.B) {
		b.ReportAllocs()
		for b.Loop() {
			result := BoundedMul(a, c)
			if math.IsNaN(result.Value) || math.IsInf(result.Value, 0) ||
				math.IsNaN(result.Bound) || math.IsInf(result.Bound, 0) {
				b.Fatalf("expected finite result and bound, got %+v", result)
			}
		}
	})
}
