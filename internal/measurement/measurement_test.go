package measurement_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/stretchr/testify/require"
)

// TestExactnessOf pins the rule root exactnessOf applied before it moved: a
// value is Exact only when its proven bound is exactly zero, so a negative
// zero is Exact and a NaN, an infinity or any positive bound is Approximate.
func TestExactnessOf(t *testing.T) {
	t.Parallel()

	tests := []struct {
		name  string
		bound float64
		want  measurement.Exactness
	}{
		{"Zero", 0, measurement.Exact},
		{"NegativeZero", math.Copysign(0, -1), measurement.Exact},
		{"Tiny", 1e-300, measurement.Approximate},
		{"PositiveInf", math.Inf(1), measurement.Approximate},
		{"NaN", math.NaN(), measurement.Approximate},
	}
	for _, tc := range tests {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			require.Equal(t, tc.want, measurement.ExactnessOf(tc.bound))
		})
	}
}

// TestFiniteValues pins the finiteness gate: every value must be neither NaN
// nor an infinity, and no values at all is finite.
func TestFiniteValues(t *testing.T) {
	t.Parallel()

	tests := []struct {
		name   string
		values []float64
		want   bool
	}{
		{"None", nil, true},
		{"Finite", []float64{0, -1.5, math.MaxFloat64, math.SmallestNonzeroFloat64}, true},
		{"NaNFirst", []float64{math.NaN(), 1}, false},
		{"NaNLast", []float64{1, 2, math.NaN()}, false},
		{"PositiveInfMiddle", []float64{1, math.Inf(1), 2}, false},
		{"NegativeInf", []float64{math.Inf(-1)}, false},
	}
	for _, tc := range tests {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			require.Equal(t, tc.want, measurement.FiniteValues(tc.values...))
		})
	}
}
