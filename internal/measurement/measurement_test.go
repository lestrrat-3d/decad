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
