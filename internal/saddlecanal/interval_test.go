package saddlecanal

import (
	"math"
	"testing"

	"github.com/stretchr/testify/require"
)

func TestIntervalMulMatchesFourProductTable(t *testing.T) {
	smallest := math.SmallestNonzeroFloat64
	tests := []struct {
		name string
		a, b iv
	}{
		{"positive-positive", iv{1, 3}, iv{2, 4}},
		{"negative-negative", iv{-3, -1}, iv{-4, -2}},
		{"positive-negative", iv{1, 3}, iv{-4, -2}},
		{"negative-positive", iv{-3, -1}, iv{2, 4}},
		{"cross-cross", iv{-3, 2}, iv{-4, 5}},
		{"cross-positive", iv{-3, 2}, iv{2, 4}},
		{"cross-negative", iv{-3, 2}, iv{-4, -2}},
		{"positive-cross", iv{1, 3}, iv{-4, 5}},
		{"negative-cross", iv{-3, -1}, iv{-4, 5}},
		{"signed-zero", iv{math.Copysign(0, -1), 0}, iv{math.Copysign(0, -1), 0}},
		{"subnormal", iv{smallest, 2 * smallest}, iv{-3, -1}},
		{"finite-overflow", iv{math.MaxFloat64 / 2, math.MaxFloat64}, iv{2, 3}},
		{"nan-fallback", iv{math.NaN(), 1}, iv{-2, 3}},
		{"infinity-fallback", iv{1, math.Inf(1)}, iv{-2, 3}},
		{"zero-times-infinity-fallback", iv{0, 1}, iv{0, math.Inf(1)}},
		{"inverted-fallback", iv{2, 1}, iv{-2, 3}},
	}
	for _, tc := range tests {
		t.Run(tc.name, func(t *testing.T) {
			got, want := tc.a.mul(tc.b), tc.a.mulFour(tc.b)
			require.Equal(t, math.Float64bits(want.lo), math.Float64bits(got.lo))
			require.Equal(t, math.Float64bits(want.hi), math.Float64bits(got.hi))
		})
	}
}
