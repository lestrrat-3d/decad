package proofbound_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestAxisDot(t *testing.T) {
	dot := proofbound.AxisDot(r3.Vec{X: 1, Y: 2, Z: 3}, r3.Vec{X: 4, Y: -5, Z: 0.5})
	got, exact := dot.Float64()
	require.True(t, exact)
	require.Equal(t, -4.5, got)
}

func TestCenterAxisInterval(t *testing.T) {
	center := r3.Vec{X: 1, Y: 2, Z: 3}
	axis := r3.Vec{X: 0, Y: 0, Z: 2}

	t.Run("encloses the projection widened by the bound", func(t *testing.T) {
		lo, hi, ok := proofbound.CenterAxisInterval(center, 0.5, axis, 2)
		require.True(t, ok)
		require.LessOrEqual(t, lo, 5.0)
		require.GreaterOrEqual(t, hi, 7.0)
		require.InDelta(t, 5.0, lo, 1e-12)
		require.InDelta(t, 7.0, hi, 1e-12)
	})
	t.Run("zero bound is a tight pair around the projection", func(t *testing.T) {
		lo, hi, ok := proofbound.CenterAxisInterval(center, 0, axis, 2)
		require.True(t, ok)
		require.Less(t, lo, 6.0)
		require.Greater(t, hi, 6.0)
		require.InDelta(t, 6.0, lo, 1e-12)
	})
	t.Run("refuses a negative or non-finite bound", func(t *testing.T) {
		for _, bound := range []float64{-1, math.NaN(), math.Inf(1)} {
			_, _, ok := proofbound.CenterAxisInterval(center, bound, axis, 2)
			require.False(t, ok)
		}
	})
	t.Run("refuses a non-finite center", func(t *testing.T) {
		_, _, ok := proofbound.CenterAxisInterval(r3.Vec{X: math.NaN()}, 0, axis, 2)
		require.False(t, ok)
	})
	t.Run("refuses an overflowing interval", func(t *testing.T) {
		_, _, ok := proofbound.CenterAxisInterval(center, math.MaxFloat64, axis, 2)
		require.False(t, ok)
	})
}
