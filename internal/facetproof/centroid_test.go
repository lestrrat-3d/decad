package facetproof_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/stretchr/testify/require"
)

func TestFacetedCentroidAllowance(t *testing.T) {
	t.Parallel()
	t.Run("no volume floor answers the diameter", func(t *testing.T) {
		t.Parallel()
		for _, floor := range []float64{0, -1} {
			require.Equal(t, 0.25, facetproof.FacetedCentroidAllowance(1e-3, 0.25, floor), "volumeFloor %v", floor)
		}
	})
	t.Run("an underflowing product stays positive", func(t *testing.T) {
		t.Parallel()
		// 1e-200 * 1e-200 flushes to zero in float64. ProductUpper publishes
		// the smallest subnormal instead, and DivUpper's division by 1 rounds
		// that up one more subnormal step.
		got := facetproof.FacetedCentroidAllowance(1e-200, 1e-200, 1)
		require.Equal(t, math.Float64frombits(2), got)
		require.Positive(t, got)
	})
	t.Run("the diameter caps the allowance", func(t *testing.T) {
		t.Parallel()
		require.Equal(t, 0.25, facetproof.FacetedCentroidAllowance(1, 0.25, 1e-3))
	})
}
