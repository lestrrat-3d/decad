package loftmesh_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/stretchr/testify/require"
)

func TestCylinderChamferBound(t *testing.T) {
	t.Run("sums the larger reach with inset and clip", func(t *testing.T) {
		got, err := loftmesh.CylinderChamferBound(0.001, 0.02, 0.03, 0.01)
		require.NoError(t, err)
		require.InDelta(t, 0.06, got, 1e-12)
		require.GreaterOrEqual(t, got, 0.06)
	})
	t.Run("refuses a bound above 0.1 mm", func(t *testing.T) {
		_, err := loftmesh.CylinderChamferBound(0.05, 0.03, 0.03, 0)
		require.ErrorIs(t, err, decaderr.ErrUnsupported)
		require.ErrorContains(t, err, "exceeds 0.1 mm (joined 0.05, bridge 0, inset 0.03, clip 0.03)")
	})
	t.Run("refuses a non-finite term", func(t *testing.T) {
		_, err := loftmesh.CylinderChamferBound(0.001, math.NaN(), 0.03, 0.01)
		require.ErrorIs(t, err, decaderr.ErrUnsupported)
	})
}
