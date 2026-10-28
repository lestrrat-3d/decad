package retainedblend

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/stretchr/testify/require"
)

func TestChainSide(t *testing.T) {
	t.Parallel()

	t.Run("a chain of one side blends on that side", func(t *testing.T) {
		t.Parallel()
		for _, s := range []float64{-1, 1} {
			side, err := chainSide([]chainEntry{{side: s}, {side: s}, {side: s}})
			require.NoError(t, err)
			require.Equal(t, s, side)
		}
	})

	t.Run("a chain that mixes convex and concave edges is refused", func(t *testing.T) {
		t.Parallel()
		_, err := chainSide([]chainEntry{{side: -1}, {side: 1}})
		require.ErrorIs(t, err, decaderr.ErrUnsupported)
		require.ErrorContains(t, err, "a tangent chain mixes convex and concave edges")
	})
}
