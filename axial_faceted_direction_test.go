package decad

import (
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestAxialFacetedChainDirectionRejectsBentAndAmbiguousChains(t *testing.T) {
	verts := []r3.Vec{{}, {Z: 1}, {Z: 2}, {X: 0.001, Z: 2}, {Z: 0.5}}
	for _, chain := range [][]int{
		{0}, {0, 3}, {0, 1, 4}, {0, 1, 1}, {0, 5},
	} {
		_, proven := axialFacetedChainDirection(verts, chain)
		require.False(t, proven, "chain %v", chain)
	}
	direction, proven := axialFacetedChainDirection(verts, []int{0, 1, 2})
	require.True(t, proven)
	require.Equal(t, r3.Vec{Z: 1}, direction)
}
