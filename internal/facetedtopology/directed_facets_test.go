package facetedtopology

import (
	"testing"

	"github.com/stretchr/testify/require"
)

// The full int range needs direct coverage without allocating a mesh with
// billions of vertices. The topology builder's real-mesh tests cover its use.
func TestDirectedFacetsPreservesDirectionAndFullIndexRange(t *testing.T) {
	t.Parallel()
	maxVertices := int(^uint(0) >> 1)
	counts := []int{1 << 16, maxVertices}
	boundary := uint64(1) << 32
	if uint64(maxVertices) > boundary {
		counts = append(counts, int(boundary), int(boundary+1))
	}
	for _, vertices := range counts {
		owners := newDirectedFacets(vertices, 0)
		pairs := [][2]int{
			{0, 1}, {1, 0}, {0, vertices - 1}, {vertices - 1, 0},
			{1, vertices - 1}, {vertices - 1, 1},
		}
		for i, pair := range pairs {
			owners.set(pair[0], pair[1], i)
		}
		require.Equal(t, len(pairs), owners.len())
		for i, pair := range pairs {
			facet, ok := owners.get(pair[0], pair[1])
			require.True(t, ok, "vertex count %d pair %d", vertices, i)
			require.Equal(t, i, facet, "vertex count %d pair %d", vertices, i)
		}
		_, ok := owners.get(2, 2)
		require.False(t, ok)
		owners.set(0, 1, 99)
		require.Equal(t, len(pairs), owners.len())
		facet, ok := owners.get(0, 1)
		require.True(t, ok)
		require.Equal(t, 99, facet)
		facet, ok = owners.get(1, 0)
		require.True(t, ok)
		require.Equal(t, 1, facet)
	}
}
