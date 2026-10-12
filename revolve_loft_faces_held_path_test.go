package decad

import (
	"context"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestRevolveLoftFacesHeldPathsRefusesBranchedSwitch(t *testing.T) {
	result := facetedPayload{
		verts: []r3.Vec{{X: 1, Y: 2}, {X: 2, Y: 2},
			{X: 3, Y: 2}, {X: 2, Y: 3}},
		vertexBound: []float64{0.01, 0.01, 0.01, 0.01},
	}
	pair := revolveLoftFacesHeldPair{edges: [][2]int{{0, 1}, {1, 2}}}
	paths, ok, err := revolveLoftFacesHeldPaths(context.Background(), result,
		[]revolveLoftFacesHeldPair{pair})
	require.NoError(t, err)
	require.True(t, ok)
	require.Len(t, paths, 1)
	require.Equal(t, []int{0, 1, 2}, paths[0].vertices)
	require.False(t, paths[0].closed)

	pair.edges = append(pair.edges, [2]int{1, 3})
	_, ok, err = revolveLoftFacesHeldPaths(context.Background(), result,
		[]revolveLoftFacesHeldPair{pair})
	require.NoError(t, err)
	require.False(t, ok)
}
