package revolveboretess

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/stretchr/testify/require"
)

func TestCapRimFollowsSharedTriangleEdges(t *testing.T) {
	face := &topo.Face{}
	mesh := &topo.Mesh{Triangles: [][3]int{{0, 1, 2}, {0, 2, 3}}, Source: []*topo.Face{face, face}}
	rim, err := capRim(mesh, face)
	require.NoError(t, err)
	require.Equal(t, []int{0, 1, 2, 3}, rim)
	mesh.Triangles = append(mesh.Triangles, [3]int{4, 5, 6})
	mesh.Source = append(mesh.Source, face)
	_, err = capRim(mesh, face)
	require.Error(t, err)
}
