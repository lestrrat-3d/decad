package decad

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestJoinedMeshOrientationChecksClosedConnectedShell(t *testing.T) {
	verts := []r3.Vec{
		r3.NewVec(0, 0, 0), r3.NewVec(1, 0, 0),
		r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1),
	}
	tris := [][3]int{{0, 2, 1}, {0, 3, 1}, {1, 2, 3}, {2, 0, 3}}
	mesh := &joinedMesh{Verts: append([]r3.Vec(nil), verts...),
		Tris: append([][3]int(nil), tris...), Src: []int{0, 1, 2, 3}}
	require.NoError(t, orientJoinedMesh(mesh))
	require.Equal(t, 1, tessellation.OrientationSign(mesh.Verts, mesh.Tris, r3.Vec{}))
	require.Equal(t, verts, mesh.Verts)
	require.Equal(t, []int{0, 1, 2, 3}, mesh.Src)

	open := &joinedMesh{Verts: verts, Tris: tris[:3]}
	require.ErrorContains(t, orientJoinedMesh(open), "does not pair every mesh edge")

	second := []r3.Vec{
		r3.NewVec(3, 0, 0), r3.NewVec(4, 0, 0),
		r3.NewVec(3, 1, 0), r3.NewVec(3, 0, 1),
	}
	disconnected := &joinedMesh{Verts: append(append([]r3.Vec(nil), verts...), second...),
		Tris: append(append([][3]int(nil), tris...),
			[3]int{4, 6, 5}, [3]int{4, 5, 7}, [3]int{5, 6, 7}, [3]int{6, 4, 7})}
	require.ErrorContains(t, orientJoinedMesh(disconnected), "another component")

	collapsed := &joinedMesh{Verts: verts, Tris: [][3]int{{0, 0, 1}}}
	require.ErrorContains(t, orientJoinedMesh(collapsed), "collapsed triangle")
}
