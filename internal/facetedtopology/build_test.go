package facetedtopology_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/facetedtopology"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestBuildAdmitsOnlyConnectedClosedSingleFaceWithoutBoundaries(t *testing.T) {
	vertices := []r3.Vec{
		r3.NewVec(0, 0, 0), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1),
	}
	triangles := [][3]int{{0, 2, 1}, {0, 1, 3}, {0, 3, 2}, {1, 2, 3}}
	build := func(verts []r3.Vec, tris [][3]int, face []int) (facetedtopology.Result, error) {
		return facetedtopology.Build(t.Context(), verts, tris, face, map[int]bool{0: false},
			make([]float64, len(verts)))
	}
	result, err := build(vertices, triangles, []int{0, 0, 0, 0})
	require.NoError(t, err)
	require.Empty(t, result.Edges)
	require.Contains(t, result.FaceLoops, 0)
	require.Empty(t, result.FaceLoops[0])

	_, err = build(vertices, triangles[:3], []int{0, 0, 0})
	require.Error(t, err)
	other := []r3.Vec{
		r3.NewVec(3, 0, 0), r3.NewVec(4, 0, 0), r3.NewVec(3, 1, 0), r3.NewVec(3, 0, 1),
	}
	allVerts := append(append([]r3.Vec(nil), vertices...), other...)
	allTris := append(append([][3]int(nil), triangles...),
		[][3]int{{4, 6, 5}, {4, 5, 7}, {4, 7, 6}, {5, 6, 7}}...)
	_, err = build(allVerts, allTris, []int{0, 0, 0, 0, 0, 0, 0, 0})
	require.Error(t, err)
}
