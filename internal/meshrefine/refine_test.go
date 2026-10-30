package meshrefine_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/meshrefine"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestRefineSucceedsNearTriangleLimit(t *testing.T) {
	triangleCount := meshrefine.MaxTriangles/4 + 1
	triangles := make([][3]int, triangleCount)
	for i := range triangles {
		triangles[i] = [3]int{0, 1, 2}
	}
	vertices := []r3.Vec{{}, r3.NewVec(3, 0, 0), r3.NewVec(1.5, 0.1, 0)}

	got, err := meshrefine.Refine(t.Context(), vertices, triangles, 2)

	require.NoError(t, err)
	require.Equal(t, triangleCount*2, len(got.Triangles))
	require.Len(t, got.Vertices, 4)
	require.Equal(t, 1, got.Rounds)
	require.Equal(t, 1, got.Splits)
}
