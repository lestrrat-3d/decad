package tessellation_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/stretchr/testify/require"
)

func TestPatchEdgeSense(t *testing.T) {
	triangles := [][3]int{{0, 1, 2}, {2, 1, 3}, {4, 5, 6}}

	require.Equal(t, 1, tessellation.PatchEdgeSense(triangles, []int{0}, 0, 1))
	require.Equal(t, -1, tessellation.PatchEdgeSense(triangles, []int{0}, 1, 0))
	require.Equal(t, 1, tessellation.PatchEdgeSense(triangles, []int{0}, 2, 0), "wrap-around edge")
	require.Equal(t, -1, tessellation.PatchEdgeSense(triangles, []int{0, 1}, 2, 1))
	require.Equal(t, 0, tessellation.PatchEdgeSense(triangles, []int{0}, 0, 3))
	require.Equal(t, 0, tessellation.PatchEdgeSense(triangles, []int{2}, 0, 1), "only the named facets count")
	require.Equal(t, 0, tessellation.PatchEdgeSense(triangles, nil, 0, 1))
}
