package facetproof_test

import (
	"math"
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// certSphere is a subdivided icosahedron of radius r about centre: 20·4^subdiv
// embedded facets.
func certSphere(subdiv int, r float64, centre r3.Vec) ([]r3.Vec, [][3]int) {
	phi := (1 + math.Sqrt(5)) / 2
	verts := []r3.Vec{{X: -1, Y: phi}, {X: 1, Y: phi}, {X: -1, Y: -phi}, {X: 1, Y: -phi},
		{Y: -1, Z: phi}, {Y: 1, Z: phi}, {Y: -1, Z: -phi}, {Y: 1, Z: -phi},
		{X: phi, Z: -1}, {X: phi, Z: 1}, {X: -phi, Z: -1}, {X: -phi, Z: 1}}
	tris := [][3]int{{0, 11, 5}, {0, 5, 1}, {0, 1, 7}, {0, 7, 10}, {0, 10, 11}, {1, 5, 9}, {5, 11, 4},
		{11, 10, 2}, {10, 7, 6}, {7, 1, 8}, {3, 9, 4}, {3, 4, 2}, {3, 2, 6}, {3, 6, 8}, {3, 8, 9},
		{4, 9, 5}, {2, 4, 11}, {6, 2, 10}, {8, 6, 7}, {9, 8, 1}}
	for range subdiv {
		mid := map[[2]int]int{}
		midpoint := func(a, b int) int {
			key := [2]int{min(a, b), max(a, b)}
			if m, ok := mid[key]; ok {
				return m
			}
			verts = append(verts, verts[a].Add(verts[b]).Scale(0.5))
			mid[key] = len(verts) - 1
			return len(verts) - 1
		}
		var next [][3]int
		for _, t := range tris {
			ab, bc, ca := midpoint(t[0], t[1]), midpoint(t[1], t[2]), midpoint(t[2], t[0])
			next = append(next, [3]int{t[0], ab, ca}, [3]int{t[1], bc, ab}, [3]int{t[2], ca, bc}, [3]int{ab, bc, ca})
		}
		tris = next
	}
	out := make([]r3.Vec, len(verts))
	for i, v := range verts {
		out[i] = v.Scale(r / v.Len()).Add(centre)
	}
	return out, tris
}

// requireSamePlacement compares two placements field by field with bounded
// failure output.
func requireSamePlacement(t *testing.T, want, got facetproof.PlacedMesh, label string) {
	t.Helper()
	require.True(t, slices.Equal(want.Verts, got.Verts), "%s: held vertices differ", label)
	require.True(t, slices.Equal(want.Tris, got.Tris), "%s: facets differ", label)
	require.True(t, slices.Equal(want.VertexBound, got.VertexBound), "%s: vertex bounds differ", label)
	require.Equal(t, math.Float64bits(want.MeshBound), math.Float64bits(got.MeshBound), "%s: mesh bound", label)
	require.Equal(t, math.Float64bits(want.VolSymDiff), math.Float64bits(got.VolSymDiff), "%s: volume bound", label)
	require.Equal(t, math.Float64bits(want.AreaSlack), math.Float64bits(got.AreaSlack), "%s: area slack", label)
	require.Equal(t, want.EmbeddingReach == nil, got.EmbeddingReach == nil, "%s: repair summary", label)
	if want.EmbeddingReach != nil {
		require.Equal(t, *want.EmbeddingReach, *got.EmbeddingReach, "%s: repair summary", label)
	}
}
