package decad

import (
	"math"
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// The combined transport must restate one triangle's face bound without
// changing the source map, even when the face points back to its handle.
func TestSpiralCombinedBoundsDetachSourceMap(t *testing.T) {
	face := &Face{}
	face.n.Owner = face
	original := &Mesh{
		n: topo.Mesh{
			Vertices:    []r3.Vec{{}, {X: 1}, {Y: 1}},
			Triangles:   [][3]int{{0, 1, 2}},
			VertexBound: []float64{0.2, 0.3, 0.4},
			Bound:       0.4,
			CoordBound:  0.2,
			VolSymDiff:  1,
			AreaSlack:   2,
		},
		source:    []*Face{face},
		faceBound: map[*Face]float64{face: 0.4},
	}
	placed := facetproof.PlacedMesh{
		Verts:       []r3.Vec{{}, {X: 2}, {Y: 2}},
		VertexBound: []float64{0.5, 0.7, 0.6},
		VolSymDiff:  3,
		AreaSlack:   4,
	}
	next, err := spiralCombinedProxyMesh(original, []facetproof.PlacedMesh{placed})
	require.NoError(t, err)
	require.True(t, next != original, "the proxy must have a separate mesh handle")
	require.Equal(t, 0.4, original.faceBound[face], "the source face bound stays unchanged")
	require.Equal(t, 0.4, original.n.Bound, "the source global bound stays unchanged")
	require.Equal(t, 1.0, original.n.VolSymDiff, "the source volume proof stays unchanged")
	require.Equal(t, 2.0, original.n.AreaSlack, "the source area proof stays unchanged")
	require.Equal(t, 0.7, next.faceBound[face], "the placed triangle sets its face bound")
	require.Equal(t, 0.7, next.n.Bound, "the global bound follows the face bound")
	require.Equal(t, 0.7, next.n.CoordBound, "the coordinate bound covers the new bound")
	require.Equal(t, math.Nextafter(3, math.Inf(1)), next.n.VolSymDiff,
		"the placed volume proof rounds upward")
	require.Equal(t, math.Nextafter(4, math.Inf(1)), next.n.AreaSlack,
		"the placed area proof rounds upward")
	require.True(t, slices.Equal(placed.VertexBound, next.n.VertexBound),
		"the placed vertex bounds are retained")
	require.True(t, next.source[0] == face, "the triangle keeps its source face")
	next.faceBound[face] = 9
	require.Equal(t, 0.4, original.faceBound[face], "mutating the proxy map must not change the source")
}
