package planar

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/stretchr/testify/require"
)

func crossingPrep(vertices [][3]float64, triangles [][3]int) *planarPrep {
	s := &PlanarSolid{Tris: triangles}
	for _, v := range vertices {
		s.Verts = append(s.Verts, proof.DyV3{
			proof.MustDyOf(v[0]), proof.MustDyOf(v[1]), proof.MustDyOf(v[2]),
		})
	}
	p := preparePlanar(s)
	p.floatBoxes()
	return p
}

func crossingEdgePrep(p, q [3]float64) (*planarPrep, [2]int) {
	edges := crossingPrep([][3]float64{p, q}, nil)
	edges.edges = [][2]int{{0, 1}}
	edges.edgeBox = []floatBox{edges.unionBox([]int{0, 1})}
	return edges, edges.edges[0]
}

func exactEdgeInFacetContact(edges *planarPrep, edge [2]int, tris *planarPrep) bool {
	k := planarKernel{a: edges, b: tris}
	k.edgeInFacet(edges, edge, tris, 0)
	return len(k.sites) != 0
}

func TestSidePlanePretestKeepsExactEdgeInFacetContacts(t *testing.T) {
	tris := crossingPrep([][3]float64{{0, 0, 0}, {2, 0, 0}, {0, 2, 0}}, [][3]int{{0, 1, 2}})
	for _, tc := range []struct {
		name string
		p, q [3]float64
	}{
		{name: "crosses the open interior", p: [3]float64{-1, 1, 0}, q: [3]float64{2, 1, 0}},
		{name: "overlaps inside the facet", p: [3]float64{0.25, 0.25, 0}, q: [3]float64{0.5, 0.5, 0}},
		{name: "starts at a boundary vertex", p: [3]float64{0, 0, 0}, q: [3]float64{0.5, 0.5, 0}},
		{name: "starts on a boundary edge", p: [3]float64{0.5, 0, 0}, q: [3]float64{0.5, 0.5, 0}},
	} {
		t.Run(tc.name, func(t *testing.T) {
			edges, edge := crossingEdgePrep(tc.p, tc.q)
			require.True(t, exactEdgeInFacetContact(edges, edge, tris), "exact edge-in-facet test must record a contact")
			require.False(t, sidePlanesRejectEdge(edges, 0, tris, 0), "pretest must keep exact contacts")
		})
	}
}

func TestSidePlanePretestRejectsOnlyEdgesOutsideFacet(t *testing.T) {
	tris := crossingPrep([][3]float64{{0, 0, 0}, {2, 0, 0}, {0, 2, 0}}, [][3]int{{0, 1, 2}})
	for _, tc := range []struct {
		name   string
		p, q   [3]float64
		reject bool
	}{
		{name: "outside one side", p: [3]float64{0, 3, 0}, q: [3]float64{1, 3, 0}, reject: true},
		{name: "touches a vertex from outside", p: [3]float64{-1, -1, 0}, q: [3]float64{0, 0, 0}},
		{name: "lies on the boundary", p: [3]float64{0, 0, 0}, q: [3]float64{2, 0, 0}},
	} {
		t.Run(tc.name, func(t *testing.T) {
			edges, edge := crossingEdgePrep(tc.p, tc.q)
			require.False(t, exactEdgeInFacetContact(edges, edge, tris), "exact method must not record an open interior chord")
			require.Equal(t, tc.reject, sidePlanesRejectEdge(edges, 0, tris, 0))
		})
	}
}

func TestSidePlanePretestDefersUncertainFloatEnclosures(t *testing.T) {
	tris := crossingPrep([][3]float64{{0, 0, 0}, {2, 0, 0}, {0, 2, 0}}, [][3]int{{0, 1, 2}})
	t.Run("wide edge box", func(t *testing.T) {
		edges, edge := crossingEdgePrep([3]float64{-1, 1, 0}, [3]float64{2, 1, 0})
		edges.edgeBox[0] = floatBox{lo: [3]float64{math.Inf(-1), math.Inf(-1), math.Inf(-1)},
			hi: [3]float64{math.Inf(1), math.Inf(1), math.Inf(1)}}
		require.True(t, exactEdgeInFacetContact(edges, edge, tris))
		require.False(t, sidePlanesRejectEdge(edges, 0, tris, 0))
	})
	t.Run("non-finite edge enclosure", func(t *testing.T) {
		edges, edge := crossingEdgePrep([3]float64{-1, 1, 0}, [3]float64{2, 1, 0})
		edges.edgeBox[0].lo[0] = math.NaN()
		require.True(t, exactEdgeInFacetContact(edges, edge, tris))
		require.False(t, sidePlanesRejectEdge(edges, 0, tris, 0))
	})
}
