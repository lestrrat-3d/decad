package meshbool

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// TestFacesOfMeshReadsSurfaceBound holds the proximity gate's face slack to
// the smaller of the mesh's face bound and a refined surface bound: a smaller
// surface bound lowers it, a larger one leaves it, and a face without one keeps
// its mesh bound.
//
// Leg shown to fail: ignoring the surface map leaves the first face at 0.3.
func TestFacesOfMeshReadsSurfaceBound(t *testing.T) {
	t.Parallel()
	refined, loose, plain := &topo.Face{}, &topo.Face{}, &topo.Face{}
	m := &topo.Mesh{
		Source:    []*topo.Face{refined, loose, plain},
		FaceBound: map[*topo.Face]float64{refined: 0.3, loose: 0.01, plain: 0.2},
	}
	boxes := make([][2]r3.Vec, len(m.Source))
	surface := map[*topo.Face]float64{refined: 0.04, loose: 0.5}
	faces, err := facesOfMesh(proofbound.NewWorkBudget(t.Context()), m, boxes, surface)
	require.NoError(t, err)
	require.Len(t, faces, 3)
	got := map[*topo.Face]float64{}
	for _, f := range faces {
		got[f.source] = f.delta
	}
	require.InDelta(t, 0.04, got[refined], 0)
	require.InDelta(t, 0.01, got[loose], 0)
	require.InDelta(t, 0.2, got[plain], 0)
}

// TestFacesOfMeshGroupsFacetsInFirstAppearanceOrder pins the grouping: facets
// are collected per source face, groups follow first appearance, and each
// group's box is the union of its facets' boxes.
//
// Leg shown to fail: mapping every facet to a fresh group breaks the counts.
func TestFacesOfMeshGroupsFacetsInFirstAppearanceOrder(t *testing.T) {
	t.Parallel()
	a, b, c := &topo.Face{}, &topo.Face{}, &topo.Face{}
	m := &topo.Mesh{
		Source:    []*topo.Face{b, a, b, c, a, b},
		FaceBound: map[*topo.Face]float64{a: 0.1, b: 0.2, c: 0.3, {}: 0.4},
	}
	boxes := make([][2]r3.Vec, len(m.Source))
	for i := range boxes {
		x := float64(i)
		boxes[i] = [2]r3.Vec{{X: x, Y: -x, Z: 0}, {X: x + 1, Y: 1, Z: 2 * x}}
	}
	faces, err := facesOfMesh(proofbound.NewWorkBudget(t.Context()), m, boxes, nil)
	require.NoError(t, err)
	require.Len(t, faces, 3)
	wantSrc := []*topo.Face{b, a, c}
	wantFacets := [][]int{{0, 2, 5}, {1, 4}, {3}}
	wantDelta := []float64{0.2, 0.1, 0.3}
	wantBox := [][2]r3.Vec{
		{{X: 0, Y: -5, Z: 0}, {X: 6, Y: 1, Z: 10}},
		{{X: 1, Y: -4, Z: 0}, {X: 5, Y: 1, Z: 8}},
		{{X: 3, Y: -3, Z: 0}, {X: 4, Y: 1, Z: 6}},
	}
	for i, f := range faces {
		require.Same(t, wantSrc[i], f.source)
		require.Equal(t, wantFacets[i], f.facets)
		require.InDelta(t, wantDelta[i], f.delta, 0)
		require.Equal(t, wantBox[i], f.box)
	}
}
