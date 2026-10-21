package spiralsector_test

import (
	"context"
	"math"
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/spiralsector"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// The combined transport must restate one triangle's face bound without
// changing the source map, even when the face points back to its handle.
func TestSpiralCombinedBoundsDetachSourceMap(t *testing.T) {
	face := new(topo.Face)
	original := &topo.Mesh{
		Vertices:    []r3.Vec{{}, {X: 1}, {Y: 1}},
		Triangles:   [][3]int{{0, 1, 2}},
		VertexBound: []float64{0.2, 0.3, 0.4},
		Bound:       0.4,
		CoordBound:  0.2,
		VolSymDiff:  1,
		AreaSlack:   2,
		Source:      []*topo.Face{face},
		FaceBound:   map[*topo.Face]float64{face: 0.4},
	}
	placed := facetproof.PlacedMesh{
		Verts:       []r3.Vec{{}, {X: 2}, {Y: 2}},
		VertexBound: []float64{0.5, 0.7, 0.6},
		VolSymDiff:  3,
		AreaSlack:   4,
	}
	next, err := spiralsector.CombinedProxyMesh(original, []facetproof.PlacedMesh{placed})
	require.NoError(t, err)
	require.True(t, next != original, "the proxy must have a separate mesh handle")
	require.Equal(t, 0.4, original.FaceBound[face], "the source face bound stays unchanged")
	require.Equal(t, 0.4, original.Bound, "the source global bound stays unchanged")
	require.Equal(t, 1.0, original.VolSymDiff, "the source volume proof stays unchanged")
	require.Equal(t, 2.0, original.AreaSlack, "the source area proof stays unchanged")
	require.Equal(t, 0.7, next.FaceBound[face], "the placed triangle sets its face bound")
	require.Equal(t, 0.7, next.Bound, "the global bound follows the face bound")
	require.Equal(t, 0.7, next.CoordBound, "the coordinate bound covers the new bound")
	require.Equal(t, math.Nextafter(3, math.Inf(1)), next.VolSymDiff,
		"the placed volume proof rounds upward")
	require.Equal(t, math.Nextafter(4, math.Inf(1)), next.AreaSlack,
		"the placed area proof rounds upward")
	require.True(t, slices.Equal(placed.VertexBound, next.VertexBound),
		"the placed vertex bounds are retained")
	require.True(t, next.Source[0] == face, "the triangle keeps its source face")
	next.FaceBound[face] = 9
	require.Equal(t, 0.4, original.FaceBound[face], "mutating the proxy map must not change the source")
}

type noMesher struct{ t *testing.T }

func (m noMesher) TessellateOperand(context.Context, *topo.Body, float64) (*topo.Mesh, bool, error) {
	m.t.Fatal("the gates must refuse before meshing")
	return nil, false, nil
}

func TestPrepareCombinedUnionGates(t *testing.T) {
	t.Parallel()

	original := &topo.Mesh{}
	pass := func(name string, op meshbool.OperationKind, payload topo.Payload) {
		t.Run(name, func(t *testing.T) {
			got, source, home, err := spiralsector.PrepareCombinedUnion(t.Context(), op, noMesher{t},
				&topo.Body{}, &topo.Body{Payload: payload}, original, 0.1)
			require.NoError(t, err)
			require.Same(t, original, got)
			require.Nil(t, source)
			require.Nil(t, home)
		})
	}
	pass("not faceted", meshbool.OpUnion, nil)
	pass("no spiral tools", meshbool.OpUnion, facetedrecord.Payload{})
	pass("not a union", meshbool.OpCut, facetedrecord.Payload{SpiralTools: &facetedrecord.SpiralCombinedTools{}})

	t.Run("incomplete source", func(t *testing.T) {
		t.Parallel()
		tool := &topo.Body{}
		move := r3.Identity()
		first := &facetedrecord.SpiralFirstJoinSource{Tooth: &facetedrecord.LoftFacesTrimRecord{}}
		tests := map[string]*facetedrecord.SpiralCombineSource{
			"no source":      nil,
			"one tool":       {First: first, Tools: []*topo.Body{tool}, Moves: []r3.Transform{move}},
			"moves mismatch": {First: first, Tools: []*topo.Body{tool, tool}, Moves: []r3.Transform{move}},
			"no first join":  {Tools: []*topo.Body{tool, tool}, Moves: []r3.Transform{move, move}},
			"no first tooth": {First: &facetedrecord.SpiralFirstJoinSource{}, Tools: []*topo.Body{tool, tool}, Moves: []r3.Transform{move, move}},
		}
		for name, source := range tests {
			_, _, _, err := spiralsector.PrepareCombinedUnion(t.Context(), meshbool.OpUnion, noMesher{t},
				&topo.Body{}, &topo.Body{Payload: facetedrecord.Payload{
					SpiralTools: &facetedrecord.SpiralCombinedTools{Source: source}}}, original, 0.1)
			require.ErrorIs(t, err, decaderr.ErrUnsupported, name)
			require.ErrorContains(t, err, "combined spiral source is incomplete", name)
		}
	})
}

func TestCombinedProxyMeshRefusals(t *testing.T) {
	t.Parallel()

	original := &topo.Mesh{Vertices: []r3.Vec{{}, {X: 1}}, VertexBound: []float64{0.1, 0.1}}
	whole := facetproof.PlacedMesh{Verts: []r3.Vec{{}, {X: 1}}, VertexBound: []float64{0.1, 0.1}}
	short := facetproof.PlacedMesh{Verts: []r3.Vec{{}}, VertexBound: []float64{0.1}}
	unbound := facetproof.PlacedMesh{Verts: []r3.Vec{{}, {X: 1}}, VertexBound: []float64{0.1}}
	tests := map[string]facetproof.PlacedMesh{"fewer vertices": short, "fewer bounds": unbound}
	nan := whole
	nan.VolSymDiff = math.NaN()
	tests["non-finite volume"] = nan
	nan = whole
	nan.AreaSlack = math.Inf(1)
	tests["non-finite area"] = nan
	for name, placed := range tests {
		_, err := spiralsector.CombinedProxyMesh(original, []facetproof.PlacedMesh{placed})
		require.ErrorIs(t, err, decaderr.ErrUnsupported, name)
		require.ErrorContains(t, err, "no complete bound", name)
	}
	_, err := spiralsector.CombinedProxyMesh(original, []facetproof.PlacedMesh{whole})
	require.NoError(t, err)
}

// The proxy's global bound is rebuilt from the placed vertex bounds alone: a
// larger bound on the source mesh does not carry over.
func TestCombinedProxyMeshDropsSourceBound(t *testing.T) {
	t.Parallel()

	face := new(topo.Face)
	original := &topo.Mesh{
		Vertices:    []r3.Vec{{}, {X: 1}, {Y: 1}},
		Triangles:   [][3]int{{0, 1, 2}},
		VertexBound: []float64{0.9, 0.9, 0.9},
		Bound:       0.9,
		Source:      []*topo.Face{face},
		FaceBound:   map[*topo.Face]float64{face: 0.9},
	}
	placed := facetproof.PlacedMesh{
		Verts:       []r3.Vec{{}, {X: 1}, {Y: 1}},
		VertexBound: []float64{0.1, 0.2, 0.3},
	}
	next, err := spiralsector.CombinedProxyMesh(original, []facetproof.PlacedMesh{placed})
	require.NoError(t, err)
	require.Equal(t, 0.3, next.Bound)
	require.Equal(t, 0.3, next.FaceBound[face])
}
