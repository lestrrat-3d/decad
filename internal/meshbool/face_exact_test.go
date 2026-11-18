package meshbool

import (
	"errors"
	"testing"

	"github.com/stretchr/testify/require"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

// TestSetExactFacets pins §3.1's admission reading: a facet is exact only when
// its own bound and its face's gate reading are both zero.
//
// Leg shown to fail: reading FacetBound alone marks the loose face's facet
// exact.
func TestSetExactFacets(t *testing.T) {
	t.Parallel()
	exactFace, looseFace, narrowed := &topo.Face{}, &topo.Face{}, &topo.Face{}
	m := &topo.Mesh{
		Source:    []*topo.Face{exactFace, exactFace, looseFace, narrowed},
		FaceBound: map[*topo.Face]float64{exactFace: 0, looseFace: 0.5, narrowed: 0.5},
	}
	bm := &BoolMesh{
		Tris:        make([][3]int, 4),
		FacetBound:  []float64{0, 1e-15, 0, 0},
		FaceSurface: map[*topo.Face]float64{narrowed: 0},
	}
	require.NoError(t, bm.SetExactFacets(m))
	require.Equal(t, []bool{true, false, false, true}, bm.Exact,
		"a facet bound, a face bound and a narrowing surface bound each decide one facet")
}

func TestSetExactFacetsUnstatedFace(t *testing.T) {
	t.Parallel()
	f := &topo.Face{}
	m := &topo.Mesh{Source: []*topo.Face{f}, FaceBound: map[*topo.Face]float64{}}
	bm := &BoolMesh{Tris: make([][3]int, 1), FacetBound: []float64{0}}
	err := bm.SetExactFacets(m)
	require.Error(t, err)
	require.True(t, errors.Is(err, decaderr.ErrBooleanFailed))
}

// TestSetExactFacetsBox pins a prepared box whose faces all read zero: every
// facet is exact.
func TestSetExactFacetsBox(t *testing.T) {
	t.Parallel()
	bm := boxMesh(t, r3.Vec{X: 1, Y: 1, Z: 1}, func(v r3.Vec) r3.Vec { return v }, r3.Vec{})
	bm.SetVertexBounds(make([]float64, len(bm.Verts)))
	m := &topo.Mesh{FaceBound: map[*topo.Face]float64{}}
	faces := make([]*topo.Face, 6)
	for i := range faces {
		faces[i] = &topo.Face{}
		m.FaceBound[faces[i]] = 0
	}
	for f := range bm.Tris {
		m.Source = append(m.Source, faces[f/2])
	}
	require.NoError(t, bm.SetExactFacets(m))
	require.Len(t, bm.Exact, 12)
	for f, ok := range bm.Exact {
		require.True(t, ok, "facet %d", f)
	}
}
