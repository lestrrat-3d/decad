package meshbool

import (
	"errors"
	"testing"

	"github.com/stretchr/testify/require"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/topo"
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
