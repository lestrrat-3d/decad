package meshbool

import (
	"errors"
	"testing"

	"github.com/stretchr/testify/require"

	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

// boxTessellation is the tessellation exactBoxMesh's facets came from: one
// source face per box side (two facets each, in boxMesh's order) with the given
// face bounds.
func boxTessellation(bound [6]float64) *topo.Mesh {
	m := &topo.Mesh{FaceBound: map[*topo.Face]float64{}}
	faces := make([]*topo.Face, 6)
	for f := range faces {
		faces[f] = &topo.Face{}
		m.FaceBound[faces[f]] = bound[f]
	}
	for f := range 12 {
		m.Source = append(m.Source, faces[f/2])
	}
	return m
}

// TestCoplanarGateInexactFace pins §3.1's admission test: a facet with a zero
// bound on a face with a positive reading is NOT exact, so the coplanar pair
// is refused with TouchCoplanar and the gap retry stays reachable.
//
// Leg shown to fail: admitting on FacetBound alone classifies the pair.
func TestCoplanarGateInexactFace(t *testing.T) {
	a := exactBoxMesh(t, r3.Vec{}, r3.Vec{X: 10, Y: 10, Z: 10})
	b := exactBoxMesh(t, r3.Vec{X: 0.5, Z: 10}, r3.Vec{X: 10.5, Y: 10, Z: 20})
	// boxMesh's quad 4 is the box's z-minimum side: B's contact face.
	require.NoError(t, a.SetExactFacets(boxTessellation([6]float64{})))
	require.NoError(t, b.SetExactFacets(boxTessellation([6]float64{4: 1e-3})))
	for _, f := range b.FacetBound {
		require.Zero(t, f, "every facet bound is zero")
	}
	_, _, err := MeshBooleanStats(t.Context(), OpUnion, a, b, NewContactMemo(a, b))
	touch, ok := errors.AsType[*TouchContactError](err)
	require.True(t, ok, "%v", err)
	require.Equal(t, touchCoplanar, touch.What)

	// The same pair with an exact face reading is classified.
	require.NoError(t, b.SetExactFacets(boxTessellation([6]float64{})))
	_, _, err = MeshBooleanStats(t.Context(), OpUnion, a, b, NewContactMemo(a, b))
	require.NoError(t, err)
}
