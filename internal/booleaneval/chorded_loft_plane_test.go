package booleaneval_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/booleaneval"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/loftrecord"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestChordedLoftPlaneStatesTheFacetDeparture(t *testing.T) {
	t.Parallel()
	frame, err := r3.NewFrame(r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	retained := facetedrecord.Group{Surface: surfacegeom.Plane{Frame: frame}, Retained: true}
	wall := &topo.Face{Origins: []provenance.Ref{{Role: "side(0,3,1)"}}}
	capFace := &topo.Face{Origins: []provenance.Ref{{Role: provenance.RoleCapStart}}}
	loft := func(chorded bool, departure float64) *topo.Body {
		return &topo.Body{Payload: loftrecord.Payload{
			Chorded: chorded, Proof: loftmesh.MeshProof{FacetDeparture: departure},
		}}
	}

	t.Run("a chorded wall states the loft's departure", func(t *testing.T) {
		t.Parallel()
		g := booleaneval.ChordedLoftPlane(retained, loft(true, 0.0137), wall)
		require.True(t, g.Retained)
		require.True(t, g.HasAxialDelta)
		require.Equal(t, 0.0137, g.AxialDelta)
		require.Nil(t, g.CapRecord)
	})
	t.Run("a larger stated displacement stays", func(t *testing.T) {
		t.Parallel()
		g := retained
		g.AxialDelta, g.HasAxialDelta = 0.5, true
		g = booleaneval.ChordedLoftPlane(g, loft(true, 0.0137), wall)
		require.Equal(t, 0.5, g.AxialDelta)
	})
	t.Run("a cap, a loft with no chorded cell and another payload state nothing", func(t *testing.T) {
		t.Parallel()
		require.Equal(t, retained, booleaneval.ChordedLoftPlane(retained, loft(true, 0.0137), capFace))
		require.Equal(t, retained, booleaneval.ChordedLoftPlane(retained, loft(false, 0.0137), wall))
		require.Equal(t, retained, booleaneval.ChordedLoftPlane(retained,
			&topo.Body{Payload: prismrecord.Payload{}}, wall))
	})
	t.Run("a group with no tag is left alone", func(t *testing.T) {
		t.Parallel()
		bare := facetedrecord.Group{}
		require.Equal(t, bare, booleaneval.ChordedLoftPlane(bare, loft(true, 0.0137), wall))
	})
	t.Run("a departure no slab can charge drops the tag", func(t *testing.T) {
		t.Parallel()
		for _, departure := range []float64{math.NaN(), math.Inf(1), -1} {
			g := booleaneval.ChordedLoftPlane(retained, loft(true, departure), wall)
			require.Nil(t, g.Surface)
			require.False(t, g.Retained)
			require.False(t, g.HasAxialDelta)
		}
	})
}
