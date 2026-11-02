package orthoshell_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/orthoshell"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestFacetedPartRequiresUnboundedMesh(t *testing.T) {
	t.Parallel()
	payload := facetedrecord.Payload{
		Verts: []r3.Vec{{}, {X: 1}, {Y: 1}},
		Tris:  [][3]int{{0, 1, 2}},
	}
	part, ok := orthoshell.FacetedPart(payload)
	require.True(t, ok)
	require.Equal(t, payload.Verts, part.Verts)
	require.Equal(t, payload.Tris, part.Tris)

	payload.VertexBound = []float64{0, 0, 1}
	_, ok = orthoshell.FacetedPart(payload)
	require.False(t, ok)

	payload.VertexBound = nil
	payload.Tris = nil
	_, ok = orthoshell.FacetedPart(payload)
	require.False(t, ok)
}
