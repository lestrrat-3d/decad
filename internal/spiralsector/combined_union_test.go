package spiralsector_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/spiralsector"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/stretchr/testify/require"
)

// combinedOwnersFixture builds two one-facet tools, each with its own face,
// and a composite body whose two triangles land on the faces faceOf names.
func combinedOwnersFixture(compositeFaces int, faceOf []int) (
	*topo.Body, *facetedrecord.SpiralCombinedTools, [2]*topo.Face, []*topo.Face) {
	var live [2]*topo.Face
	tools := make([]*topo.Body, 2)
	for i := range tools {
		live[i] = &topo.Face{}
		tools[i] = joinFacesBody(live[i])
		tools[i].Payload = facetedrecord.Payload{Tris: [][3]int{{0, 1, 2}}, FaceOf: []int{0}}
	}
	composite := make([]*topo.Face, compositeFaces)
	for i := range composite {
		composite[i] = &topo.Face{}
	}
	body := joinFacesBody(composite...)
	body.Payload = facetedrecord.Payload{FaceOf: faceOf}
	meta := &facetedrecord.SpiralCombinedTools{
		Source: &facetedrecord.SpiralCombineSource{Tools: tools},
		Owner:  make(map[*topo.Face]*topo.Face),
	}
	return body, meta, live, composite
}

func TestBindCombinedOwners(t *testing.T) {
	t.Run("each composite face copies one live face", func(t *testing.T) {
		body, meta, live, composite := combinedOwnersFixture(2, []int{0, 1})
		require.NoError(t, spiralsector.BindCombinedOwners(body, meta))
		require.Len(t, meta.Owner, 2)
		require.Same(t, live[0], meta.Owner[composite[0]])
		require.Same(t, live[1], meta.Owner[composite[1]])
	})
	t.Run("a composite face merging two live faces refuses", func(t *testing.T) {
		body, meta, _, _ := combinedOwnersFixture(1, []int{0, 0})
		require.ErrorIs(t, spiralsector.BindCombinedOwners(body, meta), decaderr.ErrBooleanFailed)
	})
	t.Run("a composite face index out of range refuses", func(t *testing.T) {
		body, meta, _, _ := combinedOwnersFixture(1, []int{0, 1})
		require.ErrorIs(t, spiralsector.BindCombinedOwners(body, meta), decaderr.ErrBooleanFailed)
	})
}
