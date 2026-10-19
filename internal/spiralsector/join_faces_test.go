package spiralsector_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/spiralsector"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/stretchr/testify/require"
)

func joinFacesBody(faces ...*topo.Face) *topo.Body {
	return &topo.Body{Lumps: []*topo.Lump{{Shells: []*topo.FaceShell{{Faces: faces}}}}}
}

func TestUniqueFaceWithOrigin(t *testing.T) {
	want := provenance.Ref{Producer: 3, Role: "side(1,0)"}
	other := provenance.Ref{Producer: 3, Role: "side(1,2)"}
	t.Run("one face carries the ref", func(t *testing.T) {
		face := &topo.Face{Origins: []provenance.Ref{other, want}}
		body := joinFacesBody(&topo.Face{Origins: []provenance.Ref{other}}, face)
		require.Same(t, face, spiralsector.UniqueFaceWithOrigin(body, want))
	})
	t.Run("a repeated ref on one face", func(t *testing.T) {
		face := &topo.Face{Origins: []provenance.Ref{want, want}}
		require.Same(t, face, spiralsector.UniqueFaceWithOrigin(joinFacesBody(face), want))
	})
	t.Run("two faces carry the ref", func(t *testing.T) {
		body := joinFacesBody(&topo.Face{Origins: []provenance.Ref{want}},
			&topo.Face{Origins: []provenance.Ref{want}})
		require.Nil(t, spiralsector.UniqueFaceWithOrigin(body, want))
	})
	t.Run("same role from another producer", func(t *testing.T) {
		body := joinFacesBody(&topo.Face{Origins: []provenance.Ref{{Producer: 4, Role: want.Role}}})
		require.Nil(t, spiralsector.UniqueFaceWithOrigin(body, want))
	})
	t.Run("nil body", func(t *testing.T) {
		require.Nil(t, spiralsector.UniqueFaceWithOrigin(nil, want))
	})
}

func TestOriginalSideRef(t *testing.T) {
	ref := provenance.Ref{Producer: 3, Role: "side(1,0)"}
	trim := func(origins ...[]provenance.Ref) *facetedrecord.LoftFacesTrimRecord {
		groups := make([]facetedrecord.Group, len(origins))
		for i, list := range origins {
			groups[i].Origins = list
		}
		return &facetedrecord.LoftFacesTrimRecord{Base: facetedrecord.Payload{Groups: groups}}
	}
	t.Run("groups agree on the producer", func(t *testing.T) {
		got, ok := spiralsector.OriginalSideRef(trim([]provenance.Ref{ref},
			[]provenance.Ref{{Producer: 3, Role: "side(1,2)"}, ref}), 1, 0)
		require.True(t, ok)
		require.Equal(t, ref, got)
	})
	t.Run("groups disagree on the producer", func(t *testing.T) {
		_, ok := spiralsector.OriginalSideRef(trim([]provenance.Ref{ref},
			[]provenance.Ref{{Producer: 4, Role: ref.Role}}), 1, 0)
		require.False(t, ok)
	})
	t.Run("no group carries the role", func(t *testing.T) {
		_, ok := spiralsector.OriginalSideRef(trim([]provenance.Ref{ref}), 2, 0)
		require.False(t, ok)
	})
	t.Run("band or segment out of range", func(t *testing.T) {
		record := trim([]provenance.Ref{{Producer: 3, Role: "side(8,0)"}, {Producer: 3, Role: "side(1,6)"}})
		_, ok := spiralsector.OriginalSideRef(record, 8, 0)
		require.False(t, ok)
		_, ok = spiralsector.OriginalSideRef(record, 1, 6)
		require.False(t, ok)
		_, ok = spiralsector.OriginalSideRef(nil, 1, 0)
		require.False(t, ok)
	})
}
