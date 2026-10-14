package decad

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/stretchr/testify/require"
)

// TestDraftReceiverClasses pins SD23 (docs/draft-design.md Table SD): only a
// prism whose recorded section is the section it denotes is drafted, and a
// draft body is refused by name.
func TestDraftReceiverClasses(t *testing.T) {
	t.Parallel()
	pp, err := draftReceiver(bornBody(&Body{payload: prismPayload{}}))
	require.NoError(t, err)
	require.Equal(t, prismPayload{}, pp)

	_, err = draftReceiver(bornBody(&Body{payload: prismPayload{Payload: prismrecord.Payload{SectionDelta: 1e-6}}}))
	require.ErrorIs(t, err, ErrUnsupported)
	require.ErrorContains(t, err, "proven displacement")

	_, err = draftReceiver(bornBody(&Body{payload: draftPayload{}}))
	require.ErrorIs(t, err, ErrUnsupported)
	require.ErrorContains(t, err, "already a draft body")

	_, err = draftReceiver(bornBody(&Body{payload: revolvePayload{}}))
	require.ErrorIs(t, err, ErrUnsupported)
}

// TestDraftKeptWallsReadsRoles pins the selection's reading: a face counts as
// a wall by its own producer's side(i, j) role, never by another producer's;
// the complete wall set keeps nothing; a subset keeps every recorded segment
// of the walls it leaves out; a selected cap is SD22 and a selected face that
// is no wall is SD21.
func TestDraftKeptWallsReadsRoles(t *testing.T) {
	t.Parallel()
	face := func(origins ...provenance.Ref) *Face { return bornFace(&Face{n: topo.Face{Origins: origins}}) }
	own := face(provenance.Ref{Producer: 7, Role: "side(0,1)"})
	coalesced := face(provenance.Ref{Producer: 7, Role: "side(1,2)"}, provenance.Ref{Producer: 7, Role: "side(1,3)"})
	foreign := face(provenance.Ref{Producer: 8, Role: "side(0,1)"})
	capStart := face(provenance.Ref{Producer: 7, Role: provenance.RoleCapStart})
	require.True(t, own.isWallOf(7))
	require.False(t, foreign.isWallOf(7))
	require.False(t, capStart.isWallOf(7))

	b := bornBody(&Body{lumps: []*Lump{{shells: []*FaceShell{{faces: []*Face{own, coalesced, capStart}}}}}})
	kept, err := draftKeptWalls(b, 7, []*Face{own, coalesced})
	require.NoError(t, err)
	require.Empty(t, kept)

	kept, err = draftKeptWalls(b, 7, []*Face{own})
	require.NoError(t, err)
	require.Equal(t, map[offset2d.DraftWall]struct{}{{Loop: 1, Seg: 2}: {}, {Loop: 1, Seg: 3}: {}}, kept)

	_, err = draftKeptWalls(b, 7, []*Face{own, capStart})
	require.ErrorIs(t, err, ErrDegenerate)
	_, err = draftKeptWalls(b, 7, []*Face{foreign})
	require.ErrorIs(t, err, ErrUnsupported)
}
