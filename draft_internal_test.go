package decad

import (
	"testing"

	"github.com/stretchr/testify/require"
)

// TestDraftReceiverClasses pins SD23 (docs/draft-design.md Table SD): only a
// prism whose recorded section is the section it denotes is drafted, and a
// draft body is refused by name.
func TestDraftReceiverClasses(t *testing.T) {
	t.Parallel()
	pp, err := draftReceiver(&Body{payload: prismPayload{}})
	require.NoError(t, err)
	require.Equal(t, prismPayload{}, pp)

	_, err = draftReceiver(&Body{payload: prismPayload{sectionDelta: 1e-6}})
	require.ErrorIs(t, err, ErrUnsupported)
	require.ErrorContains(t, err, "proven displacement")

	_, err = draftReceiver(&Body{payload: draftPayload{}})
	require.ErrorIs(t, err, ErrUnsupported)
	require.ErrorContains(t, err, "already a draft body")

	_, err = draftReceiver(&Body{payload: revolvePayload{}})
	require.ErrorIs(t, err, ErrUnsupported)
}

// TestCompleteWallSetReadsRoles pins the wall-set test: a face counts as a
// wall by its own producer's side(i, j) role, never by another producer's.
func TestCompleteWallSetReadsRoles(t *testing.T) {
	t.Parallel()
	wall := func(p producerID, role string) *Face {
		return &Face{origins: []FeatureRef{{producer: p, Role: role}}}
	}
	own, foreign := wall(7, "side(0,1)"), wall(8, "side(0,1)")
	capStart := wall(7, roleCapStart)
	require.True(t, own.isWallOf(7))
	require.False(t, foreign.isWallOf(7))
	require.False(t, capStart.isWallOf(7))

	b := &Body{lumps: []*Lump{{shells: []*Shell{{faces: []*Face{own, capStart}}}}}}
	require.NoError(t, requireCompleteWallSet(b, 7, []*Face{own}))
	require.ErrorIs(t, requireCompleteWallSet(b, 7, nil), ErrUnsupported)
	require.ErrorIs(t, requireCompleteWallSet(b, 7, []*Face{own, capStart}), ErrDegenerate)
}
