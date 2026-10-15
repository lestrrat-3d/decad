package stackedrecord

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topo/topotest"
	"github.com/stretchr/testify/require"
)

// TestLumpsNeedsOneOuterShellForACavity reads lumps directly: a connected
// face set with no anchor is a void, attached to the one outer shell, and
// several outer shells beside a cavity are ErrUnsupported.
func TestLumpsNeedsOneOuterShellForACavity(t *testing.T) {
	outerA, outerB, cavity := &topo.Face{}, &topo.Face{}, &topo.Face{}
	anchors := map[*topo.Face]struct{}{outerA: {}, outerB: {}}
	got, err := lumps(topotest.Alloc{}, []*topo.Face{outerA, cavity}, anchors)
	require.NoError(t, err)
	require.Len(t, got, 1)
	require.Len(t, got[0].Shells, 2)
	require.True(t, got[0].Shells[1].Void)
	_, err = lumps(topotest.Alloc{}, []*topo.Face{outerA, outerB, cavity}, anchors)
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
	got, err = lumps(topotest.Alloc{}, []*topo.Face{outerA, outerB}, anchors)
	require.NoError(t, err)
	require.Len(t, got, 2)
}
