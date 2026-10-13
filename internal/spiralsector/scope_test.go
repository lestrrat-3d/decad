package spiralsector_test

import (
	"context"
	"testing"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/spiralsector"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// allowedWith returns 24 allowed pairs: key plus 23 fillers that no test
// triangle uses.
func allowedWith(key [2]int) map[[2]int]bool {
	allowed := map[[2]int]bool{key: true}
	for i := range 23 {
		allowed[[2]int{100 + i, 200 + i}] = true
	}
	return allowed
}

func TestScopeWidenedContacts(t *testing.T) {
	t.Parallel()
	ctx := t.Context()
	blank, err := meshbool.PrepContext(ctx,
		[]r3.Vec{{X: 0, Y: 0, Z: 0}, {X: 2, Y: 0, Z: 0}, {X: 0, Y: 2, Z: 0}},
		[][3]int{{0, 1, 2}}, []int{5})
	require.NoError(t, err)
	tooth, err := meshbool.PrepContext(ctx,
		[]r3.Vec{{X: 0.5, Y: 0.5, Z: -1}, {X: 0.5, Y: 0.5, Z: 1}, {X: 1.5, Y: 0.2, Z: 0}},
		[][3]int{{0, 1, 2}}, []int{7})
	require.NoError(t, err)
	tooth.FacetBound = []float64{0.5}
	memo := meshbool.NewContactMemo(blank, tooth)

	t.Run("allowed pair", func(t *testing.T) {
		ok, err := spiralsector.ScopeWidenedContacts(ctx, blank, tooth, memo,
			allowedWith([2]int{5, 7}), 0.1)
		require.NoError(t, err)
		require.True(t, ok)
	})
	t.Run("unlisted pair", func(t *testing.T) {
		ok, err := spiralsector.ScopeWidenedContacts(ctx, blank, tooth, memo,
			allowedWith([2]int{99, 99}), 0.1)
		require.NoError(t, err)
		require.False(t, ok)
	})
	t.Run("facet below the ordinary gate is skipped", func(t *testing.T) {
		ok, err := spiralsector.ScopeWidenedContacts(ctx, blank, tooth, memo,
			allowedWith([2]int{99, 99}), 0.6)
		require.NoError(t, err)
		require.True(t, ok)
	})
	t.Run("allowed count must match", func(t *testing.T) {
		allowed := allowedWith([2]int{5, 7})
		delete(allowed, [2]int{100, 200})
		ok, err := spiralsector.ScopeWidenedContacts(ctx, blank, tooth, memo, allowed, 0.1)
		require.NoError(t, err)
		require.False(t, ok)
	})
	t.Run("nil memo", func(t *testing.T) {
		ok, err := spiralsector.ScopeWidenedContacts(ctx, blank, tooth, nil,
			allowedWith([2]int{5, 7}), 0.1)
		require.NoError(t, err)
		require.False(t, ok)
	})
	t.Run("cancelled context", func(t *testing.T) {
		cancelled, cancel := context.WithCancel(ctx)
		cancel()
		ok, err := spiralsector.ScopeWidenedContacts(cancelled, blank, tooth, memo,
			allowedWith([2]int{5, 7}), 0.1)
		require.ErrorIs(t, err, context.Canceled)
		require.False(t, ok)
	})
}

func TestScopeOutputSwitches(t *testing.T) {
	t.Parallel()
	ctx := t.Context()
	held := func(tris [][3]int, src []int) *meshbool.StitchedMesh {
		return &meshbool.StitchedMesh{
			Verts: []r3.Vec{{X: 0, Y: 0, Z: 0}, {X: 1, Y: 0, Z: 0}, {X: 0, Y: 1, Z: 0}, {X: 0, Y: 0, Z: 1}},
			Tris:  tris,
			Src:   src,
		}
	}
	tetra := [][3]int{{0, 2, 1}, {0, 1, 3}, {0, 3, 2}, {1, 2, 3}}
	src := []int{0, 0, 1, 1}

	ok, err := spiralsector.ScopeOutputSwitches(ctx, held(tetra, src), 1, allowedWith([2]int{0, 1}))
	require.NoError(t, err)
	require.True(t, ok)

	ok, err = spiralsector.ScopeOutputSwitches(ctx, held(tetra, src), 1, allowedWith([2]int{99, 99}))
	require.NoError(t, err)
	require.False(t, ok, "a switch between unlisted source faces is refused")

	ok, err = spiralsector.ScopeOutputSwitches(ctx, held(tetra[:3], src[:3]), 1, allowedWith([2]int{0, 1}))
	require.NoError(t, err)
	require.False(t, ok, "an open mesh has an edge with one owner")
}
