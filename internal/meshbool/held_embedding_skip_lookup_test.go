package meshbool

import (
	"context"
	"maps"
	"math/rand/v2"
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/stretchr/testify/require"
)

// heldFirstPass runs the first sweep's offenders on a fresh memo, with the
// pair memo lookups skipped or not, and returns what it listed and stored.
func heldFirstPass(t *testing.T, h HeldRounding, skip bool) ([]int, []int, map[uint64]uint64) {
	t.Helper()
	g := h
	g.Verts = slices.Clone(h.Verts)
	e := newHeldEmbeddingScan(g, proofbound.NewWorkBudget(t.Context()), true)
	require.NoError(t, e.buildGrid())
	e.prepareRepair()
	e.pairs = map[uint64]uint64{}
	e.skipLookup = skip
	verts, failing, err := e.offenders(heldSeedQuery(g))
	require.NoError(t, err)
	return verts, failing, e.pairs
}

// TestHeldFirstPassSkipsMemoLookupsWithoutChange checks that the first sweep
// lists the same vertices and failing facets, and stores the same pairs, when
// it does not read the memo it starts with empty.
func TestHeldFirstPassSkipsMemoLookupsWithoutChange(t *testing.T) {
	cases := map[string]HeldRounding{}
	for seed := range uint64(40) {
		h := stitchLayered(rand.New(rand.NewPCG(seed, 1)), 6+int(seed%40))
		h.Moved = make([]bool, len(h.Verts))
		for i := range h.Moved {
			h.Moved[i] = true
		}
		h.Movable = slices.Clone(h.Moved)
		cases["layered/"+string(rune('A'+seed%26))+string(rune('a'+seed/26))] = h
	}
	for _, kind := range []heldSeedKind{seedOne, seedCollapsed} {
		cases["grid/"+string(kind)] = heldSeedCase(t, kind)
	}
	failingCases, stored := 0, 0
	for name, h := range cases {
		wantVerts, wantFailing, wantPairs := heldFirstPass(t, h, false)
		gotVerts, gotFailing, gotPairs := heldFirstPass(t, h, true)
		requireSameInts(t, name+" vertices", wantVerts, gotVerts)
		requireSameInts(t, name+" failing facets", wantFailing, gotFailing)
		require.True(t, maps.Equal(wantPairs, gotPairs), "%s: stored pairs differ (%d want, %d got)", name,
			len(wantPairs), len(gotPairs))
		if len(wantFailing) > 0 {
			failingCases++
		}
		stored += len(wantPairs)
	}
	require.Positive(t, failingCases, "some case fails the first sweep")
	require.Positive(t, stored, "some case stores a pair")
}

// TestHeldRepairMemoKeepsMemoFreeResult checks that the memo path, whose
// first sweep skips the memo lookups, repairs layered meshes as the memo-free
// path does: the same vertices, moved count and failing facets.
func TestHeldRepairMemoKeepsMemoFreeResult(t *testing.T) {
	movedCases := 0
	for seed := range uint64(60) {
		h := stitchLayered(rand.New(rand.NewPCG(seed, 1)), 6+int(seed%40))
		plain, memo := cloneStitchHeld(h), cloneStitchHeld(h)
		wantMoved, wantFailing, err := repairHeldEmbedding(t.Context(), plain, false)
		require.NoError(t, err)
		gotMoved, gotFailing, err := repairHeldEmbedding(t.Context(), memo, true)
		require.NoError(t, err)
		require.Equal(t, wantMoved, gotMoved, "seed %d moved count", seed)
		requireSameInts(t, "failing facets", wantFailing, gotFailing)
		requireSameVerts(t, plain.Verts, memo.Verts)
		if gotMoved > 0 {
			movedCases++
		}
	}
	require.Positive(t, movedCases, "some layered case moves a vertex")
}

// TestHeldFirstPassSkippingLookupsStillPolls checks that a cancelled context
// stops the first sweep with the context error when it skips the memo.
func TestHeldFirstPassSkippingLookupsStillPolls(t *testing.T) {
	h := heldSeedCase(t, seedOne)
	e := newHeldEmbeddingScan(h, proofbound.NewWorkBudget(t.Context()), true)
	require.NoError(t, e.buildGrid())
	e.prepareRepair()
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	e.budget = proofbound.NewWorkBudget(ctx)
	e.pairs = map[uint64]uint64{}
	e.skipLookup = true
	_, _, err := e.offenders(heldSeedQuery(h))
	require.ErrorIs(t, err, context.Canceled)
}
