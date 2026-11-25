package meshbool

import (
	"context"
	"math/rand/v2"
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// heldGridBudget is an unbounded budget that still has a step function, so
// repair keeps the uniform grid: the reference the tree is compared with.
func heldGridBudget(ctx context.Context) *proofbound.WorkBudget {
	b := proofbound.NewWorkBudget(ctx)
	b.StepFn = ctx.Err
	return b
}

func TestHeldRepairUsesTreeOnlyOnUnboundedBudgets(t *testing.T) {
	require.True(t, heldRepairUsesTree(proofbound.NewWorkBudget(t.Context())))
	require.False(t, heldRepairUsesTree(heldGridBudget(t.Context())))
	bounded := proofbound.NewWorkBudget(t.Context())
	bounded.ReserveFn = func(uint64) error { return nil }
	require.False(t, heldRepairUsesTree(bounded))

	h := heldSeedCase(t, seedOne)
	for _, useTree := range []bool{false, true} {
		e := newHeldEmbeddingScan(h, proofbound.NewWorkBudget(t.Context()), true)
		e.useTree = useTree
		require.NoError(t, e.buildGrid())
		require.Equal(t, useTree, e.rtree != nil)
		require.Equal(t, useTree, e.cells == nil)
	}
}

// TestHeldRepairTreeKeepsRepairResult checks that repair over the box tree
// gives the result of repair over the grid, on the memo and memo-free paths:
// the same vertices, moved count and failing facets, with and without the
// reach stage.
func TestHeldRepairTreeKeepsRepairResult(t *testing.T) {
	for _, kind := range []heldSeedKind{seedOne, seedCollapsed} {
		for _, reach := range []float64{0, 1e-9} {
			for _, memo := range []bool{false, true} {
				h := heldSeedCase(t, kind)
				h.Reach = reach
				grid, tree := h, h
				grid.Verts, tree.Verts = slices.Clone(h.Verts), slices.Clone(h.Verts)
				wantMoved, wantFailing, err := repairHeldEmbeddingShared(t.Context(), grid, memo, nil, heldGridBudget(t.Context()))
				require.NoError(t, err)
				gotMoved, gotFailing, err := repairHeldEmbeddingShared(t.Context(), tree, memo, nil,
					proofbound.NewWorkBudget(t.Context()))
				require.NoError(t, err)
				t.Logf("%s reach=%g memo=%v: %d moved, %d failing", kind, reach, memo, gotMoved, len(gotFailing))
				require.Equal(t, wantMoved, gotMoved, "%s reach=%g memo=%v moved count", kind, reach, memo)
				requireSameInts(t, "failing facets", wantFailing, gotFailing)
				requireSameVerts(t, grid.Verts, tree.Verts)
			}
		}
	}
}

// TestHeldRepairTreeOffendersMatchGrid checks that the first sweep over the
// tree lists the vertices and failing facets the grid's first sweep lists.
func TestHeldRepairTreeOffendersMatchGrid(t *testing.T) {
	for _, kind := range []heldSeedKind{seedMany, seedCollapsed} {
		for _, memo := range []bool{false, true} {
			h := heldSeedCase(t, kind)
			query := heldSeedQuery(h)
			var verts, failing [2][]int
			for k, useTree := range []bool{false, true} {
				e := newHeldEmbeddingScan(h, proofbound.NewWorkBudget(t.Context()), memo)
				e.useTree = useTree
				require.NoError(t, e.buildGrid())
				e.prepareRepair()
				if memo {
					e.pairs = map[uint64]uint64{}
				}
				var err error
				verts[k], failing[k], err = e.offenders(query)
				require.NoError(t, err)
			}
			requireSameInts(t, "vertices to try", verts[0], verts[1])
			requireSameInts(t, "failing facets", failing[0], failing[1])
		}
	}
}

// TestHeldRepairTreeCandidatesCoverOverlaps moves vertices as repair does and
// as far as the padding allows, and checks that for every facet the tree lists
// each facet whose current box overlaps its own, which is what the grid lists.
func TestHeldRepairTreeCandidatesCoverOverlaps(t *testing.T) {
	h := heldSeedCase(t, seedMany)
	rng := rand.New(rand.NewPCG(7, 11))
	var emb [2]*heldEmbedding
	for k, useTree := range []bool{false, true} {
		g := h
		g.Verts = slices.Clone(h.Verts)
		emb[k] = newHeldEmbeddingScan(g, proofbound.NewWorkBudget(t.Context()), true)
		emb[k].useTree = useTree
		require.NoError(t, emb[k].buildGrid())
		emb[k].prepareRepair()
	}
	pad := emb[1].pad
	require.Greater(t, pad, 0.0)
	// The same moves on both: a corner of the exact point's float box, or a
	// shift of up to a quarter of the padding on each axis.
	for range 200 {
		v := rng.IntN(len(h.Verts))
		var to r3.Vec
		if rng.IntN(2) == 0 {
			corners := FloatBoxCorners(h.Exact[v])
			to = corners[rng.IntN(len(corners))]
		} else {
			d := func() float64 { return (rng.Float64()*2 - 1) * pad / 4 }
			to = h.Verts[v].Add(r3.NewVec(d(), d(), d()))
		}
		for _, e := range emb {
			e.setVert(v, to)
		}
	}
	missing, extra, checked := 0, 0, 0
	firstMissing := [2]int{-1, -1}
	for i := range h.Tris {
		if i%7 != 0 {
			continue
		}
		checked++
		var sets [2]map[int]struct{}
		for k, e := range emb {
			cands, err := e.candidates(i)
			require.NoError(t, err)
			sets[k] = map[int]struct{}{}
			box := e.triBox(i)
			for _, j := range cands {
				if j != i && BoxesOverlap(box, e.triBox(j)) {
					sets[k][j] = struct{}{}
				}
			}
		}
		box := emb[1].triBox(i)
		for j := range h.Tris {
			if j == i || !BoxesOverlap(box, emb[1].triBox(j)) {
				continue
			}
			if _, ok := sets[1][j]; !ok {
				if missing == 0 {
					firstMissing = [2]int{i, j}
				}
				missing++
			}
		}
		if len(sets[0]) != len(sets[1]) {
			extra++
		}
	}
	require.Positive(t, checked)
	require.Zero(t, missing, "overlapping facets the tree does not list, first pair %v", firstMissing)
	require.Zero(t, extra, "facets whose overlapping candidates differ between tree and grid")
}

// TestHeldRepairTreeKeepsStitchedRepairResult checks the same on layered
// meshes whose repair moves vertices.
func TestHeldRepairTreeKeepsStitchedRepairResult(t *testing.T) {
	movedCases := 0
	for seed := range uint64(60) {
		h := stitchLayered(rand.New(rand.NewPCG(seed, 1)), 6+int(seed%40))
		for _, memo := range []bool{false, true} {
			grid, tree := cloneStitchHeld(h), cloneStitchHeld(h)
			wantMoved, wantFailing, err := repairHeldEmbeddingShared(t.Context(), grid, memo, nil, heldGridBudget(t.Context()))
			require.NoError(t, err)
			gotMoved, gotFailing, err := repairHeldEmbeddingShared(t.Context(), tree, memo, nil,
				proofbound.NewWorkBudget(t.Context()))
			require.NoError(t, err)
			require.Equal(t, wantMoved, gotMoved, "seed %d memo=%v moved count", seed, memo)
			requireSameInts(t, "failing facets", wantFailing, gotFailing)
			requireSameVerts(t, grid.Verts, tree.Verts)
			if gotMoved > 0 {
				movedCases++
			}
		}
	}
	require.Positive(t, movedCases, "some layered case moves a vertex")
	t.Logf("%d of 120 runs moved a vertex", movedCases)
}
