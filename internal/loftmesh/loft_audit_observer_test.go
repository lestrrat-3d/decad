package loftmesh

import (
	"context"
	"math"
	"sync/atomic"
	"testing"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestLoftFacetSelectorParallelMasksAndBudgets(t *testing.T) {
	t.Parallel()
	var f vertexFixture
	for instance := range 16 {
		part := selectedLoftFixture(instance%2 == 0)
		offset := len(f.verts)
		for _, v := range part.verts {
			f.verts = append(f.verts, v.Add(r3.Vec{X: 100 * float64(instance)}))
		}
		for _, tri := range part.tris {
			f.tris = append(f.tris, [3]int{tri[0] + offset, tri[1] + offset, tri[2] + offset})
		}
	}
	boxes := make([][2]r3.Vec, len(f.tris))
	for i, tri := range f.tris {
		boxes[i] = meshbool.TriBox(f.verts, tri)
	}
	members := allMembers(len(f.tris))
	tree := newTreeScan(boxes, members)
	sc, err := sweepCandidateCountsWorkers(unlimitedBudget(t), tree, len(f.tris), keepEveryPair, math.MaxUint64, 1)
	require.NoError(t, err)
	newSelector := func() *loftFacetSelector {
		return &loftFacetSelector{verts: f.verts, tris: f.tris, flat: loftFlatSelection(f.verts, f.tris),
			uncertain: make([]atomic.Bool, len(f.tris)), shortcuts: loftGenericShortcuts}
	}
	want := newSelector()
	for i := range f.tris {
		for j := i + 1; j < len(f.tris); j++ {
			if meshbool.BoxesOverlap(boxes[i], boxes[j]) {
				want.Observe(i, j)
			}
		}
	}
	var total uint64
	require.NoError(t, preflightLoftRowsWorkers(countingBudget(math.MaxUint64, &total), tree,
		members, sc.starts, math.MaxUint64, 1))
	for _, workers := range []int{1, 4} {
		for _, limit := range []uint64{0, 1, total / 2, total, total + 1, math.MaxUint64} {
			got := newSelector()
			var wantSteps, gotSteps uint64
			wantErr := preflightLoftRowsWorkers(countingBudget(limit, &wantSteps), tree,
				members, sc.starts, math.MaxUint64, workers)
			gotErr := preflightLoftRowsWorkers(countingBudget(limit, &gotSteps), tree,
				members, sc.starts, math.MaxUint64, workers, got)
			require.Equal(t, wantErr, gotErr, "workers %d limit %d", workers, limit)
			require.Equal(t, wantSteps, gotSteps, "workers %d limit %d", workers, limit)
			if gotErr != nil {
				continue
			}
			for i := range f.tris {
				require.Equal(t, want.uncertain[i].Load(), got.uncertain[i].Load(),
					"workers %d triangle %d", workers, i)
			}
		}
		got := newSelector()
		budget := proofbound.NewWorkBudget(t.Context())
		budget.ErrFn = func() error { return context.Canceled }
		require.ErrorIs(t, preflightLoftRowsWorkers(budget, tree, members, sc.starts,
			math.MaxUint64, workers, got), context.Canceled)
	}
}
