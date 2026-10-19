package loftmesh

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestSparseProjectedAuditDataMatchesFullPairVerdicts(t *testing.T) {
	verts := []r3.Vec{
		{X: 0, Y: 0}, {X: 2, Y: 0}, {X: 2, Y: 2}, {X: 0, Y: 2},
		{X: 0.5, Y: 0.5, Z: -1}, {X: 1.5, Y: 0.5, Z: 1}, {X: 1, Y: 1.5, Z: -1},
		{X: 10, Y: 10}, {X: 11, Y: 10}, {X: 10, Y: 11},
	}
	tris := [][3]int{{0, 1, 2}, {0, 2, 3}, {4, 5, 6}, {7, 8, 9}}
	boxes := make([][2]r3.Vec, len(tris))
	members := make([]int, len(tris))
	for i, tri := range tris {
		boxes[i] = meshbool.TriBox(verts, tri)
		members[i] = i
	}
	order := newGroupedTreeScan(boxes, members, []int{0, 0, -1, -1})
	budget := proofbound.NewWorkBudget(t.Context())
	sc, err := sweepCandidateCounts(budget, order, len(tris), keepEveryPair, 100)
	require.NoError(t, err)
	require.False(t, sc.exceeded)
	cands, err := sweepCandidates(budget, order, sc.starts, sc.count, keepEveryPair)
	require.NoError(t, err)
	require.NotEmpty(t, cands.js)

	sparse := newLoftAuditDataForCandidates(verts, tris, cands)
	full := NewLoftAuditData(verts, tris)
	require.Nil(t, sparse.Norms[3].W)
	require.NotNil(t, full.Norms[3].W)
	require.NoError(t, cands.each(func(i, j int) error {
		require.NotNil(t, sparse.Norms[i].W)
		require.NotNil(t, sparse.Norms[j].W)
		got, gotErr := AuditLoftPairData(sparse, tris, i, j, loftGenericShortcuts)
		want, wantErr := AuditLoftPairData(full, tris, i, j, loftGenericShortcuts)
		require.Equal(t, want, got)
		if wantErr == nil {
			require.NoError(t, gotErr)
		} else {
			require.EqualError(t, gotErr, wantErr.Error())
		}
		return nil
	}))
}
