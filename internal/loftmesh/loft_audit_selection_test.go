package loftmesh

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func selectedLoftFixture(crossing bool) vertexFixture {
	f := vertexFixture{
		verts: []r3.Vec{{}, {X: 2, Z: 1}, {Y: 2}, {X: 2, Y: 2, Z: 1},
			{X: 10}, {X: 12, Z: 1}, {X: 10, Y: 2}, {X: 12, Y: 2, Z: 1}},
		tris: [][3]int{{0, 1, 2}, {1, 3, 2}, {4, 5, 6}, {5, 7, 6}},
	}
	if crossing {
		f.verts[7] = r3.Vec{X: 10.5, Y: 0.5, Z: 0.25}
		f.tris[3] = [3]int{4, 5, 7}
	}
	fan := flatFan("fan", uniformAngles(40), []float64{1}, 10, false)
	offset := len(f.verts)
	f.verts = append(f.verts, fan.verts...)
	for _, tri := range fan.tris {
		f.tris = append(f.tris, [3]int{tri[0] + offset, tri[1] + offset, tri[2] + offset})
	}
	return f
}

func TestSelectedLoftTablesReachTheSamePairVerdict(t *testing.T) {
	t.Parallel()
	for _, crossing := range []bool{false, true} {
		f := selectedLoftFixture(crossing)
		boxes := make([][2]r3.Vec, len(f.tris))
		for i, tri := range f.tris {
			boxes[i] = meshbool.TriBox(f.verts, tri)
		}
		members := allMembers(len(f.tris))
		tree := newTreeScan(boxes, members)
		sc, err := sweepCandidateCounts(unlimitedBudget(t), tree, len(f.tris), keepEveryPair, math.MaxUint64)
		require.NoError(t, err)
		require.False(t, sc.exceeded)
		data, err := prepareLoftStreamAuditData(unlimitedBudget(t), tree, members, sc.starts,
			math.MaxUint64, f.verts, f.tris, loftGenericShortcuts, true)
		require.NoError(t, err)
		require.True(t, data.preflightProofs)
		require.Nil(t, data.Norms[0].W, "the proven nonflat pair still built exact tables")
		require.NotNil(t, data.fans, "the selected layout lost the exact flat fan")
		if crossing {
			require.NotNil(t, data.Norms[2].W)
			require.NotNil(t, data.Norms[3].W)
		}
		cands, err := sweepCandidates(unlimitedBudget(t), tree, sc.starts, sc.count, keepEveryPair)
		require.NoError(t, err)
		var got, want LoftAuditWork
		gi, gj, gotErr := parallelLoftPairs(unlimitedBudget(t), f.tris, boxes, data, loftGenericShortcuts, cands, &got)
		full := newCompactLoftAuditData(f.verts, f.tris)
		// The facet certificate also covers pairs outside S8's candidate set.
		// Compare every incident pair with the independent exact classifier.
		checked := 0
		for i := range f.tris {
			for j := i + 1; j < len(f.tris); j++ {
				if !data.preflightCertified(i, j) {
					continue
				}
				_, exactErr := AuditLoftPairData(full, f.tris, i, j, LoftAuditShortcuts{})
				require.NoError(t, exactErr, "cached pair %d,%d", i, j)
				_, cachedErr := AuditLoftPairData(data, f.tris, i, j, loftGenericShortcuts)
				require.NoError(t, cachedErr, "cached pair %d,%d", i, j)
				checked++
			}
		}
		require.Positive(t, checked)
		wi, wj, wantErr := parallelLoftPairs(unlimitedBudget(t), f.tris, boxes, full, loftGenericShortcuts, cands, &want)
		require.Equal(t, wi, gi)
		require.Equal(t, wj, gj)
		if crossing {
			require.Error(t, wantErr)
			require.EqualError(t, gotErr, wantErr.Error())
		} else {
			require.NoError(t, wantErr)
			require.NoError(t, gotErr)
			require.Equal(t, int(sc.count), got.Skips+got.EdgeCerts+got.VertexCerts+got.Classifications)
			owned := append([][3]int(nil), f.tris...)
			f.verts[0] = r3.Vec{X: 1e6}
			f.tris[0][0], f.tris[0][1] = f.tris[0][1], f.tris[0][0]
			_, orderErr := AuditLoftPairData(data, f.tris, 0, 1, loftGenericShortcuts)
			require.ErrorIs(t, orderErr, ErrLoftAuditOrder)
			var unchanged LoftAuditWork
			_, _, err := parallelLoftPairs(unlimitedBudget(t), owned, boxes, data, loftGenericShortcuts, cands, &unchanged)
			require.NoError(t, err, "caller writes changed the cached proof's owned geometry")
		}
	}
}

func TestSelectedLoftTablesDoNotCertifyOmittedPairs(t *testing.T) {
	t.Parallel()
	f := selectedLoftFixture(true)
	boxes := make([][2]r3.Vec, len(f.tris))
	for i, tri := range f.tris {
		boxes[i] = meshbool.TriBox(f.verts, tri)
	}
	for _, filtered := range []bool{false, true} {
		members := allMembers(len(f.tris))
		var order pairScan
		var rows loftRowScan
		if filtered {
			grouped := newGroupedTreeScan(boxes, members, make([]int, len(f.tris)))
			order, rows = grouped, grouped
		} else {
			members = members[:1]
			partial := newTreeScan(boxes, members)
			order, rows = partial, partial
		}
		sc, err := sweepCandidateCounts(unlimitedBudget(t), order, len(f.tris), keepEveryPair, math.MaxUint64)
		require.NoError(t, err)
		data, err := prepareLoftStreamAuditData(unlimitedBudget(t), rows, members, sc.starts,
			math.MaxUint64, f.verts, f.tris, loftGenericShortcuts, true)
		require.NoError(t, err)
		require.False(t, data.preflightProofs)
		_, err = AuditLoftPairData(data, f.tris, 2, 3, LoftAuditShortcuts{})
		require.Error(t, err, "omitted overlapping triangles gained a facet certificate")
	}
}

func TestSelectedLoftTablesKeepNonfiniteVertexRefusal(t *testing.T) {
	t.Parallel()
	for _, bad := range []float64{math.NaN(), math.Inf(1)} {
		f := selectedLoftFixture(false)
		f.verts = append(f.verts, r3.Vec{X: bad})
		boxes := make([][2]r3.Vec, len(f.tris))
		for i, tri := range f.tris {
			boxes[i] = meshbool.TriBox(f.verts, tri)
		}
		members := allMembers(len(f.tris))
		tree := newTreeScan(boxes, members)
		sc, err := sweepCandidateCounts(unlimitedBudget(t), tree, len(f.tris), keepEveryPair, math.MaxUint64)
		require.NoError(t, err)
		require.Panics(t, func() {
			_, _ = prepareLoftStreamAuditData(unlimitedBudget(t), tree, members, sc.starts,
				math.MaxUint64, f.verts, f.tris, loftGenericShortcuts, true)
		}, "an unused nonfinite vertex must retain the complete constructor's refusal")
	}
}

func TestLoftPreflightObserverPreservesBudgetFailures(t *testing.T) {
	t.Parallel()
	boxes := randomBoxes(71, 80, 0.3)
	members := allMembers(len(boxes))
	tree := newTreeScan(boxes, members)
	sc, err := sweepCandidateCountsWorkers(unlimitedBudget(t), tree, len(boxes), keepEveryPair, math.MaxUint64, 1)
	require.NoError(t, err)
	var total uint64
	require.NoError(t, preflightLoftRowsWorkers(countingBudget(math.MaxUint64, &total), tree, members,
		sc.starts, math.MaxUint64, 1))
	for _, workers := range []int{1, 4} {
		for _, ceiling := range []uint64{total / 2, total - 1, total, math.MaxUint64} {
			for _, limit := range []uint64{0, 1, total / 2, total, total + 1, math.MaxUint64} {
				var wantSteps, gotSteps uint64
				wantErr := preflightLoftRowsWorkers(countingBudget(limit, &wantSteps), tree, members,
					sc.starts, ceiling, workers)
				gotErr := preflightLoftRowsWorkers(countingBudget(limit, &gotSteps), tree, members,
					sc.starts, ceiling, workers, loftPairObserver(func(i, j int) {
						require.Greater(t, j, i)
					}))
				require.Equal(t, wantSteps, gotSteps, "workers %d ceiling %d limit %d", workers, ceiling, limit)
				require.Equal(t, wantErr, gotErr, "workers %d ceiling %d limit %d", workers, ceiling, limit)
			}
		}
	}
}

func TestSelectedLoftTablesFallBackAtExtremeScales(t *testing.T) {
	t.Parallel()
	for _, scale := range []float64{0x1p-600, 0x1p600} {
		f := selectedLoftFixture(false)
		for i := range f.verts {
			f.verts[i] = f.verts[i].Scale(scale)
		}
		boxes := make([][2]r3.Vec, len(f.tris))
		for i, tri := range f.tris {
			boxes[i] = meshbool.TriBox(f.verts, tri)
		}
		members := allMembers(len(f.tris))
		tree := newTreeScan(boxes, members)
		sc, err := sweepCandidateCounts(unlimitedBudget(t), tree, len(f.tris), keepEveryPair, math.MaxUint64)
		require.NoError(t, err)
		data, err := prepareLoftStreamAuditData(unlimitedBudget(t), tree, members, sc.starts,
			math.MaxUint64, f.verts, f.tris, loftGenericShortcuts, true)
		require.NoError(t, err)
		require.False(t, data.preflightProofs, "scale %g should require the complete exact tables", scale)
		for i := range f.tris {
			require.NotNil(t, data.Norms[i].W, "scale %g triangle %d lacks exact data", scale, i)
		}
	}
}

func TestSelectedLoftTablesKeepProjectedPreparation(t *testing.T) {
	t.Parallel()
	f := selectedLoftFixture(false)
	boxes := make([][2]r3.Vec, len(f.tris))
	for i, tri := range f.tris {
		boxes[i] = meshbool.TriBox(f.verts, tri)
	}
	members := allMembers(len(f.tris))
	tree := newTreeScan(boxes, members)
	sc, err := sweepCandidateCounts(unlimitedBudget(t), tree, len(f.tris), keepEveryPair, math.MaxUint64)
	require.NoError(t, err)
	data, err := prepareLoftStreamAuditData(unlimitedBudget(t), tree, members, sc.starts,
		math.MaxUint64, f.verts, f.tris, loftGenericShortcuts, false)
	require.NoError(t, err)
	require.False(t, data.preflightProofs)
	for i := range f.tris {
		require.NotNil(t, data.Norms[i].W, "triangle %d lost its complete exact data", i)
	}
}
