package loftmesh

import (
	"math"
	"testing"

	"github.com/stretchr/testify/require"
)

// groupedWalkRowReference is groupedTreeScan.walkRow without the root test on
// rootBox and rootMax: every group's own tree walk, in group order.
func groupedWalkRowReference(s groupedTreeScan, i int, limit uint64, fn func(j int)) (uint64, bool) {
	var scanned uint64
	for k, group := range s.groups {
		if s.keys[k] >= 0 && s.keys[k] == s.family[i] {
			continue
		}
		used, exceeded := group.walkRow(i, limit-scanned, fn)
		scanned += used
		if exceeded {
			return scanned, true
		}
	}
	return scanned, false
}

// TestGroupedWalkRowMatchesReference compares walkRow and walkRowAppend with
// the per-group walk at limits that stop the walk inside a rejected root,
// inside a tree and after the last group: the partners, their order, the work
// count and the exceed must all agree. The radial fixture checks every row,
// the larger random fixtures every fifth.
func TestGroupedWalkRowMatchesReference(t *testing.T) {
	fixtures := append(groupFixtures(), radialLugFixture(12, 60, 20, 200))
	for _, fixture := range fixtures {
		t.Run(fixture.name, func(t *testing.T) {
			scan := fixture.scan()
			stride := 5
			if len(scan.groups) > 5 {
				stride = 1
			}
			rejected := 0
			for i := 0; i < len(fixture.boxes); i += stride {
				full, exceeded := groupedWalkRowReference(scan, i, math.MaxUint64, func(int) {})
				require.False(t, exceeded)
				limits := []uint64{0, 1, 2, full / 3, full / 2, max(full, 1) - 1, full, full + 1, math.MaxUint64}
				for _, limit := range limits {
					var want, got []int32
					wantWork, wantExceeded := groupedWalkRowReference(scan, i, limit,
						func(j int) { want = append(want, int32(j)) })
					gotWork, gotExceeded := scan.walkRow(i, limit, func(j int) { got = append(got, int32(j)) })
					require.Equal(t, wantWork, gotWork, "row %d limit %d", i, limit)
					require.Equal(t, wantExceeded, gotExceeded, "row %d limit %d", i, limit)
					require.Equal(t, want, got, "row %d limit %d", i, limit)
					appended, appendWork, appendExceeded := scan.walkRowAppend(i, limit, nil)
					require.Equal(t, wantWork, appendWork, "row %d limit %d", i, limit)
					require.Equal(t, wantExceeded, appendExceeded, "row %d limit %d", i, limit)
					require.Equal(t, want, appended, "row %d limit %d", i, limit)
				}
				bi := box6(fixture.boxes[i])
				for k := range scan.groups {
					if scan.rootRejects(&bi, i, k) {
						rejected++
					}
				}
			}
			if stride == 1 {
				// The radial fixture's rows meet far groups whose roots reject.
				require.Greater(t, rejected, len(fixture.boxes))
			}
		})
	}
}

// TestGroupedRowFillMatchesSweepCandidates compares the parallel row fill of
// the radial fixture, whose rows mostly meet rejected roots, with the serial
// pair walk's list.
func TestGroupedRowFillMatchesSweepCandidates(t *testing.T) {
	g := radialLugFixture(40, 100, 20, 400)
	scan := g.scan()
	members := allMembers(len(g.boxes))
	require.GreaterOrEqual(t, len(members), loftRowParallelMin)
	sc, err := sweepCandidateCountsWorkers(unlimitedBudget(t), scan, len(g.boxes), keepEveryPair, math.MaxUint64, 1)
	require.NoError(t, err)
	require.Positive(t, sc.count)
	want, err := sweepCandidates(unlimitedBudget(t), scan, sc.starts, sc.count, keepEveryPair)
	require.NoError(t, err)
	got, err := sweepCandidatesRowsWorkers(unlimitedBudget(t), scan, members, sc.starts, sc.count, sc.scanned, 4)
	require.NoError(t, err)
	require.Equal(t, want.starts, got.starts)
	require.Equal(t, want.js, got.js)
}
