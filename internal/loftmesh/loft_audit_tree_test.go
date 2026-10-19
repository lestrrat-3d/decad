package loftmesh

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/stretchr/testify/require"
)

func TestTreeScanEnumeratesEveryOverlappingPairOnce(t *testing.T) {
	t.Parallel()
	for _, fixture := range gridFixtures() {
		t.Run(fixture.name, func(t *testing.T) {
			members := make([]int, len(fixture.boxes))
			for i := range members {
				members[i] = i
			}
			tree := newTreeScan(fixture.boxes, members)
			wantWork, complete := tree.workBound(math.MaxUint64)
			require.True(t, complete)
			seen := map[[2]int]int{}
			scanned, exceeded, err := tree.visit(proofbound.NewWorkBudget(t.Context()), math.MaxUint64,
				func(i, j int) { seen[[2]int{i, j}]++ })
			require.NoError(t, err)
			require.False(t, exceeded)
			require.Equal(t, wantWork, scanned)
			want := 0
			for i := range fixture.boxes {
				for j := i + 1; j < len(fixture.boxes); j++ {
					if !meshbool.BoxesOverlap(fixture.boxes[i], fixture.boxes[j]) {
						continue
					}
					want++
					require.Equal(t, 1, seen[[2]int{i, j}], "pair (%d, %d)", i, j)
				}
			}
			require.Len(t, seen, want)
			if wantWork > 0 {
				_, complete = tree.workBound(wantWork - 1)
				require.False(t, complete)
				_, exceeded, err = tree.visit(proofbound.NewWorkBudget(t.Context()), wantWork-1,
					func(int, int) {})
				require.NoError(t, err)
				require.True(t, exceeded)
			}
		})
	}
}

func TestTreeScanRowsMatchPairCandidatesAndBoundWork(t *testing.T) {
	t.Parallel()
	for _, fixture := range gridFixtures() {
		t.Run(fixture.name, func(t *testing.T) {
			members := make([]int, 0, len(fixture.boxes))
			for i := range fixture.boxes {
				if i%7 != 3 {
					members = append(members, i)
				}
			}
			tree := newTreeScan(fixture.boxes, members)
			want := map[[2]int]struct{}{}
			_, exceeded, err := tree.visit(proofbound.NewWorkBudget(t.Context()), math.MaxUint64,
				func(i, j int) { want[[2]int{i, j}] = struct{}{} })
			require.NoError(t, err)
			require.False(t, exceeded)
			got := map[[2]int]struct{}{}
			var total uint64
			for _, i := range members {
				row := make([]int, 0)
				scanned, over, err := tree.visitRow(proofbound.NewWorkBudget(t.Context()), i,
					math.MaxUint64, func(j int) { row = append(row, j) })
				require.NoError(t, err)
				require.False(t, over)
				total += scanned
				for _, j := range row {
					require.Less(t, i, j)
					pair := [2]int{i, j}
					_, duplicate := got[pair]
					require.False(t, duplicate, "pair %v repeated", pair)
					got[pair] = struct{}{}
				}
			}
			require.Equal(t, want, got)
			if total > 0 {
				var capped uint64
				for _, i := range members {
					scanned, over, err := tree.visitRow(proofbound.NewWorkBudget(t.Context()), i,
						total-1-capped, func(int) {})
					require.NoError(t, err)
					capped += scanned
					if over {
						break
					}
				}
				require.Equal(t, total, capped)
			}
		})
	}
}

func TestPairScanChoosesTheLowestWork(t *testing.T) {
	t.Parallel()
	for _, fixture := range gridFixtures() {
		t.Run(fixture.name, func(t *testing.T) {
			members := make([]int, len(fixture.boxes))
			for i := range members {
				members[i] = i
			}
			sweepWork := newSweepOrder(fixture.boxes, members).work()
			grid, ok := newGridScan(fixture.boxes, members, math.MaxUint64)
			require.True(t, ok)
			treeWork, ok := newTreeScan(fixture.boxes, members).workBound(math.MaxUint64)
			require.True(t, ok)
			chosen := newPairScan(fixture.boxes, members, math.MaxUint64)
			var chosenWork uint64
			switch scan := chosen.(type) {
			case sweepOrder:
				chosenWork = scan.work()
			case gridScan:
				chosenWork = scan.work
			case treeScan:
				chosenWork, ok = scan.workBound(math.MaxUint64)
				require.True(t, ok)
			default:
				t.Fatalf("unexpected scan %T", chosen)
			}
			require.Equal(t, min(sweepWork, grid.work, treeWork), chosenWork)
		})
	}
}
