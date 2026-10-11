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
