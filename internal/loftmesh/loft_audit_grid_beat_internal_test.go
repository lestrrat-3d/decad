package loftmesh

import (
	"math"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// This file pins the grid's early stop in newPairScan. FALSIFICATION LOG.
// Each break was made in loft_audit_grid.go, these tests re-run, and the
// change reverted; each went RED:
//
//   - stopping at half the beat (g.work >= beat/2): the equivalence, tie and
//     beat tests, and TestPairScanChoosesTheLowestWork.
//   - g.work > beat instead of >=: the beat test and the tie test, at
//     beat == work.
//   - no stop on the last column: the beat test, at beat == work.
//   - a count that omits the registrations (work-registrations >= beat): the
//     beat and tie tests.
//   - passing min(bestWork, limit) as the beat: the equivalence test, at a
//     limit between the grid's work and bestWork.
//   - no stop at all: the beat and tie tests.
//
// Two breaks stay green, and both are the same scan: checking before an
// entry's comparisons instead of after (the last entry of a column adds no
// comparisons, so the same totals are checked), and checking once per column
// instead of once per entry (a coarser stop, still at or past beat before the
// column ends).

// pairScanReference is newPairScan before the early stop: the grid's work is
// counted to the end, and the grid is kept when that count is below bestWork.
func pairScanReference(boxes [][2]r3.Vec, members []int, limit uint64) pairScan {
	sweep := newSweepOrder(boxes, members)
	best, bestWork := pairScan(sweep), sweep.work()
	if bestWork > 1 {
		tree := newTreeScan(boxes, members)
		if treeWork, ok := tree.workBound(min(bestWork, limit)); ok && treeWork < bestWork {
			best, bestWork = tree, treeWork
		}
	}
	grid, ok := newGridScan(boxes, members, min(bestWork, limit))
	if ok && grid.work < bestWork {
		best = grid
	}
	return best
}

// beatFixtures are the grid fixtures plus the shapes the early stop
// targets or could mishandle: nothing, one box, a cap fan whose boxes all
// share one corner, flat and identical boxes, and random small sets.
func beatFixtures() []gridFixture {
	out := gridFixtures()
	out = append(out, gridFixture{"empty", nil})
	out = append(out, gridFixture{"one box", [][2]r3.Vec{gridBox(r3.NewVec(0, 0, 0), r3.NewVec(1, 1, 1))}})
	out = append(out, gridFixture{"two boxes", [][2]r3.Vec{
		gridBox(r3.NewVec(0, 0, 0), r3.NewVec(1, 1, 1)), gridBox(r3.NewVec(1, 1, 1), r3.NewVec(2, 2, 2)),
	}})
	// One-sided: every box lies on one side of the others along x.
	var oneSided [][2]r3.Vec
	for i := range 40 {
		oneSided = append(oneSided, gridBox(r3.NewVec(float64(i)*3, 0, 0), r3.NewVec(float64(i)*3+1, 1, 1)))
	}
	out = append(out, gridFixture{"disjoint along x", oneSided})
	// A cap fan: thin triangles around an apex, so every box holds the apex.
	var fan [][2]r3.Vec
	const spokes = 300
	for k := range spokes {
		a := 2 * math.Pi * float64(k) / spokes
		b := 2 * math.Pi * float64(k+1) / spokes
		fan = append(fan, gridBox(r3.NewVec(0, 0, 0), r3.NewVec(5*math.Cos(a), 5*math.Sin(b), 0.01)))
	}
	out = append(out, gridFixture{"cap fan", fan})
	rng := rand.New(rand.NewPCG(0x51ed, 0x270f))
	for n := range 60 {
		var boxes [][2]r3.Vec
		scale := []float64{0.1, 1, 3, 10}[n%4]
		for range 2 + n {
			lo := r3.NewVec(rng.Float64()*10, rng.Float64()*10, rng.Float64()*10*float64(n%2))
			boxes = append(boxes, gridBox(lo, lo.Add(r3.NewVec(rng.Float64()*scale, rng.Float64()*scale, rng.Float64()*scale))))
		}
		out = append(out, gridFixture{"random", boxes})
	}
	return append(out, tieFixtures(rng)...)
}

// tieFixtures searches small random sets for ones whose grid work equals the
// smaller of the sweep and tree work, the case the early stop must keep on the
// earlier scan. Each trial is a handful of boxes, so the search is cheap.
func tieFixtures(rng *rand.Rand) []gridFixture {
	var out []gridFixture
	for trial := 0; trial < 40000 && len(out) < 12; trial++ {
		// Small cubes on a coarse lattice: the grid puts each in about one
		// cell, so its work is near the box count and can equal a scan's.
		var boxes [][2]r3.Vec
		for range 3 + rng.IntN(6) {
			lo := r3.NewVec(float64(rng.IntN(3))*4, float64(rng.IntN(3))*4, float64(rng.IntN(3))*4)
			e := 0.1 + 0.8*rng.Float64()
			boxes = append(boxes, gridBox(lo, lo.Add(r3.NewVec(e, e, e))))
		}
		members := allMembers(len(boxes))
		grid, ok := newGridScan(boxes, members, math.MaxUint64)
		if !ok {
			continue
		}
		bestWork := newSweepOrder(boxes, members).work()
		if bestWork > 1 {
			if treeWork, ok := newTreeScan(boxes, members).workBound(bestWork); ok && treeWork < bestWork {
				bestWork = treeWork
			}
		}
		if grid.work == bestWork {
			out = append(out, gridFixture{"tie", boxes})
		}
	}
	return out
}

// TestGridScanStopsAtBeat checks that newGridScanBelow keeps the grid exactly
// when its finished work is below beat, at and around every boundary.
func TestGridScanStopsAtBeat(t *testing.T) {
	t.Parallel()
	for _, fx := range beatFixtures() {
		t.Run(fx.name, func(t *testing.T) {
			members := allMembers(len(fx.boxes))
			full, ok := newGridScan(fx.boxes, members, math.MaxUint64)
			if !ok {
				_, ok = newGridScanBelow(fx.boxes, members, math.MaxUint64, math.MaxUint64)
				require.False(t, ok, "a grid that cannot be built stays unbuilt")
				return
			}
			beats := []uint64{0, 1, full.registrations, full.registrations + 1, full.work - 1, full.work, full.work + 1,
				full.work / 2, math.MaxUint64}
			for _, beat := range beats {
				got, ok := newGridScanBelow(fx.boxes, members, math.MaxUint64, beat)
				require.Equal(t, full.work < beat, ok, "beat %d, work %d", beat, full.work)
				if ok {
					require.Equal(t, full, got, "a grid kept under beat %d is the whole grid", beat)
				}
			}
		})
	}
}

// TestPairScanMatchesCountingToTheEnd compares newPairScan with the version
// that counts the grid to the end, for limits at and around every work count.
func TestPairScanMatchesCountingToTheEnd(t *testing.T) {
	t.Parallel()
	gridWins, gridLoses, ties := 0, 0, 0
	for _, fx := range beatFixtures() {
		members := allMembers(len(fx.boxes))
		sweepWork := newSweepOrder(fx.boxes, members).work()
		treeWork, _ := newTreeScan(fx.boxes, members).workBound(math.MaxUint64)
		grid, ok := newGridScan(fx.boxes, members, math.MaxUint64)
		limits := []uint64{0, 1, math.MaxUint64, sweepWork, treeWork, grid.work, grid.work - 1, grid.work + 1,
			sweepWork - 1, sweepWork + 1, min(sweepWork, treeWork), min(sweepWork, treeWork) - 1}
		for _, limit := range limits {
			want := pairScanReference(fx.boxes, members, limit)
			got := newPairScan(fx.boxes, members, limit)
			require.Equal(t, want, got, "%s, limit %d", fx.name, limit)
		}
		want := pairScanReference(fx.boxes, members, math.MaxUint64)
		_, isGrid := want.(gridScan)
		switch {
		case ok && isGrid:
			gridWins++
		case ok && grid.work == min(sweepWork, treeWork):
			ties++
		case ok:
			gridLoses++
		}
	}
	// The comparison is only worth something when the three outcomes occur:
	// the grid wins, the grid loses, and its work equals the best other work.
	require.Positive(t, gridWins)
	require.Positive(t, gridLoses)
	require.Positive(t, ties, "no fixture ties the grid with the best other scan")
}

// TestPairScanTieKeepsTheEarlierScan runs a tie exactly at the bound: the
// grid's finished work equals the best other scan's, and the other scan wins.
func TestPairScanTieKeepsTheEarlierScan(t *testing.T) {
	t.Parallel()
	found := false
	for _, fx := range beatFixtures() {
		members := allMembers(len(fx.boxes))
		grid, ok := newGridScan(fx.boxes, members, math.MaxUint64)
		if !ok {
			continue
		}
		sweepWork := newSweepOrder(fx.boxes, members).work()
		bestWork := sweepWork
		if sweepWork > 1 {
			if treeWork, ok := newTreeScan(fx.boxes, members).workBound(sweepWork); ok && treeWork < bestWork {
				bestWork = treeWork
			}
		}
		if grid.work != bestWork {
			continue
		}
		found = true
		_, isGrid := newPairScan(fx.boxes, members, math.MaxUint64).(gridScan)
		require.False(t, isGrid, "%s: a tie keeps the earlier scan", fx.name)
		_, ok = newGridScanBelow(fx.boxes, members, math.MaxUint64, bestWork)
		require.False(t, ok)
		_, ok = newGridScanBelow(fx.boxes, members, math.MaxUint64, bestWork+1)
		require.True(t, ok)
	}
	require.True(t, found, "no fixture ties the grid with the best other scan")
}
