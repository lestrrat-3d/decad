package loftmesh

import (
	"cmp"
	"fmt"
	"math"
	"math/rand/v2"
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// This file checks the scan builds against reference builds kept here:
// gridScanBelowReference sorts the grid's items by comparison, and
// newSortTreeScan (loft_audit_select_test.go) sorts every tree node's range.
// The production builds must give the same structures, the same scan choice
// and the same walks. FALSIFICATION LOG. Each break was made in
// loft_audit_grid.go or loft_audit_tree.go, these tests re-run, and the
// change reverted; each went RED:
//
//   - sortGridItems scattering each pass from the last item down (an
//     unstable pass): both tests.
//   - sortGridItems running its lowest radix pass only: both tests.
//   - fillColumnsByKey leaving columns unsorted by zLo: the grid comparison.
//   - fillColumnsByKey scattering members in descending order: the grid
//     comparison.
//   - treeKey.less breaking ties by descending id: the tree comparison.
//   - buildSelected leaving leaf children unsorted: the tree comparison.
//
// Three breaks stay green, as they must, since each path gives the one
// sequence the (start, pos) order allows: sortGridItems comparing at every
// size, newGridScanBelow never taking fillColumnsByKey, and taking it for
// every grid under 2^24 columns.

// gridScanBelowReference is newGridScanBelow with its items sorted by
// comparison on (start, pos), and each member's cell range computed again
// for registration.
func gridScanBelowReference(boxes [][2]r3.Vec, members []int, budget, beat uint64) (gridScan, bool) {
	g := gridScan{boxes: boxes}
	if len(members) < 2 {
		return g, false
	}
	count := float64(len(members))
	side := math.Ceil(math.Cbrt(count))
	extents := make([]float64, min(len(members), gridMedianExtentSamples))
	for axis := range 3 {
		lo, hi := math.Inf(1), math.Inf(-1)
		for _, i := range members {
			a, b := vecAxis(boxes[i][0], axis), vecAxis(boxes[i][1], axis)
			lo, hi = math.Min(lo, a), math.Max(hi, b)
		}
		span := hi - lo
		if proofbound.IsNonFinite(span) {
			return g, false
		}
		for sample := range extents {
			i := members[sample*len(members)/len(extents)]
			a, b := vecAxis(boxes[i][0], axis), vecAxis(boxes[i][1], axis)
			extents[sample] = b - a
		}
		slices.Sort(extents)
		size := extents[len(extents)/2]
		if !(size > span/count) {
			size = span / side
		}
		if !(size > 0) {
			size = 1
		}
		g.lo[axis], g.size[axis] = lo, size
		g.n[axis] = int(math.Min(span/size, count)) + 1
	}
	total, columnTotal := uint64(0), uint64(0)
	for _, i := range members {
		lo, hi := g.cellRange(boxes[i])
		columns := uint64(hi[0]-lo[0]+1) * uint64(hi[1]-lo[1]+1)
		total += columns * uint64(hi[2]-lo[2]+1)
		columnTotal += columns
		if total > budget {
			return g, false
		}
	}
	g.registrations = total
	items := make([]gridItem, 0, columnTotal)
	for k, i := range members {
		lo, hi := g.cellRange(boxes[i])
		for x := lo[0]; x <= hi[0]; x++ {
			for y := lo[1]; y <= hi[1]; y++ {
				items = append(items, gridItem{start: g.key([3]int{x, y, lo[2]}), pos: int32(k), zHi: int32(hi[2])})
			}
		}
	}
	slices.SortFunc(items, func(p, q gridItem) int {
		if p.start != q.start {
			return cmp.Compare(p.start, q.start)
		}
		return cmp.Compare(p.pos, q.pos)
	})
	nz := int64(g.n[2])
	entries := make([]gridEntry, len(items))
	g.work = g.registrations
	for a := 0; a < len(items); {
		key := items[a].start / nz
		b := a
		for b < len(items) && items[b].start/nz == key {
			entries[b] = gridEntry{box: int32(members[items[b].pos]), zLo: int32(items[b].start % nz), zHi: items[b].zHi}
			b++
		}
		col := entries[a:b:b]
		for p, ep := range col {
			for _, eq := range col[p+1:] {
				if eq.zLo > ep.zHi {
					break
				}
				g.work += uint64(min(ep.zHi, eq.zHi) - eq.zLo + 1)
			}
			if g.work >= beat {
				return g, false
			}
		}
		g.columns = append(g.columns, gridColumn{key: key, entries: col})
		a = b
	}
	return g, true
}

// pairScanBuildReference is newPairScan over the reference builds.
func pairScanBuildReference(boxes [][2]r3.Vec, members []int, limit uint64) pairScan {
	sweep := newSweepOrder(boxes, members)
	best, bestWork := pairScan(sweep), sweep.work()
	if bestWork > 1 {
		tree := newSortTreeScan(boxes, members)
		if treeWork, ok := tree.workBound(min(bestWork, limit)); ok && treeWork < bestWork {
			best, bestWork = tree, treeWork
		}
	}
	grid, ok := gridScanBelowReference(boxes, members, min(bestWork, limit), bestWork)
	if ok && grid.work < bestWork {
		best = grid
	}
	return best
}

// requireSameGrid compares two complete grids field by field and reports
// only the first differing column, so a failure prints bounded data.
func requireSameGrid(t *testing.T, want, got gridScan, label string) {
	t.Helper()
	require.Equal(t, want.n, got.n, "%s: cell counts", label)
	require.True(t, sameFloats(want.lo[:], got.lo[:]), "%s: grid origin", label)
	require.True(t, sameFloats(want.size[:], got.size[:]), "%s: cell sizes", label)
	require.Equal(t, want.registrations, got.registrations, "%s: registrations", label)
	require.Equal(t, want.work, got.work, "%s: work", label)
	require.Len(t, got.columns, len(want.columns), "%s: column count", label)
	for c, w := range want.columns {
		g := got.columns[c]
		if w.key != g.key || !slices.Equal(w.entries, g.entries) {
			require.Failf(t, "grid", "%s: column %d differs: key %d/%d, %d/%d entries", label, c,
				w.key, g.key, len(w.entries), len(g.entries))
		}
	}
}

// scanWalk is one complete visit: the pairs in visit order and the work.
type scanWalk struct {
	pairs    [][2]int
	scanned  uint64
	exceeded bool
}

func walkScan(t *testing.T, s pairScan, limit uint64) scanWalk {
	t.Helper()
	var w scanWalk
	var err error
	w.scanned, w.exceeded, err = s.visit(proofbound.NewWorkBudget(t.Context()), limit, func(i, j int) {
		w.pairs = append(w.pairs, [2]int{i, j})
	})
	require.NoError(t, err)
	return w
}

func requireSameWalk(t *testing.T, want, got scanWalk, label string) {
	t.Helper()
	require.Equal(t, want.scanned, got.scanned, "%s: scanned", label)
	require.Equal(t, want.exceeded, got.exceeded, "%s: exceeded", label)
	require.Len(t, got.pairs, len(want.pairs), "%s: pair count", label)
	for k, p := range want.pairs {
		if got.pairs[k] != p {
			require.Failf(t, "walk", "%s: pair %d is %v, want %v", label, k, got.pairs[k], p)
		}
	}
}

// scanBuildFixtures are the beat fixtures plus a small lobed tube,
// boxes with duplicate and coincident vertices (points, segments, repeats),
// and scattered boxes, coarse and fine: the fine set has more (x, y) columns
// than registrations, which sends its grid to the item sort.
func scanBuildFixtures() []gridFixture {
	out := beatFixtures()
	out = append(out, gridFixture{"lobed tube", lobedTubeBoxes(40, 96, 12, 0.26, 0.02)})
	rng := rand.New(rand.NewPCG(0x5ca1, 0xb0e5))
	var degenerate [][2]r3.Vec
	for i := range 900 {
		p := r3.NewVec(float64(rng.IntN(6)), float64(rng.IntN(6)), float64(rng.IntN(6)))
		switch i % 3 {
		case 0: // all three vertices equal: a point box
			degenerate = append(degenerate, gridBox(p, p))
		case 1: // two vertices equal: a segment box
			degenerate = append(degenerate, gridBox(p, p.Add(r3.NewVec(0, 0, 1))))
		default: // a repeat of an earlier box
			degenerate = append(degenerate, degenerate[rng.IntN(len(degenerate))])
		}
	}
	out = append(out, gridFixture{"duplicate vertices", degenerate})
	var scattered [][2]r3.Vec
	for range 5000 {
		lo := r3.NewVec(rng.Float64()*1e6, rng.Float64()*1e6, rng.Float64()*1e6)
		scattered = append(scattered, gridBox(lo, lo.Add(r3.NewVec(1, 1, 1))))
	}
	out = append(out, gridFixture{"scattered", scattered})
	// Unit boxes in a cube a thousand units wide: unit cells, so many more
	// (x, y) columns than registrations, and grid keys of several digits.
	var fine [][2]r3.Vec
	for range 3000 {
		lo := r3.NewVec(rng.Float64()*1000, rng.Float64()*1000, rng.Float64()*1000)
		fine = append(fine, gridBox(lo, lo.Add(r3.NewVec(1, 1, 1))))
	}
	out = append(out, gridFixture{"scattered fine", fine})
	return out
}

// TestScanBuildsMatchReference compares the grid, tree and chosen scan, and
// the walks they yield, with the reference builds, over member sets in
// ascending, shuffled and reversed-subset order and under several budgets
// and beats.
func TestScanBuildsMatchReference(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(0x7e57, 0x5eed))
	for fi, fx := range scanBuildFixtures() {
		ident := allMembers(len(fx.boxes))
		shuffled := slices.Clone(ident)
		rng.Shuffle(len(shuffled), func(a, b int) { shuffled[a], shuffled[b] = shuffled[b], shuffled[a] })
		var subset []int
		for i := len(fx.boxes) - 1; i >= 0; i -= 2 {
			subset = append(subset, i)
		}
		for name, members := range map[string][]int{"ascending": ident, "shuffled": shuffled, "subset": subset} {
			label := fmt.Sprintf("%s#%d/%s", fx.name, fi, name)

			want, wantOK := gridScanBelowReference(fx.boxes, members, math.MaxUint64, math.MaxUint64)
			got, gotOK := newGridScan(fx.boxes, members, math.MaxUint64)
			require.Equal(t, wantOK, gotOK, "%s: grid ok", label)
			if wantOK {
				requireSameGrid(t, want, got, label)
				requireSameWalk(t, walkScan(t, want, math.MaxUint64), walkScan(t, got, math.MaxUint64), label+" grid walk")
				// The budget and beat stops, at and around the exact counts.
				for _, cut := range [][2]uint64{
					{want.registrations, math.MaxUint64}, {want.registrations - 1, math.MaxUint64},
					{math.MaxUint64, want.work}, {math.MaxUint64, want.work + 1}, {math.MaxUint64, want.work / 2},
				} {
					_, wantOK := gridScanBelowReference(fx.boxes, members, cut[0], cut[1])
					got, gotOK := newGridScanBelow(fx.boxes, members, cut[0], cut[1])
					require.Equal(t, wantOK, gotOK, "%s: grid ok at budget %d beat %d", label, cut[0], cut[1])
					if wantOK {
						requireSameGrid(t, want, got, label)
					}
				}
			}

			requireSameTree(t, newSortTreeScan(fx.boxes, members), newTreeScan(fx.boxes, members), label)

			for _, limit := range []uint64{math.MaxUint64, 1000, 10} {
				wantScan := pairScanBuildReference(fx.boxes, members, limit)
				gotScan := newPairScan(fx.boxes, members, limit)
				require.Equal(t, fmt.Sprintf("%T", wantScan), fmt.Sprintf("%T", gotScan), "%s: chosen scan at %d", label, limit)
				for _, walkLimit := range []uint64{math.MaxUint64, 100} {
					requireSameWalk(t, walkScan(t, wantScan, walkLimit), walkScan(t, gotScan, walkLimit),
						fmt.Sprintf("%s: chosen walk at %d/%d", label, limit, walkLimit))
				}
			}
		}
	}
}

// TestSortGridItemsMatchesComparisonSort pins sortGridItems against a
// comparison sort on (start, pos) for item sets with many shared starts,
// starts that need up to five radix digits, and sizes around gridRadixMin.
func TestSortGridItemsMatchesComparisonSort(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(0x50f7, 0x1735))
	for _, n := range []int{0, 1, 2, gridRadixMin - 1, gridRadixMin, gridRadixMin + 1, 5000} {
		for _, maxStart := range []int64{0, 1, 7, 1<<gridRadixBits - 1, 1 << gridRadixBits, 1 << 40, 1<<62 - 1} {
			// Items come in ascending pos, as newGridScanBelow appends them.
			items := make([]gridItem, n)
			for k := range items {
				start := rng.Int64N(maxStart + 1)
				if k > 0 && rng.IntN(3) == 0 {
					start = items[rng.IntN(k)].start // a shared start
				}
				items[k] = gridItem{start: start, pos: int32(k), zHi: int32(rng.IntN(100))}
			}
			want := slices.Clone(items)
			slices.SortFunc(want, func(p, q gridItem) int {
				if p.start != q.start {
					return cmp.Compare(p.start, q.start)
				}
				return cmp.Compare(p.pos, q.pos)
			})
			got := sortGridItems(slices.Clone(items), maxStart)
			label := fmt.Sprintf("n=%d max=%d", n, maxStart)
			require.Len(t, got, len(want), label)
			for k := range want {
				if got[k] != want[k] {
					require.Failf(t, "sort", "%s: item %d is %+v, want %+v", label, k, got[k], want[k])
				}
			}
		}
	}
}
