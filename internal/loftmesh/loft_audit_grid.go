package loftmesh

import (
	"cmp"
	"math"
	"slices"
	"sort"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// This file is the crossing audit's second way to enumerate box-overlapping
// pairs (agent-docs/loft-design.md §6): a uniform grid over the member boxes.
// newPairScan counts, before any runs, the sweep, grid and tree work and
// hands the audit the cheapest one. All report exactly the box-overlapping
// pairs, each once, so the candidate set, its
// lexicographic test order and every verdict are the same whichever runs.

// pairScan enumerates every pair of member triangles whose boxes overlap on
// all three axes, as (i, j) with i < j, each exactly once and in no
// particular order. visit counts its own work: every unit steps the budget
// once, and visit reports true as soon as the count passes limit.
type pairScan interface {
	visit(budget *proofbound.WorkBudget, limit uint64, fn func(i, j int)) (uint64, bool, error)
}

// newPairScan returns the lowest-work scan. The tree checks descendant boxes
// before leaf pairs; the grid registers boxes in cells; the sweep compares
// boxes overlapping on one axis. A tie keeps the earlier scan.
func newPairScan(boxes [][2]r3.Vec, members []int, limit uint64) pairScan {
	sweep := newSweepOrder(boxes, members)
	best, bestWork := pairScan(sweep), sweep.work()
	// A nonempty tree visit costs at least one unit. Avoid sorting all boxes
	// when a sparse sweep already costs zero or one comparison.
	if bestWork > 1 {
		tree := newTreeScan(boxes, members)
		if treeWork, ok := tree.workBound(min(bestWork, limit)); ok && treeWork < bestWork {
			best, bestWork = tree, treeWork
		}
	}
	grid, ok := newGridScanBelow(boxes, members, min(bestWork, limit), bestWork)
	if ok && grid.work < bestWork {
		best = grid
	}
	return best
}

// work is the number of pairs visit compares when it runs to completion: for
// each member in sweep order, the later members whose lower bound on the
// sweep axis lies at or below its upper bound there.
func (s sweepOrder) work() uint64 {
	los := make([]float64, len(s.order))
	for k, i := range s.order {
		los[k], _ = boxAxis(s.boxes[i], s.axis)
	}
	total := uint64(0)
	for k, i := range s.order {
		_, hi := boxAxis(s.boxes[i], s.axis)
		// The first position past k whose lower bound exceeds hi; los is
		// sorted, so every position between compares with k.
		end := k + 1 + sort.Search(len(los)-k-1, func(q int) bool { return los[k+1+q] > hi })
		total += uint64(end - k - 1)
	}
	return total
}

// gridScan is a uniform grid over the member boxes. A box meets the block of
// cells between the cells of its two corners; the grid records it once per
// (x, y) column of that block, with the block's z cell range, so two boxes in
// one column share exactly the cells where their z ranges meet. A pair whose
// boxes overlap meets in the cell holding the componentwise maximum of their
// two lower corners, a point of both boxes, and is reported from that cell's
// column alone.
type gridScan struct {
	boxes [][2]r3.Vec
	lo    [3]float64
	size  [3]float64
	n     [3]int
	// columns holds every occupied (x, y) column in ascending column key;
	// a column's entries run in ascending zLo, ties in member order.
	columns []gridColumn
	// registrations counts every (box, cell) incidence; work adds, for every
	// cell, one comparison per pair of boxes sharing it, so work is what a
	// complete visit counts.
	registrations uint64
	work          uint64
}

// gridColumn is one occupied (x, y) column: key is x·n[1] + y.
type gridColumn struct {
	key     int64
	entries []gridEntry
}

// gridEntry is one box's z cell range [zLo, zHi] in one column.
type gridEntry struct {
	box      int32
	zLo, zHi int32
}

func vecAxis(v r3.Vec, axis int) float64 {
	switch axis {
	case 0:
		return v.X
	case 1:
		return v.Y
	default:
		return v.Z
	}
}

// newGridScan sizes the grid and records every member. Each axis's cell is
// the members' median extent on it when that exceeds the members' span over
// F, else the span over ⌈∛F⌉, so no axis has more than F + 1 cells. ok is
// false when the members span no finite extent, or when they meet more than
// budget cells; the caller then keeps the sweep.
func newGridScan(boxes [][2]r3.Vec, members []int, budget uint64) (gridScan, bool) {
	return newGridScanBelow(boxes, members, budget, math.MaxUint64)
}

// newGridScanBelow is newGridScan for a caller that keeps the grid only when
// its work is below beat. ok is also false as soon as the work counted so far
// reaches beat: the count only grows, so the finished grid could not be kept.
// A grid returned with ok false is partly built and must be dropped.
func newGridScanBelow(boxes [][2]r3.Vec, members []int, budget, beat uint64) (gridScan, bool) {
	g := gridScan{boxes: boxes}
	if len(members) < 2 {
		return g, false
	}
	count := float64(len(members))
	side := math.Ceil(math.Cbrt(count))
	extents := make([]float64, len(members))
	for axis := range 3 {
		lo, hi := math.Inf(1), math.Inf(-1)
		for k, i := range members {
			a, b := vecAxis(boxes[i][0], axis), vecAxis(boxes[i][1], axis)
			lo, hi = math.Min(lo, a), math.Max(hi, b)
			extents[k] = b - a
		}
		span := hi - lo
		if proofbound.IsNonFinite(span) {
			return g, false
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
	// Count the incidences before registering any, so a grid that would cost
	// more than budget is never built.
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
	// One item per (box, column) incidence: at most total, already within
	// budget. start is the key of the box's lowest cell in the column, so
	// sorting by (start, member position) orders the columns by key and each
	// column's boxes by zLo, ties in member order.
	type gridItem struct {
		start int64
		pos   int32
		zHi   int32
	}
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

// index is the cell along axis holding x, clamped into the grid. It is
// non-decreasing in x, which is what places a point of a box inside that
// box's own cell range.
func (g gridScan) index(axis int, x float64) int {
	k := int(math.Floor((x - g.lo[axis]) / g.size[axis]))
	return max(0, min(k, g.n[axis]-1))
}

func (g gridScan) cellRange(b [2]r3.Vec) ([3]int, [3]int) {
	var lo, hi [3]int
	for axis := range 3 {
		lo[axis] = g.index(axis, vecAxis(b[0], axis))
		hi[axis] = g.index(axis, vecAxis(b[1], axis))
	}
	return lo, hi
}

func (g gridScan) key(c [3]int) int64 {
	return (int64(c[0])*int64(g.n[1])+int64(c[1]))*int64(g.n[2]) + int64(c[2])
}

// visit charges the registrations, then, column by column, every pair of
// boxes whose z ranges meet: one unit per cell the pair shares, the same
// comparisons a cell-by-cell pass makes. An overlapping pair is reported
// only from the column holding the maximum of its two lower corners; that
// point's z cell lies in both boxes' z ranges, so the pair is reported once.
func (g gridScan) visit(budget *proofbound.WorkBudget, limit uint64, fn func(i, j int)) (uint64, bool, error) {
	scanned := g.registrations
	if scanned > limit {
		return scanned, true, nil
	}
	ny := int64(g.n[1])
	for _, col := range g.columns {
		x, y := int(col.key/ny), int(col.key%ny)
		for a, ea := range col.entries {
			boxA := g.boxes[ea.box]
			for _, eb := range col.entries[a+1:] {
				if eb.zLo > ea.zHi {
					break
				}
				zHi := min(ea.zHi, eb.zHi)
				for range zHi - eb.zLo + 1 {
					if err := budget.Step(); err != nil {
						return scanned, false, err
					}
					scanned++
					if scanned > limit {
						return scanned, true, nil
					}
				}
				boxB := g.boxes[eb.box]
				if !meshbool.BoxesOverlap(boxA, boxB) {
					continue
				}
				var c [3]int
				for axis := range 3 {
					c[axis] = g.index(axis, math.Max(vecAxis(boxA[0], axis), vecAxis(boxB[0], axis)))
				}
				if c[0] != x || c[1] != y {
					continue
				}
				i, j := int(ea.box), int(eb.box)
				if i > j {
					i, j = j, i
				}
				fn(i, j)
			}
		}
	}
	return scanned, false, nil
}
