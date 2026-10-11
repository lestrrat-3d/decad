package loftmesh

import (
	"math"
	"slices"
	"sort"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// This file is the crossing audit's second way to enumerate box-overlapping
// pairs (docs/loft-design.md §6): a uniform grid over the member boxes.
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
	tree := newTreeScan(boxes, members)
	if treeWork, ok := tree.workBound(min(bestWork, limit)); ok && treeWork < bestWork {
		best, bestWork = tree, treeWork
	}
	grid, ok := newGridScan(boxes, members, min(bestWork, limit))
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

// gridScan is a uniform grid over the member boxes. Every box is registered
// in each cell its extent meets; a pair whose boxes overlap meets in the
// cell holding the componentwise maximum of their two lower corners, a point
// of both boxes, and is reported from that cell alone.
type gridScan struct {
	boxes [][2]r3.Vec
	lo    [3]float64
	size  [3]float64
	n     [3]int
	keys  []int64
	cells [][]int32
	// registrations counts every (box, cell) incidence; work adds every
	// in-cell pair comparison, so work is what a complete visit counts.
	registrations uint64
	work          uint64
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

// newGridScan sizes the grid and registers every member. Each axis's cell
// is the larger of the members' median extent on it and the members' span
// over ⌈∛F⌉, so no axis has more than ⌈∛F⌉ + 1 cells. ok is false when the
// members span no finite extent, or when registering them would pass budget
// incidences; the caller then keeps the sweep.
func newGridScan(boxes [][2]r3.Vec, members []int, budget uint64) (gridScan, bool) {
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
	total := uint64(0)
	for _, i := range members {
		lo, hi := g.cellRange(boxes[i])
		count := uint64(1)
		for axis := range 3 {
			count *= uint64(hi[axis] - lo[axis] + 1)
		}
		total += count
		if total > budget {
			return g, false
		}
	}
	g.registrations = total
	byKey := map[int64][]int32{}
	for _, i := range members {
		lo, hi := g.cellRange(boxes[i])
		for x := lo[0]; x <= hi[0]; x++ {
			for y := lo[1]; y <= hi[1]; y++ {
				for z := lo[2]; z <= hi[2]; z++ {
					key := g.key([3]int{x, y, z})
					byKey[key] = append(byKey[key], int32(i))
				}
			}
		}
	}
	g.keys = make([]int64, 0, len(byKey))
	for key := range byKey {
		g.keys = append(g.keys, key)
	}
	slices.Sort(g.keys)
	g.cells = make([][]int32, len(g.keys))
	g.work = g.registrations
	for k, key := range g.keys {
		g.cells[k] = byKey[key]
		m := uint64(len(g.cells[k]))
		g.work += m * (m - 1) / 2
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

// visit charges the registrations, then compares every pair sharing a cell,
// in key order, reporting an overlapping pair only from the cell that holds
// the maximum of its two lower corners.
func (g gridScan) visit(budget *proofbound.WorkBudget, limit uint64, fn func(i, j int)) (uint64, bool, error) {
	scanned := g.registrations
	if scanned > limit {
		return scanned, true, nil
	}
	for k, cell := range g.cells {
		key := g.keys[k]
		for a, ia := range cell {
			boxA := g.boxes[ia]
			for _, ib := range cell[a+1:] {
				if err := budget.Step(); err != nil {
					return scanned, false, err
				}
				scanned++
				if scanned > limit {
					return scanned, true, nil
				}
				boxB := g.boxes[ib]
				if !meshbool.BoxesOverlap(boxA, boxB) {
					continue
				}
				var c [3]int
				for axis := range 3 {
					c[axis] = g.index(axis, math.Max(vecAxis(boxA[0], axis), vecAxis(boxB[0], axis)))
				}
				if g.key(c) != key {
					continue
				}
				i, j := int(ia), int(ib)
				if i > j {
					i, j = j, i
				}
				fn(i, j)
			}
		}
	}
	return scanned, false, nil
}
