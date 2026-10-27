package diameter

import (
	"cmp"
	"math"
	"math/bits"
	"slices"

	"github.com/lestrrat-3d/r3"
)

type spatialPoint struct {
	value r3.Vec
	index int
}

type spatialBlock struct {
	box        pointBlock
	begin, end int
}

// spatialPointBlocks groups nearby witnesses so one box bound can reject all
// pairs in two leaves. The original indices stay with each point for tie order.
//
// Each node splits its segment at the middle of its widest axis's order, and
// each leaf lists its points in its parent's axis order. That order,
// spatialCompare, breaks coordinate ties by the distinct original index, so
// it is a total order and admits exactly one sorted arrangement. A node
// therefore needs only to SELECT its middle: which points fall in each half
// is fixed by the order, not by how the node arranged them, and every child
// arranges its own half from scratch. Sorting each leaf on its parent's axis
// then restores the one arrangement a full sort at every node produced, so
// ordered and blocks are unchanged and the pair scan's polls and result are
// those of the full sort.
func spatialPointBlocks(points []r3.Vec) ([]spatialPoint, []spatialBlock) {
	ordered := make([]spatialPoint, len(points))
	for i, point := range points {
		ordered[i] = spatialPoint{value: point, index: i}
	}
	blocks := make([]spatialBlock, 0, (len(points)+pointBlockSize-1)/pointBlockSize)
	var partition func(int, int, int)
	// parentAxis is the axis the enclosing node split on, or -1 at the root.
	partition = func(begin, end, parentAxis int) {
		lo, hi := ordered[begin].value, ordered[begin].value
		for _, entry := range ordered[begin+1 : end] {
			p := entry.value
			lo.X, lo.Y, lo.Z = math.Min(lo.X, p.X), math.Min(lo.Y, p.Y), math.Min(lo.Z, p.Z)
			hi.X, hi.Y, hi.Z = math.Max(hi.X, p.X), math.Max(hi.Y, p.Y), math.Max(hi.Z, p.Z)
		}
		if end-begin <= pointBlockSize {
			if parentAxis >= 0 {
				sortSpatial(ordered[begin:end], parentAxis)
			}
			blocks = append(blocks, spatialBlock{box: pointBlock{min: lo, max: hi}, begin: begin, end: end})
			return
		}
		axis := 0
		span := hi.X - lo.X
		if hi.Y-lo.Y > span {
			axis, span = 1, hi.Y-lo.Y
		}
		if hi.Z-lo.Z > span {
			axis = 2
		}
		mid := begin + (end-begin)/2
		selectSpatial(ordered[begin:end], mid-begin, axis)
		partition(begin, mid, axis)
		partition(mid, end, axis)
	}
	partition(0, len(ordered), -1)
	return ordered, blocks
}

// spatialCompare orders points by their axis coordinate, then by original
// index. The scan reaches it only with finite coordinates, so two
// coordinates neither below nor above each other are equal (signed zeros
// included), and the distinct indices make the order total.
func spatialCompare(a, b spatialPoint, axis int) int {
	ca, cb := coordinate(a.value, axis), coordinate(b.value, axis)
	if ca < cb {
		return -1
	}
	if ca > cb {
		return 1
	}
	return cmp.Compare(a.index, b.index)
}

func sortSpatial(s []spatialPoint, axis int) {
	slices.SortFunc(s, func(a, b spatialPoint) int { return spatialCompare(a, b, axis) })
}

// selectSpatial rearranges s so that s[k] is the point a full sort under
// spatialCompare places there, every point before it orders below it and
// every point after it above it.
func selectSpatial(s []spatialPoint, k, axis int) {
	selectSpatialRounds(s, k, axis, 2*bits.Len(uint(len(s))))
}

// selectSpatialRounds is selectSpatial with at most limit partition rounds;
// a range still open after them is sorted whole, which bounds the work on
// inputs whose pivots split badly.
func selectSpatialRounds(s []spatialPoint, k, axis, limit int) {
	lo, hi := 0, len(s)-1
	for rounds := 0; lo < hi; rounds++ {
		if rounds == limit {
			sortSpatial(s[lo:hi+1], axis)
			return
		}
		// Median of three moved to hi, then a Lomuto partition around it.
		mid := lo + (hi-lo)/2
		if spatialCompare(s[mid], s[lo], axis) < 0 {
			s[mid], s[lo] = s[lo], s[mid]
		}
		if spatialCompare(s[hi], s[lo], axis) < 0 {
			s[hi], s[lo] = s[lo], s[hi]
		}
		if spatialCompare(s[mid], s[hi], axis) < 0 {
			s[mid], s[hi] = s[hi], s[mid]
		}
		pivot := s[hi]
		store := lo
		for j := lo; j < hi; j++ {
			if spatialCompare(s[j], pivot, axis) < 0 {
				s[store], s[j] = s[j], s[store]
				store++
			}
		}
		s[store], s[hi] = s[hi], s[store]
		switch {
		case k == store:
			return
		case k < store:
			hi = store - 1
		default:
			lo = store + 1
		}
	}
}

func coordinate(p r3.Vec, axis int) float64 {
	switch axis {
	case 0:
		return p.X
	case 1:
		return p.Y
	default:
		return p.Z
	}
}

func blockPairDistanceUpperSq(a, b pointBlock) float64 {
	dx := math.Max(math.Abs(a.min.X-b.max.X), math.Abs(a.max.X-b.min.X))
	dy := math.Max(math.Abs(a.min.Y-b.max.Y), math.Abs(a.max.Y-b.min.Y))
	dz := math.Max(math.Abs(a.min.Z-b.max.Z), math.Abs(a.max.Z-b.min.Z))
	return dx*dx + dy*dy + dz*dz
}

// extremePair supplies a real witness distance before the block scan. It
// changes only pruning: the winning pair still gets the exact-rational read.
func extremePair(points []r3.Vec) pairMaximum {
	extremes := [6]int{}
	for i, p := range points {
		for axis := range 3 {
			if coordinate(p, axis) < coordinate(points[extremes[axis*2]], axis) {
				extremes[axis*2] = i
			}
			if coordinate(p, axis) > coordinate(points[extremes[axis*2+1]], axis) {
				extremes[axis*2+1] = i
			}
		}
	}
	best := pairMaximum{}
	for a, i := range extremes {
		for _, j := range extremes[a+1:] {
			if i == j {
				continue
			}
			first, second := min(i, j), max(i, j)
			candidate := pairMaximum{
				distance: points[first].Sub(points[second]).Len(), i: first, j: second,
			}
			if betterPair(candidate, best) {
				best = candidate
			}
		}
	}
	return best
}

func betterPair(candidate, best pairMaximum) bool {
	return candidate.distance > best.distance ||
		candidate.distance == best.distance &&
			(candidate.i < best.i || candidate.i == best.i && candidate.j < best.j)
}
