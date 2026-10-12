package diameter

import (
	"math"
	"sort"

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
func spatialPointBlocks(points []r3.Vec) ([]spatialPoint, []spatialBlock) {
	ordered := make([]spatialPoint, len(points))
	for i, point := range points {
		ordered[i] = spatialPoint{value: point, index: i}
	}
	blocks := make([]spatialBlock, 0, (len(points)+pointBlockSize-1)/pointBlockSize)
	var partition func(int, int)
	partition = func(begin, end int) {
		lo, hi := ordered[begin].value, ordered[begin].value
		for _, entry := range ordered[begin+1 : end] {
			p := entry.value
			lo.X, lo.Y, lo.Z = math.Min(lo.X, p.X), math.Min(lo.Y, p.Y), math.Min(lo.Z, p.Z)
			hi.X, hi.Y, hi.Z = math.Max(hi.X, p.X), math.Max(hi.Y, p.Y), math.Max(hi.Z, p.Z)
		}
		if end-begin <= pointBlockSize {
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
		segment := ordered[begin:end]
		sort.Slice(segment, func(i, j int) bool {
			a, b := coordinate(segment[i].value, axis), coordinate(segment[j].value, axis)
			if a == b {
				return segment[i].index < segment[j].index
			}
			return a < b
		})
		mid := begin + (end-begin)/2
		partition(begin, mid)
		partition(mid, end)
	}
	partition(0, len(ordered))
	return ordered, blocks
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
