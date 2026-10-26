package stackedrecord

import (
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
)

// levelOrder compares two denoted levels by their exact rationals. Distinct
// levels order only when their gap exceeds both held displacements together,
// and equal levels count as one only when both are undisplaced; a gap inside
// the displacements, or equal levels that carry one, order nothing.
func levelOrder(a, b UnionLevel) (int, bool) {
	cmp := a.Exact.Cmp(b.Exact)
	if cmp == 0 {
		return 0, a.Delta == 0 && b.Delta == 0
	}
	margin := proofarith.FloatRat(a.Delta)
	slack := proofarith.FloatRat(b.Delta)
	if margin == nil || slack == nil {
		return 0, false
	}
	margin.Add(margin, slack)
	gap := new(big.Rat).Sub(a.Exact, b.Exact)
	gap.Abs(gap)
	return cmp, gap.Cmp(margin) > 0
}

// CrossingBlindLevels reads the levels of a one-slab target and one-slab tool
// whose blind cut crosses the target's through hole. The tool must end at one
// target cap, or pass it, and start strictly inside the target at the other
// end, or open at or past that cap: the tool is clipped to the target's
// interval. Every comparison goes through levelOrder, so a level displacement
// never admits an order it does not prove. It returns the three result levels
// (target start, interior level, target end) and, per result slab, the target
// slab and the tool slab reaching it (-1 for none).
func CrossingBlindLevels(target, tool []Slab, shift *big.Rat) ([]UnionLevel, [][2]int, bool) {
	if len(target) != 1 || len(tool) != 1 {
		return nil, nil, false
	}
	a, ok := UnionLevels(target, nil, shift)
	if !ok || len(a) != 2 {
		return nil, nil, false
	}
	t, ok := UnionLevels(nil, tool, shift)
	if !ok || len(t) != 2 {
		return nil, nil, false
	}
	// pos is -2 below the target's start, -1 at it, 0 inside, 1 at its end
	// and 2 past it.
	pos := func(level UnionLevel) (int, bool) {
		lo, ok := levelOrder(level, a[0])
		if !ok {
			return 0, false
		}
		if lo < 0 {
			return -2, true
		}
		if lo == 0 {
			return -1, true
		}
		hi, ok := levelOrder(level, a[1])
		if !ok {
			return 0, false
		}
		return hi + 1, true
	}
	p0, ok := pos(t[0])
	if !ok {
		return nil, nil, false
	}
	p1, ok := pos(t[1])
	if !ok {
		return nil, nil, false
	}
	zero := new(big.Rat)
	var levels []UnionLevel
	switch {
	case p0 == 0 && p1 >= 1:
		levels = []UnionLevel{a[0], t[0], a[1]}
	case p0 <= -1 && p1 == 0:
		levels = []UnionLevel{a[0], t[1], a[1]}
	default:
		return nil, nil, false
	}
	for i := 1; i < len(levels); i++ {
		if levels[i-1].Held >= levels[i].Held {
			return nil, nil, false
		}
	}
	reach := make([][2]int, 2)
	for k := range reach {
		reach[k][0] = UnionSlabOf(target, zero, levels[k].Exact, levels[k+1].Exact)
		reach[k][1] = UnionSlabOf(tool, shift, levels[k].Exact, levels[k+1].Exact)
	}
	return levels, reach, true
}
