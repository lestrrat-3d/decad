package stackedrecord

import (
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
)

// levelOrder compares two held levels by their exact rationals. Distinct
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

// crossingLevel is one end level of a crossing blind cut operand, with the
// exact rational it denotes on the target's axis, or nil where no record
// proves one (agent-docs/stacked-prism-design.md §2.1).
type crossingLevel struct {
	UnionLevel
	denoted *big.Rat
}

// crossingOrder is levelOrder, except that two levels whose recorded denoted
// levels are the same rational are one level, whatever their displacements.
// Equal held floats never stand in for that: each may round a different
// rational. Distinct or unrecorded denotations fall to levelOrder's margin,
// which a denotation never contradicts, since each lies within its own
// level's displacement.
func crossingOrder(a, b crossingLevel) (int, bool) {
	if a.denoted != nil && b.denoted != nil && a.denoted.Cmp(b.denoted) == 0 {
		return 0, true
	}
	return levelOrder(a.UnionLevel, b.UnionLevel)
}

// shiftedDenoted lifts a tool level's denotation onto the target's axis, or
// returns nil when the tool records none.
func shiftedDenoted(level, shift *big.Rat) *big.Rat {
	if level == nil {
		return nil
	}
	return level.Add(level, shift)
}

// CrossingBlindLevels reads the levels of a one-slab target and one-slab tool
// whose blind cut crosses the target's through hole. The tool must end at one
// target cap, or pass it, and start strictly inside the target at the other
// end, or open at or past that cap: the tool is clipped to the target's
// interval. Every comparison goes through crossingOrder, so a level
// displacement never admits an order it does not prove, and a tool end meets
// a target cap when both levels are undisplaced and equal or both record one
// denoted rational. It returns the three result levels (target start,
// interior level, target end) and, per result slab, the target slab and the
// tool slab reaching it (-1 for none).
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
	start := crossingLevel{a[0], target[0].DenotedZ0()}
	end := crossingLevel{a[1], target[0].DenotedZ1()}
	// pos is -2 below the target's start, -1 at it, 0 inside, 1 at its end
	// and 2 past it.
	pos := func(level crossingLevel) (int, bool) {
		lo, ok := crossingOrder(level, start)
		if !ok {
			return 0, false
		}
		if lo < 0 {
			return -2, true
		}
		if lo == 0 {
			return -1, true
		}
		hi, ok := crossingOrder(level, end)
		if !ok {
			return 0, false
		}
		return hi + 1, true
	}
	p0, ok := pos(crossingLevel{t[0], shiftedDenoted(tool[0].DenotedZ0(), shift)})
	if !ok {
		return nil, nil, false
	}
	p1, ok := pos(crossingLevel{t[1], shiftedDenoted(tool[0].DenotedZ1(), shift)})
	if !ok {
		return nil, nil, false
	}
	// The tool is clipped to the target, so a tool end at or past a cap
	// reaches the result slab next to that cap whatever its held float.
	var levels []UnionLevel
	var reach [][2]int
	switch {
	case p0 == 0 && p1 >= 1:
		levels = []UnionLevel{a[0], t[0], a[1]}
		reach = [][2]int{{0, -1}, {0, 0}}
	case p0 <= -1 && p1 == 0:
		levels = []UnionLevel{a[0], t[1], a[1]}
		reach = [][2]int{{0, 0}, {0, -1}}
	default:
		return nil, nil, false
	}
	for i := 1; i < len(levels); i++ {
		if levels[i-1].Held >= levels[i].Held {
			return nil, nil, false
		}
	}
	return levels, reach, true
}
