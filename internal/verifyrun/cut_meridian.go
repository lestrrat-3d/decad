package verifyrun

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/boolchain"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// This file is the Cut-tool meridian separator
// (agent-docs/interference-design.md §3.2, "The Cut-tool meridian"). A Cut
// whose tool is a prism over one whole circle removes that tool's solid
// cylinder over its levels from its target. Whatever the target's shape,
// it lies between two levels along the tool's axis, read exactly from its
// reach, so the Cut result lies in the revolution about the tool's axis of
// the band between those levels less the tool's own rectangle. A screw whose
// shank sits in the pilot the tool bored and whose head rests on the
// target's face is then compared with that region by its own meridian.

// cutToolMeridianApart reports whether b misses Cut(target, tool) through
// the revolution region the Cut result lies in about the axis of the tool, or
// of any circle prism a Union tool joins: target less the Union lies in
// target less each of its operands.
func cutToolMeridianApart(ctx context.Context, budget *proofbound.WorkBudget, history boolchain.Source,
	target, tool, b *topo.Body) (bool, error) {
	for _, part := range unionParts(history, tool, 0) {
		if apart, err := cutCircleMeridianApart(ctx, budget, history, target, part, b); err != nil || apart {
			return apart, err
		}
	}
	return false, nil
}

// unionParts lists body, or the operands of the Unions it was built from.
func unionParts(history boolchain.Source, body *topo.Body, depth int) []*topo.Body {
	body = sameSetOf(history, body)
	if depth > coaxialMaxDepth {
		return nil
	}
	op, inputs, ok := expandRecorded(history, body)
	if !ok || op != meshbool.OpUnion {
		return []*topo.Body{body}
	}
	var out []*topo.Body
	for _, part := range inputs {
		out = append(out, unionParts(history, part, depth+1)...)
	}
	return out
}

// cutCircleMeridianApart is cutToolMeridianApart for one circle prism tool.
func cutCircleMeridianApart(ctx context.Context, budget *proofbound.WorkBudget, history boolchain.Source,
	target, tool, b *topo.Body) (bool, error) {
	return cutCircleMeridianApartRegions(ctx, budget, history, target, tool, meridiansOf(b))
}

// cutCircleMeridianApartRegions is cutCircleMeridianApart for a body known to
// lie in the revolution of any one of bms, each compared in turn.
func cutCircleMeridianApartRegions(ctx context.Context, budget *proofbound.WorkBudget, history boolchain.Source,
	target, tool *topo.Body, bms []meridian) (bool, error) {
	pt, ok := tool.Payload.(prismrecord.Payload)
	if !ok || len(pt.Profile.Holes) != 0 {
		return false, nil
	}
	if _, ok := wholeCircle(pt.Profile.Outer); !ok {
		return false, nil
	}
	tms := prismMeridians(pt)
	if len(tms) != 1 {
		return false, nil
	}
	tm := tms[0]
	r := tm.region[1][0]
	t0, t1 := tm.region[0][1], tm.region[2][1]
	lo, hi, ok := axialRange(history, target, pt, 0)
	if !ok {
		return false, nil
	}
	if t1.Cmp(lo) <= 0 || t0.Cmp(hi) >= 0 {
		// The tool misses the target's levels; the band alone separates
		// nothing more than the target does.
		return false, nil
	}
	a, c := lo, hi
	if t0.Cmp(a) > 0 {
		a = t0
	}
	if t1.Cmp(c) < 0 {
		c = t1
	}
	zero := new(big.Rat)
	// The band [lo, hi] less the tool rectangle [0, r] × [a, c]; a nil ρ is a
	// radius past both regions, as for a hole's strip.
	var region [][2]*big.Rat
	switch {
	case a.Cmp(lo) == 0 && c.Cmp(hi) == 0:
		region = [][2]*big.Rat{{r, lo}, {nil, lo}, {nil, hi}, {r, hi}}
	case a.Cmp(lo) == 0:
		region = [][2]*big.Rat{{r, lo}, {nil, lo}, {nil, hi}, {zero, hi}, {zero, c}, {r, c}}
	case c.Cmp(hi) == 0:
		region = [][2]*big.Rat{{zero, lo}, {nil, lo}, {nil, hi}, {r, hi}, {r, a}, {zero, a}}
	default:
		region = [][2]*big.Rat{{zero, lo}, {nil, lo}, {nil, hi}, {zero, hi}, {zero, c}, {r, c}, {r, a}, {zero, a}}
	}
	cut := tm
	cut.region = region
	for _, mb := range bms {
		if err := ctx.Err(); err != nil {
			return false, err
		}
		pa, pb, ok := sharedMeridians(cut, mb)
		if !ok {
			continue
		}
		apart, err := prismcells.SectionsApart(ctx, budget, []momentinput.Profile{pa}, []momentinput.Profile{pb})
		if err != nil || apart {
			return apart, err
		}
	}
	return false, nil
}

// axialRange bounds body's levels along the axis of prism pp: with pp's lift
// t + L·(x, y, z), every point x of body has g·(x − t) in [lo, hi], Lᵀg = e₃,
// each end read from body's reach, exactly where it is rational and from
// above where it holds a square root. A Cut lies where its target lies, an
// Intersect where any operand lies, a Union where every operand lies.
func axialRange(history boolchain.Source, body *topo.Body, pp prismrecord.Payload, depth int) (*big.Rat, *big.Rat, bool) {
	if depth > coaxialMaxDepth {
		return nil, nil, false
	}
	body = sameSetOf(history, body)
	l, t := prismLift(pp)
	var cols [3][3]*big.Rat
	for i := range 3 {
		cols[i] = [3]*big.Rat{l[0][i], l[1][i], l[2][i]}
	}
	zero, one := new(big.Rat), big.NewRat(1, 1)
	g, ok := solve(cols, [3]*big.Rat{zero, zero, one})
	if !ok {
		return nil, nil, false
	}
	base := dotVec(g, t)
	if up, ok := reachUpper(body, g); ok {
		down, ok := reachUpper(body, scaleVec(g, big.NewRat(-1, 1)))
		if !ok {
			return nil, nil, false
		}
		return new(big.Rat).Neg(new(big.Rat).Add(down, base)), new(big.Rat).Sub(up, base), true
	}
	op, inputs, ok := expandRecorded(history, body)
	if !ok {
		return nil, nil, false
	}
	switch op {
	case meshbool.OpCut:
		return axialRange(history, inputs[0], pp, depth+1)
	case meshbool.OpIntersect:
		for _, part := range inputs {
			if lo, hi, ok := axialRange(history, part, pp, depth+1); ok {
				return lo, hi, true
			}
		}
	case meshbool.OpUnion:
		var lo, hi *big.Rat
		for _, part := range inputs {
			l, h, ok := axialRange(history, part, pp, depth+1)
			if !ok {
				return nil, nil, false
			}
			if lo == nil || l.Cmp(lo) < 0 {
				lo = l
			}
			if hi == nil || h.Cmp(hi) > 0 {
				hi = h
			}
		}
		return lo, hi, lo != nil
	}
	return nil, nil, false
}
