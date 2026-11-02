package prismcells

import (
	"context"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/sketchrecord"
	"github.com/lestrrat-3d/sketch"
)

// ReplayWholeArcCharge bounds reconstruction of one whole arc. Its radius
// difference uses |h²-r²|/h; the natural end's radial residual uses
// |r1²-r0²|/max(|dx0|,|dy0|). Both replace a positive denominator by a
// proven lower bound and are exact rational calculations until rounded upward.
func ReplayWholeArcCharge(ctx context.Context, arc sectionrecord.ArcSeg, budget proof.ReplayKeyBudget) (float64, bool, error) {
	if budget == nil {
		return 0, false, nil
	}
	if err := budget.SpendWork(ctx, 256); err != nil {
		return 0, false, err
	}
	if !WholeSegmentRange(arc.TStart, arc.TEnd) || arc.TStart == arc.TEnd {
		return 0, false, nil
	}
	h := math.Hypot(arc.Start.U-arc.Center.U, arc.Start.V-arc.Center.V)
	if math.IsNaN(h) || math.IsInf(h, 0) || h <= 0 {
		return 0, false, nil
	}
	c := clearanceReplay{ctx, budget}
	center, err := c.point(arc.Center)
	if err != nil {
		return 0, false, err
	}
	start, err := c.point(arc.Start)
	if err != nil {
		return 0, false, err
	}
	end, err := c.point(arc.End)
	if err != nil {
		return 0, false, err
	}
	squared := func(p [2]*big.Rat) (*big.Rat, [2]*big.Rat, error) {
		var d [2]*big.Rat
		var sum *big.Rat
		for i := range d {
			var err error
			d[i], err = c.op(p[i], center[i], '-')
			if err != nil {
				return nil, d, err
			}
			q, err := c.op(d[i], d[i], '*')
			if err != nil {
				return nil, d, err
			}
			if sum == nil {
				sum = q
			} else {
				sum, err = c.op(sum, q, '+')
				if err != nil {
					return nil, d, err
				}
			}
		}
		return sum, d, nil
	}
	s0, d, err := squared(start)
	if err != nil {
		return 0, false, err
	}
	s1, _, err := squared(end)
	if err != nil {
		return 0, false, err
	}
	hRat, err := c.lift(h)
	if err != nil {
		return 0, false, err
	}
	h2, err := c.op(hRat, hRat, '*')
	if err != nil {
		return 0, false, err
	}
	diff, err := c.op(s0, h2, '-')
	if err != nil {
		return 0, false, err
	}
	diff, err = c.absolute(diff)
	if err != nil {
		return 0, false, err
	}
	radiusError, err := c.op(diff, hRat, '/')
	if err != nil {
		return 0, false, err
	}
	dx, err := c.absolute(d[0])
	if err != nil {
		return 0, false, err
	}
	dy, err := c.absolute(d[1])
	if err != nil {
		return 0, false, err
	}
	cmp, err := c.cmp(dx, dy)
	if err != nil {
		return 0, false, err
	}
	if cmp < 0 {
		dx = dy
	}
	if dx.Sign() <= 0 {
		return 0, false, nil
	}
	diff, err = c.op(s1, s0, '-')
	if err != nil {
		return 0, false, err
	}
	diff, err = c.absolute(diff)
	if err != nil {
		return 0, false, err
	}
	residual, err := c.op(diff, dx, '/')
	if err != nil {
		return 0, false, err
	}
	total, err := c.op(radiusError, residual, '+')
	if err != nil {
		return 0, false, err
	}
	if total.Sign() == 0 {
		return 0, true, nil
	}
	value, err := proof.ReplayRatFloat64(ctx, total, budget)
	if err != nil {
		return 0, false, err
	}
	if math.IsNaN(value) || math.IsInf(value, 0) {
		return 0, false, nil
	}
	value = math.Nextafter(value, math.Inf(1))
	if math.IsInf(value, 0) {
		return 0, false, nil
	}
	return value, true, nil
}

// replayWholeArcIdentity binds every target whole arc in a matched scene back
// to its exact natural-order source record. It returns the largest separately
// charged reconstruction allowance. The caller must supply GroupCutMatch's
// whole-loop result and the tag map from the same scene.
func replayWholeArcIdentity(ctx context.Context, target momentinput.Profile, match *sketch.Profile,
	tags map[sketch.Entity]Origin, budget proof.ReplayKeyBudget) (float64, bool, error) {
	if budget == nil {
		return 0, false, nil
	}
	if err := budget.SpendWork(ctx, 1); err != nil {
		return 0, false, err
	}
	if match == nil {
		return 0, false, nil
	}
	expected := 0
	var maximum float64
	chargeLoop := func(loop sectionrecord.LoopRecord) (bool, error) {
		for _, segment := range loop.Segments {
			if err := budget.SpendWork(ctx, 1); err != nil {
				return false, err
			}
			arc, yes := segment.(sectionrecord.ArcSeg)
			if !yes {
				continue
			}
			charge, ok, err := ReplayWholeArcCharge(ctx, arc, budget)
			if err != nil || !ok {
				return false, err
			}
			maximum = math.Max(maximum, charge)
			expected++
		}
		return true, nil
	}
	ok, err := chargeLoop(target.Outer)
	if err != nil || !ok {
		return 0, false, err
	}
	for _, loop := range target.Holes {
		if err := budget.SpendWork(ctx, 1); err != nil {
			return 0, false, err
		}
		ok, err = chargeLoop(loop)
		if err != nil || !ok {
			return 0, false, err
		}
	}
	found := 0
	checkLoop := func(edges []sketch.BoundaryEdge) (bool, error) {
		for _, edge := range edges {
			if err := budget.SpendWork(ctx, 1); err != nil {
				return false, err
			}
			origin, yes := tags[edge.Entity]
			if !yes {
				return false, nil
			}
			if origin.IsB {
				continue
			}
			if origin.Region != 0 || origin.Hole < -1 || origin.Hole >= len(target.Holes) {
				return false, nil
			}
			loop := target.Outer
			if origin.Hole >= 0 {
				loop = target.Holes[origin.Hole]
			}
			if origin.Segment < 0 || origin.Segment >= len(loop.Segments) {
				return false, nil
			}
			want, yes := loop.Segments[origin.Segment].(sectionrecord.ArcSeg)
			if !yes {
				continue
			}
			if edge.Partial {
				return false, nil
			}
			if err := budget.SpendWork(ctx, 128); err != nil {
				return false, err
			}
			if err := budget.ReserveRecords(ctx, 2); err != nil {
				return false, err
			}
			if err := budget.ReserveBytes(ctx, 512); err != nil {
				return false, err
			}
			recorded, err := sketchrecord.RecordEdge(edge)
			if err != nil {
				return false, err
			}
			got, yes := recorded.(sectionrecord.ArcSeg)
			if !yes || got != want {
				return false, nil
			}
			found++
		}
		return true, nil
	}
	ok, err = checkLoop(match.Outer)
	if err != nil || !ok {
		return 0, false, err
	}
	for _, hole := range match.Holes {
		if err := budget.SpendWork(ctx, 1); err != nil {
			return 0, false, err
		}
		ok, err = checkLoop(hole)
		if err != nil || !ok {
			return 0, false, err
		}
	}
	return maximum, found == expected, nil
}
