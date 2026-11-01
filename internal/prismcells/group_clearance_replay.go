package prismcells

import (
	"context"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismplacement"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

type replayBox struct{ lo, hi [2]*big.Rat }
type clearanceReplay struct {
	ctx    context.Context //nolint:containedctx // The context is confined to one synchronous replay or test.
	budget proof.ReplayKeyBudget
}

func (c clearanceReplay) lift(x float64) (*big.Rat, error) {
	return proof.ReplayFloatRat(c.ctx, x, c.budget)
}
func (c clearanceReplay) op(a, b *big.Rat, op byte) (*big.Rat, error) {
	return proof.ReplayRatArithmetic(c.ctx, a, b, op, c.budget)
}
func (c clearanceReplay) cmp(a, b *big.Rat) (int, error) {
	return proof.ReplayRatCompare(c.ctx, a, b, c.budget)
}
func (c clearanceReplay) absolute(a *big.Rat) (*big.Rat, error) {
	if a.Sign() >= 0 {
		return a, nil
	}
	z, e := c.lift(0)
	if e != nil {
		return nil, e
	}
	return c.op(z, a, '-')
}
func (c clearanceReplay) circle(center [2]*big.Rat, radius *big.Rat) (replayBox, error) {
	var b replayBox
	for i := range b.lo {
		var err error
		b.lo[i], err = c.op(center[i], radius, '-')
		if err != nil {
			return b, err
		}
		b.hi[i], err = c.op(center[i], radius, '+')
		if err != nil {
			return b, err
		}
	}
	return b, nil
}
func (c clearanceReplay) point(p sectionrecord.Point2) ([2]*big.Rat, error) {
	var out [2]*big.Rat
	var err error
	out[0], err = c.lift(p.U)
	if err != nil {
		return out, err
	}
	out[1], err = c.lift(p.V)
	return out, err
}
func (c clearanceReplay) l1Radius(center, point [2]*big.Rat) (*big.Rat, error) {
	dx, err := c.op(point[0], center[0], '-')
	if err != nil {
		return nil, err
	}
	dx, err = c.absolute(dx)
	if err != nil {
		return nil, err
	}
	dy, err := c.op(point[1], center[1], '-')
	if err != nil {
		return nil, err
	}
	dy, err = c.absolute(dy)
	if err != nil {
		return nil, err
	}
	return c.op(dx, dy, '+')
}
func (c clearanceReplay) segmentBox(seg sectionrecord.CurveSegment) (replayBox, bool, error) {
	var box replayBox
	if err := c.budget.SpendWork(c.ctx, 1); err != nil {
		return box, false, err
	}
	switch s := seg.(type) {
	case sectionrecord.LineSeg:
		if !(s.TStart >= 0 && s.TStart <= 1 && s.TEnd >= 0 && s.TEnd <= 1) {
			return box, false, nil
		}
		a, err := c.point(s.Start)
		if err != nil {
			return box, false, err
		}
		b, err := c.point(s.End)
		if err != nil {
			return box, false, err
		}
		for i := range box.lo {
			cmp, err := c.cmp(a[i], b[i])
			if err != nil {
				return box, false, err
			}
			if cmp <= 0 {
				box.lo[i], box.hi[i] = a[i], b[i]
			} else {
				box.lo[i], box.hi[i] = b[i], a[i]
			}
		}
		return box, true, nil
	case sectionrecord.CircleSeg:
		if s.Radius.Unit() != units.Millimeter || !(s.Radius.Mag() > 0) {
			return box, false, nil
		}
		center, err := c.point(s.Center)
		if err != nil {
			return box, false, err
		}
		r, err := c.lift(s.Radius.Mag())
		if err != nil {
			return box, false, err
		}
		box, err = c.circle(center, r)
		return box, err == nil, err
	case sectionrecord.ArcSeg:
		if !WholeSegmentRange(s.TStart, s.TEnd) || s.TStart == s.TEnd {
			return box, false, nil
		}
		center, err := c.point(s.Center)
		if err != nil {
			return box, false, err
		}
		start, err := c.point(s.Start)
		if err != nil {
			return box, false, err
		}
		end, err := c.point(s.End)
		if err != nil {
			return box, false, err
		}
		r0, err := c.l1Radius(center, start)
		if err != nil {
			return box, false, err
		}
		r1, err := c.l1Radius(center, end)
		if err != nil {
			return box, false, err
		}
		cmp, err := c.cmp(r0, r1)
		if err != nil {
			return box, false, err
		}
		if cmp < 0 {
			r0 = r1
		}
		if r0.Sign() <= 0 {
			return box, false, nil
		}
		box, err = c.circle(center, r0)
		return box, err == nil, err
	default:
		return box, false, nil
	}
}
func (c clearanceReplay) separated(a, b replayBox, margin *big.Rat) (bool, error) {
	for i := range 2 {
		aEnd, err := c.op(a.hi[i], margin, '+')
		if err != nil {
			return false, err
		}
		cmp, err := c.cmp(aEnd, b.lo[i])
		if err != nil {
			return false, err
		}
		if cmp < 0 {
			return true, nil
		}
		bEnd, err := c.op(b.hi[i], margin, '+')
		if err != nil {
			return false, err
		}
		cmp, err = c.cmp(bEnd, a.lo[i])
		if err != nil {
			return false, err
		}
		if cmp < 0 {
			return true, nil
		}
	}
	return false, nil
}
func (c clearanceReplay) groupCircle(profile momentinput.Profile, m prismplacement.ReplaySignedMap) (replayBox, bool, error) {
	var box replayBox
	if err := c.budget.SpendWork(c.ctx, 1); err != nil {
		return box, false, err
	}
	if len(profile.Holes) != 0 || len(profile.Outer.Segments) != 1 {
		return box, false, nil
	}
	s, ok := profile.Outer.Segments[0].(sectionrecord.CircleSeg)
	if !ok || s.Radius.Unit() != units.Millimeter || !(s.Radius.Mag() > 0) || s.TStart != 0 || s.TEnd != 1 {
		return box, false, nil
	}
	center := [2]*big.Rat{m.OffsetU, m.OffsetV}
	for i, val := range [2]float64{s.Center.U, s.Center.V} {
		q, err := c.lift(val)
		if err != nil {
			return box, false, err
		}
		axis := m.U
		if i == 1 {
			axis = m.V
		}
		idx := axis
		if idx < 0 {
			idx = -idx
		}
		idx--
		if idx < 0 || idx > 1 {
			return box, false, nil
		}
		op := byte('+')
		if axis < 0 {
			op = '-'
		}
		center[idx], err = c.op(center[idx], q, op)
		if err != nil {
			return box, false, err
		}
	}
	radius, err := c.lift(s.Radius.Mag())
	if err != nil {
		return box, false, err
	}
	box, err = c.circle(center, radius)
	return box, err == nil, err
}

// ReplayBlindScene keeps its authenticated scene and input snapshots private.
type ReplayBlindScene struct {
	budget                          *proofbound.ReplayBudget
	match                           *sketch.Profile
	scene                           GroupScene
	target                          momentinput.Profile
	group                           []momentinput.Profile
	targetPlacement, groupPlacement prismplacement.Operand
	targetDelta, groupDelta         float64
}

// ReplayGroupSourceClearance checks the exact input snapshots used by the
// corresponding whole-loop scene. The caller separately seals the source bodies.
func ReplayGroupSourceClearance(ctx context.Context, cert *ReplayBlindScene, budget *proofbound.ReplayBudget) (bool, error) {
	if budget == nil {
		return false, nil
	}
	if err := budget.SpendWork(ctx, 1); err != nil {
		return false, err
	}
	if cert == nil || cert.budget != budget {
		return false, nil
	}
	target, group := cert.target, cert.group
	targetPlacement, groupPlacement := cert.targetPlacement, cert.groupPlacement
	targetDelta, groupDelta, scene := cert.targetDelta, cert.groupDelta, cert.scene
	if len(group) < 2 || len(group) > 4096 || len(target.Outer.Segments) == 0 {
		return false, nil
	}
	c := clearanceReplay{ctx, budget}
	m, ok, err := prismplacement.ReplayParallelSignedMap(ctx, targetPlacement, groupPlacement, budget)
	if err != nil || !ok {
		return false, err
	}
	sum := func(values ...float64) (*big.Rat, error) {
		var out *big.Rat
		for _, v := range values {
			if math.IsNaN(v) || math.IsInf(v, 0) || v < 0 {
				return nil, nil //nolint:nilnil // A nil result without an error means the bounded proof declined.
			}
			q, e := c.lift(v)
			if e != nil {
				return nil, e
			}
			if out == nil {
				out = q
			} else {
				out, e = c.op(out, q, '+')
				if e != nil {
					return nil, e
				}
			}
		}
		return out, nil
	}
	margin, err := sum(targetDelta, groupDelta, scene.Delta.A, scene.Delta.B, scene.ReexpressionDelta, scene.Delta.Crossing)
	if err != nil || margin == nil {
		return false, err
	}
	pairMargin, err := sum(groupDelta, groupDelta, scene.Delta.B, scene.Delta.B, scene.ReexpressionDelta, scene.ReexpressionDelta, scene.Delta.Crossing, scene.Delta.Crossing)
	if err != nil || pairMargin == nil {
		return false, err
	}
	for i := range group {
		if err := budget.SpendWork(ctx, 1); err != nil {
			return false, err
		}
		profile := group[i]
		a, ok, err := c.groupCircle(profile, m)
		if err != nil || !ok {
			return false, err
		}
		checkLoop := func(loop sectionrecord.LoopRecord) (bool, error) {
			if len(loop.Segments) == 0 {
				return false, nil
			}
			for i := range loop.Segments {
				if err := budget.SpendWork(ctx, 1); err != nil {
					return false, err
				}
				segment := loop.Segments[i]
				b, ok, e := c.segmentBox(segment)
				if e != nil || !ok {
					return false, e
				}
				separate, e := c.separated(a, b, margin)
				if e != nil || !separate {
					return false, e
				}
			}
			return true, nil
		}
		ok, err = checkLoop(target.Outer)
		if err != nil || !ok {
			return false, err
		}
		for i := range target.Holes {
			if err := budget.SpendWork(ctx, 1); err != nil {
				return false, err
			}
			ok, err = checkLoop(target.Holes[i])
			if err != nil || !ok {
				return false, err
			}
		}
		for j := range i {
			if err := budget.SpendWork(ctx, 1); err != nil {
				return false, err
			}
			b, ok, e := c.groupCircle(group[j], m)
			if e != nil || !ok {
				return false, e
			}
			separate, e := c.separated(a, b, pairMargin)
			if e != nil || !separate {
				return false, e
			}
		}
	}
	return true, nil
}

// ReplayBlindGroupScene constructs one whole-loop Cut scene only after checking
// that its source kinds have supported reconstruction bounds. Target whole
// arcs are charged before arrangement and bound to their source afterward.
func ReplayBlindGroupScene(ctx context.Context, target momentinput.Profile, group []momentinput.Profile,
	targetPlacement, groupPlacement prismplacement.Operand, targetDelta, groupDelta float64,
	budget *proofbound.ReplayBudget) (*ReplayBlindScene, bool, error) {
	if budget == nil {
		return nil, false, nil
	}
	if err := budget.SpendWork(ctx, 1); err != nil {
		return nil, false, err
	}
	var arcCharge float64
	if math.IsNaN(targetDelta) || math.IsInf(targetDelta, 0) || targetDelta < 0 || math.IsNaN(groupDelta) || math.IsInf(groupDelta, 0) || groupDelta < 0 {
		return nil, false, nil
	}
	check := func(loop sectionrecord.LoopRecord, allowArc bool) (bool, error) {
		if len(loop.Segments) == 0 {
			return false, nil
		}
		for i := range loop.Segments {
			if err := budget.SpendWork(ctx, 1); err != nil {
				return false, err
			}
			switch value := loop.Segments[i].(type) {
			case sectionrecord.LineSeg:
			case sectionrecord.CircleSeg:
				if !WholeSegmentRange(value.TStart, value.TEnd) || value.TStart == value.TEnd || value.CCW != (value.TStart < value.TEnd) {
					return false, nil
				}
			case sectionrecord.ArcSeg:
				if !allowArc {
					return false, nil
				}
				charge, ok, err := ReplayWholeArcCharge(ctx, value, budget)
				if err != nil || !ok {
					return false, err
				}
				arcCharge = math.Max(arcCharge, charge)

			default:
				return false, nil
			}
		}
		return true, nil
	}
	ok, err := check(target.Outer, true)
	if err != nil || !ok {
		return nil, false, err
	}
	for i := range target.Holes {
		if err := budget.SpendWork(ctx, 1); err != nil {
			return nil, false, err
		}
		ok, err = check(target.Holes[i], true)
		if err != nil || !ok {
			return nil, false, err
		}
	}
	for i := range group {
		if err := budget.SpendWork(ctx, 1); err != nil {
			return nil, false, err
		}
		region := group[i]
		if len(region.Holes) != 0 {
			return nil, false, nil
		}
		ok, err = check(region.Outer, false)
		if err != nil || !ok {
			return nil, false, err
		}
	}
	if len(group) < 2 || len(group) > 4096 {
		return nil, false, nil
	}
	cloneLoop := func(loop sectionrecord.LoopRecord) (sectionrecord.LoopRecord, error) {
		if err := budget.ReserveRecords(ctx, uint64(len(loop.Segments))); err != nil {
			return sectionrecord.LoopRecord{}, err
		}
		if err := budget.ReserveArray(ctx, uint64(len(loop.Segments)), 16); err != nil {
			return sectionrecord.LoopRecord{}, err
		}
		out := sectionrecord.LoopRecord{Segments: make([]sectionrecord.CurveSegment, len(loop.Segments))}
		for i := range loop.Segments {
			if err := budget.SpendWork(ctx, 1); err != nil {
				return sectionrecord.LoopRecord{}, err
			}
			out.Segments[i] = loop.Segments[i]
		}
		return out, nil
	}
	cloneProfile := func(profile momentinput.Profile) (momentinput.Profile, error) {
		var out momentinput.Profile
		var err error
		out.Outer, err = cloneLoop(profile.Outer)
		if err != nil {
			return out, err
		}
		if err := budget.ReserveRecords(ctx, uint64(len(profile.Holes))); err != nil {
			return out, err
		}
		if err := budget.ReserveArray(ctx, uint64(len(profile.Holes)), 24); err != nil {
			return out, err
		}
		out.Holes = make([]sectionrecord.LoopRecord, len(profile.Holes))
		for i := range profile.Holes {
			if err := budget.SpendWork(ctx, 1); err != nil {
				return out, err
			}
			out.Holes[i], err = cloneLoop(profile.Holes[i])
			if err != nil {
				return out, err
			}
		}
		return out, nil
	}
	target, err = cloneProfile(target)
	if err != nil {
		return nil, false, err
	}
	if err := budget.ReserveRecords(ctx, uint64(len(group))); err != nil {
		return nil, false, err
	}
	if err := budget.ReserveArray(ctx, uint64(len(group)), 48); err != nil {
		return nil, false, err
	}
	cloned := make([]momentinput.Profile, len(group))
	for i := range group {
		if err := budget.SpendWork(ctx, 1); err != nil {
			return nil, false, err
		}
		cloned[i], err = cloneProfile(group[i])
		if err != nil {
			return nil, false, err
		}
	}
	group = cloned
	if err := budget.SpendWork(ctx, 64); err != nil {
		return nil, false, err
	}
	chargedTarget := proofbound.AbsSumUpper(targetDelta, arcCharge)
	if math.IsNaN(chargedTarget) || math.IsInf(chargedTarget, 0) {
		return nil, false, nil
	}
	scene, ok, err := BuildGroupScene(ctx, budget.WorkBudget(ctx), []momentinput.Profile{target}, group, targetPlacement, groupPlacement, chargedTarget, groupDelta, "cut")
	if err != nil || !ok {
		return nil, false, err
	}
	match, matched, err := replayGroupCutMatchLoops(ctx, budget, scene.Profiles, scene.Tags, len(target.Holes), len(group))
	if err != nil || !matched {
		return nil, false, err
	}
	if !match.Valid {
		return nil, false, InvalidRegionError("cut")
	}
	identityCharge, identity, err := replayWholeArcIdentity(ctx, target, match, scene.Tags, budget)
	if err != nil || !identity || identityCharge > arcCharge {
		return nil, false, err
	}
	if err := budget.SpendWork(ctx, 64); err != nil {
		return nil, false, err
	}
	scene.Delta.A = proofbound.AbsSumUpper(scene.Delta.A, arcCharge)
	if math.IsNaN(scene.Delta.A) || math.IsInf(scene.Delta.A, 0) {
		return nil, false, nil
	}
	if err := budget.ReserveRecords(ctx, 1); err != nil {
		return nil, false, err
	}
	if err := budget.ReserveBytes(ctx, 1024); err != nil {
		return nil, false, err
	}
	cert := &ReplayBlindScene{budget: budget, match: match, scene: scene, target: target, group: group, targetPlacement: targetPlacement, groupPlacement: groupPlacement, targetDelta: targetDelta, groupDelta: groupDelta}
	// Exact source clearance covers every cross-region pair previously checked
	// by RegionContact. The host topology is authenticated separately.
	clearance, err := ReplayGroupSourceClearance(ctx, cert, budget)
	if err != nil || !clearance {
		return nil, false, err
	}
	return cert, true, nil
}
