package decad

import (
	"context"
	"fmt"
	"math/big"
)

type thickenAxisDir struct{ u, v int }

type thickenExactPoint struct{ u, v *big.Rat }

type thickenAxisJoin struct {
	arc              bool
	m, before, after thickenExactPoint
}

func thickenPrismAxis(ctx context.Context, d *Document, pp prismPayload, side ThickenSide,
	amount float64, budget *workBudget) (*Body, error) {
	for _, seg := range pp.profile.Outer.Segments {
		if _, ok := seg.(LineSeg); !ok {
			return nil, fmt.Errorf(`%w: the prism sheet requires line-only axis-parallel walks`, ErrUnsupported)
		}
	}
	loops, err := prismCornerLoopsBudget(budget, pp)
	if err != nil {
		return nil, err
	}
	if len(loops) != 1 {
		return nil, fmt.Errorf(`%w: the prism sheet requires one outer loop`, ErrUnsupported)
	}
	loop := loops[0]
	dirs, err := thickenAxisDirections(loop)
	if err != nil {
		return nil, err
	}
	var outer, inner ProfileRecord
	switch side {
	case ThickenPositive:
		outer, err = thickenAxisOffset(budget, pp.profile, loop, dirs, -1, amount)
		inner = pp.profile
	case ThickenNegative:
		inner, err = thickenAxisOffset(budget, pp.profile, loop, dirs, +1, amount)
		outer = pp.profile
	case ThickenCentered:
		outer, err = thickenAxisOffset(budget, pp.profile, loop, dirs, -1, amount)
		if err == nil {
			inner, err = thickenAxisOffset(budget, pp.profile, loop, dirs, +1, amount)
		}
	}
	if err != nil {
		return nil, err
	}
	// Every offset point stays within sqrt(2)*amount of its source feature.
	// A strict source gap greater than 2*sqrt(2)*amount therefore excludes
	// every unintended contact between non-neighbouring features throughout
	// the interval, including between the two centered offsets. Neighbouring
	// pieces meet only at the joins certified below; offsetLoopBudget already
	// rejects a consumed walk. This exact rational sufficient condition may
	// refuse safe narrow parts; it cannot admit an earlier pinch.
	if err := thickenAxisIntervalClear(loop, amount, budget); err != nil {
		return nil, err
	}
	hole, err := reverseLoopRecordContext(ctx, inner.Outer)
	if err != nil {
		return nil, err
	}
	annulus := ProfileRecord{Outer: outer.Outer, Holes: []LoopRecord{hole}}
	entries, err := buildSegEntriesBudget(budget, []LoopRecord{annulus.Outer, hole})
	if err != nil {
		return nil, err
	}
	if err := thickenAuditRefusal(crossingAuditBudget(budget, entries)); err != nil {
		return nil, err
	}
	if err := thickenAuditRefusal(nestingAuditBudget(budget, entries, 2)); err != nil {
		return nil, err
	}
	if side == ThickenPositive {
		return evalTubeContext(ctx, d, d.nextProducerID(), pp, outer, -1)
	}
	if side == ThickenNegative {
		return evalTubeContext(ctx, d, d.nextProducerID(), pp, inner, +1)
	}
	pp.profile = annulus
	pp.surfaceResult = false
	pp.walks = nil
	return evalPrismContext(ctx, d, d.nextProducerID(), pp, newFreeformWork())
}

func thickenAxisDirections(loop cornerLoop) ([]thickenAxisDir, error) {
	n := len(loop.walks)
	if n < 4 {
		return nil, fmt.Errorf(`%w: an axis-parallel prism loop needs at least four walks`, ErrUnsupported)
	}
	dirs := make([]thickenAxisDir, n)
	for i, w := range loop.walks {
		next := loop.walks[(i+1)%n]
		if w.startBound.u != 0 || w.startBound.v != 0 || w.endBound.u != 0 || w.endBound.v != 0 {
			return nil, fmt.Errorf(`%w: a source line endpoint has an unresolved coordinate bound`, ErrUnsupported)
		}
		if w.endU != next.startU || w.endV != next.startV {
			return nil, fmt.Errorf(`%w: the prism loop has no exact adjacent joins`, ErrUnsupported)
		}
		switch {
		case w.startU == w.endU && w.startV < w.endV:
			dirs[i] = thickenAxisDir{v: 1}
		case w.startU == w.endU && w.startV > w.endV:
			dirs[i] = thickenAxisDir{v: -1}
		case w.startV == w.endV && w.startU < w.endU:
			dirs[i] = thickenAxisDir{u: 1}
		case w.startV == w.endV && w.startU > w.endU:
			dirs[i] = thickenAxisDir{u: -1}
		default:
			return nil, fmt.Errorf(`%w: the prism loop is not axis-parallel`, ErrUnsupported)
		}
		if floatRat(w.startU) == nil || floatRat(w.startV) == nil ||
			floatRat(w.endU) == nil || floatRat(w.endV) == nil {
			return nil, fmt.Errorf(`%w: a prism boundary coordinate is not finite`, ErrUnsupported)
		}
	}
	for i := range dirs {
		p, q := dirs[(i+n-1)%n], dirs[i]
		if p.u*q.v-p.v*q.u == 0 {
			return nil, fmt.Errorf(`%w: a prism corner is not a right angle`, ErrUnsupported)
		}
	}
	return dirs, nil
}

func thickenAxisOffset(budget *workBudget, source ProfileRecord, loop cornerLoop,
	dirs []thickenAxisDir, sense int, amount float64) (ProfileRecord, error) {
	offset, err := offsetProfile(budget, source, float64(sense), amount)
	if err != nil {
		return ProfileRecord{}, err
	}
	if err := thickenCertifyAxisOffset(loop, dirs, offset.Outer, sense, amount); err != nil {
		return ProfileRecord{}, err
	}
	if err := thickenAuditRefusal(auditOffsetSectionBudget(budget, source, offset)); err != nil {
		return ProfileRecord{}, err
	}
	return offset, nil
}

func thickenCertifyAxisOffset(loop cornerLoop, dirs []thickenAxisDir, generated LoopRecord,
	sense int, amount float64) error {
	n := len(dirs)
	t := floatRat(amount)
	joins := make([]thickenAxisJoin, n)
	for i := range dirs {
		prev, cur := dirs[(i+n-1)%n], dirs[i]
		turn := prev.u*cur.v - prev.v*cur.u
		vertex := loop.walks[i]
		j := &joins[i]
		j.arc = turn == -sense
		j.before = thickenExactOffset(vertex.startU, vertex.startV, sense*(-prev.v), sense*prev.u, t)
		j.after = thickenExactOffset(vertex.startU, vertex.startV, sense*(-cur.v), sense*cur.u, t)
		j.m = thickenExactOffset(vertex.startU, vertex.startV,
			sense*(-prev.v-cur.v), sense*(prev.u+cur.u), t)
	}
	idx := 0
	for i := range dirs {
		if idx >= len(generated.Segments) {
			return fmt.Errorf(`%w: the offset dropped a line walk`, ErrUnsupported)
		}
		line, ok := generated.Segments[idx].(LineSeg)
		if !ok || line.TStart != 0 || line.TEnd != 1 {
			return fmt.Errorf(`%w: the offset line is not the generated walk`, ErrUnsupported)
		}
		start, end := joins[i].m, joins[(i+1)%n].m
		if joins[i].arc {
			start = joins[i].after
		}
		if joins[(i+1)%n].arc {
			end = joins[(i+1)%n].before
		}
		if !thickenPointIsExact(line.Start, start) || !thickenPointIsExact(line.End, end) {
			return fmt.Errorf(`%w: a generated offset line coordinate is rounded`, ErrUnsupported)
		}
		idx++
		corner := (i + 1) % n
		if !joins[corner].arc {
			continue
		}
		if idx >= len(generated.Segments) {
			return fmt.Errorf(`%w: the offset dropped a corner arc`, ErrUnsupported)
		}
		arc, ok := generated.Segments[idx].(ArcSeg)
		if !ok {
			return fmt.Errorf(`%w: the offset corner is not an arc`, ErrUnsupported)
		}
		before, after := joins[corner].before, joins[corner].after
		if sense > 0 {
			before, after = after, before
		}
		v := loop.walks[corner]
		center := thickenExactOffset(v.startU, v.startV, 0, 0, t)
		if !thickenPointIsExact(arc.Center, center) ||
			!thickenPointIsExact(arc.Start, before) || !thickenPointIsExact(arc.End, after) ||
			(sense < 0 && (arc.TStart != 0 || arc.TEnd != 1)) ||
			(sense > 0 && (arc.TStart != 1 || arc.TEnd != 0)) {
			return fmt.Errorf(`%w: a generated corner arc coordinate is rounded`, ErrUnsupported)
		}
		idx++
	}
	if idx != len(generated.Segments) {
		return fmt.Errorf(`%w: the offset added an unexpected feature`, ErrUnsupported)
	}
	return nil
}

func thickenExactOffset(u, v float64, du, dv int, t *big.Rat) thickenExactPoint {
	return thickenExactPoint{
		u: new(big.Rat).Add(floatRat(u), new(big.Rat).Mul(big.NewRat(int64(du), 1), t)),
		v: new(big.Rat).Add(floatRat(v), new(big.Rat).Mul(big.NewRat(int64(dv), 1), t)),
	}
}

func thickenPointIsExact(got Point2, want thickenExactPoint) bool {
	return rationalFloatError(want.u, got.U) == 0 && rationalFloatError(want.v, got.V) == 0
}

// thickenAxisIntervalClear gives a conservative, exact all-parameter proof.
// Distances between non-neighbouring source walks exceed the sum of their
// maximum possible displacements, so none can touch at an earlier parameter.
func thickenAxisIntervalClear(loop cornerLoop, amount float64, budget *workBudget) error {
	n := len(loop.walks)
	t := floatRat(amount)
	limit := new(big.Rat).Mul(t, t)
	limit.Mul(limit, big.NewRat(8, 1))
	for i := range n {
		for j := i + 1; j < n; j++ {
			if err := wallBudgetStep(budget); err != nil {
				return err
			}
			if j == i+1 || (i == 0 && j == n-1) {
				continue
			}
			a, b := loop.walks[i], loop.walks[j]
			du := thickenIntervalGap(a.startU, a.endU, b.startU, b.endU)
			dv := thickenIntervalGap(a.startV, a.endV, b.startV, b.endV)
			d2 := new(big.Rat).Mul(du, du)
			d2.Add(d2, new(big.Rat).Mul(dv, dv))
			if d2.Cmp(limit) <= 0 {
				return fmt.Errorf(`%w: the offset interval cannot certify separation of nonadjacent walks`, ErrUnsupported)
			}
		}
	}
	return nil
}

func thickenIntervalGap(a0, a1, b0, b1 float64) *big.Rat {
	aLo, aHi := floatRat(a0), floatRat(a1)
	bLo, bHi := floatRat(b0), floatRat(b1)
	if aLo.Cmp(aHi) > 0 {
		aLo, aHi = aHi, aLo
	}
	if bLo.Cmp(bHi) > 0 {
		bLo, bHi = bHi, bLo
	}
	if aHi.Cmp(bLo) < 0 {
		return new(big.Rat).Sub(bLo, aHi)
	}
	if bHi.Cmp(aLo) < 0 {
		return new(big.Rat).Sub(aLo, bHi)
	}
	return new(big.Rat)
}
