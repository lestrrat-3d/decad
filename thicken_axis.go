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
		if err := ctx.Err(); err != nil {
			return nil, err
		}
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
	dirs, err := thickenAxisDirections(loop, budget)
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
	if side != ThickenNegative {
		if err := thickenAxisIntervalClear(ctx, loop, dirs, -1, amount, budget); err != nil {
			return nil, err
		}
	}
	if side != ThickenPositive {
		if err := thickenAxisIntervalClear(ctx, loop, dirs, +1, amount, budget); err != nil {
			return nil, err
		}
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

func thickenAxisDirections(loop cornerLoop, budget *workBudget) ([]thickenAxisDir, error) {
	n := len(loop.walks)
	if n < 4 {
		return nil, fmt.Errorf(`%w: an axis-parallel prism loop needs at least four walks`, ErrUnsupported)
	}
	dirs := make([]thickenAxisDir, n)
	for i, w := range loop.walks {
		if err := wallBudgetStep(budget); err != nil {
			return nil, err
		}
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
		return ProfileRecord{}, thickenAuditRefusal(err)
	}
	if err := thickenCertifyAxisOffset(loop, dirs, offset.Outer, sense, amount, budget); err != nil {
		return ProfileRecord{}, err
	}
	if err := auditOffsetSectionBudget(budget, source, offset); err != nil {
		return ProfileRecord{}, thickenAuditRefusal(err)
	}
	return offset, nil
}

func thickenCertifyAxisOffset(loop cornerLoop, dirs []thickenAxisDir, generated LoopRecord,
	sense int, amount float64, budget *workBudget) error {
	n := len(dirs)
	t := floatRat(amount)
	joins := make([]thickenAxisJoin, n)
	for i := range dirs {
		if err := wallBudgetStep(budget); err != nil {
			return err
		}
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
		if err := wallBudgetStep(budget); err != nil {
			return err
		}
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

// thickenAffine is an exact coordinate a+bτ. Every axis-line endpoint and
// corner-arc endpoint has this form because right-angle miter coefficients
// and axis normals are integers. No intermediate offset is constructed.
type thickenAffine struct{ a, b *big.Rat }

type thickenMovingPoint struct{ u, v thickenAffine }

type thickenMovingPiece struct {
	line       bool
	horizontal bool
	start, end thickenMovingPoint
	center     thickenExactPoint
}

type thickenExactBox struct{ minU, maxU, minV, maxV *big.Rat }

func thickenAffineCoord(x float64, step int) thickenAffine {
	return thickenAffine{a: floatRat(x), b: big.NewRat(int64(step), 1)}
}

func thickenMovingOffset(u, v float64, du, dv int) thickenMovingPoint {
	return thickenMovingPoint{u: thickenAffineCoord(u, du), v: thickenAffineCoord(v, dv)}
}

func thickenAffineSub(a, b thickenAffine) ratPoly {
	return ratPoly{new(big.Rat).Sub(a.a, b.a), new(big.Rat).Sub(a.b, b.b)}
}

func thickenAffineConst(a thickenAffine, b *big.Rat) ratPoly {
	return ratPoly{new(big.Rat).Sub(a.a, b), new(big.Rat).Set(a.b)}
}

func thickenAffineAt(a thickenAffine, at *big.Rat) *big.Rat {
	return new(big.Rat).Add(a.a, new(big.Rat).Mul(a.b, at))
}

func thickenRange(values ...*big.Rat) (*big.Rat, *big.Rat) {
	lo, hi := values[0], values[0]
	for _, v := range values[1:] {
		if v.Cmp(lo) < 0 {
			lo = v
		}
		if v.Cmp(hi) > 0 {
			hi = v
		}
	}
	return lo, hi
}

// thickenPieceBox encloses a piece over every τ in [0, limit] using only
// affine endpoint extrema. A corner arc stays in its fixed radial quadrant.
func thickenPieceBox(p thickenMovingPiece, limit *big.Rat) thickenExactBox {
	if p.line {
		zero := new(big.Rat)
		minU, maxU := thickenRange(thickenAffineAt(p.start.u, zero), thickenAffineAt(p.end.u, zero),
			thickenAffineAt(p.start.u, limit), thickenAffineAt(p.end.u, limit))
		minV, maxV := thickenRange(thickenAffineAt(p.start.v, zero), thickenAffineAt(p.end.v, zero),
			thickenAffineAt(p.start.v, limit), thickenAffineAt(p.end.v, limit))
		return thickenExactBox{minU: minU, maxU: maxU, minV: minV, maxV: maxV}
	}
	sx, sy := p.start.u.b, p.start.v.b
	if sx.Sign() == 0 {
		sx = p.end.u.b
	}
	if sy.Sign() == 0 {
		sy = p.end.v.b
	}
	minU, maxU := thickenRange(p.center.u, new(big.Rat).Add(p.center.u, new(big.Rat).Mul(sx, limit)))
	minV, maxV := thickenRange(p.center.v, new(big.Rat).Add(p.center.v, new(big.Rat).Mul(sy, limit)))
	return thickenExactBox{minU: minU, maxU: maxU, minV: minV, maxV: maxV}
}

func thickenBoxesDisjoint(a, b thickenExactBox) bool {
	return a.maxU.Cmp(b.minU) < 0 || b.maxU.Cmp(a.minU) < 0 ||
		a.maxV.Cmp(b.minV) < 0 || b.maxV.Cmp(a.minV) < 0
}

func thickenContactEvent(ctx context.Context, p ratPoly, limit *big.Rat) (bool, error) {
	p = rpSquareFree(p)
	if rpDeg(p) < 1 {
		return false, nil
	}
	chain, err := sturmChainIntContext(ctx, p)
	if err != nil {
		return false, err
	}
	return sturmCount(chain, new(big.Rat), limit) > 0, nil
}

// thickenAxisIntervalClear isolates every possible first contact event of
// nonadjacent moving pieces. Their supporting equations have degree at most
// two in τ: line endpoints and carriers are affine, and a corner radius is τ.
// A root of a supporting equation can be a false contact on a trimmed piece;
// refusing it is conservative. If no such root lies in (0, amount], no actual
// contact can begin there. The audited endpoint decides the final winding.
func thickenAxisIntervalClear(ctx context.Context, loop cornerLoop, dirs []thickenAxisDir,
	sense int, amount float64, budget *workBudget) error {
	n := len(dirs)
	joins := make([]struct {
		arc              bool
		m, before, after thickenMovingPoint
	}, n)
	for i := range dirs {
		if err := wallBudgetStep(budget); err != nil {
			return err
		}
		prev, cur := dirs[(i+n-1)%n], dirs[i]
		v := loop.walks[i]
		j := &joins[i]
		j.arc = prev.u*cur.v-prev.v*cur.u == -sense
		j.before = thickenMovingOffset(v.startU, v.startV, sense*-prev.v, sense*prev.u)
		j.after = thickenMovingOffset(v.startU, v.startV, sense*-cur.v, sense*cur.u)
		j.m = thickenMovingOffset(v.startU, v.startV,
			sense*(-prev.v-cur.v), sense*(prev.u+cur.u))
	}
	var pieces []thickenMovingPiece
	for i, dir := range dirs {
		if err := wallBudgetStep(budget); err != nil {
			return err
		}
		start, end := joins[i].m, joins[(i+1)%n].m
		if joins[i].arc {
			start = joins[i].after
		}
		if joins[(i+1)%n].arc {
			end = joins[(i+1)%n].before
		}
		pieces = append(pieces, thickenMovingPiece{line: true, horizontal: dir.u != 0,
			start: start, end: end})
		if joins[(i+1)%n].arc {
			v := loop.walks[(i+1)%n]
			pieces = append(pieces, thickenMovingPiece{
				start: joins[(i+1)%n].before, end: joins[(i+1)%n].after,
				center: thickenExactPoint{u: floatRat(v.startU), v: floatRat(v.startV)},
			})
		}
	}
	limit := floatRat(amount)
	boxes := make([]thickenExactBox, len(pieces))
	for i, piece := range pieces {
		if err := wallBudgetStep(budget); err != nil {
			return err
		}
		boxes[i] = thickenPieceBox(piece, limit)
	}
	check := func(p ratPoly) error {
		contact, err := thickenContactEvent(ctx, p, limit)
		if err != nil {
			return err
		}
		if contact {
			return fmt.Errorf(`%w: the offset interval contains a possible nonadjacent contact`, ErrUnsupported)
		}
		return nil
	}
	for i, a := range pieces {
		if a.line {
			advance := thickenAffineSub(a.end.v, a.start.v)
			if a.horizontal {
				advance = thickenAffineSub(a.end.u, a.start.u)
			}
			if err := check(advance); err != nil {
				return err
			}
		}
		for j := i + 1; j < len(pieces); j++ {
			if err := wallBudgetStep(budget); err != nil {
				return err
			}
			if j == i+1 || (i == 0 && j == len(pieces)-1) {
				continue
			}
			if thickenBoxesDisjoint(boxes[i], boxes[j]) {
				continue
			}
			b := pieces[j]
			if a.line && b.line {
				if thickenLineLineContact(a, b, limit) {
					return fmt.Errorf(`%w: the offset interval contains a nonadjacent line contact`, ErrUnsupported)
				}
				continue
			}
			var candidates []ratPoly
			switch {
			case a.line:
				candidates = thickenLineArcEvents(a, b)
			case b.line:
				candidates = thickenLineArcEvents(b, a)
			default:
				du := new(big.Rat).Sub(a.center.u, b.center.u)
				dv := new(big.Rat).Sub(a.center.v, b.center.v)
				d2 := new(big.Rat).Add(new(big.Rat).Mul(du, du), new(big.Rat).Mul(dv, dv))
				candidates = []ratPoly{{new(big.Rat).Neg(d2), new(big.Rat), big.NewRat(4, 1)}}
			}
			for _, p := range candidates {
				if err := check(p); err != nil {
					return err
				}
			}
		}
	}
	return nil
}

// A first finite line-line contact occurs when one endpoint reaches the
// other's support, or parallel supports become equal. These parameters are
// rational; test finite segment inclusion at each exact root.
func thickenLineLineContact(a, b thickenMovingPiece, limit *big.Rat) bool {
	for _, p := range thickenLineLineEvents(a, b) {
		p = rpTrim(p)
		if len(p) != 2 || p[1].Sign() == 0 {
			continue
		}
		root := new(big.Rat).Quo(new(big.Rat).Neg(p[0]), p[1])
		if root.Sign() <= 0 || root.Cmp(limit) > 0 {
			continue
		}
		if thickenLinesTouchAt(a, b, root) {
			return true
		}
	}
	return false
}

func thickenRangesOverlap(a0, a1, b0, b1 *big.Rat) bool {
	aLo, aHi := thickenRange(a0, a1)
	bLo, bHi := thickenRange(b0, b1)
	return aLo.Cmp(bHi) <= 0 && bLo.Cmp(aHi) <= 0
}

func thickenWithin(value, a, b *big.Rat) bool {
	lo, hi := thickenRange(a, b)
	return lo.Cmp(value) <= 0 && value.Cmp(hi) <= 0
}

func thickenLinesTouchAt(a, b thickenMovingPiece, at *big.Rat) bool {
	au0, av0 := thickenAffineAt(a.start.u, at), thickenAffineAt(a.start.v, at)
	au1, av1 := thickenAffineAt(a.end.u, at), thickenAffineAt(a.end.v, at)
	bu0, bv0 := thickenAffineAt(b.start.u, at), thickenAffineAt(b.start.v, at)
	bu1, bv1 := thickenAffineAt(b.end.u, at), thickenAffineAt(b.end.v, at)
	if a.horizontal && b.horizontal {
		return av0.Cmp(bv0) == 0 && thickenRangesOverlap(au0, au1, bu0, bu1)
	}
	if !a.horizontal && !b.horizontal {
		return au0.Cmp(bu0) == 0 && thickenRangesOverlap(av0, av1, bv0, bv1)
	}
	if a.horizontal {
		return thickenWithin(bu0, au0, au1) && thickenWithin(av0, bv0, bv1)
	}
	return thickenWithin(au0, bu0, bu1) && thickenWithin(bv0, av0, av1)
}

func thickenLineLineEvents(a, b thickenMovingPiece) []ratPoly {
	if a.horizontal && b.horizontal {
		return []ratPoly{
			thickenAffineSub(a.start.v, b.start.v),
			thickenAffineSub(a.start.u, b.start.u), thickenAffineSub(a.start.u, b.end.u),
			thickenAffineSub(a.end.u, b.start.u), thickenAffineSub(a.end.u, b.end.u),
		}
	}
	if !a.horizontal && !b.horizontal {
		return []ratPoly{
			thickenAffineSub(a.start.u, b.start.u),
			thickenAffineSub(a.start.v, b.start.v), thickenAffineSub(a.start.v, b.end.v),
			thickenAffineSub(a.end.v, b.start.v), thickenAffineSub(a.end.v, b.end.v),
		}
	}
	horizontal, vertical := a, b
	if !a.horizontal {
		horizontal, vertical = b, a
	}
	return []ratPoly{
		thickenAffineSub(vertical.start.u, horizontal.start.u),
		thickenAffineSub(vertical.start.u, horizontal.end.u),
		thickenAffineSub(horizontal.start.v, vertical.start.v),
		thickenAffineSub(horizontal.start.v, vertical.end.v),
	}
}

func thickenLineArcEvents(line, arc thickenMovingPiece) []ratPoly {
	var distance ratPoly
	var arcStart, arcEnd, lineCoord thickenAffine
	if line.horizontal {
		distance = thickenAffineConst(line.start.v, arc.center.v)
		arcStart, arcEnd = arc.start.v, arc.end.v
		lineCoord = line.start.v
	} else {
		distance = thickenAffineConst(line.start.u, arc.center.u)
		arcStart, arcEnd = arc.start.u, arc.end.u
		lineCoord = line.start.u
	}
	tau := ratPoly{new(big.Rat), big.NewRat(1, 1)}
	contact := rpSub(rpMul(distance, distance), rpMul(tau, tau))
	endpoint := func(p thickenMovingPoint) ratPoly {
		du := thickenAffineConst(p.u, arc.center.u)
		dv := thickenAffineConst(p.v, arc.center.v)
		return rpSub(rpAdd(rpMul(du, du), rpMul(dv, dv)), rpMul(tau, tau))
	}
	return []ratPoly{contact, endpoint(line.start), endpoint(line.end),
		thickenAffineSub(arcStart, lineCoord), thickenAffineSub(arcEnd, lineCoord)}
}
