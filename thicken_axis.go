package decad

import (
	"context"
	"fmt"
	"math/big"
	"slices"
)

type thickenAxisDir struct{ u, v int }

type thickenExactPoint struct{ u, v *big.Rat }

type thickenAxisJoin struct {
	arc              bool
	m, before, after thickenExactPoint
}

func thickenAxisSection(ctx context.Context, profile ProfileRecord, side ThickenSide,
	amount float64, budget *workBudget, radial *thickenRadial) (thickenSection, error) {
	for _, seg := range profile.Outer.Segments {
		if err := ctx.Err(); err != nil {
			return thickenSection{}, err
		}
		if _, ok := seg.(LineSeg); !ok {
			return thickenSection{}, fmt.Errorf(`%w: the sheet requires line-only axis-parallel walks`, ErrUnsupported)
		}
	}
	loops, err := prismCornerLoopsBudget(budget, prismPayload{profile: profile})
	if err != nil {
		return thickenSection{}, err
	}
	if len(loops) != 1 {
		return thickenSection{}, fmt.Errorf(`%w: the sheet requires one outer loop`, ErrUnsupported)
	}
	loop := loops[0]
	dirs, err := thickenAxisDirections(loop, budget)
	if err != nil {
		return thickenSection{}, err
	}
	sec := thickenSection{source: profile, outer: profile, inner: profile}
	if side != ThickenNegative {
		if sec.outer, err = thickenAxisOffset(budget, profile, loop, dirs, -1, amount); err != nil {
			return thickenSection{}, err
		}
	}
	if side != ThickenPositive {
		if sec.inner, err = thickenAxisOffset(budget, profile, loop, dirs, +1, amount); err != nil {
			return thickenSection{}, err
		}
	}
	if side != ThickenNegative {
		if err := thickenAxisIntervalClear(ctx, loop, dirs, -1, amount, budget, radial); err != nil {
			return thickenSection{}, err
		}
	}
	if side != ThickenPositive {
		if err := thickenAxisIntervalClear(ctx, loop, dirs, +1, amount, budget, radial); err != nil {
			return thickenSection{}, err
		}
	}
	hole, err := reverseLoopRecordContext(ctx, sec.inner.Outer)
	if err != nil {
		return thickenSection{}, err
	}
	entries, err := buildSegEntriesBudget(budget, []LoopRecord{sec.outer.Outer, hole})
	if err != nil {
		return thickenSection{}, err
	}
	if err := thickenAuditRefusal(crossingAuditBudget(budget, entries)); err != nil {
		return thickenSection{}, err
	}
	if err := thickenAuditRefusal(nestingAuditBudget(budget, entries, 2)); err != nil {
		return thickenSection{}, err
	}
	return sec, nil
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
	sense int, amount float64, budget *workBudget, radial *thickenRadial) error {
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
	return thickenPiecesIntervalClear(ctx, pieces, floatRat(amount), budget, radial)
}

// thickenPiecesIntervalClear is the interval scan itself, over one CLOSED ring
// of moving pieces in loop order: consecutive pieces are the joins the
// construction prescribes and are excluded, and every other pair is isolated
// exactly.
func thickenPiecesIntervalClear(ctx context.Context, pieces []thickenMovingPiece,
	limit *big.Rat, budget *workBudget, radial *thickenRadial) error {
	boxes := make([]thickenExactBox, len(pieces))
	for i, piece := range pieces {
		if err := wallBudgetStep(budget); err != nil {
			return err
		}
		boxes[i] = thickenPieceBox(piece, limit)
	}
	if radial != nil {
		// Each box encloses its own piece over every τ in [0, amount], so the
		// least radius the whole swept family reaches is the least any box
		// corner reaches. The boxes at τ = 0 are the SOURCE walks, so one scan
		// certifies the source section, the requested offset, and every
		// intermediate offset between them.
		for _, box := range boxes {
			if err := radial.require(radial.leastOverBox(box)); err != nil {
				return err
			}
		}
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

// This section is the OPEN-walk counterpart of the closed offsets above: the
// ribbon and the uncapped chain shell of docs/surface-design.md §16.6 and
// §16.7. An open walk has no interior to erode, so there is no P ⊖ t / P ⊕ t
// to take. What a thickened ribbon sweeps is one CLOSED section assembled in
// closed form: the walk's two offset copies — or the walk and one copy —
// joined by one cap line at each free end, walked right copy forward, end cap,
// left copy backward, start cap. Every coordinate of that section is a
// recorded walk coordinate plus an INTEGER multiple of the offset parameter,
// so the whole section is affine in τ and the interval scan above reads it
// unchanged.

// thickenRibbonSide names one side of the walk and how far its copy sits from
// it: normal is the left-normal sign (−1 the walk's right-hand side, +1 its
// left), and steps is 0 where that copy IS the recorded walk and 1 where it is
// an offset copy.
type thickenRibbonSide struct{ normal, steps int }

// thickenRibbonSides is the pair of copies one thicken side assembles, right
// copy first.
func thickenRibbonSides(side ThickenSide) (right, left thickenRibbonSide) {
	switch side {
	case ThickenNegative:
		return thickenRibbonSide{normal: -1, steps: 0}, thickenRibbonSide{normal: +1, steps: 1}
	case ThickenCentered:
		return thickenRibbonSide{normal: -1, steps: 1}, thickenRibbonSide{normal: +1, steps: 1}
	default:
		return thickenRibbonSide{normal: -1, steps: 1}, thickenRibbonSide{normal: +1, steps: 0}
	}
}

// thickenOpenDirections is thickenAxisDirections without wraparound: it reads
// one open walk's per-segment axis direction, refusing an inexact endpoint, a
// junction the two walks do not share exactly, a segment that is not
// axis-parallel, and an interior corner that is not a right angle.
func thickenOpenDirections(walks []sideWalk, budget *workBudget) ([]thickenAxisDir, error) {
	n := len(walks)
	if n == 0 {
		return nil, fmt.Errorf(`%w: an open walk holds no segment`, ErrUnsupported)
	}
	dirs := make([]thickenAxisDir, n)
	for i, w := range walks {
		if err := wallBudgetStep(budget); err != nil {
			return nil, err
		}
		if w.startBound.u != 0 || w.startBound.v != 0 || w.endBound.u != 0 || w.endBound.v != 0 {
			return nil, fmt.Errorf(`%w: an open walk endpoint has an unresolved coordinate bound`, ErrUnsupported)
		}
		if i+1 < n {
			next := walks[i+1]
			if w.endU != next.startU || w.endV != next.startV {
				return nil, fmt.Errorf(`%w: the open walk has no exact interior joins`, ErrUnsupported)
			}
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
			return nil, fmt.Errorf(`%w: the open walk is not axis-parallel`, ErrUnsupported)
		}
		if floatRat(w.startU) == nil || floatRat(w.startV) == nil ||
			floatRat(w.endU) == nil || floatRat(w.endV) == nil {
			return nil, fmt.Errorf(`%w: an open walk coordinate is not finite`, ErrUnsupported)
		}
	}
	for i := 1; i < n; i++ {
		p, q := dirs[i-1], dirs[i]
		if p.u*q.v-p.v*q.u == 0 {
			return nil, fmt.Errorf(`%w: an open walk corner is not a right angle`, ErrUnsupported)
		}
	}
	return dirs, nil
}

// thickenRibbonCopy is one side's moving copy of the walk, in walk order: the
// offset lines and the corner arcs between them, plus the two free-end points
// the caps attach to.
type thickenRibbonCopy struct {
	pieces     []thickenMovingPiece
	head, tail thickenMovingPoint
}

// thickenRibbonCopyOf builds one copy in closed form. A corner arc appears
// exactly where the walk turns AWAY from this copy's side, the same
// sign(cross) == −s rule the closed offset takes (shell_offset.go); the other
// corner miters. A copy with steps == 0 is the recorded walk itself and takes
// neither.
func thickenRibbonCopyOf(walks []sideWalk, dirs []thickenAxisDir, c thickenRibbonSide) thickenRibbonCopy {
	n := len(dirs)
	k := c.steps * c.normal
	offset := func(u, v float64, d thickenAxisDir) thickenMovingPoint {
		return thickenMovingOffset(u, v, k*-d.v, k*d.u)
	}
	type join struct {
		arc              bool
		m, before, after thickenMovingPoint
	}
	joins := make([]join, n)
	for i := 1; i < n; i++ {
		prev, cur := dirs[i-1], dirs[i]
		v := walks[i]
		joins[i] = join{
			arc:    c.steps != 0 && prev.u*cur.v-prev.v*cur.u == -c.normal,
			before: offset(v.startU, v.startV, prev),
			after:  offset(v.startU, v.startV, cur),
			m: thickenMovingOffset(v.startU, v.startV,
				k*(-prev.v-cur.v), k*(prev.u+cur.u)),
		}
	}
	out := thickenRibbonCopy{
		head: offset(walks[0].startU, walks[0].startV, dirs[0]),
		tail: offset(walks[n-1].endU, walks[n-1].endV, dirs[n-1]),
	}
	for i, dir := range dirs {
		start, end := out.head, out.tail
		if i > 0 {
			start = joins[i].m
			if joins[i].arc {
				start = joins[i].after
			}
		}
		if i+1 < n {
			end = joins[i+1].m
			if joins[i+1].arc {
				end = joins[i+1].before
			}
		}
		out.pieces = append(out.pieces, thickenMovingPiece{line: true, horizontal: dir.u != 0,
			start: start, end: end})
		if i+1 < n && joins[i+1].arc {
			v := walks[i+1]
			out.pieces = append(out.pieces, thickenMovingPiece{
				start: joins[i+1].before, end: joins[i+1].after,
				center: thickenExactPoint{u: floatRat(v.startU), v: floatRat(v.startV)},
			})
		}
	}
	return out
}

// thickenReverseMovingPiece walks one piece the other way. A line swaps its
// two ends; an arc swaps them about the same fixed centre.
func thickenReverseMovingPiece(p thickenMovingPiece) thickenMovingPiece {
	p.start, p.end = p.end, p.start
	return p
}

// thickenRibbonLoop assembles the two copies into ONE closed moving boundary
// in loop order: the right copy forward, the cap at the walk's far end, the
// left copy backward, and the cap at its near end. The order is what makes the
// interval scan's own adjacency exclusion (consecutive pieces of one loop)
// correct with no second rule.
func thickenRibbonLoop(right, left thickenRibbonCopy) []thickenMovingPiece {
	pieces := append([]thickenMovingPiece{}, right.pieces...)
	pieces = append(pieces, thickenCapPiece(right.tail, left.tail))
	for _, piece := range slices.Backward(left.pieces) {
		pieces = append(pieces, thickenReverseMovingPiece(piece))
	}
	return append(pieces, thickenCapPiece(left.head, right.head))
}

// thickenCapPiece is one free end's cap: the straight join between the two
// copies' own endpoints there. Both endpoints offset along the SAME normal
// line, so the cap is axis-parallel wherever the walk's end segment is.
func thickenCapPiece(from, to thickenMovingPoint) thickenMovingPiece {
	horizontal := from.v.a.Cmp(to.v.a) == 0 && from.v.b.Cmp(to.v.b) == 0
	return thickenMovingPiece{line: true, horizontal: horizontal, start: from, end: to}
}

// thickenExactPointAt evaluates one moving point at τ and refuses unless every
// held float64 equals its closed-form value exactly (R27).
func thickenExactPointAt(p thickenMovingPoint, at *big.Rat) (Point2, error) {
	u, v := thickenAffineAt(p.u, at), thickenAffineAt(p.v, at)
	held := Point2{U: ratToFloat(u), V: ratToFloat(v)}
	if rationalFloatError(u, held.U) != 0 || rationalFloatError(v, held.V) != 0 {
		return Point2{}, fmt.Errorf(`%w: a generated ribbon coordinate is rounded`, ErrUnsupported)
	}
	return held, nil
}

func ratToFloat(r *big.Rat) float64 {
	f, _ := r.Float64()
	return f
}

// thickenRibbonSection evaluates the assembled moving boundary at the
// requested offset, refusing any coordinate that does not land exactly.
func thickenRibbonSection(pieces []thickenMovingPiece, at *big.Rat) (ProfileRecord, error) {
	segs := make([]CurveSegment, 0, len(pieces))
	for _, p := range pieces {
		start, err := thickenExactPointAt(p.start, at)
		if err != nil {
			return ProfileRecord{}, err
		}
		end, err := thickenExactPointAt(p.end, at)
		if err != nil {
			return ProfileRecord{}, err
		}
		if p.line {
			if start == end {
				return ProfileRecord{}, fmt.Errorf(`%w: a generated ribbon segment has no length`, ErrUnsupported)
			}
			segs = append(segs, LineSeg{Start: start, End: end, TStart: 0, TEnd: 1})
			continue
		}
		center := Point2{U: ratToFloat(p.center.u), V: ratToFloat(p.center.v)}
		if rationalFloatError(p.center.u, center.U) != 0 || rationalFloatError(p.center.v, center.V) != 0 {
			return ProfileRecord{}, fmt.Errorf(`%w: a generated ribbon corner centre is rounded`, ErrUnsupported)
		}
		segs = append(segs, arcSegment(center, start, end, thickenArcIsCCW(start, end, center)))
	}
	return ProfileRecord{Outer: LoopRecord{Segments: segs}}, nil
}

// thickenArcIsCCW reads a right-angle corner arc's own sense from the exact
// integer cross product of its two radii — never an angle.
func thickenArcIsCCW(start, end, center Point2) bool {
	return (start.U-center.U)*(end.V-center.V)-(start.V-center.V)*(end.U-center.U) > 0
}

// thickenRibbon certifies the closed section one open axis-parallel walk
// sweeps when it is thickened: the assembled boundary at the requested offset,
// proven simple there and proven free of any nonadjacent contact over the
// whole interval 0 < τ ≤ amount.
func thickenRibbon(ctx context.Context, chain ChainRecord, side ThickenSide, amount float64,
	budget *workBudget, work *freeformWork) (ProfileRecord, error) {
	raw := make([]sideWalk, len(chain.Segments))
	for i, seg := range chain.Segments {
		if err := ctx.Err(); err != nil {
			return ProfileRecord{}, err
		}
		if _, ok := seg.(LineSeg); !ok {
			return ProfileRecord{}, fmt.Errorf(`%w: the open walk requires line-only axis-parallel segments`, ErrUnsupported)
		}
		w, err := walkOf(seg, work)
		if err != nil {
			return ProfileRecord{}, err
		}
		raw[i] = sideWalk{segmentWalk: w, segs: []int{i}}
	}
	walks, err := coalesceChainWalksContext(ctx, raw)
	if err != nil {
		return ProfileRecord{}, err
	}
	dirs, err := thickenOpenDirections(walks, budget)
	if err != nil {
		return ProfileRecord{}, err
	}
	rightSide, leftSide := thickenRibbonSides(side)
	pieces := thickenRibbonLoop(
		thickenRibbonCopyOf(walks, dirs, rightSide),
		thickenRibbonCopyOf(walks, dirs, leftSide),
	)
	limit := floatRat(amount)
	if limit == nil {
		return ProfileRecord{}, fmt.Errorf(`%w: the thicken offset is not finite`, ErrUnsupported)
	}
	if err := thickenPiecesIntervalClear(ctx, pieces, limit, budget, nil); err != nil {
		return ProfileRecord{}, err
	}
	section, err := thickenRibbonSection(pieces, limit)
	if err != nil {
		return ProfileRecord{}, err
	}
	entries, err := buildSegEntriesBudget(budget, []LoopRecord{section.Outer})
	if err != nil {
		return ProfileRecord{}, err
	}
	if err := thickenAuditRefusal(crossingAuditBudget(budget, entries)); err != nil {
		return ProfileRecord{}, err
	}
	return section, nil
}
