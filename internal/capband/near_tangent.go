package capband

import (
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/capcontour"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// This file is agent-docs/modify-design.md Part D §9.1.1: the near-tangent join of a
// draft or cap-loop chamfer band, a line–arc or arc–arc corner the build reads as G1 whose exact
// tangents turn by a sliver θ. Its true corner m(t), where the moved line
// meets the moved circle or the two moved circles meet, leaves the radial
// foot f(t) the ideal polyhedron B1 holds there, and the slice-wise proof
// charges the gap instead of refusing it.
// Every test here is exact rational arithmetic over the record's floats, and
// every published number is rounded upward from a rational.

// SharpAmounts states a band's per-walk offset amounts for loop li, walk i
// moved amounts[i] into the material (zero for a wall a subset draft keeps),
// and the half-width of every nonzero amount's span. A draft band passes its
// sharp rule's amounts; a cap-loop chamfer passes its setback on every walk,
// since its shell offset meets a convex sliver at the sharp corner and closes
// a reflex one with a connector arc the same charge covers
// (agent-docs/modify-design.md Part D §9.1.1, "On a cap-loop chamfer band").
type SharpAmounts func(li int, walks []survey2d.SideWalk) ([]float64, float64)

// NearTangentCharge is one corner's terms in the near-tangent volume charge:
// Rho bounds |m(t) − f(t)| at every level, and Eps bounds the distance of f(t)
// from the moved line. Both are zero at a line-line miter, at an exactly
// tangent join and at a corner neither of whose walks moves. TwoArcs marks a
// corner of two circular walls (Lemma 1′), where Rho also bounds the gap
// between the two walks' radial feet and Eps is zero, since no straight wall
// meets it.
type NearTangentCharge struct {
	Rho, Eps float64
	TwoArcs  bool
}

// notTangentWhy and notTangentHint are the refusal of a corner no rule here
// admits: neither a line-line miter, nor exactly tangent, nor a near-tangent
// join (the last only on a draft band).
const (
	notTangentWhy  = `a corner this evaluator cannot prove a line-line miter or an exactly tangent join`
	notTangentHint = `; the two recorded segments' exact tangents there are not parallel or their recorded ends differ, and a join recorded exactly tangent, its arc's centre on the line's normal through the shared end, is admitted`
)

// nearTangentSinSqLimit is the largest sin²θ a near-tangent join may turn by:
// 2⁻⁶⁴, so sin θ <= 2⁻³², under a quarter of the build's held-tangent
// dead zone. A join a sketch tangent constraint solves turns by rounding, many
// orders below it; a corner turning more was drawn as a corner.
var nearTangentSinSqLimit = new(big.Rat).SetFrac(big.NewInt(1), new(big.Int).Lsh(big.NewInt(1), 64))

// NearTangentCorner reads the corner where prev's walk ends and cur's starts,
// both normalized recorded segments the exact-tangent test JoinIsG1 rejects,
// as agent-docs/modify-design.md Part D §9.1.1's near-tangent join. prevAmount and
// curAmount are the two walks' offset amounts and amountDelta the half-width
// of a nonzero amount's span. It returns the corner's charge, or the reason
// and hint of the refusal: a corner whose recorded ends differ, whose exact
// tangents do not run the same way, or whose walks are neither a line and an
// arc nor two arcs keeps the exact-tangent refusal, and a turn past
// sin²θ = 2⁻⁶⁴, two walks moving by different amounts, or a moved circle
// within the charge of the corner each refuses by name.
func NearTangentCorner(prev, cur sectionrecord.CurveSegment, prevAmount, curAmount, amountDelta float64) (NearTangentCharge, string, string) {
	pc, okP := recordedCorner(prev)
	cc, okC := recordedCorner(cur)
	if !okP || !okC {
		return NearTangentCharge{}, notTangentWhy, notTangentHint
	}
	return nearTangentCorner(pc, cc, prevAmount, curAmount, amountDelta)
}

// nearTangentCorner is NearTangentCorner over two segments read about the
// centres they carry.
func nearTangentCorner(prev, cur cornerSegment, prevAmount, curAmount, amountDelta float64) (NearTangentCharge, string, string) {
	p, okP := joinEnds(prev)
	c, okC := joinEnds(cur)
	if !okP || !okC {
		return NearTangentCharge{}, notTangentWhy, notTangentHint
	}
	v := p.end
	if v[0].Cmp(c.start[0]) != 0 || v[1].Cmp(c.start[1]) != 0 {
		return NearTangentCharge{}, notTangentWhy, notTangentHint
	}
	a, b := p.tanEnd, c.tanStart
	if ratDot(a, b).Sign() <= 0 {
		return NearTangentCharge{}, notTangentWhy, notTangentHint
	}
	arc, ok := lineArcPair(prev, cur)
	first, second, twoArcs := arcArcPair(prev, cur)
	if !ok && !twoArcs {
		return NearTangentCharge{}, notTangentWhy, notTangentHint
	}
	if prevAmount != curAmount {
		return NearTangentCharge{}, `a near-tangent join whose two walls move by different amounts`, ""
	}
	cross := new(big.Rat).Sub(new(big.Rat).Mul(a[0], b[1]), new(big.Rat).Mul(a[1], b[0]))
	sinSq := new(big.Rat).Mul(cross, cross)
	sinSq.Quo(sinSq, new(big.Rat).Mul(ratDot(a, a), ratDot(b, b)))
	if sinSq.Cmp(nearTangentSinSqLimit) > 0 {
		return NearTangentCharge{}, `a near-tangent join whose exact turn exceeds the admitted sin²θ ≤ 2⁻⁶⁴`, ""
	}
	if twoArcs {
		charge, ok := twoArcCharge(first, second, v, sinSq, curAmount, amountDelta)
		if !ok {
			return NearTangentCharge{}, nearTangentArcsWhy, ""
		}
		return charge, "", ""
	}
	charge, ok := nearTangentCharge(arc, v, sinSq, curAmount, amountDelta)
	if !ok {
		return NearTangentCharge{}, `a near-tangent join whose moved circle comes within the near-tangent corner charge of the corner`, ""
	}
	return charge, "", ""
}

// nearTangentCharge composes the corner's ρ and ε (agent-docs/modify-design.md Part D
// §9.1.1, Lemma 1) from the exact sin²θ, the corner v on arc's circle and the
// common amount, after checking condition 6: the moved radius r stays above
// ρ + R·sin²θ at every level, which keeps the miter's discriminant positive
// and the arc from f to m under a quarter turn. ok is false where it does not,
// or where a number does not lift.
func nearTangentCharge(arc cornerSegment, v [2]*big.Rat, sinSq *big.Rat, amount, amountDelta float64) (NearTangentCharge, bool) {
	rho, eps, width, ok := nearTangentReach(sinSq, amount, amountDelta)
	if !ok {
		return NearTangentCharge{}, false
	}
	radiusHi, movedLo, ok := movedRadius(arc, v, amount, width)
	if !ok {
		return NearTangentCharge{}, false
	}
	need := new(big.Rat).Add(rho, new(big.Rat).Mul(radiusHi, sinSq))
	if movedLo.Cmp(need) <= 0 {
		return NearTangentCharge{}, false
	}
	return NearTangentCharge{Rho: proofbound.RatFloatUp(rho), Eps: proofbound.RatFloatUp(eps)}, true
}

// nearTangentReach composes ρ = T·sinUp·(1 + S) and ε = T·S
// (agent-docs/modify-design.md Part D §9.1.1) from the exact sin²θ and the common
// amount, with T = |amount| + width bounding |t| at every level, and returns
// width, the span's half-width (zero for a walk that does not move). ok is
// false where a number does not lift.
func nearTangentReach(sinSq *big.Rat, amount, amountDelta float64) (*big.Rat, *big.Rat, *big.Rat, bool) {
	held := ratAbs(proofarith.FloatRat(amount))
	width := new(big.Rat)
	if amount != 0 {
		width = proofarith.FloatRat(amountDelta)
	}
	if held == nil || width == nil || width.Sign() < 0 {
		return nil, nil, nil, false
	}
	// t spans [min(0, amount − width), max(0, amount + width)] over the sweep,
	// so |t| <= reach at every level.
	reach := new(big.Rat).Add(held, width)
	sinUp := proofarith.FloatRat(proofbound.RatSqrtUp(sinSq))
	if sinUp == nil {
		return nil, nil, nil, false
	}
	rho := new(big.Rat).Mul(reach, sinUp)
	rho.Mul(rho, new(big.Rat).Add(big.NewRat(1, 1), sinSq))
	return rho, new(big.Rat).Mul(reach, sinSq), width, true
}

// nearTangentArcsWhy is the refusal of a corner of two circular walls whose
// moved circles do not meet condition 6 (agent-docs/modify-design.md Part D §9.1.1).
const nearTangentArcsWhy = `a near-tangent join of two circular walls whose moved circles do not both clear the near-tangent corner charge`

// twoArcCharge composes the charge of a near-tangent corner where arc first
// arrives at v and arc second leaves it (agent-docs/modify-design.md Part D §9.1.1,
// Lemma 1′), after checking condition 6 for two arcs: each moved radius stays
// above ρ at every level. That keeps each arc from a radial foot to the true
// corner under a quarter turn and, with sin θ ≤ 2⁻³², the two moved circles
// meeting in two distinct points at every level. ok is false where it does
// not, or where a number does not lift.
func twoArcCharge(first, second cornerSegment, v [2]*big.Rat, sinSq *big.Rat, amount, amountDelta float64) (NearTangentCharge, bool) {
	rho, _, width, ok := nearTangentReach(sinSq, amount, amountDelta)
	if !ok {
		return NearTangentCharge{}, false
	}
	for _, arc := range []cornerSegment{first, second} {
		_, movedLo, ok := movedRadius(arc, v, amount, width)
		if !ok || movedLo.Cmp(rho) <= 0 {
			return NearTangentCharge{}, false
		}
	}
	return NearTangentCharge{Rho: proofbound.RatFloatUp(rho), TwoArcs: true}, true
}

// NearTangentCharges reads every corner of one coalesced loop of a draft band
// the occupied-volume admission admitted, walk i moved amounts[i] and every
// nonzero amount spanning amountDelta: corner i, at walk i's start, carries a
// near-tangent join's charge, and a line-line miter or an exactly tangent join
// carries none. A lone closed circle has no corner and returns nil. A corner
// NearTangentCorner refuses is ErrUnsupported, which the admission rules out
// before any mesh reads this.
//
// restate reads every off-radius arc about its restatedCentre, as a chamfer
// band's admission does (agent-docs/tessellation-reach-design.md §7.1).
func NearTangentCharges(budget *proofbound.WorkBudget, loop sectionrecord.LoopRecord, walks []survey2d.SideWalk, amounts []float64, amountDelta float64, restate bool, reflex []bool) ([]NearTangentCharge, error) {
	n := len(walks)
	if n == 1 && walks[0].Closed {
		return nil, nil
	}
	if len(amounts) != n {
		return nil, fmt.Errorf(`%w: %d offset amounts for %d walks`, decaderr.ErrDegenerate, len(amounts), n)
	}
	out := make([]NearTangentCharge, n)
	for i := range n {
		if err := budget.Step(); err != nil {
			return nil, err
		}
		p := (i + n - 1) % n
		prev, cur := walks[p], walks[i]
		if prev.IsLine() && cur.IsLine() {
			continue
		}
		if reflex != nil && reflex[i] {
			continue
		}
		prevSeg, err := sectionrecord.NormalizeSegment(loop.Segments[prev.Segs[len(prev.Segs)-1]])
		if err != nil {
			return nil, err
		}
		curSeg, err := sectionrecord.NormalizeSegment(loop.Segments[cur.Segs[0]])
		if err != nil {
			return nil, err
		}
		pc, okP := readCorner(prevSeg, restate)
		cc, okC := readCorner(curSeg, restate)
		if !okP || !okC {
			return nil, fmt.Errorf(`%w: %s at %s`, decaderr.ErrUnsupported, restateRefusal, cornerAt(prev, cur))
		}
		if joinIsG1(pc, cc) {
			continue
		}
		charge, why, _ := nearTangentCorner(pc, cc, amounts[p], amounts[i], amountDelta)
		if why != "" {
			return nil, fmt.Errorf(`%w: %s at %s`, decaderr.ErrUnsupported, why, cornerAt(prev, cur))
		}
		out[i] = charge
	}
	return out, nil
}

// lineArcPair returns the arc of a corner joining one LineSeg and one
// ArcSeg, in either order, and ok == false for any other pair.
func lineArcPair(prev, cur cornerSegment) (cornerSegment, bool) {
	if _, ok := prev.seg.(sectionrecord.LineSeg); ok {
		_, isArc := cur.seg.(sectionrecord.ArcSeg)
		return cur, isArc
	}
	if _, ok := cur.seg.(sectionrecord.LineSeg); ok {
		_, isArc := prev.seg.(sectionrecord.ArcSeg)
		return prev, isArc
	}
	return cornerSegment{}, false
}

// arcArcPair returns the two arcs of a corner joining two arcs, the one
// arriving first, and ok == false for any other pair.
func arcArcPair(prev, cur cornerSegment) (cornerSegment, cornerSegment, bool) {
	_, okP := prev.seg.(sectionrecord.ArcSeg)
	_, okC := cur.seg.(sectionrecord.ArcSeg)
	return prev, cur, okP && okC
}

// ratDot is a·b over the rationals.
func ratDot(a, b [2]*big.Rat) *big.Rat {
	return new(big.Rat).Add(new(big.Rat).Mul(a[0], b[0]), new(big.Rat).Mul(a[1], b[1]))
}

// movedRadius bounds the circle of arc through the recorded point v: R_hi, an
// upper bound on R = |v − Center|, and r_lo, a lower bound on the moved radius
// at every level of an amount within width of amount. A counter-clockwise walk
// holds its material inside, so its circle moves to R − t; a clockwise one
// moves to R + t. The radius falls by at most the largest inward step any
// level takes.
func movedRadius(arc cornerSegment, v [2]*big.Rat, amount float64, width *big.Rat) (*big.Rat, *big.Rat, bool) {
	rec, isArc := arc.seg.(sectionrecord.ArcSeg)
	center := arc.center
	if !isArc || center[0] == nil || center[1] == nil {
		return nil, nil, false
	}
	du := new(big.Rat).Sub(v[0], center[0])
	dv := new(big.Rat).Sub(v[1], center[1])
	radiusSq := new(big.Rat).Add(new(big.Rat).Mul(du, du), new(big.Rat).Mul(dv, dv))
	radiusLo := proofarith.FloatRat(proofbound.RatSqrtDown(radiusSq))
	radiusHi := proofarith.FloatRat(proofbound.RatSqrtUp(radiusSq))
	signed := proofarith.FloatRat(amount)
	if radiusLo == nil || radiusHi == nil || signed == nil {
		return nil, nil, false
	}
	inside := big.NewRat(1, 1)
	if rec.TStart == 1 {
		inside = big.NewRat(-1, 1)
	}
	shrink := new(big.Rat)
	for _, t := range []*big.Rat{new(big.Rat).Add(signed, width), new(big.Rat).Sub(signed, width)} {
		if s := new(big.Rat).Mul(inside, t); s.Cmp(shrink) > 0 {
			shrink = s
		}
	}
	return radiusHi, new(big.Rat).Sub(radiusLo, shrink), true
}

// ratAbs is |q|, or nil for a nil q.
func ratAbs(q *big.Rat) *big.Rat {
	if q == nil {
		return nil
	}
	return new(big.Rat).Abs(q)
}

// ratPoint2 lifts a recorded point to the rationals, ok == false for a
// coordinate that denotes no rational.
func ratPoint2(p sectionrecord.Point2) ([2]*big.Rat, bool) {
	u, v := proofarith.FloatRat(p.U), proofarith.FloatRat(p.V)
	return [2]*big.Rat{u, v}, u != nil && v != nil
}

// NearTangentWallRefusal checks Lemma 2's survival assumption
// (agent-docs/modify-design.md Part D §9.1.1) for walk i of a draft loop with a
// near-tangent join at an end: the true edge runs forward from its start corner
// to its end corner at every level. A circular walk reads
// nearTangentArcRefusal; for a straight one, B1's two corner points move affinely
// between the near and far sections, and each true corner lies within its ρ of
// B1's (rhoStart, rhoEnd; zero at an exact end), so the edge survives when its
// advance along the recorded direction exceeds rhoStart + rhoEnd at both
// sections. The near section's advance is the recorded length, compared
// exactly; the far section's corners are enclosed over the amount span, the
// circle's radial foot at a circular neighbour and the moved carriers' miter
// at a straight one, and the enclosure's least advance is compared against the
// upward root of the recorded length. It returns the refusal's reason, or ""
// when the wall survives.
//
// restate reads every off-radius arc about its restatedCentre
// (agent-docs/tessellation-reach-design.md §7.1). B1's foot on a restated
// circular neighbour then sits within that arc's OffRadiusShift of the foot
// the held neighbour's enclosure states, so a straight wall's charge grows by
// each restated neighbour's shift.
func NearTangentWallRefusal(loop sectionrecord.LoopRecord, walks []survey2d.SideWalk, i int, amounts []float64, amountDelta, rhoStart, rhoEnd float64, restate bool) string {
	const why = nearTangentWallWhy
	n := len(walks)
	w := walks[i]
	if len(amounts) != n || len(w.Segs) == 0 {
		return why
	}
	if w.IsCircular() {
		return nearTangentArcRefusal(loop, w, amounts[i], amountDelta, rhoStart, rhoEnd, restate)
	}
	first, err := sectionrecord.NormalizeSegment(loop.Segments[w.Segs[0]])
	if err != nil {
		return why
	}
	last, err := sectionrecord.NormalizeSegment(loop.Segments[w.Segs[len(w.Segs)-1]])
	if err != nil {
		return why
	}
	fc, okFC := recordedCorner(first)
	lc, okLC := recordedCorner(last)
	fe, okF := joinEnds(fc)
	le, okL := joinEnds(lc)
	rs, re := proofarith.FloatRat(rhoStart), proofarith.FloatRat(rhoEnd)
	if !okFC || !okLC || !okF || !okL || rs == nil || re == nil {
		return why
	}
	p, q := (i+n-1)%n, (i+1)%n
	slack := 0.0
	if restate {
		before, okB := neighbourShift(loop, walks[p], false, amounts[p], amountDelta)
		after, okA := neighbourShift(loop, walks[q], true, amounts[q], amountDelta)
		if !okB || !okA {
			return why
		}
		slack = proofbound.AbsSumUpper(before, after)
	}
	slackRat := proofarith.FloatRat(slack)
	if slackRat == nil {
		return why
	}
	dir := [2]*big.Rat{new(big.Rat).Sub(le.end[0], fe.start[0]), new(big.Rat).Sub(le.end[1], fe.start[1])}
	need := new(big.Rat).Add(new(big.Rat).Add(rs, re), slackRat)
	lengthSq := ratDot(dir, dir)
	if lengthSq.Cmp(new(big.Rat).Mul(need, need)) <= 0 {
		return why
	}
	span := func(k int) (proofbound.RatInterval, bool) {
		width := amountDelta
		if amounts[k] == 0 {
			width = 0
		}
		return capcontour.OffsetSpan(amounts[k], width)
	}
	start, okS := farCorner(walks[p], w, span, p, i, true)
	end, okE := farCorner(w, walks[q], span, i, q, false)
	lengthUp := proofarith.FloatRat(proofbound.RatSqrtUp(lengthSq))
	if !okS || !okE || lengthUp == nil {
		return why
	}
	advance := proofbound.IntervalAdd(
		proofbound.IntervalScale(proofbound.IntervalSub(end.U, start.U), dir[0]),
		proofbound.IntervalScale(proofbound.IntervalSub(end.V, start.V), dir[1]),
	)
	if advance.Lo.Cmp(new(big.Rat).Mul(need, lengthUp)) <= 0 {
		return why
	}
	return ""
}

// farCorner encloses B1's far-section point at the corner where prev arrives
// at cur over the two walks' amount spans: the radial foot of the circular
// one (atWallStart says the straight wall is cur, so the neighbour is prev)
// when the neighbour is circular, else the miter of the two moved carriers
// nearest the corner.
func farCorner(prev, cur survey2d.SideWalk, span func(int) (proofbound.RatInterval, bool), pi, ci int, atWallStart bool) (capcontour.Point, bool) {
	sp, okP := span(pi)
	sc, okC := span(ci)
	if !okP || !okC {
		return capcontour.Point{}, false
	}
	neighbour, atEnd, s := cur, false, sc
	if atWallStart {
		neighbour, atEnd, s = prev, true, sp
	}
	if neighbour.IsCircular() {
		u, v, bound := neighbour.StartU, neighbour.StartV, neighbour.StartBound
		if atEnd {
			u, v, bound = neighbour.EndU, neighbour.EndV, neighbour.EndBound
		}
		corner, ok := capcontour.WalkPointEnclosure(u, v, bound)
		if !ok {
			return capcontour.Point{}, false
		}
		return capcontour.OffsetFootEnclosure(corner, neighbour, atEnd, s)
	}
	ca, okA := capcontour.CarrierOver(prev, sp)
	cb, okB := capcontour.CarrierOver(cur, sc)
	if !okA || !okB {
		return capcontour.Point{}, false
	}
	cands, ok := capcontour.Intersect(ca, cb)
	if !ok {
		return capcontour.Point{}, false
	}
	return capcontour.Nearest(cands, cur.StartU, cur.StartV)
}

// nearTangentWallWhy is the refusal of a straight wall the survival gate does
// not prove; nearTangentArcWhy is that of a circular wall.
const (
	nearTangentWallWhy = `a straight wall at a near-tangent join whose advance at the near or far section does not exceed the near-tangent corner charge, so its edge is not proven to survive every level`
	nearTangentArcWhy  = `a circular wall at a near-tangent join whose window does not exceed the turn the near-tangent corner charge allows its ends, so its arc is not proven to survive every level`
)

// nearTangentArcRefusal is NearTangentWallRefusal for circular walk w. Each
// true end lies on the moved circle within its ρ of the wall's own radial
// foot at the recorded window's end (Lemma 1, or Lemma 1′ at a corner of two
// arcs, where B1 holds the other arc's foot instead), so it turns from that
// foot by at most 2·asin(ρ/2r) <= (π/2)·ρ/r_lo. The arc survives every level
// when those two turns together stay below the window W. W is at least π
// where the window passes a half turn, and at least |v₂ − v₁|/R otherwise (a
// chord of length c subtends 2·asin(c/2R) >= c/R), so the gate asks for
// (π/2)(ρ_start + ρ_end) < W_lo·r_lo over the rationals with π/2 rounded up
// and π rounded down.
func nearTangentArcRefusal(loop sectionrecord.LoopRecord, w survey2d.SideWalk, amount, amountDelta, rhoStart, rhoEnd float64, restate bool) string {
	first, err := sectionrecord.NormalizeSegment(loop.Segments[w.Segs[0]])
	if err != nil {
		return nearTangentArcWhy
	}
	last, err := sectionrecord.NormalizeSegment(loop.Segments[w.Segs[len(w.Segs)-1]])
	if err != nil {
		return nearTangentArcWhy
	}
	firstArc, okA := first.(sectionrecord.ArcSeg)
	arc, okR := readCorner(first, restate)
	lastC, okRL := readCorner(last, restate)
	fe, okF := joinEnds(arc)
	le, okL := joinEnds(lastC)
	rs, re := proofarith.FloatRat(rhoStart), proofarith.FloatRat(rhoEnd)
	width := new(big.Rat)
	if amount != 0 {
		width = proofarith.FloatRat(amountDelta)
	}
	if !okA || !okR || !okRL || !okF || !okL || rs == nil || re == nil || width == nil {
		return nearTangentArcWhy
	}
	v1, v2 := fe.start, le.end
	radiusHi, movedLo, ok := movedRadius(arc, v1, amount, width)
	center := arc.center
	if !ok || movedLo.Sign() <= 0 {
		return nearTangentArcWhy
	}
	a := [2]*big.Rat{new(big.Rat).Sub(v1[0], center[0]), new(big.Rat).Sub(v1[1], center[1])}
	b := [2]*big.Rat{new(big.Rat).Sub(v2[0], center[0]), new(big.Rat).Sub(v2[1], center[1])}
	cross := new(big.Rat).Sub(new(big.Rat).Mul(a[0], b[1]), new(big.Rat).Mul(a[1], b[0]))
	if firstArc.TStart == 1 {
		cross.Neg(cross)
	}
	// 1.5708 > π/2 and 3 < π.
	turn := new(big.Rat).Mul(new(big.Rat).Add(rs, re), big.NewRat(15708, 10000))
	if cross.Sign() <= 0 {
		// The window passes a half turn: W >= π > 3.
		if turn.Cmp(new(big.Rat).Mul(big.NewRat(3, 1), movedLo)) >= 0 {
			return nearTangentArcWhy
		}
		return ""
	}
	chord := [2]*big.Rat{new(big.Rat).Sub(v2[0], v1[0]), new(big.Rat).Sub(v2[1], v1[1])}
	chordLo := proofarith.FloatRat(proofbound.RatSqrtDown(ratDot(chord, chord)))
	if chordLo == nil {
		return nearTangentArcWhy
	}
	// (π/2)(ρ_start + ρ_end) < (|v₂ − v₁|/R_hi)·r_lo.
	if new(big.Rat).Mul(turn, radiusHi).Cmp(new(big.Rat).Mul(chordLo, movedLo)) >= 0 {
		return nearTangentArcWhy
	}
	return ""
}

// neighbourShift is the OffRadiusShift of the recorded segment of walk w
// that meets a straight neighbour: w's first segment when atStart, else its
// last, at the walk's own amount span. It is zero for a straight walk and for
// an arc whose ends lie on one circle.
func neighbourShift(loop sectionrecord.LoopRecord, w survey2d.SideWalk, atStart bool, amount, amountDelta float64) (float64, bool) {
	if !w.IsCircular() || len(w.Segs) == 0 {
		return 0, true
	}
	si := w.Segs[len(w.Segs)-1]
	if atStart {
		si = w.Segs[0]
	}
	seg, err := sectionrecord.NormalizeSegment(loop.Segments[si])
	if err != nil {
		return 0, false
	}
	arc, ok := seg.(sectionrecord.ArcSeg)
	if !ok {
		return 0, true
	}
	reach := math.Abs(amount)
	if amount != 0 {
		reach = proofbound.AbsSumUpper(reach, amountDelta)
	}
	return OffRadiusShift(arc, reach)
}
