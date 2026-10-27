package capband

import (
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/capcontour"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// This file is agent-docs/draft-design.md §9.1.1: the near-tangent join of a
// draft band, a line–arc corner the build reads as G1 whose exact tangents
// turn by a sliver θ. Its true corner m(t), where the moved line meets the
// moved circle, leaves the radial foot f(t) the ideal polyhedron B1 holds
// there, and the slice-wise proof charges the gap instead of refusing it.
// Every test here is exact rational arithmetic over the record's floats, and
// every published number is rounded upward from a rational.

// SharpAmounts states a draft band's per-walk offset amounts for loop li,
// walk i moved amounts[i] into the material (zero for a wall a subset draft
// keeps), and the half-width of every nonzero amount's span. A chamfer band
// passes none: its offset family is the shell's (agent-docs/modify-design.md
// §7), whose reflex sliver takes a connector arc the near-tangent proof does
// not follow.
type SharpAmounts func(li int, walks []survey2d.SideWalk) ([]float64, float64)

// NearTangentCharge is one corner's terms in the near-tangent volume charge:
// Rho bounds |m(t) − f(t)| at every level, and Eps bounds the distance of f(t)
// from the moved line. Both are zero at a line-line miter, at an exactly
// tangent join and at a corner neither of whose walks moves.
type NearTangentCharge struct {
	Rho, Eps float64
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
// as agent-docs/draft-design.md §9.1.1's near-tangent join. prevAmount and
// curAmount are the two walks' offset amounts and amountDelta the half-width
// of a nonzero amount's span. It returns the corner's charge, or the reason
// and hint of the refusal: a corner whose recorded ends differ or whose exact
// tangents do not run the same way keeps the exact-tangent refusal, and a
// join of two circular walls, a turn past sin²θ = 2⁻⁶⁴, two walks moving by
// different amounts, or a moved circle within the charge of the corner each
// refuses by name.
func NearTangentCorner(prev, cur sectionrecord.CurveSegment, prevAmount, curAmount, amountDelta float64) (NearTangentCharge, string, string) {
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
	if !ok {
		return NearTangentCharge{}, `a near-tangent join between two circular walls, which the near-tangent corner charge does not cover`, ""
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
	charge, ok := nearTangentCharge(arc, v, sinSq, curAmount, amountDelta)
	if !ok {
		return NearTangentCharge{}, `a near-tangent join whose moved circle comes within the near-tangent corner charge of the corner`, ""
	}
	return charge, "", ""
}

// nearTangentCharge composes the corner's ρ and ε (agent-docs/draft-design.md
// §9.1.1, Lemma 1) from the exact sin²θ, the corner v on arc's circle and the
// common amount, after checking condition 6: the moved radius r stays above
// ρ + R·sin²θ at every level, which keeps the miter's discriminant positive
// and the arc from f to m under a quarter turn. ok is false where it does not,
// or where a number does not lift.
func nearTangentCharge(arc sectionrecord.ArcSeg, v [2]*big.Rat, sinSq *big.Rat, amount, amountDelta float64) (NearTangentCharge, bool) {
	held := ratAbs(proofarith.FloatRat(amount))
	width := new(big.Rat)
	if amount != 0 {
		width = proofarith.FloatRat(amountDelta)
	}
	if held == nil || width == nil || width.Sign() < 0 {
		return NearTangentCharge{}, false
	}
	// t spans [min(0, amount − width), max(0, amount + width)] over the sweep,
	// so |t| <= reach at every level.
	reach := new(big.Rat).Add(held, width)
	sinUp := proofarith.FloatRat(proofbound.RatSqrtUp(sinSq))
	if sinUp == nil {
		return NearTangentCharge{}, false
	}
	rho := new(big.Rat).Mul(reach, sinUp)
	rho.Mul(rho, new(big.Rat).Add(big.NewRat(1, 1), sinSq))
	eps := new(big.Rat).Mul(reach, sinSq)

	center, ok := ratPoint2(arc.Center)
	if !ok {
		return NearTangentCharge{}, false
	}
	du := new(big.Rat).Sub(v[0], center[0])
	dv := new(big.Rat).Sub(v[1], center[1])
	radiusSq := new(big.Rat).Add(new(big.Rat).Mul(du, du), new(big.Rat).Mul(dv, dv))
	radiusLo := proofarith.FloatRat(proofbound.RatSqrtDown(radiusSq))
	radiusHi := proofarith.FloatRat(proofbound.RatSqrtUp(radiusSq))
	signed := proofarith.FloatRat(amount)
	if radiusLo == nil || radiusHi == nil || signed == nil {
		return NearTangentCharge{}, false
	}
	// A counter-clockwise walk holds its material inside, so its circle moves
	// to R − t; a clockwise one moves to R + t. The radius falls by at most the
	// largest inward step any level takes.
	inside := big.NewRat(1, 1)
	if arc.TStart == 1 {
		inside = big.NewRat(-1, 1)
	}
	shrink := new(big.Rat)
	for _, t := range []*big.Rat{new(big.Rat).Add(signed, width), new(big.Rat).Sub(signed, width)} {
		if s := new(big.Rat).Mul(inside, t); s.Cmp(shrink) > 0 {
			shrink = s
		}
	}
	movedLo := new(big.Rat).Sub(radiusLo, shrink)
	need := new(big.Rat).Add(rho, new(big.Rat).Mul(radiusHi, sinSq))
	if movedLo.Cmp(need) <= 0 {
		return NearTangentCharge{}, false
	}
	return NearTangentCharge{Rho: proofbound.RatFloatUp(rho), Eps: proofbound.RatFloatUp(eps)}, true
}

// NearTangentCharges reads every corner of one coalesced loop of a draft band
// the occupied-volume admission admitted, walk i moved amounts[i] and every
// nonzero amount spanning amountDelta: corner i, at walk i's start, carries a
// near-tangent join's charge, and a line-line miter or an exactly tangent join
// carries none. A lone closed circle has no corner and returns nil. A corner
// NearTangentCorner refuses is ErrUnsupported, which the admission rules out
// before any mesh reads this.
func NearTangentCharges(budget *proofbound.WorkBudget, loop sectionrecord.LoopRecord, walks []survey2d.SideWalk, amounts []float64, amountDelta float64) ([]NearTangentCharge, error) {
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
		prevSeg, err := sectionrecord.NormalizeSegment(loop.Segments[prev.Segs[len(prev.Segs)-1]])
		if err != nil {
			return nil, err
		}
		curSeg, err := sectionrecord.NormalizeSegment(loop.Segments[cur.Segs[0]])
		if err != nil {
			return nil, err
		}
		if JoinIsG1(prevSeg, curSeg) {
			continue
		}
		charge, why, _ := NearTangentCorner(prevSeg, curSeg, amounts[p], amounts[i], amountDelta)
		if why != "" {
			return nil, fmt.Errorf(`%w: %s at %s`, decaderr.ErrUnsupported, why, cornerAt(prev, cur))
		}
		out[i] = charge
	}
	return out, nil
}

// lineArcPair returns the ArcSeg of a corner joining one LineSeg and one
// ArcSeg, in either order, and ok == false for any other pair.
func lineArcPair(prev, cur sectionrecord.CurveSegment) (sectionrecord.ArcSeg, bool) {
	if _, ok := prev.(sectionrecord.LineSeg); ok {
		arc, isArc := cur.(sectionrecord.ArcSeg)
		return arc, isArc
	}
	if _, ok := cur.(sectionrecord.LineSeg); ok {
		arc, isArc := prev.(sectionrecord.ArcSeg)
		return arc, isArc
	}
	return sectionrecord.ArcSeg{}, false
}

// ratDot is a·b over the rationals.
func ratDot(a, b [2]*big.Rat) *big.Rat {
	return new(big.Rat).Add(new(big.Rat).Mul(a[0], b[0]), new(big.Rat).Mul(a[1], b[1]))
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
// (agent-docs/draft-design.md §9.1.1) for straight walk i of a draft loop with a
// near-tangent join at an end: the true edge runs forward from its start corner
// to its end corner at every level. B1's two corner points move affinely
// between the near and far sections, and each true corner lies within its ρ of
// B1's (rhoStart, rhoEnd; zero at an exact end), so the edge survives when its
// advance along the recorded direction exceeds rhoStart + rhoEnd at both
// sections. The near section's advance is the recorded length, compared
// exactly; the far section's corners are enclosed over the amount span, the
// circle's radial foot at a circular neighbour and the moved carriers' miter
// at a straight one, and the enclosure's least advance is compared against the
// upward root of the recorded length. It returns the refusal's reason, or ""
// when the wall survives.
func NearTangentWallRefusal(loop sectionrecord.LoopRecord, walks []survey2d.SideWalk, i int, amounts []float64, amountDelta, rhoStart, rhoEnd float64) string {
	const why = `a straight wall at a near-tangent join whose advance at the near or far section does not exceed the near-tangent corner charge, so its edge is not proven to survive every level`
	n := len(walks)
	w := walks[i]
	if w.IsCircular() || len(amounts) != n || len(w.Segs) == 0 {
		return why
	}
	first, err := sectionrecord.NormalizeSegment(loop.Segments[w.Segs[0]])
	if err != nil {
		return why
	}
	last, err := sectionrecord.NormalizeSegment(loop.Segments[w.Segs[len(w.Segs)-1]])
	if err != nil {
		return why
	}
	fe, okF := joinEnds(first)
	le, okL := joinEnds(last)
	rs, re := proofarith.FloatRat(rhoStart), proofarith.FloatRat(rhoEnd)
	if !okF || !okL || rs == nil || re == nil {
		return why
	}
	dir := [2]*big.Rat{new(big.Rat).Sub(le.end[0], fe.start[0]), new(big.Rat).Sub(le.end[1], fe.start[1])}
	need := new(big.Rat).Add(rs, re)
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
	p, q := (i+n-1)%n, (i+1)%n
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
