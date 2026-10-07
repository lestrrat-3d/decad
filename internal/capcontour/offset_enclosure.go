package capcontour

import (
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// WalkPointEnclosure lifts a walk endpoint to the box its own stated bound
// allows. A recorded endpoint states zero and lifts to its exact point.
func WalkPointEnclosure(u, v float64, bound proofbound.WalkEndBound) (Point, bool) {
	p, ok := ExactPoint(u, v)
	allow := proofbound.WalkEndBoundAllow(bound)
	if !ok || proofbound.IsNonFinite(allow) {
		return Point{}, false
	}
	if allow == 0 {
		return p, true
	}
	ra := proofarith.FloatRat(allow)
	widen := func(c proofbound.RatInterval) proofbound.RatInterval {
		return proofbound.Interval(new(big.Rat).Sub(c.Lo, ra), new(big.Rat).Add(c.Hi, ra))
	}
	return Point{U: widen(p.U), V: widen(p.V)}, true
}

// ivUnitOf encloses the unit vector of every vector its argument encloses.
func ivUnitOf(p Point) (Point, bool) {
	l, ok := survey2d.IntervalSqrt(proofbound.IntervalAdd(survey2d.IntervalSquare(p.U), survey2d.IntervalSquare(p.V)))
	if !ok || l.Lo.Sign() <= 0 {
		return Point{}, false
	}
	u, okU := survey2d.IntervalQuo(p.U, l)
	v, okV := survey2d.IntervalQuo(p.V, l)
	return Point{U: u, V: v}, okU && okV
}

// walkTangentEnclosure encloses a walk's unit travel tangent at one end: a
// line's chord direction, or a circle's radius at that end turned a quarter in
// the walk's sense.
func walkTangentEnclosure(w survey2d.SideWalk, atEnd bool) (Point, bool) {
	start, okS := WalkPointEnclosure(w.StartU, w.StartV, w.StartBound)
	end, okE := WalkPointEnclosure(w.EndU, w.EndV, w.EndBound)
	if !okS || !okE {
		return Point{}, false
	}
	switch {
	case w.IsLine():
		return ivUnitOf(Point{U: proofbound.IntervalSub(end.U, start.U), V: proofbound.IntervalSub(end.V, start.V)})
	case w.IsCircular():
		c, ok := ExactPoint(w.CU, w.CV)
		if !ok {
			return Point{}, false
		}
		p := start
		if atEnd {
			p = end
		}
		ru, rv := proofbound.IntervalSub(p.U, c.U), proofbound.IntervalSub(p.V, c.V)
		if w.Th1 > w.Th0 {
			return ivUnitOf(Point{U: proofbound.IntervalNeg(rv), V: ru})
		}
		return ivUnitOf(Point{U: rv, V: proofbound.IntervalNeg(ru)})
	default:
		return Point{}, false
	}
}

// OffsetFootEnclosure encloses corner + amount·n̂, n̂ the walk's left unit
// normal at that end — the point the float build spells v + s·t·(−ty, tx).
func OffsetFootEnclosure(corner Point, w survey2d.SideWalk, atEnd bool, amount proofbound.RatInterval) (Point, bool) {
	tan, ok := walkTangentEnclosure(w, atEnd)
	if !ok {
		return Point{}, false
	}
	return Point{
		U: proofbound.IntervalAdd(corner.U, proofbound.IntervalMul(amount, proofbound.IntervalNeg(tan.V))),
		V: proofbound.IntervalAdd(corner.V, proofbound.IntervalMul(amount, tan.U)),
	}, true
}

// OffsetCarrierEnclosure encloses a walk's offset carrier over every signed
// offset in amount: the line through the start moved along the left normal,
// or the concentric circle (offsetCarrier's two shapes).
func OffsetCarrierEnclosure(w survey2d.SideWalk, amount proofbound.RatInterval) (Carrier, bool) {
	if w.IsCircular() {
		r, ok := OffsetCircleRadius(w, amount)
		c, okC := ExactPoint(w.CU, w.CV)
		return Carrier{C: c, R: r}, ok && okC
	}
	if !w.IsLine() {
		return Carrier{}, false
	}
	start, okS := WalkPointEnclosure(w.StartU, w.StartV, w.StartBound)
	dir, okD := walkTangentEnclosure(w, false)
	if !okS || !okD {
		return Carrier{}, false
	}
	p := Point{
		U: proofbound.IntervalAdd(start.U, proofbound.IntervalMul(amount, proofbound.IntervalNeg(dir.V))),
		V: proofbound.IntervalAdd(start.V, proofbound.IntervalMul(amount, dir.U)),
	}
	return Carrier{IsLine: true, P: p, Dir: dir}, true
}

// OffsetCircleRadius encloses offsetRadius's R − insideSign·(s·t) over every
// signed offset in amount, R the walk's radius widened by its own bracket.
func OffsetCircleRadius(w survey2d.SideWalk, amount proofbound.RatInterval) (proofbound.RatInterval, bool) {
	rr, rb := proofarith.FloatRat(w.Radius), proofarith.FloatRat(w.RadiusBound)
	if rr == nil || rb == nil || rb.Sign() < 0 {
		return proofbound.RatInterval{}, false
	}
	inside := big.NewRat(1, 1)
	if w.Th1 < w.Th0 { // a clockwise walk has its material outside the circle
		inside = big.NewRat(-1, 1)
	}
	base := proofbound.Interval(new(big.Rat).Sub(rr, rb), new(big.Rat).Add(rr, rb))
	r := proofbound.IntervalSub(base, proofbound.IntervalScale(amount, inside))
	if r.Lo.Sign() <= 0 {
		return proofbound.RatInterval{}, false
	}
	return r, true
}

// CircularWalkEndGap bounds how far either end of a circular walk sits off
// the circle its walk radius brackets, measured radially.
func CircularWalkEndGap(w survey2d.SideWalk) (float64, bool) {
	c, okC := ExactPoint(w.CU, w.CV)
	r, okR := OffsetCircleRadius(w, proofbound.PointInterval(new(big.Rat)))
	if !okC || !okR {
		return 0, false
	}
	gap := new(big.Rat)
	reach := func(u, v float64, bound proofbound.WalkEndBound) bool {
		p, ok := WalkPointEnclosure(u, v, bound)
		if !ok {
			return false
		}
		d, ok := survey2d.IntervalSqrt(proofbound.IntervalAdd(survey2d.IntervalSquare(proofbound.IntervalSub(p.U, c.U)), survey2d.IntervalSquare(proofbound.IntervalSub(p.V, c.V))))
		if !ok {
			return false
		}
		for _, x := range []*big.Rat{new(big.Rat).Sub(d.Hi, r.Lo), new(big.Rat).Sub(r.Hi, d.Lo)} {
			if x.Cmp(gap) > 0 {
				gap = x
			}
		}
		return true
	}
	if !reach(w.StartU, w.StartV, w.StartBound) || !reach(w.EndU, w.EndV, w.EndBound) {
		return 0, false
	}
	return proofbound.RatFloatUp(gap), true
}
