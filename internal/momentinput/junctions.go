package momentinput

import (
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/momentline"
	"github.com/lestrrat-3d/decad/internal/momentregion"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// segmentEnds is one recorded segment's walk start and end as held floats,
// each beside a proven per-component bound on its distance from the point the
// record denotes there. exactStart and exactEnd are those denoted points
// themselves where the record states them as exact rationals — a line's lerp
// at its recorded parameter, a free-form chain's end control point — and nil
// for a circular walk, whose ends carry trigonometry.
type segmentEnds struct {
	start, end           Point2
	startBound, endBound proofbound.WalkEndBound
	exactStart, exactEnd *freeform.RatPoint
}

// lineExactEnds states a line's two denoted walk ends exactly (momentline.RatLerp).
func lineExactEnds(line LineSeg) (*freeform.RatPoint, *freeform.RatPoint) {
	at := func(t float64) *freeform.RatPoint {
		u := momentline.RatLerp(line.Start.U, line.End.U, t)
		v := momentline.RatLerp(line.Start.V, line.End.V, t)
		if u == nil || v == nil {
			return nil
		}
		return &freeform.RatPoint{U: u, V: v}
	}
	return at(line.TStart), at(line.TEnd)
}

// copyRatPoint detaches a converted chain's control point from the chain,
// which the moments pass later shifts in place.
func copyRatPoint(p freeform.RatPoint) *freeform.RatPoint {
	return &freeform.RatPoint{U: new(big.Rat).Set(p.U), V: new(big.Rat).Set(p.V)}
}

// endsOfWalk reads a segment's ends off its walk (survey2d.SegmentWalk's
// StartBound/EndBound), each measured against the point the record denotes
// there: an arc's natural t == 1 end also carries the radial residual between
// the recorded End and Start's radius (boundarywalk.DenotedStartBound and
// DenotedEndBound).
func endsOfWalk(segment CurveSegment, walk survey2d.SegmentWalk) segmentEnds {
	ends := segmentEnds{
		start:      Point2{U: walk.StartU, V: walk.StartV},
		end:        Point2{U: walk.EndU, V: walk.EndV},
		startBound: boundarywalk.DenotedStartBound(segment, walk),
		endBound:   boundarywalk.DenotedEndBound(segment, walk),
	}
	if line, ok := segment.(LineSeg); ok {
		ends.exactStart, ends.exactEnd = lineExactEnds(line)
	}
	return ends
}

// chargeLoopJunctions charges every junction of one recorded loop at which
// the walk ending there and the walk starting there need not denote the same
// point (docs/evaluator-design.md §4). A cut bound is one such place: each
// side denotes its own entity at its own recorded parameter, and neither
// parameter is the exact crossing, so the two denoted points differ by the
// cut parameters' own error. The region a loop denotes is the one its
// segments bound with every such junction closed between the two denoted
// points, and the segment sums omit that closing path, so each one is charged
// at its largest possible contribution (momentregion.State.ChargeJunction).
// Two line fragments close through the exact crossing of their supports where
// that crossing lies near the junction (lineCorner), so a line-only region is
// the polygon sketch arranged; every other junction closes with the straight
// chord. A loop of one segment is a whole closed curve and states no
// junction.
func chargeLoopJunctions(ig *Integrals, loop LoopRecord, ends []segmentEnds, anchor Point2, order freeform.MomentIntegralOrder) {
	n := len(loop.Segments)
	if n < 2 || len(ends) != n {
		return
	}
	for i := range n {
		j := (i + 1) % n
		if sameDenotedJunction(loop.Segments[i], loop.Segments[j], ends[i], ends[j]) {
			continue
		}
		if corner, ok := lineCorner(loop.Segments[i], loop.Segments[j], ends[i].exactEnd, ends[j].exactStart); ok {
			chargeExactLeg(ig, *ends[i].exactEnd, corner, anchor, order)
			chargeExactLeg(ig, corner, *ends[j].exactStart, anchor, order)
			continue
		}
		p, pb := ends[i].end, ends[i].endBound
		q, qb := ends[j].start, ends[j].startBound
		gap := pointGapUpper(p, pb, q, qb)
		anchorReach := math.Max(pointReachUpper(p, pb, anchor), pointReachUpper(q, qb, anchor))
		originReach := math.Max(pointReachUpper(p, pb, Point2{}), pointReachUpper(q, qb, Point2{}))
		var chord *momentregion.ExactChord
		if ends[i].exactEnd != nil && ends[j].exactStart != nil {
			chord = &momentregion.ExactChord{From: *ends[i].exactEnd, To: *ends[j].exactStart}
		}
		ig.state().ChargeJunction(gap, anchorReach, originReach, chord, anchor, order)
	}
}

// sameDenotedJunction reports whether two consecutive walks provably meet at
// one denoted point, so the junction needs no chord. Two ends meet when both
// are stated exactly — zero bounds — at the same coordinate, which is every
// junction of two natural line ends, or when both exact rationals agree. A
// circle cut at its own seam meets itself too (boundarywalk.SameCircleSeam).
func sameDenotedJunction(prev, next CurveSegment, prevEnds, nextEnds segmentEnds) bool {
	zero := proofbound.WalkEndBound{}
	if prevEnds.endBound == zero && nextEnds.startBound == zero && prevEnds.end == nextEnds.start {
		return true
	}
	if p, q := prevEnds.exactEnd, nextEnds.exactStart; p != nil && q != nil {
		return p.U.Cmp(q.U) == 0 && p.V.Cmp(q.V) == 0
	}
	return boundarywalk.SameCircleSeam(prev, next)
}

// pointGapUpper bounds the distance between the two points p and q denote,
// each within its own per-component bound of its held coordinate. A
// non-finite bound answers +Inf.
func pointGapUpper(p Point2, pb proofbound.WalkEndBound, q Point2, qb proofbound.WalkEndBound) float64 {
	du := componentSpan(p.U, q.U, pb.U, qb.U)
	dv := componentSpan(p.V, q.V, pb.V, qb.V)
	if du == nil || dv == nil {
		return math.Inf(1)
	}
	return proofbound.RatSqrtUp(new(big.Rat).Add(new(big.Rat).Mul(du, du), new(big.Rat).Mul(dv, dv)))
}

// pointReachUpper bounds the distance from x to the point p denotes, within
// pb of its held coordinate. A non-finite bound answers +Inf.
func pointReachUpper(p Point2, pb proofbound.WalkEndBound, x Point2) float64 {
	du := componentSpan(p.U, x.U, pb.U, 0)
	dv := componentSpan(p.V, x.V, pb.V, 0)
	if du == nil || dv == nil {
		return math.Inf(1)
	}
	return proofbound.RatSqrtUp(new(big.Rat).Add(new(big.Rat).Mul(du, du), new(big.Rat).Mul(dv, dv)))
}

// componentSpan is |a − b| + ba + bb over exact rationals, or nil where any
// operand is not finite.
func componentSpan(a, b, ba, bb float64) *big.Rat {
	ra, rb := proofarith.FloatRat(a), proofarith.FloatRat(b)
	rba, rbb := proofarith.FloatRat(math.Abs(ba)), proofarith.FloatRat(math.Abs(bb))
	if ra == nil || rb == nil || rba == nil || rbb == nil {
		return nil
	}
	d := new(big.Rat).Sub(ra, rb)
	d.Abs(d)
	return d.Add(d, rba.Add(rba, rbb))
}

// cornerReach is how far, as a multiple of a junction's own gap, the path
// through a line corner may run: the corner closes a junction only where the
// two legs to it together measure at most cornerReach times the straight
// chord between the junction's two ends. The ratio exceeds it only where the
// two supports meet at well under a degree, and there the chord closes the
// junction instead.
const cornerReach = 1024

// lineCorner is the exact crossing of the two supporting lines of a junction
// between two line fragments, p the end of prev and q the start of next, each
// the exact point its record denotes (docs/evaluator-design.md §4). It is
// stated over exact rationals from the recorded endpoints, so a T-junction —
// one line ending on another — closes at the same exact crossing as two
// fragments cut at it. It reports false where either segment is not a line,
// either end has no exact rational, the supports are parallel, or the corner
// is not provably within cornerReach of the junction: the path p → corner → q
// is longer than cornerReach times |pq|, each leg rounded up and the chord
// rounded down.
func lineCorner(prev, next CurveSegment, p, q *freeform.RatPoint) (freeform.RatPoint, bool) {
	a, ok := prev.(LineSeg)
	if !ok || p == nil || q == nil {
		return freeform.RatPoint{}, false
	}
	b, ok := next.(LineSeg)
	if !ok {
		return freeform.RatPoint{}, false
	}
	corner, ok := supportCrossing(a, b)
	if !ok {
		return freeform.RatPoint{}, false
	}
	legs := proofbound.AbsSumUpper(ratDistanceUpper(*p, corner), ratDistanceUpper(corner, *q))
	chord := proofbound.RatSqrtDown(ratDistanceSquared(*p, *q))
	if proofbound.IsNonFinite(legs) || legs > cornerReach*chord {
		return freeform.RatPoint{}, false
	}
	return corner, true
}

// supportCrossing is the exact crossing of the lines through a's and b's
// recorded endpoints, or false where they are parallel or a field is not
// finite.
func supportCrossing(a, b LineSeg) (freeform.RatPoint, bool) {
	rats := make([]*big.Rat, 8)
	for i, x := range []float64{a.Start.U, a.Start.V, a.End.U, a.End.V, b.Start.U, b.Start.V, b.End.U, b.End.V} {
		if rats[i] = proofarith.FloatRat(x); rats[i] == nil {
			return freeform.RatPoint{}, false
		}
	}
	au, av := rats[0], rats[1]
	du, dv := new(big.Rat).Sub(rats[2], au), new(big.Rat).Sub(rats[3], av)
	eu, ev := new(big.Rat).Sub(rats[6], rats[4]), new(big.Rat).Sub(rats[7], rats[5])
	det := new(big.Rat).Sub(new(big.Rat).Mul(du, ev), new(big.Rat).Mul(dv, eu))
	if det.Sign() == 0 {
		return freeform.RatPoint{}, false
	}
	// a.Start + s·d meets b.Start + r·e at s = ((b.Start − a.Start) × e) / (d × e).
	wu, wv := new(big.Rat).Sub(rats[4], au), new(big.Rat).Sub(rats[5], av)
	s := new(big.Rat).Sub(new(big.Rat).Mul(wu, ev), new(big.Rat).Mul(wv, eu))
	s.Quo(s, det)
	return freeform.RatPoint{
		U: new(big.Rat).Add(au, new(big.Rat).Mul(s, du)),
		V: new(big.Rat).Add(av, new(big.Rat).Mul(s, dv)),
	}, true
}

// chargeExactLeg charges one straight leg of a closing path whose two ends are
// exact rationals: its exact integral joins the rational sum, and every held
// field is widened by the leg's largest contribution
// (momentregion.State.ChargeJunction).
func chargeExactLeg(ig *Integrals, from, to freeform.RatPoint, anchor Point2, order freeform.MomentIntegralOrder) {
	gap := ratDistanceUpper(from, to)
	anchorReach := math.Inf(1)
	if au, av := proofarith.FloatRat(anchor.U), proofarith.FloatRat(anchor.V); au != nil && av != nil {
		at := freeform.RatPoint{U: au, V: av}
		anchorReach = math.Max(ratDistanceUpper(from, at), ratDistanceUpper(to, at))
	}
	origin := freeform.RatPoint{U: new(big.Rat), V: new(big.Rat)}
	originReach := math.Max(ratDistanceUpper(from, origin), ratDistanceUpper(to, origin))
	ig.state().ChargeJunction(gap, anchorReach, originReach, &momentregion.ExactChord{From: from, To: to}, anchor, order)
}

// ratDistanceSquared is |p − q|² over exact rationals.
func ratDistanceSquared(p, q freeform.RatPoint) *big.Rat {
	du, dv := new(big.Rat).Sub(p.U, q.U), new(big.Rat).Sub(p.V, q.V)
	return du.Add(du.Mul(du, du), dv.Mul(dv, dv))
}

// ratDistanceUpper is |p − q| rounded up.
func ratDistanceUpper(p, q freeform.RatPoint) float64 {
	return proofbound.RatSqrtUp(ratDistanceSquared(p, q))
}
