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
// segments bound with every such junction closed by the straight chord
// between the two denoted points, and the segment sums omit that chord, so
// each one is charged at its largest possible contribution
// (momentregion.State.ChargeJunction). A loop of one segment is a whole
// closed curve and states no junction.
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
