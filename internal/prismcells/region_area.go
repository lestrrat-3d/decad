package prismcells

import (
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/circularbounds"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/momentline"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
)

// LoopAreaEnclosure encloses the signed area (positive counter-clockwise) of
// the section S* a recorded loop denotes (agent-docs/prism-boolean-design.md Part K
// §5.3, "Selection and merge"): the sum of every segment's area term about
// one anchor, a line's exact and a circular segment's from
// circularbounds.AreaInterval, widened by the area its junction gaps can
// move and by proofbound.SectionDisplacementArea of hLoop. ok=false means no
// enclosure.
func LoopAreaEnclosure(budget *proofbound.WorkBudget, loop sectionrecord.LoopRecord, hLoop float64) (proofbound.RatInterval, bool, error) {
	var zero proofbound.RatInterval
	if len(loop.Segments) == 0 || proofbound.IsNonFinite(hLoop) {
		return zero, false, nil
	}
	anchor := sectionrecord.Point2{}
	total := proofbound.PointInterval(new(big.Rat))
	widen := 0.0
	perimeter := 0.0
	type end struct {
		u, v     float64
		allowReg float64
	}
	var starts, ends []end
	for _, seg := range loop.Segments {
		if err := budget.Step(); err != nil {
			return zero, false, err
		}
		w, err := boundarywalk.WalkOf(seg, nil)
		if err != nil {
			return zero, false, nil //nolint:nilerr // no walk: no enclosure
		}
		starts = append(starts, end{w.StartU, w.StartV, proofbound.WalkEndBoundAllow(w.StartBound)})
		ends = append(ends, end{w.EndU, w.EndV, proofbound.WalkEndBoundAllow(w.EndBound)})
		switch s := seg.(type) {
		case sectionrecord.LineSeg:
			u0 := momentline.RatLerp(s.Start.U, s.End.U, s.TStart)
			v0 := momentline.RatLerp(s.Start.V, s.End.V, s.TStart)
			u1 := momentline.RatLerp(s.Start.U, s.End.U, s.TEnd)
			v1 := momentline.RatLerp(s.Start.V, s.End.V, s.TEnd)
			if u0 == nil || v0 == nil || u1 == nil || v1 == nil {
				return zero, false, nil
			}
			term := new(big.Rat).Sub(new(big.Rat).Mul(u0, v1), new(big.Rat).Mul(u1, v0))
			term.Mul(term, big.NewRat(1, 2))
			total = proofbound.IntervalAdd(total, proofbound.PointInterval(term))
			perimeter = proofbound.AbsSumUpper(perimeter, math.Hypot(s.End.U-s.Start.U, s.End.V-s.Start.V)*(1+0x1p-50))
		case sectionrecord.CircleSeg, sectionrecord.ArcSeg:
			var cs circularbounds.CurveSegment
			if c, isCircle := seg.(sectionrecord.CircleSeg); isCircle {
				cs = circularbounds.CircleSeg{Center: circularbounds.Point2(c.Center), Radius: c.Radius,
					CCW: c.CCW, TStart: c.TStart, TEnd: c.TEnd}
			} else {
				a, isArc := seg.(sectionrecord.ArcSeg)
				if !isArc {
					return zero, false, nil
				}
				cs = circularbounds.ArcSeg{Center: circularbounds.Point2(a.Center), Start: circularbounds.Point2(a.Start),
					End: circularbounds.Point2(a.End), TStart: a.TStart, TEnd: a.TEnd}
			}
			iv, ok := circularbounds.AreaInterval(cs, circularbounds.Point2(anchor))
			if !ok {
				return zero, false, nil
			}
			total = proofbound.IntervalAdd(total, iv)
			_, hi, ok := fullLengthBounds(seg)
			if !ok {
				return zero, false, nil
			}
			perimeter = proofbound.AbsSumUpper(perimeter, hi)
		default:
			return zero, false, nil
		}
	}
	n := len(loop.Segments)
	if n > 1 {
		for i := range n {
			p, q := ends[i], starts[(i+1)%n]
			gap := proofbound.AbsSumUpper(math.Hypot(p.u-q.u, p.v-q.v)*(1+0x1p-50), p.allowReg, q.allowReg)
			reach := proofbound.AbsSumUpper(math.Max(math.Hypot(p.u, p.v), math.Hypot(q.u, q.v)), gap)
			widen = proofbound.AbsSumUpper(widen, proofbound.ProductUpper(gap, reach))
			// Two lines close through their supports' corner where the
			// evaluator does (momentinput.JunctionCorner): the triangle the
			// chord leaves out adds up to half the product of its legs.
			tri, ok := cornerTriangleUpper(loop.Segments[i], loop.Segments[(i+1)%n])
			if !ok {
				return zero, false, nil
			}
			widen = proofbound.AbsSumUpper(widen, tri)
		}
	}
	widen = proofbound.AbsSumUpper(widen, proofbound.SectionDisplacementArea(hLoop, n, perimeter))
	w := proofarith.FloatRat(widen)
	if w == nil {
		return zero, false, nil
	}
	return proofbound.Interval(new(big.Rat).Sub(total.Lo, w), new(big.Rat).Add(total.Hi, w)), true, nil
}

// cornerTriangleUpper bounds the area between a junction's chord and the
// closing path through the supports' corner, ½·|p − corner|·|corner − q|, and
// is zero where the junction closes with the chord. ok=false is an end the
// path cannot be stated from.
func cornerTriangleUpper(prev, next sectionrecord.CurveSegment) (float64, bool) {
	_, l1 := prev.(sectionrecord.LineSeg)
	_, l2 := next.(sectionrecord.LineSeg)
	if !l1 || !l2 {
		return 0, true
	}
	p, ok1 := segmentJunctionEnd(prev, true)
	q, ok2 := segmentJunctionEnd(next, false)
	if !ok1 || !ok2 {
		return 0, false
	}
	corner, ok := momentinput.JunctionCorner(prev, next, &p.pt, &q.pt)
	if !ok {
		return 0, true
	}
	return proofbound.ProductUpper(ratDistUpper(p.pt, corner), ratDistUpper(corner, q.pt)) / 2, true
}
