package patternassembly

import (
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/circularbounds"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
)

// instanceRecord is the carrier record of a frame-keeping pattern instance of
// an undisplaced source (agent-docs/carrier-record-design.md §5.1). moved is
// the instance's recorded section, charge the motion's section charge (the
// instance's SectionDelta before it is raised) and e the largest point charge
// the motion took.
//
// Every segment and junction entry is charge, except the junction at an
// ArcSeg's End, which takes δ_End = up(3e + 4e·(R + 2e)/(R_E − 2e)) with R the
// recorded radius and R_E a lower bound on the moved |End − Center|. The
// returned SectionDelta is raised to the largest entry and every loop's
// Lemma H bound (CR4). ok=false means no record: a segment the arm cannot
// have produced, a line–line junction whose two read ends differ (its closing
// path may run through the supports' corner, which a motion's rounding
// moves), or an arc whose Lemma H condition fails.
func instanceRecord(moved momentinput.Profile, charge, e float64) (prismrecord.CarrierRecord, float64, bool) {
	cr := prismrecord.UniformCarrierRecord(moved, charge)
	loops := append([]sectionrecord.LoopRecord{moved.Outer}, moved.Holes...)
	for l, loop := range loops {
		n := len(loop.Segments)
		for j, seg := range loop.Segments {
			switch s := seg.(type) {
			case sectionrecord.LineSeg:
				if !wholeRange(s.TStart, s.TEnd) {
					return prismrecord.CarrierRecord{}, 0, false
				}
				next, isLine := loop.Segments[(j+1)%n].(sectionrecord.LineSeg)
				if isLine && (!wholeRange(next.TStart, next.TEnd) || walkEnd(s) != walkStart(next)) {
					return prismrecord.CarrierRecord{}, 0, false
				}
			case sectionrecord.CircleSeg:
				if !wholeRange(s.TStart, s.TEnd) {
					return prismrecord.CarrierRecord{}, 0, false
				}
			case sectionrecord.ArcSeg:
				if !wholeRange(s.TStart, s.TEnd) {
					return prismrecord.CarrierRecord{}, 0, false
				}
				d, ok := arcEndCharge(s, e)
				if !ok {
					return prismrecord.CarrierRecord{}, 0, false
				}
				// δ_End sits at ArcSeg.End: for a clockwise arc (TStart 1) the End is
				// the walk's start, the junction before the segment (§5.1).
				at := j
				if s.TStart == 1 {
					at = (j + n - 1) % n
				}
				cr.Junction[l][at] = math.Max(cr.Junction[l][at], d)
			default:
				return prismrecord.CarrierRecord{}, 0, false
			}
		}
	}
	delta := math.Max(math.Max(charge, cr.Max()), cr.HMax())
	if math.IsInf(delta, 0) || math.IsNaN(delta) {
		return prismrecord.CarrierRecord{}, 0, false
	}
	cr.SectionDelta = delta
	return cr, delta, true
}

func wholeRange(t0, t1 float64) bool { return (t0 == 0 && t1 == 1) || (t0 == 1 && t1 == 0) }

// arcEndCharge is δ_End for a moved arc.
func arcEndCharge(arc sectionrecord.ArcSeg, e float64) (float64, bool) {
	radius := func(p sectionrecord.Point2) *big.Rat {
		du := circularbounds.ExactCoordinateDelta(p.U, arc.Center.U)
		dv := circularbounds.ExactCoordinateDelta(p.V, arc.Center.V)
		return new(big.Rat).Add(new(big.Rat).Mul(du, du), new(big.Rat).Mul(dv, dv))
	}
	r := proofbound.RatSqrtUp(radius(arc.Start))
	rEnd := proofbound.RatSqrtDown(radius(arc.End))
	twoE := proofbound.ProductUpper(2, e)
	den := math.Nextafter(rEnd-twoE, math.Inf(-1))
	if !(den > 0) || math.IsInf(r, 0) {
		return 0, false
	}
	num := proofbound.ProductUpper(proofbound.ProductUpper(4, e), proofbound.AbsSumUpper(r, twoE))
	d := proofbound.AbsSumUpper(proofbound.ProductUpper(3, e), proofbound.DivUpper(num, den))
	return d, !math.IsInf(d, 0)
}

// walkStart and walkEnd are a whole line's read ends in walk order: a line
// recorded against its natural sense (TStart 1, TEnd 0) walks End to Start.
func walkStart(l sectionrecord.LineSeg) sectionrecord.Point2 {
	if l.TStart == 1 {
		return l.End
	}
	return l.Start
}

func walkEnd(l sectionrecord.LineSeg) sectionrecord.Point2 {
	if l.TStart == 1 {
		return l.Start
	}
	return l.End
}
