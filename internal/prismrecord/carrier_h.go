package prismrecord

import (
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/circularbounds"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/units"
)

// This file is Lemma H of agent-docs/carrier-record-design.md §3.3: the
// pointwise bound h that a record's entries give on how far the denoted
// section's segment lies from the recorded one.

// wholeRange reports a circular segment's parameter range as the whole
// carrier, either walk sense.
func wholeRange(t0, t1 float64) bool { return (t0 == 0 && t1 == 1) || (t0 == 1 && t1 == 0) }

func subDown(a, b float64) float64 { return math.Nextafter(a-b, math.Inf(-1)) }

// recordedRadius returns exact-enough bounds on a circular segment's recorded
// radius (an ArcSeg's is |Start − Center|) and its sweep as the fraction of a
// turn, and whether the segment is a whole circle. ok=false means the data
// is not usable.
func circularRadius(seg sectionrecord.CurveSegment) (lo, hi float64, ok bool) {
	switch s := seg.(type) {
	case sectionrecord.CircleSeg:
		r, err := s.Radius.In(units.Millimeter)
		if err != nil || !(r > 0) || math.IsInf(r, 0) {
			return 0, 0, false
		}
		return r, r, true
	case sectionrecord.ArcSeg:
		du := circularbounds.ExactCoordinateDelta(s.Start.U, s.Center.U)
		dv := circularbounds.ExactCoordinateDelta(s.Start.V, s.Center.V)
		q := new(big.Rat).Add(new(big.Rat).Mul(du, du), new(big.Rat).Mul(dv, dv))
		lo, hi = proofbound.RatSqrtDown(q), proofbound.RatSqrtUp(q)
		if !(lo > 0) || math.IsInf(hi, 0) {
			return 0, 0, false
		}
		return lo, hi, true
	}
	return 0, 0, false
}

// SweepLowerUpper is a proven enclosure of the sweep σ of a circular segment
// in radians: an ArcSeg's wrapSweep of its recorded bearings (agent-docs
// carrier-record-design.md §3.3) times its parameter range, a narrowed
// CircleSeg's 2π·|TEnd − TStart|. ok=false means no enclosure (a bearing
// difference straddling zero, or unusable data).
func SweepLowerUpper(seg sectionrecord.CurveSegment) (lo, hi float64, ok bool) {
	twoPi := proofbound.TwoPiInterval()
	frac := func(t0, t1 float64) *big.Rat {
		d := proofarith.FloatRat(t1)
		e := proofarith.FloatRat(t0)
		if d == nil || e == nil {
			return nil
		}
		return new(big.Rat).Abs(d.Sub(d, e))
	}
	switch s := seg.(type) {
	case sectionrecord.CircleSeg:
		f := frac(s.TStart, s.TEnd)
		if f == nil {
			return 0, 0, false
		}
		iv := proofbound.IntervalScale(twoPi, f)
		return proofbound.RatFloatDown(iv.Lo), proofbound.RatFloatUp(iv.Hi), true
	case sectionrecord.ArcSeg:
		y0 := circularbounds.ExactCoordinateDelta(s.Start.V, s.Center.V)
		x0 := circularbounds.ExactCoordinateDelta(s.Start.U, s.Center.U)
		y1 := circularbounds.ExactCoordinateDelta(s.End.V, s.Center.V)
		x1 := circularbounds.ExactCoordinateDelta(s.End.U, s.Center.U)
		if y0 == nil || x0 == nil || y1 == nil || x1 == nil || (x0.Sign() == 0 && y0.Sign() == 0) ||
			(x1.Sign() == 0 && y1.Sign() == 0) {
			return 0, 0, false
		}
		a0 := proofbound.Atan2Interval(y0, x0, false)
		a1 := proofbound.Atan2Interval(y1, x1, false)
		d := proofbound.IntervalSub(a1, a0)
		switch {
		case d.Lo.Sign() > 0:
		case d.Hi.Sign() <= 0:
			d = proofbound.IntervalAdd(d, twoPi)
		default:
			return 0, 0, false
		}
		if !wholeRange(s.TStart, s.TEnd) {
			f := frac(s.TStart, s.TEnd)
			if f == nil {
				return 0, 0, false
			}
			d = proofbound.IntervalScale(d, f)
		}
		return proofbound.RatFloatDown(d.Lo), proofbound.RatFloatUp(d.Hi), true
	}
	return 0, 0, false
}

// SegmentH is Lemma H's h for segment j of loop l: how far every point of the
// denoted segment lies from the matched point of the recorded one. It is
// +Inf where the lemma's condition fails, and a consumer then misses or
// refuses as its own section says.
func (cr CarrierRecord) SegmentH(l, j int) float64 {
	loops := carrierLoops(cr.Profile)
	if l < 0 || l >= len(loops) || l >= len(cr.Segment) || j < 0 || j >= len(loops[l].Segments) ||
		j >= len(cr.Segment[l]) || len(cr.Junction[l]) != len(cr.Segment[l]) {
		return math.Inf(1)
	}
	n := len(loops[l].Segments)
	eps := cr.Segment[l][j]
	ds, de := cr.Junction[l][(j+n-1)%n], cr.Junction[l][j]
	dm := math.Max(ds, de)
	switch seg := loops[l].Segments[j].(type) {
	case sectionrecord.LineSeg:
		return dm
	case sectionrecord.CircleSeg:
		if wholeRange(seg.TStart, seg.TEnd) {
			return eps
		}
	case sectionrecord.ArcSeg:
	default:
		return math.Inf(1)
	}
	return arcH(loops[l].Segments[j], eps, dm)
}

// arcH is Lemma H's arc row.
func arcH(seg sectionrecord.CurveSegment, eps, dm float64) float64 {
	if eps == 0 && dm == 0 {
		return 0 // nothing slides: S* is S as evaluator §4 denotes it
	}
	rLo, _, ok := circularRadius(seg)
	if !ok {
		return math.Inf(1)
	}
	slide := proofbound.AbsSumUpper(eps, dm)
	rEps := subDown(rLo, eps)
	if !(rEps > 0) || !(slide <= proofbound.ProductUpper(2, rEps)) {
		return math.Inf(1)
	}
	alpha := proofbound.DivUpper(proofbound.ProductUpper(2, slide), rEps)
	sLo, sHi, ok := SweepLowerUpper(seg)
	if !ok {
		return math.Inf(1)
	}
	twoPiLo := proofbound.RatFloatDown(proofbound.TwoPiIv.Lo)
	room := math.Min(sLo, subDown(twoPiLo, sHi))
	if !(proofbound.ProductUpper(2, alpha) < room) {
		return math.Inf(1)
	}
	return proofbound.AbsSumUpper(eps, proofbound.ProductUpper(2, slide))
}

// LoopH is loop l's h_loop: the largest h of its segments and the largest
// junction entry (Lemma H, "Loops"). It is +Inf when any segment's condition
// fails.
func (cr CarrierRecord) LoopH(l int) float64 {
	if l < 0 || l >= len(cr.Segment) {
		return math.Inf(1)
	}
	worst := 0.0
	for j := range cr.Segment[l] {
		worst = math.Max(worst, cr.SegmentH(l, j))
		worst = math.Max(worst, cr.Junction[l][j])
	}
	return worst
}

// HMax is the largest h_loop over the record's loops.
func (cr CarrierRecord) HMax() float64 {
	worst := 0.0
	for l := range cr.Segment {
		worst = math.Max(worst, cr.LoopH(l))
	}
	return worst
}

// Clone is a deep copy of the record: the profile shares no slice with cr's.
func (cr CarrierRecord) Clone() CarrierRecord {
	out := CarrierRecord{SectionDelta: cr.SectionDelta, Profile: cloneProfile(cr.Profile),
		Segment: make([][]float64, len(cr.Segment)), Junction: make([][]float64, len(cr.Junction))}
	for l := range cr.Segment {
		out.Segment[l] = append([]float64(nil), cr.Segment[l]...)
	}
	for l := range cr.Junction {
		out.Junction[l] = append([]float64(nil), cr.Junction[l]...)
	}
	return out
}
