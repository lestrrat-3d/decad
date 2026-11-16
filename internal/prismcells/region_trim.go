package prismcells

import (
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/circularbounds"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/units"
)

type walkT = survey2d.SegmentWalk

// arcResidual is an ArcSeg's radial residual ρ: the distance between its
// recorded End and the point at Start's radius on End's bearing.
func arcResidual(a sectionrecord.ArcSeg) float64 { return circularbounds.ArcRadialResidualUpper(a) }

// fullLengthBounds bounds the arc length of the whole carrier a recorded
// segment's parameter runs over: a line's length, a circle's 2πR, an arc's
// Rσ. ok=false means no enclosure.
func fullLengthBounds(seg sectionrecord.CurveSegment) (lo, hi float64, ok bool) {
	switch s := seg.(type) {
	case sectionrecord.LineSeg:
		du := new(big.Rat).Sub(proofarith.FloatRat(s.End.U), proofarith.FloatRat(s.Start.U))
		dv := new(big.Rat).Sub(proofarith.FloatRat(s.End.V), proofarith.FloatRat(s.Start.V))
		q := new(big.Rat).Add(new(big.Rat).Mul(du, du), new(big.Rat).Mul(dv, dv))
		return proofbound.RatSqrtDown(q), proofbound.RatSqrtUp(q), q.Sign() > 0
	case sectionrecord.CircleSeg:
		r, err := s.Radius.In(units.Millimeter)
		if err != nil || !(r > 0) {
			return 0, 0, false
		}
		rr := proofarith.FloatRat(r)
		iv := proofbound.IntervalScale(proofbound.TwoPiInterval(), rr)
		return proofbound.RatFloatDown(iv.Lo), proofbound.RatFloatUp(iv.Hi), true
	case sectionrecord.ArcSeg:
		whole := s
		whole.TStart, whole.TEnd = 0, 1
		sLo, sHi, ok := prismrecord.SweepLowerUpper(whole)
		if !ok {
			return 0, 0, false
		}
		du := circularbounds.ExactCoordinateDelta(s.Start.U, s.Center.U)
		dv := circularbounds.ExactCoordinateDelta(s.Start.V, s.Center.V)
		q := new(big.Rat).Add(new(big.Rat).Mul(du, du), new(big.Rat).Mul(dv, dv))
		rLo, rHi := proofbound.RatSqrtDown(q), proofbound.RatSqrtUp(q)
		return proofbound.RatFloatDown(new(big.Rat).Mul(proofarith.FloatRat(rLo), proofarith.FloatRat(sLo))),
			proofbound.ProductUpper(rHi, sHi), rLo > 0
	}
	return 0, 0, false
}

// setRange returns seg with its parameter range replaced.
func setRange(seg sectionrecord.CurveSegment, t0, t1 float64) (sectionrecord.CurveSegment, bool) {
	switch s := seg.(type) {
	case sectionrecord.LineSeg:
		s.TStart, s.TEnd = t0, t1
		return s, true
	case sectionrecord.CircleSeg:
		s.TStart, s.TEnd = t0, t1
		return s, true
	case sectionrecord.ArcSeg:
		s.TStart, s.TEnd = t0, t1
		return s, true
	}
	return nil, false
}

func rangeOf(seg sectionrecord.CurveSegment) (float64, float64) {
	switch s := seg.(type) {
	case sectionrecord.LineSeg:
		return s.TStart, s.TEnd
	case sectionrecord.CircleSeg:
		return s.TStart, s.TEnd
	case sectionrecord.ArcSeg:
		return s.TStart, s.TEnd
	}
	return 0, 0
}

// trimSegment removes from a recorded edge the part within the arc length
// target of its end at the start of its range (atStart) or at its end,
// by narrowing the segment's own parameter range. It rounds the trimmed
// parameter in float; kappa is the overshoot of the removed length beyond
// target (an upper bound) plus the trimmed end's own walk bound, so the ball
// of §6.3 holds every exact point the trim removes, over-trim included.
// empty means nothing is left of the edge. ok=false means no enclosure.
func trimSegment(seg sectionrecord.CurveSegment, atStart bool, target float64) (trimmed sectionrecord.CurveSegment,
	kappa float64, empty, ok bool) {
	lenLo, lenHi, ok := fullLengthBounds(seg)
	if !ok || !(lenLo > 0) || proofbound.IsNonFinite(lenHi) {
		return nil, 0, false, false
	}
	t0, t1 := rangeOf(seg)
	dt := proofbound.ProductUpper(proofbound.DivUpper(target, lenLo), 1+0x1p-50)
	sign := 1.0
	if t1 < t0 {
		sign = -1
	}
	span := math.Abs(t1 - t0)
	if !(span > dt) {
		return nil, 0, true, true
	}
	var moved float64
	var newT0, newT1 = t0, t1
	if atStart {
		newT0 = t0 + sign*dt
		moved = math.Abs(newT0 - t0)
	} else {
		newT1 = t1 - sign*dt
		moved = math.Abs(t1 - newT1)
	}
	if math.Abs(newT1-newT0) <= 0 || (newT1-newT0)*sign <= 0 {
		return nil, 0, true, true
	}
	exact := new(big.Rat).Sub(proofarith.FloatRat(newT0), proofarith.FloatRat(t0))
	if !atStart {
		exact = new(big.Rat).Sub(proofarith.FloatRat(t1), proofarith.FloatRat(newT1))
	}
	exact.Abs(exact)
	_ = moved
	removed := proofbound.RatFloatUp(new(big.Rat).Mul(exact, proofarith.FloatRat(lenHi)))
	next, ok := setRange(seg, newT0, newT1)
	if !ok {
		return nil, 0, false, false
	}
	w, err := boundarywalk.WalkOf(next, nil)
	if err != nil {
		return nil, 0, false, false
	}
	bound := w.EndBound
	if atStart {
		bound = w.StartBound
	}
	over := math.Max(0, math.Nextafter(removed-target, math.Inf(1)))
	return next, proofbound.AbsSumUpper(over, proofbound.WalkEndBoundAllow(bound)), false, true
}
