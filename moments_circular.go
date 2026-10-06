package decad

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/circularmoments"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// circularSegment transfers only the recorded fields used by circular proofs.
func circularSegment(seg CurveSegment) circularmoments.CurveSegment {
	switch seg := seg.(type) {
	case CircleSeg:
		return circularmoments.CircleSeg{
			Center: circularPoint(seg.Center), Radius: seg.Radius, CCW: seg.CCW,
			TStart: seg.TStart, TEnd: seg.TEnd,
		}
	case ArcSeg:
		return circularmoments.ArcSeg{
			Center: circularPoint(seg.Center), Start: circularPoint(seg.Start), End: circularPoint(seg.End),
			TStart: seg.TStart, TEnd: seg.TEnd,
		}
	default:
		return nil
	}
}

func circularPoint(p Point2) circularmoments.Point2 {
	return circularmoments.Point2{U: p.U, V: p.V}
}

func exactCoordinateDelta(a, b float64) *big.Rat {
	return circularmoments.ExactCoordinateDelta(a, b)
}

func arcEndRadialRatio(r2, endR2 *big.Rat) (proofbound.RatInterval, bool) {
	return circularmoments.ArcEndRadialRatio(r2, endR2)
}

func circularAreaInterval(seg CurveSegment, anchor Point2) (proofbound.RatInterval, bool) {
	return circularmoments.AreaInterval(circularSegment(seg), circularPoint(anchor))
}

func circularWalkEnclosures(seg CurveSegment) (proofbound.RatInterval, proofbound.RatInterval, bool) {
	return circularmoments.WalkEnclosures(circularSegment(seg))
}

func circularLengthInterval(seg CurveSegment) (proofbound.RatInterval, bool) {
	return circularmoments.LengthInterval(circularSegment(seg))
}

func circularEndpointInterval(seg CurveSegment, rt *big.Rat) (proofbound.RatInterval, proofbound.RatInterval, bool) {
	return circularmoments.EndpointInterval(circularSegment(seg), rt)
}

func circularOffsetEndpointInterval(seg CurveSegment, rt, radiusOffset *big.Rat) (proofbound.RatInterval, proofbound.RatInterval, bool) {
	return circularmoments.OffsetEndpointInterval(circularSegment(seg), rt, radiusOffset)
}

func quarterTurnSinCos(t *big.Rat) (proofbound.RatInterval, proofbound.RatInterval) {
	return circularmoments.QuarterTurnSinCos(t)
}

func circularAxisMomentInterval(seg CurveSegment, ax axisFrame) (proofbound.RatInterval, bool) {
	frame := circularmoments.NewAxisFrame(ax.aU, ax.aV, ax.aUBound, ax.aVBound, ax.dU, ax.dV, ax.dUBound, ax.dVBound)
	return circularmoments.AxisMomentInterval(circularSegment(seg), frame)
}

func circularFirstMomentInterval(seg CurveSegment, anchor Point2) (proofbound.RatInterval, proofbound.RatInterval, bool) {
	return circularmoments.FirstMomentInterval(circularSegment(seg), circularPoint(anchor))
}

func circularSecondMomentInterval(seg CurveSegment, anchor Point2) (proofbound.RatInterval, proofbound.RatInterval, proofbound.RatInterval, bool) {
	return circularmoments.SecondMomentInterval(circularSegment(seg), circularPoint(anchor))
}

func circularThirdMomentInterval(seg CurveSegment) ([4]proofbound.RatInterval, bool) {
	return circularmoments.ThirdMomentInterval(circularSegment(seg))
}

type circularMomentWalk = circularmoments.MomentWalk

func circularMomentWalkOf(seg CurveSegment) (circularMomentWalk, bool) {
	return circularmoments.MomentWalkOf(circularSegment(seg))
}

func circularMonomials(walk circularMomentWalk, degree int) [][]proofbound.RatInterval {
	return circularmoments.Monomials(walk, degree)
}

func circularGreenMoment(walk circularMomentWalk, j [][]proofbound.RatInterval, p, q int) proofbound.RatInterval {
	return circularmoments.GreenMoment(walk, j, p, q)
}
