package decad

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/circularmoments"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// circularSegment transfers only the recorded fields used by circular proofs.
func circularSegment(seg CurveSegment) circularmoments.CurveSegment {
	return circularmoments.RecordSegment(seg)
}

func circularPoint(p Point2) circularmoments.Point2 {
	return circularmoments.RecordPoint(p)
}

func exactCoordinateDelta(a, b float64) *big.Rat {
	return circularmoments.ExactCoordinateDelta(a, b)
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
