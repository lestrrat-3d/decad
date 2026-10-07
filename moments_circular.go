package decad

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/circularbounds"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// circularSegment transfers only the recorded fields used by circular proofs.
func circularSegment(seg CurveSegment) circularbounds.CurveSegment {
	return circularbounds.RecordSegment(seg)
}

func circularPoint(p Point2) circularbounds.Point2 {
	return circularbounds.RecordPoint(p)
}

func exactCoordinateDelta(a, b float64) *big.Rat {
	return circularbounds.ExactCoordinateDelta(a, b)
}

func circularAreaInterval(seg CurveSegment, anchor Point2) (proofbound.RatInterval, bool) {
	return circularbounds.AreaInterval(circularSegment(seg), circularPoint(anchor))
}

func circularWalkEnclosures(seg CurveSegment) (proofbound.RatInterval, proofbound.RatInterval, bool) {
	return circularbounds.WalkEnclosures(circularSegment(seg))
}

func circularLengthInterval(seg CurveSegment) (proofbound.RatInterval, bool) {
	return circularbounds.LengthInterval(circularSegment(seg))
}

func circularEndpointInterval(seg CurveSegment, rt *big.Rat) (proofbound.RatInterval, proofbound.RatInterval, bool) {
	return circularbounds.EndpointInterval(circularSegment(seg), rt)
}

func circularOffsetEndpointInterval(seg CurveSegment, rt, radiusOffset *big.Rat) (proofbound.RatInterval, proofbound.RatInterval, bool) {
	return circularbounds.OffsetEndpointInterval(circularSegment(seg), rt, radiusOffset)
}

func circularAxisMomentInterval(seg CurveSegment, ax axisFrame) (proofbound.RatInterval, bool) {
	frame := circularbounds.NewAxisFrame(ax.aU, ax.aV, ax.aUBound, ax.aVBound, ax.dU, ax.dV, ax.dUBound, ax.dVBound)
	return circularbounds.AxisMomentInterval(circularSegment(seg), frame)
}

func circularFirstMomentInterval(seg CurveSegment, anchor Point2) (proofbound.RatInterval, proofbound.RatInterval, bool) {
	return circularbounds.FirstMomentInterval(circularSegment(seg), circularPoint(anchor))
}

func circularSecondMomentInterval(seg CurveSegment, anchor Point2) (proofbound.RatInterval, proofbound.RatInterval, proofbound.RatInterval, bool) {
	return circularbounds.SecondMomentInterval(circularSegment(seg), circularPoint(anchor))
}

func circularThirdMomentInterval(seg CurveSegment) ([4]proofbound.RatInterval, bool) {
	return circularbounds.ThirdMomentInterval(circularSegment(seg))
}
