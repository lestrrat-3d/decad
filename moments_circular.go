package decad

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/circularbounds"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// circularSegment transfers only the recorded fields used by circular proofs.
func circularSegment(seg curveSegment) circularbounds.CurveSegment {
	return circularbounds.RecordSegment(seg)
}

func exactCoordinateDelta(a, b float64) *big.Rat {
	return circularbounds.ExactCoordinateDelta(a, b)
}

func circularWalkEnclosures(seg curveSegment) (proofbound.RatInterval, proofbound.RatInterval, bool) {
	return circularbounds.WalkEnclosures(circularSegment(seg))
}

func circularLengthInterval(seg curveSegment) (proofbound.RatInterval, bool) {
	return circularbounds.LengthInterval(circularSegment(seg))
}

func circularEndpointInterval(seg curveSegment, rt *big.Rat) (proofbound.RatInterval, proofbound.RatInterval, bool) {
	return circularbounds.EndpointInterval(circularSegment(seg), rt)
}
