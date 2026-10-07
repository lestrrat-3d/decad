package momentregion

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/circularbounds"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/momentline"
	"github.com/lestrrat-3d/decad/internal/polynomial"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/splinebezier"
)

// SegmentThirdMoments encloses one normalized segment's third-order integral.
func SegmentThirdMoments(segment sectionrecord.CurveSegment, plan Plan) ([4]proofbound.RatInterval, bool) {
	var exact [4]*big.Rat
	switch segment := segment.(type) {
	case sectionrecord.LineSeg:
		u0 := momentline.RatLerp(segment.Start.U, segment.End.U, segment.TStart)
		v0 := momentline.RatLerp(segment.Start.V, segment.End.V, segment.TStart)
		u1 := momentline.RatLerp(segment.Start.U, segment.End.U, segment.TEnd)
		v1 := momentline.RatLerp(segment.Start.V, segment.End.V, segment.TEnd)
		if u0 == nil || v0 == nil || u1 == nil || v1 == nil {
			return [4]proofbound.RatInterval{}, false
		}
		exact = freeform.PolyThirdMoments(
			polynomial.RatPoly{u0, new(big.Rat).Sub(u1, u0)},
			polynomial.RatPoly{v0, new(big.Rat).Sub(v1, v0)},
		)
	case sectionrecord.CircleSeg, sectionrecord.ArcSeg:
		return circularbounds.ThirdMomentInterval(circularbounds.RecordSegment(segment))
	default:
		if !splinebezier.IsFreeformSegment(segment) || len(plan.Spans) == 0 {
			return [4]proofbound.RatInterval{}, false
		}
		exact = freeform.FreeformThirdMoments(plan.Spans, plan.Reversed)
	}
	var out [4]proofbound.RatInterval
	for i, value := range exact {
		out[i] = proofbound.PointInterval(value)
	}
	return out, true
}
