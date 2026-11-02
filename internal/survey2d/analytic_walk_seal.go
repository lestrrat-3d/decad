package survey2d

import (
	"math"

	"github.com/lestrrat-3d/decad/internal/sectionrecord"
)

// AnalyticSealView states the bitwise fields of a supported analytic walk.
func (w SegmentWalk) AnalyticSealView() (sectionrecord.AnalyticWalkView, bool) {
	if (w.Kind != WalkLine && w.Kind != WalkCircular) || len(w.Spans) != 0 || w.FitInterpolated || w.Reversed {
		return sectionrecord.AnalyticWalkView{}, false
	}
	floats := [...]float64{w.StartU, w.StartV, w.EndU, w.EndV, w.StartBound.U, w.StartBound.V, w.EndBound.U, w.EndBound.V,
		w.TanInU, w.TanInV, w.TanOutU, w.TanOutV, w.TanInBound, w.TanOutBound, w.Length, w.LengthBound,
		w.LengthUpper, w.CoordUpper, w.AxisRadiusUpper, w.AxisMomentUpper, w.StartVBound, w.EndVBound, w.CVBound,
		w.CU, w.CV, w.Radius, w.RadiusBound, w.Th0, w.Th1}
	out := sectionrecord.AnalyticWalkView{Kind: uint8(w.Kind), Closed: w.Closed}
	for i, v := range floats {
		out.Values[i] = math.Float64bits(v)
	}
	return out, true
}
