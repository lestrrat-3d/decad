package circularmoments

import (
	"math"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// FloatMomentValues holds a circular walk's float moments and proven bounds.
type FloatMomentValues struct {
	Values, Bounds [6]float64
	CoordUpper     float64
}

// EvaluateFloat evaluates a circular path about center from th0 to th1.
// area is the caller's float Green's-theorem term; the returned bounds enclose
// that value and the remaining moment evaluations.
func EvaluateFloat(
	c Point2,
	r, th0, th1, radiusUpper, sweepUpper, area float64,
	areaProof proofbound.RatInterval,
	haveAreaProof bool,
	muProof, mvProof proofbound.RatInterval,
	haveMomentProof bool,
	muuProof, muvProof, mvvProof proofbound.RatInterval,
	haveSecondMomentProof bool,
	order freeform.MomentIntegralOrder,
) FloatMomentValues {
	var out FloatMomentValues
	sin0, cos0 := math.Sincos(th0)
	sin1, cos1 := math.Sincos(th1)
	dth := th1 - th0
	absR, absU, absV, absDth := radiusUpper, math.Abs(c.U), math.Abs(c.V), sweepUpper
	out.CoordUpper = proofbound.AbsSumUpper(c.U, c.V, absR, absR)

	areaScale := 0.5 * (absR*absR*absDth + 2*absU*absR + 2*absV*absR)

	intCos := sin1 - sin0
	intCos2 := dth/2 + (math.Sin(2*th1)-math.Sin(2*th0))/4
	intCos3 := (sin1 - sin1*sin1*sin1/3) - (sin0 - sin0*sin0*sin0/3)
	mu := 0.5 * r * (c.U*c.U*intCos + 2*c.U*r*intCos2 + r*r*intCos3)

	intSin := cos0 - cos1
	intSin2 := dth/2 - (math.Sin(2*th1)-math.Sin(2*th0))/4
	intSin3 := (cos0 - cos0*cos0*cos0/3) - (cos1 - cos1*cos1*cos1/3)
	mv := 0.5 * r * (c.V*c.V*intSin + 2*c.V*r*intSin2 + r*r*intSin3)

	int2Scale := absDth/2 + 0.5
	int3Scale := 4.0 / 3
	muScale := 0.5 * absR * (2*absU*absU + 2*absU*absR*int2Scale + absR*absR*int3Scale)
	mvScale := 0.5 * absR * (2*absV*absV + 2*absV*absR*int2Scale + absR*absR*int3Scale)

	areaScale = proofbound.ProductUpper(2, areaScale)
	muScale = proofbound.ProductUpper(2, muScale)
	mvScale = proofbound.ProductUpper(2, mvScale)

	areaBound := proofbound.ConservativeValueError(area, areaScale)
	if haveAreaProof {
		areaBound = math.Min(areaBound, proofbound.IntervalFloatError(areaProof, area))
	}
	muBound := proofbound.ConservativeValueError(mu, muScale)
	mvBound := proofbound.ConservativeValueError(mv, mvScale)
	if haveMomentProof {
		muBound = math.Min(muBound, proofbound.IntervalFloatError(muProof, mu))
		mvBound = math.Min(mvBound, proofbound.IntervalFloatError(mvProof, mv))
	}
	out.Values[0], out.Bounds[0] = area, areaBound
	out.Values[1], out.Bounds[1] = mu, muBound
	out.Values[2], out.Bounds[2] = mv, mvBound
	if order < freeform.MomentSecondOrder {
		return out
	}

	intCos4 := 3*dth/8 + (math.Sin(2*th1)-math.Sin(2*th0))/4 + (math.Sin(4*th1)-math.Sin(4*th0))/32
	intSin4 := 3*dth/8 - (math.Sin(2*th1)-math.Sin(2*th0))/4 + (math.Sin(4*th1)-math.Sin(4*th0))/32
	int4Scale := 3*absDth/8 + 0.5 + 1.0/16

	// ∫u² dA = ⅓ ∮ u³ dv, dv = r cos θ dθ, u³ expanded about the center.
	muu := r / 3 * (c.U*c.U*c.U*intCos + 3*c.U*c.U*r*intCos2 + 3*c.U*r*r*intCos3 + r*r*r*intCos4)
	muuScale := absR / 3 * (2*absU*absU*absU + 3*absU*absU*absR*int2Scale +
		3*absU*absR*absR*int3Scale + absR*absR*absR*int4Scale)

	// ∫v² dA = −⅓ ∮ v³ du, du = −r sin θ dθ, v³ expanded about the center.
	mvv := r / 3 * (c.V*c.V*c.V*intSin + 3*c.V*c.V*r*intSin2 + 3*c.V*r*r*intSin3 + r*r*r*intSin4)
	mvvScale := absR / 3 * (2*absV*absV*absV + 3*absV*absV*absR*int2Scale +
		3*absV*absR*absR*int3Scale + absR*absR*absR*int4Scale)

	intSC := (sin1*sin1 - sin0*sin0) / 2
	intSC2 := (cos0*cos0*cos0 - cos1*cos1*cos1) / 3
	intSC3 := (cos0*cos0*cos0*cos0 - cos1*cos1*cos1*cos1) / 4
	muv := 0.5 * r * (c.V*(c.U*c.U*intCos+2*c.U*r*intCos2+r*r*intCos3) +
		r*(c.U*c.U*intSC+2*c.U*r*intSC2+r*r*intSC3))
	muvScale := 0.5 * absR * (absV*(2*absU*absU+2*absU*absR*int2Scale+absR*absR*int3Scale) +
		absR*(0.5*absU*absU+4*absU*absR/3+absR*absR/4))
	muuScale = proofbound.ProductUpper(2, muuScale)
	muvScale = proofbound.ProductUpper(2, muvScale)
	mvvScale = proofbound.ProductUpper(2, mvvScale)
	muuBound := proofbound.ConservativeValueError(muu, muuScale)
	muvBound := proofbound.ConservativeValueError(muv, muvScale)
	mvvBound := proofbound.ConservativeValueError(mvv, mvvScale)
	if haveSecondMomentProof {
		muuBound = math.Min(muuBound, proofbound.IntervalFloatError(muuProof, muu))
		muvBound = math.Min(muvBound, proofbound.IntervalFloatError(muvProof, muv))
		mvvBound = math.Min(mvvBound, proofbound.IntervalFloatError(mvvProof, mvv))
	}
	out.Values[3], out.Bounds[3] = muu, muuBound
	out.Values[4], out.Bounds[4] = muv, muvBound
	out.Values[5], out.Bounds[5] = mvv, mvvBound
	return out
}
