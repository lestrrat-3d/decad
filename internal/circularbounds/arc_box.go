package circularbounds

import (
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// ArcBox encloses every point of a recorded arc's walked range. The angle
// interval comes from the recorded endpoint rays; sine and cosine are bounded
// over the entire range before they are scaled by the exact radius interval.
func ArcBox(seg ArcSeg) (proofbound.RatInterval, proofbound.RatInterval, bool) {
	d, ok := arcDeltasOf(seg)
	if !ok {
		return proofbound.RatInterval{}, proofbound.RatInterval{}, false
	}
	a0, sweep := arcAngleSweep(d.dx0, d.dy0, d.dx1, d.dy1,
		seg.Start.U-seg.Center.U, seg.Start.V-seg.Center.V,
		seg.End.U-seg.Center.U, seg.End.V-seg.Center.V)
	t0, t1 := new(big.Rat).SetFloat64(seg.TStart), new(big.Rat).SetFloat64(seg.TEnd)
	if t0 == nil || t1 == nil {
		return proofbound.RatInterval{}, proofbound.RatInterval{}, false
	}
	if t0.Cmp(t1) > 0 {
		t0, t1 = t1, t0
	}
	angles := proofbound.IntervalAdd(a0, proofbound.IntervalMul(sweep, proofbound.Interval(t0, t1)))
	// RadSinCosSpan widens by the width of its whole input. Split a short
	// fillet arc into bounded pieces so that the resulting box stays useful
	// when a nearby spline's control hull is only a small gap away.
	width := new(big.Rat).Sub(angles.Hi, angles.Lo)
	var sin, cos proofbound.RatInterval
	for i := range 64 {
		lo := new(big.Rat).Add(angles.Lo, new(big.Rat).Mul(width, big.NewRat(int64(i), 64)))
		hi := new(big.Rat).Add(angles.Lo, new(big.Rat).Mul(width, big.NewRat(int64(i+1), 64)))
		s, c, ok := proofbound.RadSinCosSpan(proofbound.Interval(lo, hi))
		if !ok {
			return proofbound.RatInterval{}, proofbound.RatInterval{}, false
		}
		if i == 0 {
			sin, cos = s, c
		} else {
			sin.Lo, sin.Hi = proofbound.RatMin(sin.Lo, s.Lo), proofbound.RatMax(sin.Hi, s.Hi)
			cos.Lo, cos.Hi = proofbound.RatMin(cos.Lo, c.Lo), proofbound.RatMax(cos.Hi, c.Hi)
		}
	}
	rlo, rhi := proofbound.RatSqrtDown(d.r2), proofbound.RatSqrtUp(d.r2)
	if math.IsInf(rlo, 0) || math.IsInf(rhi, 0) || math.IsNaN(rlo) || math.IsNaN(rhi) {
		return proofbound.RatInterval{}, proofbound.RatInterval{}, false
	}
	r := proofbound.Interval(new(big.Rat).SetFloat64(rlo), new(big.Rat).SetFloat64(rhi))
	u := proofbound.IntervalAdd(proofbound.PointInterval(new(big.Rat).SetFloat64(seg.Center.U)),
		proofbound.IntervalMul(r, cos))
	v := proofbound.IntervalAdd(proofbound.PointInterval(new(big.Rat).SetFloat64(seg.Center.V)),
		proofbound.IntervalMul(r, sin))
	return u, v, true
}
