package spherepath

import (
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
)

// CardinalNormal reports whether a rounded contact normal is exactly one
// signed coordinate axis.
func CardinalNormal(n r3.Vec) bool {
	return n == (r3.Vec{X: 1}) || n == (r3.Vec{X: -1}) ||
		n == (r3.Vec{Y: 1}) || n == (r3.Vec{Y: -1}) ||
		n == (r3.Vec{Z: 1}) || n == (r3.Vec{Z: -1})
}

// PairMotion is the exact center and radius record of two source spheres
// following affine center paths over one sweep.
type PairMotion struct {
	CenterA, CenterB proofarith.DyV3
	RadiusA, RadiusB proofarith.Dyadic
	DeltaA, DeltaB   [3]proofarith.Dyadic
}

// AxialGap proves that both center paths share an unchanged transverse
// coordinate pair. The signed support gap is affine until first touch.
func (m PairMotion) AxialGap() (proofarith.Dyadic, proofarith.Dyadic, bool) {
	axis, sign, nonzero := 0, 0, 0
	for i := range 3 {
		start := proofarith.DySubScalar(m.CenterB[i], m.CenterA[i])
		travel := proofarith.DySubScalar(m.DeltaB[i], m.DeltaA[i])
		if start.Sign() != 0 || travel.Sign() != 0 {
			axis, sign, nonzero = i, start.Sign(), nonzero+1
		}
	}
	if nonzero != 1 || sign == 0 {
		return proofarith.Dyadic{}, proofarith.Dyadic{}, false
	}
	initial := proofarith.DyAbs(proofarith.DySubScalar(m.CenterB[axis], m.CenterA[axis]))
	gap := proofarith.DySubScalar(initial, proofarith.DyAdd(m.RadiusA, m.RadiusB))
	slope := proofarith.DySubScalar(m.DeltaB[axis], m.DeltaA[axis])
	if sign < 0 {
		slope = proofarith.DyNeg(slope)
	}
	return gap, slope, true
}

// SquaredGap is |centerB-centerA+f*(deltaB-deltaA)|²-(radiusA+radiusB)².
// Its coefficients are exact over the held source coordinates and path.
func (m PairMotion) SquaredGap() (proofarith.Dyadic, proofarith.Dyadic, proofarith.Dyadic) {
	a, b, c := proofarith.DyZero(), proofarith.DyZero(), proofarith.DyZero()
	for i := range 3 {
		p := proofarith.DySubScalar(m.CenterB[i], m.CenterA[i])
		v := proofarith.DySubScalar(m.DeltaB[i], m.DeltaA[i])
		a = proofarith.DyAdd(a, proofarith.DyMul(v, v))
		b = proofarith.DyAdd(b, proofarith.DyMul(p, v))
		c = proofarith.DyAdd(c, proofarith.DyMul(p, p))
	}
	radius := proofarith.DyAdd(m.RadiusA, m.RadiusB)
	return a, proofarith.DyAdd(b, b), proofarith.DySubScalar(c, proofarith.DyMul(radius, radius))
}
