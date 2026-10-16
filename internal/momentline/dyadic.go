package momentline

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/proof"
)

// Recorded coordinates, parameters and anchors are binary fractions. Interpolation
// and anchor subtraction therefore stay exact in Dyadic, including trimmed lines.
func exactLineCoordinates(seg Line, anchor Point) ([4]proof.Dyadic, bool) {
	u0, ok0 := proof.DyLerp(seg.Start.U, seg.End.U, seg.TStart)
	v0, ok1 := proof.DyLerp(seg.Start.V, seg.End.V, seg.TStart)
	u1, ok2 := proof.DyLerp(seg.Start.U, seg.End.U, seg.TEnd)
	v1, ok3 := proof.DyLerp(seg.Start.V, seg.End.V, seg.TEnd)
	au, okU := proof.DyOf(anchor.U)
	av, okV := proof.DyOf(anchor.V)
	if !ok0 || !ok1 || !ok2 || !ok3 || !okU || !okV {
		return [4]proof.Dyadic{}, false
	}
	return [4]proof.Dyadic{proof.DySubScalar(u0, au), proof.DySubScalar(v0, av),
		proof.DySubScalar(u1, au), proof.DySubScalar(v1, av)}, true
}

func dyadicChordArea(p [4]proof.Dyadic) *big.Rat {
	return proof.DyShift(proof.DySubScalar(proof.DyMul(p[0], p[3]), proof.DyMul(p[2], p[1])), -1).Rat()
}

// Only the polynomial numerators are dyadic. Division by three enters a general
// rational, so convert at the final scale and keep the existing rational bounds.
// ExactChordMoments remains the general-rational implementation for junctions.
func dyadicChordMoments(p [4]proof.Dyadic, order freeform.MomentIntegralOrder) freeform.ExactMoments {
	u0, v0, u1, v1 := p[0], p[1], p[2], p[3]
	du, dv := proof.DySubScalar(u1, u0), proof.DySubScalar(v1, v0)
	u0sq, u1sq := proof.DyMul(u0, u0), proof.DyMul(u1, u1)
	v0sq, v1sq := proof.DyMul(v0, v0), proof.DyMul(v1, v1)
	area := dyadicChordArea(p)
	mu := RatScale(proof.DyMul(dv, dyadicSum3(u0sq, proof.DyMul(u0, u1), u1sq)).Rat(), 1, 6)
	mv := RatScale(proof.DyMul(du, dyadicSum3(v0sq, proof.DyMul(v0, v1), v1sq)).Rat(), -1, 6)
	if order < freeform.MomentSecondOrder {
		return freeform.ExactMoments{Area: area, Mu: mu, Mv: mv,
			Muu: new(big.Rat), Muv: new(big.Rat), Mvv: new(big.Rat)}
	}
	// a^3+a^2*b+a*b^2+b^3 = (a+b)*(a^2+b^2), exactly.
	muu := RatScale(proof.DyMul(dv, proof.DyMul(proof.DyAdd(u0, u1), proof.DyAdd(u0sq, u1sq))).Rat(), 1, 12)
	mvv := RatScale(proof.DyMul(du, proof.DyMul(proof.DyAdd(v0, v1), proof.DyAdd(v0sq, v1sq))).Rat(), -1, 12)
	duSq, u0du := proof.DyMul(du, du), proof.DyMul(u0, du)
	// Clear the original 1/3, 1/2, 2/3 and 1/4 factors to a common
	// denominator of 12, then include the outer 1/2 as a final division by 24.
	left := proof.DyMul(v0, dyadicSum3(proof.DyMul(proof.DyInt(12), u0sq),
		proof.DyMul(proof.DyInt(12), u0du), proof.DyShift(duSq, 2)))
	right := proof.DyMul(dv, dyadicSum3(proof.DyMul(proof.DyInt(6), u0sq),
		proof.DyShift(u0du, 3), proof.DyMul(proof.DyInt(3), duSq)))
	muv := RatScale(proof.DyMul(dv, proof.DyAdd(left, right)).Rat(), 1, 24)
	return freeform.ExactMoments{Area: area, Mu: mu, Mv: mv, Muu: muu, Muv: muv, Mvv: mvv}
}

func dyadicSum3(a, b, c proof.Dyadic) proof.Dyadic {
	return proof.DyAdd(proof.DyAdd(a, b), c)
}
