// Package momentline computes line-segment area and moments over recorded plane coordinates.
package momentline

import (
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/freeform"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// Point stores one plane-local coordinate in millimetres.
type Point struct{ U, V float64 }

// Line stores the recorded endpoints and the walked parameter range.
type Line struct {
	Start, End   Point
	TStart, TEnd float64
}

// Evaluate returns the held line moments and their exact-rational error bounds.
// The exact result reads recorded coordinates before the float path shifts its anchor.
func Evaluate(seg Line, anchor Point, order freeform.MomentIntegralOrder) ([6]float64, [6]float64, freeform.ExactMoments) {
	exact := ExactLineMoments(seg, anchor, order)
	start := Point{U: seg.Start.U - anchor.U, V: seg.Start.V - anchor.V}
	end := Point{U: seg.End.U - anchor.U, V: seg.End.V - anchor.V}
	u0, v0 := lerp(start, end, seg.TStart)
	u1, v1 := lerp(start, end, seg.TEnd)

	area := 0.5 * (u0*v1 - u1*v0)
	mu := (v1 - v0) * (u0*u0 + u0*u1 + u1*u1) / 6
	mv := -(u1 - u0) * (v0*v0 + v0*v1 + v1*v1) / 6
	var values, bounds [6]float64
	values[0], values[1], values[2] = area, mu, mv
	bounds[0] = proofarith.RationalFloatError(exact.Area, area)
	bounds[1] = proofarith.RationalFloatError(exact.Mu, mu)
	bounds[2] = proofarith.RationalFloatError(exact.Mv, mv)
	if order < freeform.MomentSecondOrder {
		return values, bounds, exact
	}

	muu := (v1 - v0) * (u0*u0*u0 + u0*u0*u1 + u0*u1*u1 + u1*u1*u1) / 12
	mvv := -(u1 - u0) * (v0*v0*v0 + v0*v0*v1 + v0*v1*v1 + v1*v1*v1) / 12
	du, dv := u1-u0, v1-v0
	intU2V := v0*(u0*u0+u0*du+du*du/3) + dv*(u0*u0/2+2*u0*du/3+du*du/4)
	muv := 0.5 * dv * intU2V
	values[3], values[4], values[5] = muu, muv, mvv
	bounds[3] = proofarith.RationalFloatError(exact.Muu, muu)
	bounds[4] = proofarith.RationalFloatError(exact.Muv, muv)
	bounds[5] = proofarith.RationalFloatError(exact.Mvv, mvv)
	return values, bounds, exact
}

func lerp(start, end Point, t float64) (float64, float64) {
	switch t {
	case 0:
		return start.U, start.V
	case 1:
		return end.U, end.V
	}
	return start.U + t*(end.U-start.U), start.V + t*(end.V-start.V)
}

// TranslateExactMoments re-references the exact accumulator from the walk
// anchor back to the profile origin, mirroring translateMomentIntegrals step
// for step but over rationals — the anchor coordinates are floats, hence exact
// rationals, and the shift is only sums and products, so nothing rounds here.
func TranslateExactMoments(exact freeform.ExactMoments, anchor Point, order freeform.MomentIntegralOrder) freeform.ExactMoments {
	if order == freeform.MomentAreaOrder || !exact.Complete() {
		return exact
	}
	anchorU, anchorV := proofarith.FloatRat(anchor.U), proofarith.FloatRat(anchor.V)
	if anchorU == nil || anchorV == nil {
		return freeform.ExactMoments{}
	}
	if order >= freeform.MomentSecondOrder {
		// Second-order terms read the PRE-shift first moments, so they are
		// re-referenced before mu and mv are.
		exact.Muu = proofbound.RatAdd(
			exact.Muu,
			proofbound.RatMul(big.NewRat(2, 1), anchorU, exact.Mu),
			proofbound.RatMul(anchorU, anchorU, exact.Area),
		)
		exact.Muv = proofbound.RatAdd(
			exact.Muv,
			proofbound.RatMul(anchorV, exact.Mu),
			proofbound.RatMul(anchorU, exact.Mv),
			proofbound.RatMul(anchorU, anchorV, exact.Area),
		)
		exact.Mvv = proofbound.RatAdd(
			exact.Mvv,
			proofbound.RatMul(big.NewRat(2, 1), anchorV, exact.Mv),
			proofbound.RatMul(anchorV, anchorV, exact.Area),
		)
	}
	exact.Mu = proofbound.RatAdd(exact.Mu, proofbound.RatMul(anchorU, exact.Area))
	exact.Mv = proofbound.RatAdd(exact.Mv, proofbound.RatMul(anchorV, exact.Area))
	return exact
}

// RatScale multiplies an exact rational by num/den.
func RatScale(value *big.Rat, num, den int64) *big.Rat {
	return new(big.Rat).Mul(value, big.NewRat(num, den))
}

// RatLerp returns the exact rational value of P(t) = start + t·(end − start).
// At the two natural bounds the answer is the record's own coordinate —
// P(0) is start and P(1) is end, exactly — the same identity lerp2 already
// applies on the float side; this is that twin lerp2's doc comment already
// names. The non-finite check on the case's own operand keeps the nil
// contract callers read as "no bound available": a caller widening an
// infinite bound to a finite one because of a missed non-finite operand
// would be an inadmissible repair.
func RatLerp(start, end, t float64) *big.Rat {
	if t == 0 || t == 1 {
		near, far := start, end
		if t == 1 {
			near, far = end, start
		}
		if math.IsNaN(far) || math.IsInf(far, 0) {
			return nil
		}
		return proofarith.FloatRat(near)
	}
	rs, re, rt := proofarith.FloatRat(start), proofarith.FloatRat(end), proofarith.FloatRat(t)
	if rs == nil || re == nil || rt == nil {
		return nil
	}
	return new(big.Rat).Add(rs, new(big.Rat).Mul(rt, new(big.Rat).Sub(re, rs)))
}

// ExactLineMoments evaluates the polynomial line formulas over exact rationals,
// about the walk anchor. The public values retain the existing float
// evaluation; the rational result proves whether its rounding is exact and,
// when it is not, the precise error.
//
// The segment handed in holds the RECORDED coordinates and the anchor is
// subtracted here, over rationals. A lerp is affine, so lerping then
// subtracting is identical to subtracting then lerping — but only in exact
// arithmetic: fl(p−anchor) rounds, and the rational taken over those rounded
// coordinates would be a different chord's exact area.
func ExactLineMoments(seg Line, anchor Point, order freeform.MomentIntegralOrder) freeform.ExactMoments {
	u0 := RatLerp(seg.Start.U, seg.End.U, seg.TStart)
	v0 := RatLerp(seg.Start.V, seg.End.V, seg.TStart)
	u1 := RatLerp(seg.Start.U, seg.End.U, seg.TEnd)
	v1 := RatLerp(seg.Start.V, seg.End.V, seg.TEnd)
	anchorU, anchorV := proofarith.FloatRat(anchor.U), proofarith.FloatRat(anchor.V)
	if u0 == nil || v0 == nil || u1 == nil || v1 == nil || anchorU == nil || anchorV == nil {
		return freeform.ExactMoments{}
	}
	u0.Sub(u0, anchorU)
	u1.Sub(u1, anchorU)
	v0.Sub(v0, anchorV)
	v1.Sub(v1, anchorV)
	du := new(big.Rat).Sub(u1, u0)
	dv := new(big.Rat).Sub(v1, v0)

	u0sq, u1sq := proofbound.RatMul(u0, u0), proofbound.RatMul(u1, u1)
	v0sq, v1sq := proofbound.RatMul(v0, v0), proofbound.RatMul(v1, v1)
	area := RatScale(new(big.Rat).Sub(proofbound.RatMul(u0, v1), proofbound.RatMul(u1, v0)), 1, 2)
	mu := RatScale(proofbound.RatMul(dv, proofbound.RatAdd(u0sq, proofbound.RatMul(u0, u1), u1sq)), 1, 6)
	mv := RatScale(proofbound.RatMul(du, proofbound.RatAdd(v0sq, proofbound.RatMul(v0, v1), v1sq)), -1, 6)
	if order < freeform.MomentSecondOrder {
		// The accumulator still requires six non-nil fields. These zero
		// placeholders are never read by an area- or first-order caller; they
		// let the region publish its exact area and centroid without cubic work.
		return freeform.ExactMoments{
			Area: area, Mu: mu, Mv: mv,
			Muu: new(big.Rat), Muv: new(big.Rat), Mvv: new(big.Rat),
		}
	}

	muu := RatScale(proofbound.RatMul(dv, proofbound.RatAdd(
		proofbound.RatMul(u0, u0, u0),
		proofbound.RatMul(u0, u0, u1),
		proofbound.RatMul(u0, u1, u1),
		proofbound.RatMul(u1, u1, u1),
	)), 1, 12)
	mvv := RatScale(proofbound.RatMul(du, proofbound.RatAdd(
		proofbound.RatMul(v0, v0, v0),
		proofbound.RatMul(v0, v0, v1),
		proofbound.RatMul(v0, v1, v1),
		proofbound.RatMul(v1, v1, v1),
	)), -1, 12)

	duSq := proofbound.RatMul(du, du)
	u2v0 := proofbound.RatMul(v0, proofbound.RatAdd(u0sq, proofbound.RatMul(u0, du), RatScale(duSq, 1, 3)))
	u2dv := proofbound.RatMul(dv, proofbound.RatAdd(
		RatScale(u0sq, 1, 2),
		RatScale(proofbound.RatMul(u0, du), 2, 3),
		RatScale(duSq, 1, 4),
	))
	muv := RatScale(proofbound.RatMul(dv, proofbound.RatAdd(u2v0, u2dv)), 1, 2)
	return freeform.ExactMoments{Area: area, Mu: mu, Mv: mv, Muu: muu, Muv: muv, Mvv: mvv}
}
