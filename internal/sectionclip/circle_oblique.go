package sectionclip

import (
	"math"
	"math/big"
	"slices"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// ObliqueCircleFinite reports whether every value is finite.
func ObliqueCircleFinite(values ...float64) bool {
	for _, value := range values {
		if math.IsNaN(value) || math.IsInf(value, 0) {
			return false
		}
	}
	return true
}

// obliqueCircleUnit samples the unit circle with the rational half-angle
// parameter t in [0,1). Quarter-turns cover the other three quadrants.
func obliqueCircleUnit(quadrant, numerator, denominator int) SectionRatPoint {
	t := big.NewRat(int64(numerator), int64(denominator))
	t2 := SectionRatMul(t, t)
	d := SectionRatAdd(big.NewRat(1, 1), t2)
	x := SectionRatQuo(SectionRatSub(big.NewRat(1, 1), t2), d)
	y := SectionRatQuo(SectionRatMul(big.NewRat(2, 1), t), d)
	switch quadrant {
	case 1:
		return SectionRatPoint{U: new(big.Rat).Neg(y), V: x}
	case 2:
		return SectionRatPoint{U: new(big.Rat).Neg(x), V: new(big.Rat).Neg(y)}
	case 3:
		return SectionRatPoint{U: y, V: new(big.Rat).Neg(x)}
	default:
		return SectionRatPoint{U: x, V: y}
	}
}

// Each sampled circular arc and its chord are within d²/4 for a unit
// circle, where d is the chord length. The exact station polygon may be inside
// its source circle, but coordinate rounding can move the held polygon outside.
// The affine map and vertex rounding carry that two-sided gap to the exact
// ellipse and its held line polygon.
func ObliqueCirclePolygon(center, a, b SectionRatPoint, orientation int,
	plane r3.Frame, perQuadrant int, extraLocal, extraWorld float64) ([]planePoint, float64, bool) {
	unit := make([]SectionRatPoint, 0, 4*perQuadrant)
	for quadrant := range 4 {
		for j := range perQuadrant {
			unit = append(unit, obliqueCircleUnit(quadrant, j, perQuadrant))
		}
	}
	if orientation < 0 {
		slices.Reverse(unit)
	}
	exact := make([]SectionRatPoint, len(unit))
	maxChordSquared := new(big.Rat)
	for i, point := range unit {
		exact[i] = SectionRatPoint{
			U: SectionRatAdd(center.U, SectionRatAdd(SectionRatMul(a.U, point.U), SectionRatMul(b.U, point.V))),
			V: SectionRatAdd(center.V, SectionRatAdd(SectionRatMul(a.V, point.U), SectionRatMul(b.V, point.V))),
		}
		next := unit[(i+1)%len(unit)]
		dx, dy := SectionRatSub(next.U, point.U), SectionRatSub(next.V, point.V)
		d2 := SectionRatAdd(SectionRatMul(dx, dx), SectionRatMul(dy, dy))
		if d2.Cmp(maxChordSquared) > 0 {
			maxChordSquared = d2
		}
	}
	sagitta := SectionRatQuo(maxChordSquared, big.NewRat(4, 1))
	uGap := SectionRatMul(sagitta, SectionRatAdd(new(big.Rat).Abs(a.U), new(big.Rat).Abs(b.U)))
	vGap := SectionRatMul(sagitta, SectionRatAdd(new(big.Rat).Abs(a.V), new(big.Rat).Abs(b.V)))
	uNorm := proofbound.DvLenUpper(proofarith.DyVec(plane.U()))
	vNorm := proofbound.DvLenUpper(proofarith.DyVec(plane.V()))
	curveLocal := proofbound.RatFloatUp(SectionRatAdd(uGap, vGap))
	curveWorld := proofbound.AbsSumUpper(proofbound.ProductUpper(proofbound.RatFloatUp(uGap), uNorm),
		proofbound.ProductUpper(proofbound.RatFloatUp(vGap), vNorm))
	proxy := make([]planePoint, len(exact))
	vertexLocal, vertexWorld := float64(0), float64(0)
	for i, point := range exact {
		u, _ := point.U.Float64()
		v, _ := point.V.Float64()
		if !ObliqueCircleFinite(u, v) {
			return nil, 0, false
		}
		proxy[i] = planePoint{U: u, V: v}
		uError := proofarith.RationalFloatError(point.U, u)
		vError := proofarith.RationalFloatError(point.V, v)
		vertexLocal = max(vertexLocal, proofbound.AbsSumUpper(uError, vError))
		vertexWorld = max(vertexWorld, proofbound.AbsSumUpper(
			proofbound.ProductUpper(uError, uNorm), proofbound.ProductUpper(vError, vNorm)))
	}
	delta := proofbound.UpRound(max(proofbound.AbsSumUpper(curveLocal, vertexLocal, extraLocal),
		proofbound.AbsSumUpper(curveWorld, vertexWorld, extraWorld)))
	if !ObliqueCircleFinite(delta) || delta <= 0 || delta > 1.0/20 ||
		!SectionMatchedConvex(exact, proxy) {
		return nil, 0, false
	}
	for i, point := range exact {
		next := exact[(i+1)%len(exact)]
		edgeL1 := SectionRatAdd(new(big.Rat).Abs(SectionRatSub(next.U, point.U)),
			new(big.Rat).Abs(SectionRatSub(next.V, point.V)))
		if edgeL1.Sign() <= 0 || SectionRatMul(proofarith.FloatRat(delta), big.NewRat(4, 1)).Cmp(edgeL1) >= 0 {
			return nil, 0, false
		}
	}
	return proxy, delta, true
}
