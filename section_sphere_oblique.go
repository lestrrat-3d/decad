package decad

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/pair/box"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// centralSphereObliquePolygon bounds the disk cut from an authenticated ball
// when the requested frame origin is exactly its center. The held U,V basis
// need not be exactly orthonormal, so its local cut can be an ellipse.
func centralSphereObliquePolygon(center proofarith.DyV3, radius proofarith.Dyadic,
	plane r3.Frame) ([]Point2, float64, bool) {
	if radius.Sign() <= 0 || box.CardinalBasis(plane.U(), plane.V(), plane.N()) ||
		!proofarith.DvEqual(center, proofarith.DyVec(plane.Origin())) {
		return nil, 0, false
	}
	u, v := proofarith.DyVec(plane.U()), proofarith.DyVec(plane.V())
	guu, gvv, guv := proofarith.DvDot(u, u), proofarith.DvDot(v, v),
		proofarith.DyAbs(proofarith.DvDot(u, v))
	minUU, minVV := proofarith.DySubScalar(guu, guv), proofarith.DySubScalar(gvv, guv)
	maxUU, maxVV := proofarith.DyAdd(guu, guv), proofarith.DyAdd(gvv, guv)
	minimum, maximum := minUU, maxUU
	if proofarith.DyCmp(minVV, minimum) < 0 {
		minimum = minVV
	}
	if proofarith.DyCmp(maxVV, maximum) > 0 {
		maximum = maxVV
	}
	if minimum.Sign() <= 0 {
		return nil, 0, false
	}
	// For every local vector x, minimum*|x|² <= |U*x.u+V*x.v|²
	// <= maximum*|x|². These exact Gram bounds enclose the true local
	// ellipse between two concentric disks, without treating the rounded
	// frame axes as an exactly orthonormal basis.
	sqrtMaxUp := proofarith.DySqrtUp(maximum)
	sqrtMinDown := proofarith.DySqrtDown(minimum)
	if !obliqueCircleFinite(sqrtMaxUp) || sqrtMinDown <= 0 {
		return nil, 0, false
	}
	inner := proofbound.RatFloatDown(new(big.Rat).Quo(radius.Rat(), proofarith.FloatRat(sqrtMaxUp)))
	outer := proofbound.RatFloatUp(new(big.Rat).Quo(radius.Rat(), proofarith.FloatRat(sqrtMinDown)))
	if !obliqueCircleFinite(inner, outer) || inner <= 0 || outer < inner {
		return nil, 0, false
	}
	// The true ellipse contains the inner disk and lies in the outer disk.
	// Radial movement between those disks is at most outer-inner. Twice
	// that distance bounds the local L1 move, and the held basis norms
	// convert the same coordinate bound to world displacement.
	gap := proofarith.DySubScalar(proofarith.DyOfFinite(outer), proofarith.DyOfFinite(inner))
	gapLocal := proofbound.RatFloatUp(proofarith.DyMul(gap, proofarith.DyInt(2)).Rat())
	gapWorld := proofbound.ProductUpper(gapLocal,
		max(proofbound.DvLenUpper(u), proofbound.DvLenUpper(v)))
	if !obliqueCircleFinite(gapLocal, gapWorld) {
		return nil, 0, false
	}
	zero := proofarith.FloatRat(0)
	centerLocal := sectionRatPoint{u: zero, v: zero}
	a := sectionRatPoint{u: proofarith.FloatRat(inner), v: zero}
	b := sectionRatPoint{u: zero, v: proofarith.FloatRat(inner)}
	for _, perQuadrant := range [...]int{8, 16, 32} {
		polygon, delta, proved := obliqueCirclePolygon(centerLocal, a, b, 1,
			plane, perQuadrant, gapLocal, gapWorld)
		if proved {
			return polygon, delta, true
		}
	}
	return nil, 0, false
}
