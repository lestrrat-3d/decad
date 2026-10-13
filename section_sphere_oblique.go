package decad

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/pair/box"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// obliqueSpherePolygon bounds an off-center plane cut of an authenticated
// ball. The held U,V basis need not be exactly orthonormal, so its local cut
// can be an ellipse. The Gram solve and its squared radius are exact.
func obliqueSpherePolygon(center proofarith.DyV3, radius proofarith.Dyadic,
	plane r3.Frame) ([]Point2, float64, bool) {
	if radius.Sign() <= 0 || box.CardinalBasis(plane.U(), plane.V(), plane.N()) {
		return nil, 0, false
	}
	u, v := proofarith.DyVec(plane.U()), proofarith.DyVec(plane.V())
	guu, gvv, guv := proofarith.DvDot(u, u), proofarith.DvDot(v, v), proofarith.DvDot(u, v)
	absGuv := proofarith.DyAbs(guv)
	minUU, minVV := proofarith.DySubScalar(guu, absGuv), proofarith.DySubScalar(gvv, absGuv)
	maxUU, maxVV := proofarith.DyAdd(guu, absGuv), proofarith.DyAdd(gvv, absGuv)
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
	determinant := new(big.Rat).Sub(new(big.Rat).Mul(guu.Rat(), gvv.Rat()),
		new(big.Rat).Mul(guv.Rat(), guv.Rat()))
	if determinant.Sign() <= 0 {
		return nil, 0, false
	}
	fromCenter := proofarith.DvSub(proofarith.DyVec(plane.Origin()), center)
	bu, bv := proofarith.DvDot(u, fromCenter).Rat(), proofarith.DvDot(v, fromCenter).Rat()
	localCenter := sectionRatPoint{
		u: new(big.Rat).Quo(new(big.Rat).Sub(new(big.Rat).Mul(guv.Rat(), bv),
			new(big.Rat).Mul(gvv.Rat(), bu)), determinant),
		v: new(big.Rat).Quo(new(big.Rat).Sub(new(big.Rat).Mul(guv.Rat(), bu),
			new(big.Rat).Mul(guu.Rat(), bv)), determinant),
	}
	// Completing the square gives (x-localCenter)^T G (x-localCenter)
	// <= q, where q is the sphere radius squared less the plane distance
	// squared. A tangent or exterior plane has no material profile.
	q := new(big.Rat).Sub(new(big.Rat).Sub(new(big.Rat).Mul(radius.Rat(), radius.Rat()),
		proofarith.DvDot(fromCenter, fromCenter).Rat()),
		new(big.Rat).Add(new(big.Rat).Mul(bu, localCenter.u),
			new(big.Rat).Mul(bv, localCenter.v)))
	if q.Sign() <= 0 {
		return nil, 0, false
	}
	// For every local vector x, minimum*|x|² <= |U*x.u+V*x.v|²
	// <= maximum*|x|². These exact Gram bounds enclose the true local
	// ellipse between disks centered on the exact Gram solution.
	inner := proofbound.RatSqrtDown(new(big.Rat).Quo(q, maximum.Rat()))
	outer := proofbound.RatSqrtUp(new(big.Rat).Quo(q, minimum.Rat()))
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
	a := sectionRatPoint{u: proofarith.FloatRat(inner), v: zero}
	b := sectionRatPoint{u: zero, v: proofarith.FloatRat(inner)}
	for _, perQuadrant := range [...]int{8, 16, 32} {
		polygon, delta, proved := obliqueCirclePolygon(localCenter, a, b, 1,
			plane, perQuadrant, gapLocal, gapWorld)
		if proved {
			return polygon, delta, true
		}
	}
	return nil, 0, false
}
