package sectionclip

import (
	"github.com/lestrrat-3d/decad/internal/pair/box"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// CardinalSphereSection cuts the whole ball of the given centre and radius,
// which the caller has authenticated as a source sphere, with a cardinal
// plane. An exactly representable circle stays exact. Otherwise a certified
// polygon carries the two-sided radial, chord and rounding gaps to Extrude.
func CardinalSphereSection(ballCenter proofarith.DyV3, ballRadius proofarith.Dyadic,
	plane r3.Frame) (profileRecord, []planePoint, float64, bool) {
	if !box.CardinalBasis(plane.U(), plane.V(), plane.N()) {
		return profileRecord{}, nil, 0, false
	}
	u := proofarith.DyVec(plane.U())
	v := proofarith.DyVec(plane.V())
	n := proofarith.DyVec(plane.N())
	if !proofarith.DvEqual(proofarith.DvCross(u, v), n) {
		return profileRecord{}, nil, 0, false
	}
	delta := proofarith.DvSub(ballCenter, proofarith.DyVec(plane.Origin()))
	offset := proofarith.DvDot(delta, n)
	q := proofarith.DySubScalar(proofarith.DyMul(ballRadius, ballRadius),
		proofarith.DyMul(offset, offset))
	if q.Sign() <= 0 {
		return profileRecord{}, nil, 0, false
	}
	radius := proofarith.DySqrtDown(q)
	if radius <= 0 {
		return profileRecord{}, nil, 0, false
	}
	centerU, exactU := proofarith.DvDot(delta, u).Float64()
	centerV, exactV := proofarith.DvDot(delta, v).Float64()
	if !exactU || !exactV {
		return profileRecord{}, nil, 0, false
	}
	if proofarith.DySquareEquals(radius, q) {
		return profileRecord{Outer: loopRecord{Segments: []curveSegment{circleSeg{
			Center: planePoint{U: centerU, V: centerV}, Radius: units.Millimeters(radius),
			CCW: true, TStart: 0, TEnd: 1,
		}}}}, nil, 0, true
	}
	upper := proofarith.DySqrtUp(q)
	if !ObliqueCircleFinite(upper) || upper <= radius {
		return profileRecord{}, nil, 0, false
	}
	// Both brackets are proven by exact square comparisons. Every point on
	// the true circle moves at most upper-radius radially to the lower one.
	// Twice that gap bounds the local L1 move; the held basis norms convert
	// the same coordinate bound to world distance.
	gap := proofarith.DySubScalar(proofarith.DyOfFinite(upper), proofarith.DyOfFinite(radius))
	gapL1 := proofbound.RatFloatUp(proofarith.DyMul(gap, proofarith.DyInt(2)).Rat())
	gapWorld := proofbound.ProductUpper(gapL1,
		max(proofbound.DvLenUpper(u), proofbound.DvLenUpper(v)))
	if !ObliqueCircleFinite(gapL1, gapWorld) {
		return profileRecord{}, nil, 0, false
	}
	center := SectionRatPoint{U: proofarith.FloatRat(centerU), V: proofarith.FloatRat(centerV)}
	a := SectionRatPoint{U: proofarith.FloatRat(radius), V: proofarith.FloatRat(0)}
	basisV := SectionRatPoint{U: proofarith.FloatRat(0), V: proofarith.FloatRat(radius)}
	for _, perQuadrant := range [...]int{8, 16, 32} {
		polygon, bound, proved := ObliqueCirclePolygon(center, a, basisV, 1,
			plane, perQuadrant, gapL1, gapWorld)
		if proved {
			return profileRecord{}, polygon, bound, true
		}
	}
	return profileRecord{}, nil, 0, false
}
