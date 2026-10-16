package sectionclip

import (
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/pair/box"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// CardinalConeSection cuts one authenticated full right cone. The three
// complete meridian lines must join a base-axis point, base-rim point and apex.
// All comparisons use their exact recorded binary values. An inexact radius
// is represented by a bounded polygon with a certified two-sided displacement.
func CardinalConeSection(rp revolvePayload, plane r3.Frame) (profileRecord, []planePoint, float64, bool) {
	if !rp.Full || rp.SurfaceResult || rp.SectionDelta != 0 ||
		len(rp.Profile.Holes) != 0 || len(rp.Profile.Outer.Segments) != 3 ||
		!box.CardinalBasis(rp.Frame.U(), rp.Frame.V(), rp.Frame.N()) ||
		!box.CardinalBasis(plane.U(), plane.V(), plane.N()) ||
		!box.SignedAxisTransform(rp.Xform) || !proofbound.FiniteVec(rp.Frame.Origin()) ||
		rp.Ax.AUBound != 0 || rp.Ax.AVBound != 0 ||
		rp.Ax.DUBound != 0 || rp.Ax.DVBound != 0 ||
		!((math.Abs(rp.Ax.DU) == 1 && rp.Ax.DV == 0) ||
			(rp.Ax.DU == 0 && math.Abs(rp.Ax.DV) == 1)) ||
		!measurement.FiniteValues(rp.Ax.AU, rp.Ax.AV) {
		return profileRecord{}, nil, 0, false
	}

	var corners [3]planePoint
	for i, segment := range rp.Profile.Outer.Segments {
		line, ok := segment.(lineSeg)
		if !ok || line.TStart != 0 || line.TEnd != 1 ||
			!measurement.FiniteValues(line.Start.U, line.Start.V, line.End.U, line.End.V) {
			return profileRecord{}, nil, 0, false
		}
		corners[i] = line.Start
		next, ok := rp.Profile.Outer.Segments[(i+1)%3].(lineSeg)
		if !ok || line.End != next.Start {
			return profileRecord{}, nil, 0, false
		}
	}
	if corners[0] == corners[1] || corners[1] == corners[2] || corners[2] == corners[0] {
		return profileRecord{}, nil, 0, false
	}

	// The resolved axis has a zero error bound, and the original meridian has
	// zero section displacement. Exactly two triangle corners must lie on it;
	// the third must share the base corner's axial coordinate.
	au, av := proofarith.DyOfFinite(rp.Ax.AU), proofarith.DyOfFinite(rp.Ax.AV)
	du, dv := proofarith.DyOfFinite(rp.Ax.DU), proofarith.DyOfFinite(rp.Ax.DV)
	var axial, radial [3]proofarith.Dyadic
	axisCorners := make([]int, 0, 2)
	outer := -1
	for i, p := range corners {
		u := proofarith.DySubScalar(proofarith.DyOfFinite(p.U), au)
		v := proofarith.DySubScalar(proofarith.DyOfFinite(p.V), av)
		axial[i] = proofarith.DyAdd(proofarith.DyMul(u, du), proofarith.DyMul(v, dv))
		radial[i] = proofarith.DySubScalar(proofarith.DyMul(v, du), proofarith.DyMul(u, dv))
		if radial[i].IsZero() {
			axisCorners = append(axisCorners, i)
		} else if outer < 0 {
			outer = i
		} else {
			return profileRecord{}, nil, 0, false
		}
	}
	if len(axisCorners) != 2 || outer < 0 {
		return profileRecord{}, nil, 0, false
	}
	base, apex := axisCorners[0], axisCorners[1]
	if proofarith.DyCmp(axial[base], axial[outer]) != 0 {
		base, apex = apex, base
	}
	if proofarith.DyCmp(axial[base], axial[outer]) != 0 ||
		proofarith.DyCmp(axial[base], axial[apex]) == 0 {
		return profileRecord{}, nil, 0, false
	}

	// Compare the placed base and apex with the requested cardinal plane.
	// Opposite strict signs prove an interior cut; the cone radius varies
	// linearly along that axis, so the cut is one complete disk.
	point := func(p planePoint) proofarith.DyV3 {
		local := proofarith.DvAdd(proofarith.DyVec(rp.Frame.Origin()),
			proofarith.DvAdd(
				proofarith.DvScale(proofarith.DyVec(rp.Frame.U()), proofarith.DyOfFinite(p.U)),
				proofarith.DvScale(proofarith.DyVec(rp.Frame.V()), proofarith.DyOfFinite(p.V)),
			))
		return proofarith.DvTransform(rp.Xform, local)
	}
	baseWorld, apexWorld := point(corners[base]), point(corners[apex])
	u, v, n := proofarith.DyVec(plane.U()), proofarith.DyVec(plane.V()), proofarith.DyVec(plane.N())
	if !proofarith.DvEqual(proofarith.DvCross(u, v), n) ||
		!proofarith.DvIsZero(proofarith.DvCross(proofarith.DvSub(apexWorld, baseWorld), n)) {
		return profileRecord{}, nil, 0, false
	}
	baseDelta := proofarith.DvSub(baseWorld, proofarith.DyVec(plane.Origin()))
	apexDelta := proofarith.DvSub(apexWorld, proofarith.DyVec(plane.Origin()))
	baseSide, apexSide := proofarith.DvDot(baseDelta, n), proofarith.DvDot(apexDelta, n)
	if baseSide.Sign() == 0 || apexSide.Sign() == 0 || baseSide.Sign() == apexSide.Sign() {
		return profileRecord{}, nil, 0, false
	}
	height := proofarith.DySubScalar(apexSide, baseSide)
	cutFromBase := new(big.Rat).Neg(baseSide.Rat())
	cutFromBase.Quo(cutFromBase, height.Rat())
	radius := radial[outer].Rat()
	radius.Abs(radius)
	radius.Mul(radius, new(big.Rat).Sub(big.NewRat(1, 1), cutFromBase))
	radiusMM, radiusExact := radius.Float64()
	centerU, exactU := proofarith.DvDot(baseDelta, u).Float64()
	centerV, exactV := proofarith.DvDot(baseDelta, v).Float64()
	if radiusMM <= 0 || math.IsInf(radiusMM, 0) || !exactU || !exactV {
		return profileRecord{}, nil, 0, false
	}
	if radiusExact {
		return profileRecord{Outer: loopRecord{Segments: []curveSegment{circleSeg{
			Center: planePoint{U: centerU, V: centerV}, Radius: units.Millimeters(radiusMM),
			CCW: true, TStart: 0, TEnd: 1,
		}}}}, nil, 0, true
	}
	lower, upper := proofbound.RatFloatDown(radius), proofbound.RatFloatUp(radius)
	if !ObliqueCircleFinite(lower, upper) || lower <= 0 || upper <= lower {
		return profileRecord{}, nil, 0, false
	}
	// The exact disk contains the lower disk and lies in the upper disk.
	// The radial gap plus chord and rounded-vertex bounds covers both
	// occupied regions and their boundaries.
	gap := proofarith.DySubScalar(proofarith.DyOfFinite(upper), proofarith.DyOfFinite(lower))
	gapLocal := proofbound.RatFloatUp(proofarith.DyMul(gap, proofarith.DyInt(2)).Rat())
	gapWorld := proofbound.ProductUpper(gapLocal,
		max(proofbound.DvLenUpper(u), proofbound.DvLenUpper(v)))
	if !ObliqueCircleFinite(gapLocal, gapWorld) {
		return profileRecord{}, nil, 0, false
	}
	center := SectionRatPoint{U: proofarith.FloatRat(centerU), V: proofarith.FloatRat(centerV)}
	a := SectionRatPoint{U: proofarith.FloatRat(lower), V: proofarith.FloatRat(0)}
	b := SectionRatPoint{U: proofarith.FloatRat(0), V: proofarith.FloatRat(lower)}
	for _, perQuadrant := range [...]int{8, 16, 32} {
		polygon, delta, proved := ObliqueCirclePolygon(center, a, b, 1,
			plane, perQuadrant, gapLocal, gapWorld)
		if proved {
			return profileRecord{}, polygon, delta, true
		}
	}
	return profileRecord{}, nil, 0, false
}
