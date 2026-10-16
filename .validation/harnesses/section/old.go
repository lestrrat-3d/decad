package zzclipdiff

import (
	"context"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/pair/box"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolveplan"
	"github.com/lestrrat-3d/decad/internal/sectionclip"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

type (
	planePoint     = sectionrecord.Point2
	profileRecord  = momentinput.Profile
	loopRecord     = sectionrecord.LoopRecord
	curveSegment   = sectionrecord.CurveSegment
	lineSeg        = sectionrecord.LineSeg
	circleSeg      = sectionrecord.CircleSeg
	prismPayload   = prismrecord.Payload
	revolvePayload = revolveplan.Payload
)

type oldSphere struct {
	center proofarith.DyV3
	radius proofarith.Dyadic
}

func oldSignedAxisTransform(t r3.Transform) bool {
	if !t.IsValid() || !proofbound.FiniteVec(t.Translation()) {
		return false
	}
	b := t.Basis()
	return box.CardinalBasis(b.EX, b.EY, b.EZ)
}

// oldObliqueCircleCapClip admits one strictly crossed cap only after Sketch has
// certified the source circle-line trim. Inner and outer rational polygons
// enclose the clipped disk before its affine map into the requested plane.
func oldObliqueCircleCapClip(ctx context.Context, pp prismPayload, plane r3.Frame) ([]planePoint, float64, bool) {
	if len(pp.Profile.Holes) != 0 || len(pp.Profile.Outer.Segments) != 1 ||
		pp.BlendKind != "" || len(pp.BlendSegs) != 0 || pp.Z0 >= pp.Z1 {
		return nil, 0, false
	}
	circle, ok := pp.Profile.Outer.Segments[0].(circleSeg)
	if !ok || !circle.CCW || circle.TStart != 0 || circle.TEnd != 1 {
		return nil, 0, false
	}
	radius, err := circle.Radius.In(units.Millimeter)
	if err != nil || radius <= 0 || !sectionclip.ObliqueCircleFinite(radius, circle.Center.U, circle.Center.V,
		pp.Z0, pp.Z1) {
		return nil, 0, false
	}
	sourceOrigin := proofarith.DyVec(pp.Frame.Origin())
	placedOrigin := proofarith.DvTransform(pp.Xform, sourceOrigin)
	image := func(direction r3.Vec) proofarith.DyV3 {
		point := proofarith.DvAdd(sourceOrigin, proofarith.DyVec(direction))
		return proofarith.DvSub(proofarith.DvTransform(pp.Xform, point), placedOrigin)
	}
	radialU, radialV, axis := image(pp.Frame.U()), image(pp.Frame.V()), image(pp.Frame.N())
	centerLocal := proofarith.DvAdd(sourceOrigin, proofarith.DvAdd(
		proofarith.DvScale(proofarith.DyVec(pp.Frame.U()), proofarith.DyOfFinite(circle.Center.U)),
		proofarith.DvScale(proofarith.DyVec(pp.Frame.V()), proofarith.DyOfFinite(circle.Center.V))))
	centerWorld := proofarith.DvTransform(pp.Xform, centerLocal)
	requestedOrigin := proofarith.DyVec(plane.Origin())
	normal := proofarith.DvCross(proofarith.DyVec(plane.U()), proofarith.DyVec(plane.V()))
	denominator := proofarith.DvDot(normal, axis).Rat()
	if denominator.Sign() == 0 {
		return nil, 0, false
	}
	r := proofarith.FloatRat(radius)
	zCenter := sectionclip.SectionRatQuo(new(big.Rat).Neg(
		proofarith.DvDot(normal, proofarith.DvSub(centerWorld, requestedOrigin)).Rat()), denominator)
	zCos := sectionclip.SectionRatQuo(new(big.Rat).Neg(sectionclip.SectionRatMul(r,
		proofarith.DvDot(normal, radialU).Rat())), denominator)
	zSin := sectionclip.SectionRatQuo(new(big.Rat).Neg(sectionclip.SectionRatMul(r,
		proofarith.DvDot(normal, radialV).Rat())), denominator)
	reach2 := sectionclip.SectionRatAdd(sectionclip.SectionRatMul(zCos, zCos), sectionclip.SectionRatMul(zSin, zSin))
	if reach2.Sign() <= 0 {
		return nil, 0, false
	}
	z0, z1 := proofarith.FloatRat(pp.Z0), proofarith.FloatRat(pp.Z1)
	low, high := sectionclip.SectionRatSub(z0, zCenter), sectionclip.SectionRatSub(z1, zCenter)
	lowCross := sectionclip.SectionRatMul(low, low).Cmp(reach2) < 0
	highCross := sectionclip.SectionRatMul(high, high).Cmp(reach2) < 0
	if lowCross == highCross {
		return nil, 0, false
	}
	// The uncrossed cap must miss even the conservative L1 disk extent.
	reach1 := sectionclip.SectionRatAdd(new(big.Rat).Abs(zCos), new(big.Rat).Abs(zSin))
	if highCross && sectionclip.SectionRatSub(zCenter, reach1).Cmp(z0) <= 0 ||
		lowCross && sectionclip.SectionRatAdd(zCenter, reach1).Cmp(z1) >= 0 {
		return nil, 0, false
	}
	capLevel := high
	if lowCross {
		capLevel = low
	}
	if !sectionclip.SectionCircleCapSketch(ctx, circle, r, zCos, zSin, capLevel, highCross) {
		return nil, 0, false
	}
	center := sectionclip.ObliqueCircleWorldAdd(sectionclip.ObliqueCircleWorld(centerWorld),
		sectionclip.ObliqueCircleWorldScale(sectionclip.ObliqueCircleWorld(axis), zCenter))
	cosVector := sectionclip.ObliqueCircleWorldAdd(sectionclip.ObliqueCircleWorldScale(sectionclip.ObliqueCircleWorld(radialU), r),
		sectionclip.ObliqueCircleWorldScale(sectionclip.ObliqueCircleWorld(axis), zCos))
	sinVector := sectionclip.ObliqueCircleWorldAdd(sectionclip.ObliqueCircleWorldScale(sectionclip.ObliqueCircleWorld(radialV), r),
		sectionclip.ObliqueCircleWorldScale(sectionclip.ObliqueCircleWorld(axis), zSin))
	localCenter, ok := sectionclip.SectionPlaneCoordinates(center, plane)
	if !ok {
		return nil, 0, false
	}
	localCos, ok := sectionclip.SectionPlaneCoordinates(sectionclip.ObliqueCircleWorldAdd(center, cosVector), plane)
	if !ok {
		return nil, 0, false
	}
	localSin, ok := sectionclip.SectionPlaneCoordinates(sectionclip.ObliqueCircleWorldAdd(center, sinVector), plane)
	if !ok {
		return nil, 0, false
	}
	a, b := sectionclip.SectionRatVector(localCenter, localCos), sectionclip.SectionRatVector(localCenter, localSin)
	orientation := sectionclip.SectionRatCross(a, b).Sign()
	if orientation == 0 {
		return nil, 0, false
	}
	for _, perQuadrant := range [...]int{8, 16, 32} {
		if err := ctx.Err(); err != nil {
			return nil, 0, false
		}
		polygon, delta, ok := sectionclip.SectionCircleCapPolygon(localCenter, a, b, zCos, zSin,
			capLevel, highCross, orientation, plane, perQuadrant)
		if ok {
			return polygon, delta, true
		}
	}
	return nil, 0, false
}

// oldCardinalConeSection cuts one authenticated full right cone. The three
// complete meridian lines must join a base-axis point, base-rim point and apex.
// All comparisons use their exact recorded binary values. An inexact radius
// is represented by a bounded polygon with a certified two-sided displacement.
func oldCardinalConeSection(rp revolvePayload, plane r3.Frame) (profileRecord, []planePoint, float64, bool) {
	if !rp.Full || rp.SurfaceResult || rp.SectionDelta != 0 ||
		len(rp.Profile.Holes) != 0 || len(rp.Profile.Outer.Segments) != 3 ||
		!box.CardinalBasis(rp.Frame.U(), rp.Frame.V(), rp.Frame.N()) ||
		!box.CardinalBasis(plane.U(), plane.V(), plane.N()) ||
		!oldSignedAxisTransform(rp.Xform) || !proofbound.FiniteVec(rp.Frame.Origin()) ||
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
	if !sectionclip.ObliqueCircleFinite(lower, upper) || lower <= 0 || upper <= lower {
		return profileRecord{}, nil, 0, false
	}
	// The exact disk contains the lower disk and lies in the upper disk.
	// The radial gap plus chord and rounded-vertex bounds covers both
	// occupied regions and their boundaries.
	gap := proofarith.DySubScalar(proofarith.DyOfFinite(upper), proofarith.DyOfFinite(lower))
	gapLocal := proofbound.RatFloatUp(proofarith.DyMul(gap, proofarith.DyInt(2)).Rat())
	gapWorld := proofbound.ProductUpper(gapLocal,
		max(proofbound.DvLenUpper(u), proofbound.DvLenUpper(v)))
	if !sectionclip.ObliqueCircleFinite(gapLocal, gapWorld) {
		return profileRecord{}, nil, 0, false
	}
	center := sectionclip.SectionRatPoint{U: proofarith.FloatRat(centerU), V: proofarith.FloatRat(centerV)}
	a := sectionclip.SectionRatPoint{U: proofarith.FloatRat(lower), V: proofarith.FloatRat(0)}
	b := sectionclip.SectionRatPoint{U: proofarith.FloatRat(0), V: proofarith.FloatRat(lower)}
	for _, perQuadrant := range [...]int{8, 16, 32} {
		polygon, delta, proved := sectionclip.ObliqueCirclePolygon(center, a, b, 1,
			plane, perQuadrant, gapLocal, gapWorld)
		if proved {
			return profileRecord{}, polygon, delta, true
		}
	}
	return profileRecord{}, nil, 0, false
}

// cardinalSphereSection cuts an authenticated whole-ball Revolve with a
// cardinal plane. An exactly representable circle stays exact. Otherwise a
// certified polygon carries the two-sided radial, chord and rounding gaps to Extrude.
func oldCardinalSphereSection(sphere oldSphere, ok bool, plane r3.Frame) (profileRecord, []planePoint, float64, bool) {
	if !ok || !box.CardinalBasis(plane.U(), plane.V(), plane.N()) {
		return profileRecord{}, nil, 0, false
	}
	u := proofarith.DyVec(plane.U())
	v := proofarith.DyVec(plane.V())
	n := proofarith.DyVec(plane.N())
	if !proofarith.DvEqual(proofarith.DvCross(u, v), n) {
		return profileRecord{}, nil, 0, false
	}
	delta := proofarith.DvSub(sphere.center, proofarith.DyVec(plane.Origin()))
	offset := proofarith.DvDot(delta, n)
	q := proofarith.DySubScalar(proofarith.DyMul(sphere.radius, sphere.radius),
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
	if !sectionclip.ObliqueCircleFinite(upper) || upper <= radius {
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
	if !sectionclip.ObliqueCircleFinite(gapL1, gapWorld) {
		return profileRecord{}, nil, 0, false
	}
	center := sectionclip.SectionRatPoint{U: proofarith.FloatRat(centerU), V: proofarith.FloatRat(centerV)}
	a := sectionclip.SectionRatPoint{U: proofarith.FloatRat(radius), V: proofarith.FloatRat(0)}
	basisV := sectionclip.SectionRatPoint{U: proofarith.FloatRat(0), V: proofarith.FloatRat(radius)}
	for _, perQuadrant := range [...]int{8, 16, 32} {
		polygon, bound, proved := sectionclip.ObliqueCirclePolygon(center, a, basisV, 1,
			plane, perQuadrant, gapL1, gapWorld)
		if proved {
			return profileRecord{}, polygon, bound, true
		}
	}
	return profileRecord{}, nil, 0, false
}
