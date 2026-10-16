package decad

import (
	"context"
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/sectionclip"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// obliqueCircleCapClip admits one strictly crossed cap only after Sketch has
// certified the source circle-line trim. Inner and outer rational polygons
// enclose the clipped disk before its affine map into the requested plane.
func obliqueCircleCapClip(ctx context.Context, pp prismPayload, plane r3.Frame) ([]planePoint, float64, bool) {
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
