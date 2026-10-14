package decad

import (
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/sectionclip"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// obliqueCircleClip bounds a complete ellipse cut from an exact whole-circle
// prism. The requested plane is read as the exact span of its held U and V.
// Cuts meeting either cap, and planes parallel to the sweep axis, refuse.
func obliqueCircleClip(pp prismPayload, plane r3.Frame) ([]planePoint, float64, bool) {
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
	// |a cos(theta)+b sin(theta)| <= |a|+|b|. Strict containment
	// keeps the complete ellipse away from both cap faces.
	zReach := sectionclip.SectionRatAdd(new(big.Rat).Abs(zCos), new(big.Rat).Abs(zSin))
	if sectionclip.SectionRatSub(zCenter, zReach).Cmp(proofarith.FloatRat(pp.Z0)) <= 0 ||
		sectionclip.SectionRatAdd(zCenter, zReach).Cmp(proofarith.FloatRat(pp.Z1)) >= 0 {
		return nil, 0, false
	}
	center := obliqueCircleWorldAdd(obliqueCircleWorld(centerWorld),
		obliqueCircleWorldScale(obliqueCircleWorld(axis), zCenter))
	cosVector := obliqueCircleWorldAdd(obliqueCircleWorldScale(obliqueCircleWorld(radialU), r),
		obliqueCircleWorldScale(obliqueCircleWorld(axis), zCos))
	sinVector := obliqueCircleWorldAdd(obliqueCircleWorldScale(obliqueCircleWorld(radialV), r),
		obliqueCircleWorldScale(obliqueCircleWorld(axis), zSin))
	localCenter, ok := sectionPlaneCoordinates(center, plane)
	if !ok {
		return nil, 0, false
	}
	localCos, ok := sectionPlaneCoordinates(obliqueCircleWorldAdd(center, cosVector), plane)
	if !ok {
		return nil, 0, false
	}
	localSin, ok := sectionPlaneCoordinates(obliqueCircleWorldAdd(center, sinVector), plane)
	if !ok {
		return nil, 0, false
	}
	a, b := sectionclip.SectionRatVector(localCenter, localCos), sectionclip.SectionRatVector(localCenter, localSin)
	orientation := sectionclip.SectionRatCross(a, b).Sign()
	if orientation == 0 {
		return nil, 0, false
	}
	for _, perQuadrant := range [...]int{8, 16, 32} {
		polygon, delta, ok := sectionclip.ObliqueCirclePolygon(localCenter, a, b, orientation,
			plane, perQuadrant, 0, 0)
		if ok {
			return polygon, delta, true
		}
	}
	return nil, 0, false
}

func obliqueCircleWorld(v proofarith.DyV3) [3]*big.Rat {
	return [3]*big.Rat{v[0].Rat(), v[1].Rat(), v[2].Rat()}
}

func obliqueCircleWorldAdd(a, b [3]*big.Rat) [3]*big.Rat {
	return [3]*big.Rat{sectionclip.SectionRatAdd(a[0], b[0]), sectionclip.SectionRatAdd(a[1], b[1]),
		sectionclip.SectionRatAdd(a[2], b[2])}
}

func obliqueCircleWorldScale(v [3]*big.Rat, k *big.Rat) [3]*big.Rat {
	return [3]*big.Rat{sectionclip.SectionRatMul(v[0], k), sectionclip.SectionRatMul(v[1], k),
		sectionclip.SectionRatMul(v[2], k)}
}
