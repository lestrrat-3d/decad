package decad

import (
	"math"
	"math/big"
	"slices"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// obliqueCircleClip bounds a complete ellipse cut from an exact whole-circle
// prism. The requested plane is read as the exact span of its held U and V.
// Cuts meeting either cap, and planes parallel to the sweep axis, refuse.
func obliqueCircleClip(pp prismPayload, plane r3.Frame) ([]Point2, float64, bool) {
	if len(pp.profile.Holes) != 0 || len(pp.profile.Outer.Segments) != 1 ||
		pp.blendKind != "" || len(pp.blendSegs) != 0 || pp.z0 >= pp.z1 {
		return nil, 0, false
	}
	circle, ok := pp.profile.Outer.Segments[0].(circleSeg)
	if !ok || !circle.CCW || circle.TStart != 0 || circle.TEnd != 1 {
		return nil, 0, false
	}
	radius, err := circle.Radius.In(units.Millimeter)
	if err != nil || radius <= 0 || !obliqueCircleFinite(radius, circle.Center.U, circle.Center.V,
		pp.z0, pp.z1) {
		return nil, 0, false
	}

	sourceOrigin := proofarith.DyVec(pp.frame.Origin())
	placedOrigin := proofarith.DvTransform(pp.xform, sourceOrigin)
	image := func(direction r3.Vec) proofarith.DyV3 {
		point := proofarith.DvAdd(sourceOrigin, proofarith.DyVec(direction))
		return proofarith.DvSub(proofarith.DvTransform(pp.xform, point), placedOrigin)
	}
	radialU, radialV, axis := image(pp.frame.U()), image(pp.frame.V()), image(pp.frame.N())
	centerLocal := proofarith.DvAdd(sourceOrigin, proofarith.DvAdd(
		proofarith.DvScale(proofarith.DyVec(pp.frame.U()), proofarith.DyOfFinite(circle.Center.U)),
		proofarith.DvScale(proofarith.DyVec(pp.frame.V()), proofarith.DyOfFinite(circle.Center.V))))
	centerWorld := proofarith.DvTransform(pp.xform, centerLocal)
	requestedOrigin := proofarith.DyVec(plane.Origin())
	normal := proofarith.DvCross(proofarith.DyVec(plane.U()), proofarith.DyVec(plane.V()))
	denominator := proofarith.DvDot(normal, axis).Rat()
	if denominator.Sign() == 0 {
		return nil, 0, false
	}
	r := proofarith.FloatRat(radius)
	zCenter := sectionRatQuo(new(big.Rat).Neg(
		proofarith.DvDot(normal, proofarith.DvSub(centerWorld, requestedOrigin)).Rat()), denominator)
	zCos := sectionRatQuo(new(big.Rat).Neg(sectionRatMul(r,
		proofarith.DvDot(normal, radialU).Rat())), denominator)
	zSin := sectionRatQuo(new(big.Rat).Neg(sectionRatMul(r,
		proofarith.DvDot(normal, radialV).Rat())), denominator)
	// |a cos(theta)+b sin(theta)| <= |a|+|b|. Strict containment
	// keeps the complete ellipse away from both cap faces.
	zReach := sectionRatAdd(new(big.Rat).Abs(zCos), new(big.Rat).Abs(zSin))
	if sectionRatSub(zCenter, zReach).Cmp(proofarith.FloatRat(pp.z0)) <= 0 ||
		sectionRatAdd(zCenter, zReach).Cmp(proofarith.FloatRat(pp.z1)) >= 0 {
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
	a, b := sectionRatVector(localCenter, localCos), sectionRatVector(localCenter, localSin)
	orientation := sectionRatCross(a, b).Sign()
	if orientation == 0 {
		return nil, 0, false
	}
	for _, perQuadrant := range [...]int{8, 16, 32} {
		polygon, delta, ok := obliqueCirclePolygon(localCenter, a, b, orientation,
			plane, perQuadrant)
		if ok {
			return polygon, delta, true
		}
	}
	return nil, 0, false
}

func obliqueCircleFinite(values ...float64) bool {
	for _, value := range values {
		if math.IsNaN(value) || math.IsInf(value, 0) {
			return false
		}
	}
	return true
}

func obliqueCircleWorld(v proofarith.DyV3) [3]*big.Rat {
	return [3]*big.Rat{v[0].Rat(), v[1].Rat(), v[2].Rat()}
}

func obliqueCircleWorldAdd(a, b [3]*big.Rat) [3]*big.Rat {
	return [3]*big.Rat{sectionRatAdd(a[0], b[0]), sectionRatAdd(a[1], b[1]),
		sectionRatAdd(a[2], b[2])}
}

func obliqueCircleWorldScale(v [3]*big.Rat, k *big.Rat) [3]*big.Rat {
	return [3]*big.Rat{sectionRatMul(v[0], k), sectionRatMul(v[1], k),
		sectionRatMul(v[2], k)}
}

// obliqueCircleUnit samples the unit circle with the rational half-angle
// parameter t in [0,1). Quarter-turns cover the other three quadrants.
func obliqueCircleUnit(quadrant, numerator, denominator int) sectionRatPoint {
	t := big.NewRat(int64(numerator), int64(denominator))
	t2 := sectionRatMul(t, t)
	d := sectionRatAdd(big.NewRat(1, 1), t2)
	x := sectionRatQuo(sectionRatSub(big.NewRat(1, 1), t2), d)
	y := sectionRatQuo(sectionRatMul(big.NewRat(2, 1), t), d)
	switch quadrant {
	case 1:
		return sectionRatPoint{u: new(big.Rat).Neg(y), v: x}
	case 2:
		return sectionRatPoint{u: new(big.Rat).Neg(x), v: new(big.Rat).Neg(y)}
	case 3:
		return sectionRatPoint{u: y, v: new(big.Rat).Neg(x)}
	default:
		return sectionRatPoint{u: x, v: y}
	}
}

// Each sampled circular arc and its chord are within d²/4 for a unit
// circle, where d is the chord length. The affine map and vertex rounding
// carry that two-sided gap to the exact ellipse and its held line polygon.
func obliqueCirclePolygon(center, a, b sectionRatPoint, orientation int,
	plane r3.Frame, perQuadrant int) ([]Point2, float64, bool) {
	unit := make([]sectionRatPoint, 0, 4*perQuadrant)
	for quadrant := range 4 {
		for j := range perQuadrant {
			unit = append(unit, obliqueCircleUnit(quadrant, j, perQuadrant))
		}
	}
	if orientation < 0 {
		slices.Reverse(unit)
	}
	exact := make([]sectionRatPoint, len(unit))
	maxChordSquared := new(big.Rat)
	for i, point := range unit {
		exact[i] = sectionRatPoint{
			u: sectionRatAdd(center.u, sectionRatAdd(sectionRatMul(a.u, point.u), sectionRatMul(b.u, point.v))),
			v: sectionRatAdd(center.v, sectionRatAdd(sectionRatMul(a.v, point.u), sectionRatMul(b.v, point.v))),
		}
		next := unit[(i+1)%len(unit)]
		dx, dy := sectionRatSub(next.u, point.u), sectionRatSub(next.v, point.v)
		d2 := sectionRatAdd(sectionRatMul(dx, dx), sectionRatMul(dy, dy))
		if d2.Cmp(maxChordSquared) > 0 {
			maxChordSquared = d2
		}
	}
	sagitta := sectionRatQuo(maxChordSquared, big.NewRat(4, 1))
	uGap := sectionRatMul(sagitta, sectionRatAdd(new(big.Rat).Abs(a.u), new(big.Rat).Abs(b.u)))
	vGap := sectionRatMul(sagitta, sectionRatAdd(new(big.Rat).Abs(a.v), new(big.Rat).Abs(b.v)))
	uNorm := proofbound.DvLenUpper(proofarith.DyVec(plane.U()))
	vNorm := proofbound.DvLenUpper(proofarith.DyVec(plane.V()))
	curveLocal := proofbound.RatFloatUp(sectionRatAdd(uGap, vGap))
	curveWorld := proofbound.AbsSumUpper(proofbound.ProductUpper(proofbound.RatFloatUp(uGap), uNorm),
		proofbound.ProductUpper(proofbound.RatFloatUp(vGap), vNorm))
	proxy := make([]Point2, len(exact))
	vertexLocal, vertexWorld := float64(0), float64(0)
	for i, point := range exact {
		u, _ := point.u.Float64()
		v, _ := point.v.Float64()
		if !obliqueCircleFinite(u, v) {
			return nil, 0, false
		}
		proxy[i] = Point2{U: u, V: v}
		uError := proofarith.RationalFloatError(point.u, u)
		vError := proofarith.RationalFloatError(point.v, v)
		vertexLocal = max(vertexLocal, proofbound.AbsSumUpper(uError, vError))
		vertexWorld = max(vertexWorld, proofbound.AbsSumUpper(
			proofbound.ProductUpper(uError, uNorm), proofbound.ProductUpper(vError, vNorm)))
	}
	delta := proofbound.UpRound(max(proofbound.AbsSumUpper(curveLocal, vertexLocal),
		proofbound.AbsSumUpper(curveWorld, vertexWorld)))
	if !obliqueCircleFinite(delta) || delta <= 0 || delta > 1.0/20 ||
		!sectionMatchedConvex(exact, proxy) {
		return nil, 0, false
	}
	for i, point := range exact {
		next := exact[(i+1)%len(exact)]
		edgeL1 := sectionRatAdd(new(big.Rat).Abs(sectionRatSub(next.u, point.u)),
			new(big.Rat).Abs(sectionRatSub(next.v, point.v)))
		if edgeL1.Sign() <= 0 || sectionRatMul(proofarith.FloatRat(delta), big.NewRat(4, 1)).Cmp(edgeL1) >= 0 {
			return nil, 0, false
		}
	}
	return proxy, delta, true
}
