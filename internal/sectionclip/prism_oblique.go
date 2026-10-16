package sectionclip

import (
	"math"
	"math/big"
	"sort"

	"github.com/lestrrat-3d/decad/internal/momentinput"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// ObliqueRectangleClip intersects the exact held rectangular prism with the
// exact affine plane spanned by the requested held U and V vectors. It returns
// a rounded plane-local polygon and a displacement covering every matched
// point of its exact source boundary in both local and world coordinates.
func ObliqueRectangleClip(pp prismPayload, plane r3.Frame) ([]planePoint, float64, bool) {
	u0, u1, v0, v1, ok := momentinput.RectangleBounds(pp.Profile)
	if !ok || pp.BlendKind != "" || len(pp.BlendSegs) != 0 || pp.Z0 >= pp.Z1 {
		return nil, 0, false
	}
	base := proofarith.DyVec(pp.Frame.Origin())
	axis := [3]proofarith.DyV3{
		proofarith.DyVec(pp.Frame.U()), proofarith.DyVec(pp.Frame.V()), proofarith.DyVec(pp.Frame.N()),
	}
	limits := [3][2]float64{{u0, u1}, {v0, v1}, {pp.Z0, pp.Z1}}
	requestedOrigin := proofarith.DyVec(plane.Origin())
	normal := proofarith.DvCross(proofarith.DyVec(plane.U()), proofarith.DyVec(plane.V()))
	if proofarith.DvIsZero(normal) {
		return nil, 0, false
	}
	var corners [8]proofarith.DyV3
	var side [8]proofarith.Dyadic
	for corner := range corners {
		point := base
		for coordinate := range 3 {
			choice := (corner >> coordinate) & 1
			point = proofarith.DvAdd(point, proofarith.DvScale(axis[coordinate],
				proofarith.DyOfFinite(limits[coordinate][choice])))
		}
		corners[corner] = proofarith.DvTransform(pp.Xform, point)
		side[corner] = proofarith.DvDot(proofarith.DvSub(corners[corner], requestedOrigin), normal)
		// Contact at a box vertex needs a separate topology proof.
		if side[corner].IsZero() {
			return nil, 0, false
		}
	}
	var exact []SectionRatPoint
	for a := range corners {
		for coordinate := range 3 {
			b := a ^ (1 << coordinate)
			if b < a || side[a].Sign() == side[b].Sign() {
				continue
			}
			t := SectionRatQuo(side[a].Rat(), proofarith.DySubScalar(side[a], side[b]).Rat())
			var world [3]*big.Rat
			for component := range 3 {
				world[component] = SectionRatAdd(corners[a][component].Rat(),
					SectionRatMul(t, SectionRatSub(corners[b][component].Rat(), corners[a][component].Rat())))
			}
			point, ok := SectionPlaneCoordinates(world, plane)
			if !ok {
				return nil, 0, false
			}
			duplicate := false
			for _, other := range exact {
				if point.U.Cmp(other.U) == 0 && point.V.Cmp(other.V) == 0 {
					duplicate = true
					break
				}
			}
			if !duplicate {
				exact = append(exact, point)
			}
		}
	}
	if len(exact) < 3 || len(exact) > 6 {
		return nil, 0, false
	}
	sectionSortRatPolygon(exact)
	proxy := make([]planePoint, len(exact))
	delta := float64(0)
	uNorm := proofbound.DvLenUpper(proofarith.DyVec(plane.U()))
	vNorm := proofbound.DvLenUpper(proofarith.DyVec(plane.V()))
	for i, point := range exact {
		u, _ := point.U.Float64()
		v, _ := point.V.Float64()
		if math.IsNaN(u) || math.IsInf(u, 0) || math.IsNaN(v) || math.IsInf(v, 0) {
			return nil, 0, false
		}
		proxy[i] = planePoint{U: u, V: v}
		uError := proofarith.RationalFloatError(point.U, u)
		vError := proofarith.RationalFloatError(point.V, v)
		local := proofbound.AbsSumUpper(uError, vError)
		world := proofbound.AbsSumUpper(proofbound.ProductUpper(uError, uNorm),
			proofbound.ProductUpper(vError, vNorm))
		delta = max(delta, local, world)
	}
	if !SectionMatchedConvex(exact, proxy) || math.IsInf(delta, 0) || math.IsNaN(delta) {
		return nil, 0, false
	}
	delta = proofbound.UpRound(delta)
	// A loop whose coordinate error is comparable to a source edge has no
	// useful local feature scale for later reconstruction and verification.
	for i, point := range exact {
		next := exact[(i+1)%len(exact)]
		uSpan := new(big.Rat).Abs(SectionRatSub(next.U, point.U))
		vSpan := new(big.Rat).Abs(SectionRatSub(next.V, point.V))
		edgeL1 := SectionRatAdd(uSpan, vSpan)
		if edgeL1.Sign() <= 0 || SectionRatMul(proofarith.FloatRat(delta), big.NewRat(4, 1)).Cmp(edgeL1) >= 0 {
			return nil, 0, false
		}
	}
	return proxy, delta, true
}

// SectionPlaneCoordinates returns the plane-local U,V of an exact world point,
// or false when the point is off the plane or the frame's U and V are parallel.
func SectionPlaneCoordinates(world [3]*big.Rat, plane r3.Frame) (SectionRatPoint, bool) {
	origin := proofarith.DyVec(plane.Origin())
	uAxis := proofarith.DyVec(plane.U())
	vAxis := proofarith.DyVec(plane.V())
	for a := range 3 {
		for b := a + 1; b < 3; b++ {
			det := SectionRatSub(SectionRatMul(uAxis[a].Rat(), vAxis[b].Rat()),
				SectionRatMul(uAxis[b].Rat(), vAxis[a].Rat()))
			if det.Sign() == 0 {
				continue
			}
			da := SectionRatSub(world[a], origin[a].Rat())
			db := SectionRatSub(world[b], origin[b].Rat())
			u := SectionRatQuo(SectionRatSub(SectionRatMul(da, vAxis[b].Rat()),
				SectionRatMul(db, vAxis[a].Rat())), det)
			v := SectionRatQuo(SectionRatSub(SectionRatMul(uAxis[a].Rat(), db),
				SectionRatMul(uAxis[b].Rat(), da)), det)
			for component := range 3 {
				lifted := SectionRatAdd(origin[component].Rat(),
					SectionRatAdd(SectionRatMul(u, uAxis[component].Rat()), SectionRatMul(v, vAxis[component].Rat())))
				if lifted.Cmp(world[component]) != 0 {
					return SectionRatPoint{}, false
				}
			}
			return SectionRatPoint{U: u, V: v}, true
		}
	}
	return SectionRatPoint{}, false
}

func sectionSortRatPolygon(points []SectionRatPoint) {
	center := SectionRatPoint{U: new(big.Rat), V: new(big.Rat)}
	for _, point := range points {
		center.U.Add(center.U, point.U)
		center.V.Add(center.V, point.V)
	}
	count := new(big.Rat).SetInt64(int64(len(points)))
	center.U.Quo(center.U, count)
	center.V.Quo(center.V, count)
	half := func(p SectionRatPoint) int {
		if p.V.Sign() > 0 || (p.V.Sign() == 0 && p.U.Sign() >= 0) {
			return 0
		}
		return 1
	}
	sort.Slice(points, func(i, j int) bool {
		a, b := SectionRatVector(center, points[i]), SectionRatVector(center, points[j])
		if half(a) != half(b) {
			return half(a) < half(b)
		}
		return SectionRatCross(a, b).Sign() > 0
	})
}

// ObliqueCircleClip bounds a complete ellipse cut from an exact whole-circle
// prism. The requested plane is read as the exact span of its held U and V.
// Cuts meeting either cap, and planes parallel to the sweep axis, refuse.
func ObliqueCircleClip(pp prismPayload, plane r3.Frame) ([]planePoint, float64, bool) {
	if len(pp.Profile.Holes) != 0 || len(pp.Profile.Outer.Segments) != 1 ||
		pp.BlendKind != "" || len(pp.BlendSegs) != 0 || pp.Z0 >= pp.Z1 {
		return nil, 0, false
	}
	circle, ok := pp.Profile.Outer.Segments[0].(circleSeg)
	if !ok || !circle.CCW || circle.TStart != 0 || circle.TEnd != 1 {
		return nil, 0, false
	}
	radius, err := circle.Radius.In(units.Millimeter)
	if err != nil || radius <= 0 || !ObliqueCircleFinite(radius, circle.Center.U, circle.Center.V,
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
	zCenter := SectionRatQuo(new(big.Rat).Neg(
		proofarith.DvDot(normal, proofarith.DvSub(centerWorld, requestedOrigin)).Rat()), denominator)
	zCos := SectionRatQuo(new(big.Rat).Neg(SectionRatMul(r,
		proofarith.DvDot(normal, radialU).Rat())), denominator)
	zSin := SectionRatQuo(new(big.Rat).Neg(SectionRatMul(r,
		proofarith.DvDot(normal, radialV).Rat())), denominator)
	// |a cos(theta)+b sin(theta)| <= |a|+|b|. Strict containment
	// keeps the complete ellipse away from both cap faces.
	zReach := SectionRatAdd(new(big.Rat).Abs(zCos), new(big.Rat).Abs(zSin))
	if SectionRatSub(zCenter, zReach).Cmp(proofarith.FloatRat(pp.Z0)) <= 0 ||
		SectionRatAdd(zCenter, zReach).Cmp(proofarith.FloatRat(pp.Z1)) >= 0 {
		return nil, 0, false
	}
	center := ObliqueCircleWorldAdd(ObliqueCircleWorld(centerWorld),
		ObliqueCircleWorldScale(ObliqueCircleWorld(axis), zCenter))
	cosVector := ObliqueCircleWorldAdd(ObliqueCircleWorldScale(ObliqueCircleWorld(radialU), r),
		ObliqueCircleWorldScale(ObliqueCircleWorld(axis), zCos))
	sinVector := ObliqueCircleWorldAdd(ObliqueCircleWorldScale(ObliqueCircleWorld(radialV), r),
		ObliqueCircleWorldScale(ObliqueCircleWorld(axis), zSin))
	localCenter, ok := SectionPlaneCoordinates(center, plane)
	if !ok {
		return nil, 0, false
	}
	localCos, ok := SectionPlaneCoordinates(ObliqueCircleWorldAdd(center, cosVector), plane)
	if !ok {
		return nil, 0, false
	}
	localSin, ok := SectionPlaneCoordinates(ObliqueCircleWorldAdd(center, sinVector), plane)
	if !ok {
		return nil, 0, false
	}
	a, b := SectionRatVector(localCenter, localCos), SectionRatVector(localCenter, localSin)
	orientation := SectionRatCross(a, b).Sign()
	if orientation == 0 {
		return nil, 0, false
	}
	for _, perQuadrant := range [...]int{8, 16, 32} {
		polygon, delta, ok := ObliqueCirclePolygon(localCenter, a, b, orientation,
			plane, perQuadrant, 0, 0)
		if ok {
			return polygon, delta, true
		}
	}
	return nil, 0, false
}

// ObliqueCircleWorld converts a dyadic world vector to exact rationals.
func ObliqueCircleWorld(v proofarith.DyV3) [3]*big.Rat {
	return [3]*big.Rat{v[0].Rat(), v[1].Rat(), v[2].Rat()}
}

// ObliqueCircleWorldAdd returns a+b componentwise.
func ObliqueCircleWorldAdd(a, b [3]*big.Rat) [3]*big.Rat {
	return [3]*big.Rat{SectionRatAdd(a[0], b[0]), SectionRatAdd(a[1], b[1]),
		SectionRatAdd(a[2], b[2])}
}

// ObliqueCircleWorldScale returns k*v componentwise.
func ObliqueCircleWorldScale(v [3]*big.Rat, k *big.Rat) [3]*big.Rat {
	return [3]*big.Rat{SectionRatMul(v[0], k), SectionRatMul(v[1], k),
		SectionRatMul(v[2], k)}
}
