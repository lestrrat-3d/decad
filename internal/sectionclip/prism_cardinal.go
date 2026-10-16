package sectionclip

import (
	"math"

	"github.com/lestrrat-3d/decad/internal/momentinput"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// CardinalCircleSectionRecord derives a rectangle from a whole-circle prism
// and a cardinal plane with an exact chord. Every image and corner is compared
// in exact dyadic arithmetic before Sketch receives held coordinates.
func CardinalCircleSectionRecord(pp prismPayload, plane r3.Frame) (profileRecord, bool) {
	if len(pp.Profile.Holes) != 0 || len(pp.Profile.Outer.Segments) != 1 ||
		pp.BlendKind != "" || len(pp.BlendSegs) != 0 || pp.Z0 >= pp.Z1 {
		return profileRecord{}, false
	}
	circle, ok := pp.Profile.Outer.Segments[0].(circleSeg)
	if !ok || !circle.CCW || circle.TStart != 0 || circle.TEnd != 1 {
		return profileRecord{}, false
	}
	radius, err := circle.Radius.In(units.Millimeter)
	if err != nil || radius <= 0 || math.IsNaN(radius) || math.IsInf(radius, 0) {
		return profileRecord{}, false
	}

	sourceOrigin := proofarith.DyVec(pp.Frame.Origin())
	placedOrigin := proofarith.DvTransform(pp.Xform, sourceOrigin)
	image := func(direction r3.Vec) proofarith.DyV3 {
		point := proofarith.DvAdd(sourceOrigin, proofarith.DyVec(direction))
		return proofarith.DvSub(proofarith.DvTransform(pp.Xform, point), placedOrigin)
	}
	radialU, radialV, axis := image(pp.Frame.U()), image(pp.Frame.V()), image(pp.Frame.N())
	if !sectionCardinalDirection(radialU) || !sectionCardinalDirection(radialV) ||
		!sectionCardinalDirection(axis) || !sectionSignedDirection(proofarith.DvCross(radialU, radialV), axis) {
		return profileRecord{}, false
	}
	requestU, requestV, requestN := proofarith.DyVec(plane.U()), proofarith.DyVec(plane.V()), proofarith.DyVec(plane.N())
	if !sectionSignedDirection(requestV, axis) ||
		(!sectionSignedDirection(requestU, radialU) && !sectionSignedDirection(requestU, radialV)) ||
		!proofarith.DvEqual(requestN, proofarith.DvCross(requestU, requestV)) {
		return profileRecord{}, false
	}

	centerLocal := proofarith.DvAdd(sourceOrigin, proofarith.DvAdd(
		proofarith.DvScale(proofarith.DyVec(pp.Frame.U()), proofarith.DyOfFinite(circle.Center.U)),
		proofarith.DvScale(proofarith.DyVec(pp.Frame.V()), proofarith.DyOfFinite(circle.Center.V)),
	))
	centerWorld := proofarith.DvTransform(pp.Xform, centerLocal)
	delta := proofarith.DvSub(centerWorld, proofarith.DyVec(plane.Origin()))
	uCenter := proofarith.DvDot(delta, requestU)
	vCenter := proofarith.DvDot(delta, requestV)
	r := proofarith.DyOfFinite(radius)
	halfChord := r
	if offset := proofarith.DvDot(delta, requestN); !offset.IsZero() {
		q := proofarith.DySubScalar(proofarith.DyMul(r, r), proofarith.DyMul(offset, offset))
		if q.Sign() <= 0 {
			return profileRecord{}, false
		}
		h := proofarith.DySqrtDown(q)
		if !proofarith.DySquareEquals(h, q) {
			return profileRecord{}, false
		}
		halfChord = proofarith.DyOfFinite(h)
	}
	uLo, uHi := proofarith.DySubScalar(uCenter, halfChord), proofarith.DyAdd(uCenter, halfChord)
	axisSense := proofarith.DvDot(axis, requestV)
	vLo := proofarith.DyAdd(vCenter, proofarith.DyMul(axisSense, proofarith.DyOfFinite(pp.Z0)))
	vHi := proofarith.DyAdd(vCenter, proofarith.DyMul(axisSense, proofarith.DyOfFinite(pp.Z1)))
	if proofarith.DyCmp(vLo, vHi) > 0 {
		vLo, vHi = vHi, vLo
	}
	u0, u0Exact := uLo.Float64()
	u1, u1Exact := uHi.Float64()
	v0, v0Exact := vLo.Float64()
	v1, v1Exact := vHi.Float64()
	if !u0Exact || !u1Exact || !v0Exact || !v1Exact || u0 >= u1 || v0 >= v1 {
		return profileRecord{}, false
	}

	bottomLeft := planePoint{U: u0, V: v0}
	bottomRight := planePoint{U: u1, V: v0}
	topRight := planePoint{U: u1, V: v1}
	topLeft := planePoint{U: u0, V: v1}
	return profileRecord{Outer: loopRecord{Segments: []curveSegment{
		lineSeg{Start: bottomLeft, End: bottomRight, TStart: 0, TEnd: 1},
		lineSeg{Start: bottomRight, End: topRight, TStart: 0, TEnd: 1},
		lineSeg{Start: topRight, End: topLeft, TStart: 0, TEnd: 1},
		lineSeg{Start: topLeft, End: bottomLeft, TStart: 0, TEnd: 1},
	}}}, true
}

func sectionCardinalDirection(direction proofarith.DyV3) bool {
	for _, candidate := range [...]r3.Vec{
		r3.NewVec(1, 0, 0), r3.NewVec(-1, 0, 0),
		r3.NewVec(0, 1, 0), r3.NewVec(0, -1, 0),
		r3.NewVec(0, 0, 1), r3.NewVec(0, 0, -1),
	} {
		if proofarith.DvEqual(direction, proofarith.DyVec(candidate)) {
			return true
		}
	}
	return false
}

func sectionSignedDirection(a, b proofarith.DyV3) bool {
	return proofarith.DvEqual(a, b) ||
		proofarith.DvEqual(a, proofarith.DvScale(b, proofarith.DyOfFinite(-1)))
}

// CardinalRectangleSectionRecord derives the exact section of an axis-aligned
// rectangular prism on a cardinal plane parallel to its sweep axis.
func CardinalRectangleSectionRecord(pp prismPayload, plane r3.Frame) (profileRecord, bool) {
	u0, u1, v0, v1, ok := momentinput.RectangleBounds(pp.Profile)
	if !ok || pp.BlendKind != "" || len(pp.BlendSegs) != 0 || pp.Z0 >= pp.Z1 {
		return profileRecord{}, false
	}
	sourceOrigin := proofarith.DyVec(pp.Frame.Origin())
	placedOrigin := proofarith.DvTransform(pp.Xform, sourceOrigin)
	image := func(direction r3.Vec) proofarith.DyV3 {
		point := proofarith.DvAdd(sourceOrigin, proofarith.DyVec(direction))
		return proofarith.DvSub(proofarith.DvTransform(pp.Xform, point), placedOrigin)
	}
	radialU, radialV, axis := image(pp.Frame.U()), image(pp.Frame.V()), image(pp.Frame.N())
	if !sectionCardinalDirection(radialU) || !sectionCardinalDirection(radialV) ||
		!sectionCardinalDirection(axis) || !sectionSignedDirection(proofarith.DvCross(radialU, radialV), axis) {
		return profileRecord{}, false
	}
	requestU, requestV, requestN := proofarith.DyVec(plane.U()), proofarith.DyVec(plane.V()), proofarith.DyVec(plane.N())
	if !sectionSignedDirection(requestV, axis) ||
		(!sectionSignedDirection(requestU, radialU) && !sectionSignedDirection(requestU, radialV)) ||
		!proofarith.DvEqual(requestN, proofarith.DvCross(requestU, requestV)) {
		return profileRecord{}, false
	}

	point := func(u, v, z float64) proofarith.DyV3 {
		local := proofarith.DvAdd(sourceOrigin, proofarith.DvAdd(
			proofarith.DvScale(proofarith.DyVec(pp.Frame.U()), proofarith.DyOfFinite(u)),
			proofarith.DvAdd(
				proofarith.DvScale(proofarith.DyVec(pp.Frame.V()), proofarith.DyOfFinite(v)),
				proofarith.DvScale(proofarith.DyVec(pp.Frame.N()), proofarith.DyOfFinite(z)),
			),
		))
		return proofarith.DvTransform(pp.Xform, local)
	}
	low := proofarith.DvSub(point(u0, v0, pp.Z0), proofarith.DyVec(plane.Origin()))
	high := proofarith.DvSub(point(u1, v1, pp.Z0), proofarith.DyVec(plane.Origin()))
	cutLow, cutHigh := proofarith.DvDot(low, requestN), proofarith.DvDot(high, requestN)
	if cutLow.Sign() == 0 || cutHigh.Sign() == 0 || cutLow.Sign() == cutHigh.Sign() {
		return profileRecord{}, false
	}
	loU, hiU := proofarith.DvDot(low, requestU), proofarith.DvDot(high, requestU)
	if proofarith.DyCmp(loU, hiU) > 0 {
		loU, hiU = hiU, loU
	}
	highCap := proofarith.DvSub(point(u0, v0, pp.Z1), proofarith.DyVec(plane.Origin()))
	loV, hiV := proofarith.DvDot(low, requestV), proofarith.DvDot(highCap, requestV)
	if proofarith.DyCmp(loV, hiV) > 0 {
		loV, hiV = hiV, loV
	}
	x0, x0Exact := loU.Float64()
	x1, x1Exact := hiU.Float64()
	y0, y0Exact := loV.Float64()
	y1, y1Exact := hiV.Float64()
	if !x0Exact || !x1Exact || !y0Exact || !y1Exact || x0 >= x1 || y0 >= y1 {
		return profileRecord{}, false
	}
	a, b := planePoint{U: x0, V: y0}, planePoint{U: x1, V: y0}
	c, d := planePoint{U: x1, V: y1}, planePoint{U: x0, V: y1}
	return profileRecord{Outer: loopRecord{Segments: []curveSegment{
		lineSeg{Start: a, End: b, TStart: 0, TEnd: 1},
		lineSeg{Start: b, End: c, TStart: 0, TEnd: 1},
		lineSeg{Start: c, End: d, TStart: 0, TEnd: 1},
		lineSeg{Start: d, End: a, TStart: 0, TEnd: 1},
	}}}, true
}
