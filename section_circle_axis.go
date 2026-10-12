package decad

import (
	"math"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// cardinalCircleSectionRecord derives a rectangle from a whole-circle prism
// and a cardinal plane with an exact chord. Every image and corner is compared
// in exact dyadic arithmetic before Sketch receives held coordinates.
func cardinalCircleSectionRecord(pp prismPayload, plane r3.Frame) (profileRecord, bool) {
	if len(pp.profile.Holes) != 0 || len(pp.profile.Outer.Segments) != 1 ||
		pp.blendKind != "" || len(pp.blendSegs) != 0 || pp.z0 >= pp.z1 {
		return profileRecord{}, false
	}
	circle, ok := pp.profile.Outer.Segments[0].(circleSeg)
	if !ok || !circle.CCW || circle.TStart != 0 || circle.TEnd != 1 {
		return profileRecord{}, false
	}
	radius, err := circle.Radius.In(units.Millimeter)
	if err != nil || radius <= 0 || math.IsNaN(radius) || math.IsInf(radius, 0) {
		return profileRecord{}, false
	}

	sourceOrigin := proofarith.DyVec(pp.frame.Origin())
	placedOrigin := proofarith.DvTransform(pp.xform, sourceOrigin)
	image := func(direction r3.Vec) proofarith.DyV3 {
		point := proofarith.DvAdd(sourceOrigin, proofarith.DyVec(direction))
		return proofarith.DvSub(proofarith.DvTransform(pp.xform, point), placedOrigin)
	}
	radialU, radialV, axis := image(pp.frame.U()), image(pp.frame.V()), image(pp.frame.N())
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
		proofarith.DvScale(proofarith.DyVec(pp.frame.U()), proofarith.DyOfFinite(circle.Center.U)),
		proofarith.DvScale(proofarith.DyVec(pp.frame.V()), proofarith.DyOfFinite(circle.Center.V)),
	))
	centerWorld := proofarith.DvTransform(pp.xform, centerLocal)
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
	vLo := proofarith.DyAdd(vCenter, proofarith.DyMul(axisSense, proofarith.DyOfFinite(pp.z0)))
	vHi := proofarith.DyAdd(vCenter, proofarith.DyMul(axisSense, proofarith.DyOfFinite(pp.z1)))
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

	bottomLeft := Point2{U: u0, V: v0}
	bottomRight := Point2{U: u1, V: v0}
	topRight := Point2{U: u1, V: v1}
	topLeft := Point2{U: u0, V: v1}
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
