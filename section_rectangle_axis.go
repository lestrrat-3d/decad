package decad

import (
	"github.com/lestrrat-3d/decad/internal/momentinput"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
)

// cardinalRectangleSectionRecord derives the exact section of an axis-aligned
// rectangular prism on a cardinal plane parallel to its sweep axis.
func cardinalRectangleSectionRecord(pp prismPayload, plane r3.Frame) (profileRecord, bool) {
	u0, u1, v0, v1, ok := momentinput.RectangleBounds(pp.profile)
	if !ok || pp.blendKind != "" || len(pp.blendSegs) != 0 || pp.z0 >= pp.z1 {
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

	point := func(u, v, z float64) proofarith.DyV3 {
		local := proofarith.DvAdd(sourceOrigin, proofarith.DvAdd(
			proofarith.DvScale(proofarith.DyVec(pp.frame.U()), proofarith.DyOfFinite(u)),
			proofarith.DvAdd(
				proofarith.DvScale(proofarith.DyVec(pp.frame.V()), proofarith.DyOfFinite(v)),
				proofarith.DvScale(proofarith.DyVec(pp.frame.N()), proofarith.DyOfFinite(z)),
			),
		))
		return proofarith.DvTransform(pp.xform, local)
	}
	low := proofarith.DvSub(point(u0, v0, pp.z0), proofarith.DyVec(plane.Origin()))
	high := proofarith.DvSub(point(u1, v1, pp.z0), proofarith.DyVec(plane.Origin()))
	cutLow, cutHigh := proofarith.DvDot(low, requestN), proofarith.DvDot(high, requestN)
	if cutLow.Sign() == 0 || cutHigh.Sign() == 0 || cutLow.Sign() == cutHigh.Sign() {
		return profileRecord{}, false
	}
	loU, hiU := proofarith.DvDot(low, requestU), proofarith.DvDot(high, requestU)
	if proofarith.DyCmp(loU, hiU) > 0 {
		loU, hiU = hiU, loU
	}
	highCap := proofarith.DvSub(point(u0, v0, pp.z1), proofarith.DyVec(plane.Origin()))
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
	a, b := Point2{U: x0, V: y0}, Point2{U: x1, V: y0}
	c, d := Point2{U: x1, V: y1}, Point2{U: x0, V: y1}
	return profileRecord{Outer: loopRecord{Segments: []curveSegment{
		lineSeg{Start: a, End: b, TStart: 0, TEnd: 1},
		lineSeg{Start: b, End: c, TStart: 0, TEnd: 1},
		lineSeg{Start: c, End: d, TStart: 0, TEnd: 1},
		lineSeg{Start: d, End: a, TStart: 0, TEnd: 1},
	}}}, true
}
