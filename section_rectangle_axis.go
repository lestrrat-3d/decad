package decad

import (
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
)

// cardinalRectangleSectionRecord derives the exact section of an axis-aligned
// rectangular prism on a cardinal plane parallel to its sweep axis.
func cardinalRectangleSectionRecord(pp prismPayload, plane r3.Frame) (profileRecord, bool) {
	u0, u1, v0, v1, ok := sectionRectangleBounds(pp.profile)
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

// sectionRectangleBounds accepts only a complete four-line rectangle in its
// source coordinates. Sketch has already certified the source profile.
func sectionRectangleBounds(profile profileRecord) (float64, float64, float64, float64, bool) {
	if len(profile.Holes) != 0 || len(profile.Outer.Segments) != 4 {
		return 0, 0, 0, 0, false
	}
	first, ok := profile.Outer.Segments[0].(lineSeg)
	if !ok {
		return 0, 0, 0, 0, false
	}
	u0, u1 := first.Start.U, first.Start.U
	v0, v1 := first.Start.V, first.Start.V
	var lines [4]lineSeg
	for i, segment := range profile.Outer.Segments {
		line, ok := segment.(lineSeg)
		if !ok || !((line.TStart == 0 && line.TEnd == 1) || (line.TStart == 1 && line.TEnd == 0)) {
			return 0, 0, 0, 0, false
		}
		lines[i] = line
		for _, point := range [...]Point2{line.Start, line.End} {
			if point.U < u0 {
				u0 = point.U
			}
			if point.U > u1 {
				u1 = point.U
			}
			if point.V < v0 {
				v0 = point.V
			}
			if point.V > v1 {
				v1 = point.V
			}
		}
	}
	if u0 >= u1 || v0 >= v1 {
		return 0, 0, 0, 0, false
	}
	var sides [4]bool
	for _, line := range lines {
		index := -1
		switch {
		case line.Start.V == v0 && line.End.V == v0 &&
			((line.Start.U == u0 && line.End.U == u1) || (line.Start.U == u1 && line.End.U == u0)):
			index = 0
		case line.Start.U == u1 && line.End.U == u1 &&
			((line.Start.V == v0 && line.End.V == v1) || (line.Start.V == v1 && line.End.V == v0)):
			index = 1
		case line.Start.V == v1 && line.End.V == v1 &&
			((line.Start.U == u0 && line.End.U == u1) || (line.Start.U == u1 && line.End.U == u0)):
			index = 2
		case line.Start.U == u0 && line.End.U == u0 &&
			((line.Start.V == v0 && line.End.V == v1) || (line.Start.V == v1 && line.End.V == v0)):
			index = 3
		}
		if index < 0 || sides[index] {
			return 0, 0, 0, 0, false
		}
		sides[index] = true
	}
	return u0, u1, v0, v1, true
}
