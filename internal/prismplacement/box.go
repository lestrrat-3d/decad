package prismplacement

import (
	"github.com/lestrrat-3d/decad/internal/momentinput"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
)

// Box is an axis-aligned box in a reference frame.
type Box struct {
	Lo, Hi [3]float64
}

// ExactBoxInFrame proves that an authenticated, undisplaced rectangular
// source prism is precisely this axis-aligned box in ref. The prism is the
// rectangle [u0, u1] x [v0, v1] swept over [z0, z1] in sourceFrame. Every
// projection is evaluated as a dyadic rational and must be representable as
// a float.
func ExactBoxInFrame(u0, u1, v0, v1, z0, z1 float64, sourceFrame, ref r3.Frame) (Box, bool) {
	refAxes := [3]r3.Vec{ref.U(), ref.V(), ref.N()}
	sourceAxes := [3]r3.Vec{sourceFrame.U(), sourceFrame.V(), sourceFrame.N()}
	// Coordinate projection is an inverse map only for an exactly orthonormal
	// held frame. A rounded rotated frame keeps its existing mesh route.
	for i, direction := range refAxes {
		for j, other := range refAxes {
			want := proofarith.DyOfFinite(0)
			if i == j {
				want = proofarith.DyOfFinite(1)
			}
			if proofarith.DyCmp(proofarith.DvDot(proofarith.DyVec(direction),
				proofarith.DyVec(other)), want) != 0 {
				return Box{}, false
			}
		}
	}
	var used [3]bool
	var mapped [3]int
	var sign [3]float64
	for i, axis := range sourceAxes {
		found := false
		for j, direction := range refAxes {
			if axis != direction && axis != direction.Scale(-1) {
				continue
			}
			if used[j] {
				return Box{}, false
			}
			used[j], found, mapped[i] = true, true, j
			sign[i] = 1
			if axis == direction.Scale(-1) {
				sign[i] = -1
			}
			break
		}
		if !found {
			return Box{}, false
		}
	}
	origin := proofarith.DyVec(sourceFrame.Origin())
	base := proofarith.DvSub(origin, proofarith.DyVec(ref.Origin()))
	var box Box
	for i, values := range [3][2]float64{{u0, u1}, {v0, v1}, {z0, z1}} {
		axis := mapped[i]
		baseCoordinate := proofarith.DvDot(base, proofarith.DyVec(refAxes[axis]))
		for j, value := range values {
			coordinate, exact := proofarith.DyAdd(baseCoordinate,
				proofarith.DyMul(proofarith.DyOfFinite(sign[i]), proofarith.DyOfFinite(value))).Float64()
			if !exact {
				return Box{}, false
			}
			if j == 0 {
				box.Lo[axis], box.Hi[axis] = coordinate, coordinate
			} else {
				box.Lo[axis] = min(box.Lo[axis], coordinate)
				box.Hi[axis] = max(box.Hi[axis], coordinate)
			}
		}
	}
	if box.Lo[0] >= box.Hi[0] || box.Lo[1] >= box.Hi[1] || box.Lo[2] >= box.Hi[2] {
		return Box{}, false
	}
	return box, true
}

// BoxesShareFacePatch requires one separating face and positive overlap on
// both in-plane axes. The first prism's sweep must be an in-plane axis, so
// stacked Union can split the interface at the second prism's two levels.
func BoxesShareFacePatch(a, b Box, sweep int) bool {
	contact := -1
	for axis := range 3 {
		if a.Hi[axis] == b.Lo[axis] || b.Hi[axis] == a.Lo[axis] {
			if contact >= 0 {
				return false // an edge or point contact does not join two solids.
			}
			contact = axis
			continue
		}
		if a.Lo[axis] >= b.Hi[axis] || b.Lo[axis] >= a.Hi[axis] {
			return false
		}
	}
	return contact >= 0 && contact != sweep
}

// BoxProfile returns the rectangular profile of a box's in-plane (u, v)
// extent.
func BoxProfile(box Box) momentinput.Profile {
	a := sectionrecord.Point2{U: box.Lo[0], V: box.Lo[1]}
	b := sectionrecord.Point2{U: box.Hi[0], V: box.Lo[1]}
	c := sectionrecord.Point2{U: box.Hi[0], V: box.Hi[1]}
	d := sectionrecord.Point2{U: box.Lo[0], V: box.Hi[1]}
	return momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
		sectionrecord.LineSeg{Start: a, End: b, TStart: 0, TEnd: 1},
		sectionrecord.LineSeg{Start: b, End: c, TStart: 0, TEnd: 1},
		sectionrecord.LineSeg{Start: c, End: d, TStart: 0, TEnd: 1},
		sectionrecord.LineSeg{Start: d, End: a, TStart: 0, TEnd: 1},
	}}}
}
