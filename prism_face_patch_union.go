package decad

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
)

// tryPrismFacePatchUnion admits two exact rectangular prisms whose interiors
// are disjoint and whose boundaries share a positive-area patch on one face.
// It re-expresses the second box as a prism in the first box's frame, then
// lets the existing prism or stacked Union scenes split the shared patch.
func tryPrismFacePatchUnion(ctx context.Context, a, b *Body) (featurePayload, bool, error) {
	pa, okA := a.payload.(prismPayload)
	pb, okB := b.payload.(prismPayload)
	if !okA || !okB || pa.xform != pb.xform || pa.surfaceResult || pb.surfaceResult {
		return nil, false, nil
	}
	// A1's unequal-level BRep keeps planar box volumes exact. Try that
	// reference first, then the equal-level prism scene if neither frame
	// admits A1.
	for pass := range 2 {
		for _, pair := range [][2]prismPayload{{pa, pb}, {pb, pa}} {
			x, y := pair[0], pair[1]
			if x.frame.N() == y.frame.N() || x.frame.N() == y.frame.N().Scale(-1) {
				continue // class A already reads co-directional boxes.
			}
			xBox, ok := exactPrismBoxInFrame(x, x.frame)
			if !ok {
				continue
			}
			yBox, ok := exactPrismBoxInFrame(y, x.frame)
			if !ok || !boxesShareFacePatch(xBox, yBox, 2) {
				continue
			}
			equalLevels := xBox.lo[2] == yBox.lo[2] && xBox.hi[2] == yBox.hi[2]
			if equalLevels != (pass == 1) {
				continue
			}
			yReframed := prismPayload{
				profile: prismBoxProfile(yBox), frame: x.frame, xform: x.xform,
				z0: yBox.lo[2], z1: yBox.hi[2],
			}
			var payload featurePayload
			var built bool
			var err error
			if equalLevels {
				payload, built, err = tryPrismBoolean(ctx, meshbool.OpUnion,
					&Body{payload: x}, &Body{payload: yReframed})
			} else {
				payload, built, err = tryStackedUnion(ctx, &Body{payload: x}, &Body{payload: yReframed})
			}
			if err != nil || built {
				return payload, built, err
			}
			// A silent miss in this frame may be admitted with the operands
			// reversed: that changes which in-plane axis A1 splits into slabs.
		}
	}
	return nil, false, nil
}

type exactPrismBox struct {
	lo, hi [3]float64
}

// exactPrismBoxInFrame proves that an authenticated, undisplaced rectangular
// source prism is precisely this axis-aligned box in ref. Every projection is
// evaluated as a dyadic rational and must be representable as a float.
func exactPrismBoxInFrame(source prismPayload, ref r3.Frame) (exactPrismBox, bool) {
	r, ok := axisRectangle(source.profile)
	if !ok || source.sectionDelta != 0 || source.z0Delta != 0 || source.z1Delta != 0 ||
		source.blendKind != "" || len(source.blendSegs) != 0 || source.z0 >= source.z1 {
		return exactPrismBox{}, false
	}
	refAxes := [3]r3.Vec{ref.U(), ref.V(), ref.N()}
	sourceAxes := [3]r3.Vec{source.frame.U(), source.frame.V(), source.frame.N()}
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
				return exactPrismBox{}, false
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
				return exactPrismBox{}, false
			}
			used[j], found, mapped[i] = true, true, j
			sign[i] = 1
			if axis == direction.Scale(-1) {
				sign[i] = -1
			}
			break
		}
		if !found {
			return exactPrismBox{}, false
		}
	}
	origin := proofarith.DyVec(source.frame.Origin())
	base := proofarith.DvSub(origin, proofarith.DyVec(ref.Origin()))
	var box exactPrismBox
	for i, values := range [3][2]float64{{r.u0, r.u1}, {r.v0, r.v1}, {source.z0, source.z1}} {
		axis := mapped[i]
		baseCoordinate := proofarith.DvDot(base, proofarith.DyVec(refAxes[axis]))
		for j, value := range values {
			coordinate, exact := proofarith.DyAdd(baseCoordinate,
				proofarith.DyMul(proofarith.DyOfFinite(sign[i]), proofarith.DyOfFinite(value))).Float64()
			if !exact {
				return exactPrismBox{}, false
			}
			if j == 0 {
				box.lo[axis], box.hi[axis] = coordinate, coordinate
			} else {
				box.lo[axis] = min(box.lo[axis], coordinate)
				box.hi[axis] = max(box.hi[axis], coordinate)
			}
		}
	}
	if box.lo[0] >= box.hi[0] || box.lo[1] >= box.hi[1] || box.lo[2] >= box.hi[2] {
		return exactPrismBox{}, false
	}
	return box, true
}

// boxesShareFacePatch requires one separating face and positive overlap on
// both in-plane axes. The first prism's sweep must be an in-plane axis, so
// stacked Union can split the interface at the second prism's two levels.
func boxesShareFacePatch(a, b exactPrismBox, sweep int) bool {
	contact := -1
	for axis := range 3 {
		if a.hi[axis] == b.lo[axis] || b.hi[axis] == a.lo[axis] {
			if contact >= 0 {
				return false // an edge or point contact does not join two solids.
			}
			contact = axis
			continue
		}
		if a.lo[axis] >= b.hi[axis] || b.lo[axis] >= a.hi[axis] {
			return false
		}
	}
	return contact >= 0 && contact != sweep
}

func prismBoxProfile(box exactPrismBox) profileRecord {
	a := Point2{U: box.lo[0], V: box.lo[1]}
	b := Point2{U: box.hi[0], V: box.lo[1]}
	c := Point2{U: box.hi[0], V: box.hi[1]}
	d := Point2{U: box.lo[0], V: box.hi[1]}
	return profileRecord{Outer: loopRecord{Segments: []curveSegment{
		lineSeg{Start: a, End: b, TStart: 0, TEnd: 1},
		lineSeg{Start: b, End: c, TStart: 0, TEnd: 1},
		lineSeg{Start: c, End: d, TStart: 0, TEnd: 1},
		lineSeg{Start: d, End: a, TStart: 0, TEnd: 1},
	}}}
}
