package decad

import (
	"github.com/lestrrat-3d/decad/internal/pair/box"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// cardinalSphereSectionRecord cuts an authenticated whole-ball Revolve with
// a cardinal plane. An exact circle is returned only when its center and
// radius can be recorded without rounding in the requested Sketch frame.
func cardinalSphereSectionRecord(b *Body, plane r3.Frame) (profileRecord, bool) {
	sphere, ok := sourceSphereAtPose(b, r3.Identity())
	if !ok || !box.CardinalBasis(plane.U(), plane.V(), plane.N()) {
		return profileRecord{}, false
	}
	u := proofarith.DyVec(plane.U())
	v := proofarith.DyVec(plane.V())
	n := proofarith.DyVec(plane.N())
	if !proofarith.DvEqual(proofarith.DvCross(u, v), n) {
		return profileRecord{}, false
	}
	delta := proofarith.DvSub(sphere.center, proofarith.DyVec(plane.Origin()))
	offset := proofarith.DvDot(delta, n)
	q := proofarith.DySubScalar(proofarith.DyMul(sphere.radius, sphere.radius),
		proofarith.DyMul(offset, offset))
	if q.Sign() <= 0 {
		return profileRecord{}, false
	}
	radius := proofarith.DySqrtDown(q)
	if radius <= 0 || !proofarith.DySquareEquals(radius, q) {
		return profileRecord{}, false
	}
	centerU, exactU := proofarith.DvDot(delta, u).Float64()
	centerV, exactV := proofarith.DvDot(delta, v).Float64()
	if !exactU || !exactV {
		return profileRecord{}, false
	}
	return profileRecord{Outer: loopRecord{Segments: []curveSegment{circleSeg{
		Center: Point2{U: centerU, V: centerV}, Radius: units.Millimeters(radius),
		CCW: true, TStart: 0, TEnd: 1,
	}}}}, true
}
