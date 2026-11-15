// Package ruling certifies exact cylindrical line contacts in clearance pairs.
package ruling

import (
	"math"

	"github.com/lestrrat-3d/decad/internal/clearance"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// Extent reads a body's directional extent after the carrier gates pass.
type Extent func(r3.Vec) (float64, float64, bool)

// PlaneCylinderRuling certifies the cylinder's complete tangent ruling on the
// plane face. The cylinder axis must parallel the plane at exactly its radius
// on the outward side, the tangent azimuth and the whole axial window must be
// on the cylinder face, and the whole ruling must lie inside the plane trim.
// The plane must separate the bodies' complete extents. cylinderFirst names
// the cylinder's body as A.
func PlaneCylinderRuling(tol float64, plane, cyl *clearance.CFace,
	planeExtent, cylExtent Extent, cylinderFirst bool) *clearance.RulingContact {
	n, okN := clearance.ExactSignedAxis(plane.N)
	axis, okAxis := clearance.ExactSignedAxis(cyl.Axis)
	o, okO := clearance.DyVecOf(plane.O)
	anchor, okAnchor := clearance.DyVecOf(cyl.Anchor)
	radius, okR := proofarith.DyOf(cyl.Radius)
	lo, okLo := proofarith.DyOf(cyl.ZWin.Lo)
	hi, okHi := proofarith.DyOf(cyl.ZWin.Hi)
	if !okN || !okAxis || !okO || !okAnchor || !okR || !okLo || !okHi ||
		radius.Sign() <= 0 || proofarith.DyCmp(lo, hi) >= 0 ||
		!proofarith.DvDot(n, axis).IsZero() {
		return nil
	}
	offset := proofarith.DvDot(n, o)
	if proofarith.DyCmp(proofarith.DySubScalar(proofarith.DvDot(n, anchor), offset), radius) != 0 {
		return nil
	}
	if !tangentAzimuthAdmitted(tol, cyl, plane.N.Scale(-1)) {
		return nil
	}
	base := proofarith.DvSub(anchor, proofarith.DvScale(n, radius))
	ends := [2]proofarith.DyV3{
		proofarith.DvAdd(base, proofarith.DvScale(axis, lo)),
		proofarith.DvAdd(base, proofarith.DvScale(axis, hi)),
	}
	if !rulingInsidePlaneTrim(tol, plane, ends) {
		return nil
	}
	_, planeHi, okPlane := planeExtent(plane.N)
	cylLo, _, okCyl := cylExtent(plane.N)
	if !okPlane || !okCyl {
		return nil
	}
	planeHiDy, okPlaneHi := proofarith.DyOf(planeHi)
	cylLoDy, okCylLo := proofarith.DyOf(cylLo)
	if !okPlaneHi || !okCylLo || proofarith.DyCmp(planeHiDy, offset) > 0 ||
		proofarith.DyCmp(cylLoDy, offset) < 0 {
		return nil
	}
	ruling := &clearance.RulingContact{FaceA: plane, FaceB: cyl, Normal: n, Offset: offset, Ends: clearance.OrderedRulingEnds(ends)}
	if cylinderFirst {
		ruling.FaceA, ruling.FaceB = cyl, plane
		ruling.Normal = proofarith.DvSub(proofarith.DyV3{}, n)
		ruling.Offset = proofarith.DyNeg(offset)
	}
	return ruling
}

// CylinderPairRuling certifies the common ruling of two parallel external
// cylinders. Their axes must be exactly parallel, their center offset must be
// the radius sum along one signed coordinate axis, both tangent azimuths must
// be on their faces, and the axial windows must overlap with positive
// length. The tangent plane between them must separate the complete extents.
func CylinderPairRuling(tol float64, ca, cb *clearance.CFace, aExtent, bExtent Extent) *clearance.RulingContact {
	axisA, okAxisA := clearance.ExactSignedAxis(ca.Axis)
	axisB, okAxisB := clearance.ExactSignedAxis(cb.Axis)
	anchorA, okAnchorA := clearance.DyVecOf(ca.Anchor)
	anchorB, okAnchorB := clearance.DyVecOf(cb.Anchor)
	rA, okRA := proofarith.DyOf(ca.Radius)
	rB, okRB := proofarith.DyOf(cb.Radius)
	loA, okLoA := proofarith.DyOf(ca.ZWin.Lo)
	hiA, okHiA := proofarith.DyOf(ca.ZWin.Hi)
	loB, okLoB := proofarith.DyOf(cb.ZWin.Lo)
	hiB, okHiB := proofarith.DyOf(cb.ZWin.Hi)
	if !okAxisA || !okAxisB || !okAnchorA || !okAnchorB || !okRA || !okRB ||
		!okLoA || !okHiA || !okLoB || !okHiB || rA.Sign() <= 0 || rB.Sign() <= 0 ||
		!proofarith.DvIsZero(proofarith.DvCross(axisA, axisB)) {
		return nil
	}
	delta := proofarith.DvSub(anchorB, anchorA)
	along := proofarith.DvDot(delta, axisA)
	perp := proofarith.DvSub(delta, proofarith.DvScale(axisA, along))
	normal, ok := clearance.AxisOfLength(perp, proofarith.DyAdd(rA, rB))
	if !ok {
		return nil
	}
	normalVec := clearance.DyAxisVec(normal)
	if !tangentAzimuthAdmitted(tol, ca, normalVec) || !tangentAzimuthAdmitted(tol, cb, normalVec.Scale(-1)) {
		return nil
	}
	// Map B's axial window into A's axis parameter.
	bLo, bHi := proofarith.DyAdd(along, loB), proofarith.DyAdd(along, hiB)
	if proofarith.DvDot(axisA, axisB).Sign() < 0 {
		bLo, bHi = proofarith.DySubScalar(along, hiB), proofarith.DySubScalar(along, loB)
	}
	lo, hi := proofarith.DyMax(loA, bLo), proofarith.DyMin(hiA, bHi)
	if proofarith.DyCmp(lo, hi) >= 0 {
		return nil
	}
	offset := proofarith.DyAdd(proofarith.DvDot(normal, anchorA), rA)
	_, aHi, okA := aExtent(normalVec)
	bLoExtent, _, okB := bExtent(normalVec)
	if !okA || !okB {
		return nil
	}
	aHiDy, okAHi := proofarith.DyOf(aHi)
	bLoDy, okBLo := proofarith.DyOf(bLoExtent)
	if !okAHi || !okBLo || proofarith.DyCmp(aHiDy, offset) > 0 || proofarith.DyCmp(bLoDy, offset) < 0 {
		return nil
	}
	base := proofarith.DvAdd(anchorA, proofarith.DvScale(normal, rA))
	ends := [2]proofarith.DyV3{
		proofarith.DvAdd(base, proofarith.DvScale(axisA, lo)),
		proofarith.DvAdd(base, proofarith.DvScale(axisA, hi)),
	}
	return &clearance.RulingContact{FaceA: ca, FaceB: cb, Normal: normal, Offset: offset, Ends: clearance.OrderedRulingEnds(ends)}
}

// tangentAzimuthAdmitted reports whether the cylinder face's angular trim
// holds the ruling whose outward radial direction is dir, with margin.
func tangentAzimuthAdmitted(tol float64, f *clearance.CFace, dir r3.Vec) bool {
	phi := math.Atan2(dir.Dot(f.RefV), dir.Dot(f.RefU))
	return f.Sweep.Classify(phi, tol/math.Max(f.Radius, 1e-30)) == 1
}

// rulingInsidePlaneTrim admits the whole segment into the plane face's trim:
// one end strictly inside with margin and the segment clear of every trim
// boundary element by more than the margin, so it never leaves the region.
func rulingInsidePlaneTrim(tol float64, plane *clearance.CFace, ends [2]proofarith.DyV3) bool {
	var coords [2][2]float64
	for i, end := range ends {
		// The nearest float of each end is within an ulp; the trim margin
		// below is many orders wider.
		p := clearance.DyAxisVec(end)
		if !proofbound.FiniteVec(p) {
			return false
		}
		coords[i][0], coords[i][1] = plane.PlaneCoords(p)
	}
	if plane.Region.Classify(coords[0][0], coords[0][1], tol) != 1 {
		return false
	}
	for _, e := range plane.Region.Elems {
		if clearance.SegElemDistLB(e, coords[0][0], coords[0][1], coords[1][0], coords[1][1]) <= tol {
			return false
		}
	}
	return true
}
