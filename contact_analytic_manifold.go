package decad

import (
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// rulingSide is one body's share of a certified ruling contact: its original
// face, the exact ruling interval its own occupied set puts on the
// separating plane, and the cylinder radius that scales its normal tilt.
type rulingSide struct {
	face   *Face
	ends   [2]proofarith.DyV3
	radius proofarith.Dyadic // zero for a plane side
	curved bool
}

// publishRulingManifold publishes the manifold of
// docs/contact-geometry-design.md §4.5 from the clearance kernel's ruling
// certificate. Each cylinder side must be a full source cylinder, so its
// occupied set meets the separating plane in exactly its tangent ruling; a
// plane side holds the whole certified ruling inside its trim. The complete
// contact set is then one segment, published as its two ends.
func publishRulingManifold(report *ContactReport, ruling *rulingContact) {
	outward := ruling.normal
	sideA, reason := rulingSideOf(report.A, ruling.faceA, outward, ruling.offset)
	if reason != ContactNoReason {
		report.Reason = reason
		return
	}
	inward := proofarith.DvSub(proofarith.DyV3{}, outward)
	sideB, reason := rulingSideOf(report.B, ruling.faceB, inward, proofarith.DyNeg(ruling.offset))
	if reason != ContactNoReason {
		report.Reason = reason
		return
	}
	ends, ok := rulingContactSet(sideA, sideB)
	if !ok || !rulingEndsEqual(ends, ruling.ends) {
		report.Reason = ContactAmbiguousFeature
		return
	}
	var witnesses [2]VecMeasurement
	for i := range ends {
		witness, ok := sourceBoxPointAt(&ends[i])
		if !ok || witness.Bound.Base() > report.Request.PointResolution.Base() {
			report.Reason = ContactPointTooCoarse
			return
		}
		witnesses[i] = witness
	}
	points := make([]ContactPoint, 0, len(ends))
	for _, witness := range witnesses {
		normal, angle, ok := rulingNormal(sideA, sideB, witness, outward)
		if !ok || angle.Base() > report.Request.NormalResolution.Base() {
			report.Reason = ContactNoNormalProof
			return
		}
		points = append(points, ContactPoint{
			OnA: witness, OnB: witness, Normal: normal, NormalAngle: angle,
			Separation: Measurement{Value: units.Millimeters(0), Bound: units.Millimeters(0), Exactness: Exact},
			FaceA:      sideA.face, FaceB: sideB.face,
			FeatureA: ContactFeature{Face: sideA.face}, FeatureB: ContactFeature{Face: sideB.face},
		})
	}
	report.Manifold = &ContactManifold{Points: points}
	report.Reason = ContactNoReason
}

// rulingSideOf maps one certified carrier back to its body's original face.
// outward is the side's exact outward normal at the contact and offset the
// separating plane's offset along it.
func rulingSideOf(b *Body, carrier *cFace, outward proofarith.DyV3,
	offset proofarith.Dyadic) (rulingSide, ContactReason) {
	if carrier.kind == ckPlane {
		face, ok := uniquePlaneFace(b, dyAxisVec(outward), offset)
		if !ok {
			return rulingSide{}, ContactAmbiguousFeature
		}
		return rulingSide{face: face}, ContactNoReason
	}
	cylinder, ok := sourceCylinderAtPose(b, r3.Identity())
	if !ok {
		return rulingSide{}, ContactPayloadUnsupported
	}
	normalAxis, side, ok := signedAxis(dyAxisVec(outward))
	if !ok || normalAxis == cylinder.axis {
		return rulingSide{}, ContactAmbiguousFeature
	}
	var wall *Face
	for _, face := range b.Faces() {
		if _, isCylinder := face.surface.(Cylinder); isCylinder {
			wall = face
		}
	}
	if wall == nil {
		return rulingSide{}, ContactAmbiguousFeature
	}
	// The source box is the exact disk-by-interval box: its transverse
	// extents are the axis coordinate plus or minus the radius.
	box := cylinder.box
	half := proofarith.MustDyOf(.5)
	var point proofarith.DyV3
	var radius proofarith.Dyadic
	for i := range 3 {
		switch i {
		case cylinder.axis:
			continue
		case normalAxis:
			radius = proofarith.DyMul(proofarith.DySubScalar(box.hi[i], box.lo[i]), half)
			point[i] = box.lo[i]
			if side == 1 {
				point[i] = box.hi[i]
			}
		default:
			point[i] = proofarith.DyMul(proofarith.DyAdd(box.lo[i], box.hi[i]), half)
		}
	}
	if proofarith.DyCmp(proofarith.DvDot(outward, point), offset) != 0 {
		return rulingSide{}, ContactAmbiguousFeature
	}
	ends := [2]proofarith.DyV3{point, point}
	ends[0][cylinder.axis], ends[1][cylinder.axis] = box.lo[cylinder.axis], box.hi[cylinder.axis]
	return rulingSide{face: wall, ends: ends, radius: radius, curved: true}, ContactNoReason
}

// uniquePlaneFace finds the one original planar face whose outward normal is
// exactly normal and whose plane lies exactly at offset along it.
func uniquePlaneFace(b *Body, normal r3.Vec, offset proofarith.Dyadic) (*Face, bool) {
	var found *Face
	for _, face := range b.Faces() {
		plane, ok := face.surface.(Plane)
		if !ok {
			continue
		}
		faceNormal := plane.Frame.N()
		if face.reversed {
			faceNormal = faceNormal.Scale(-1)
		}
		origin, okOrigin := dyVecOf(plane.Frame.Origin())
		if faceNormal != normal || !okOrigin ||
			proofarith.DyCmp(proofarith.DvDot(proofarith.DyVec(normal), origin), offset) != 0 {
			continue
		}
		if found != nil {
			return nil, false
		}
		found = face
	}
	return found, found != nil
}

// rulingContactSet intersects the two sides' rulings. A plane side states no
// ruling of its own: the certificate already put the whole cylinder ruling
// inside its trim.
func rulingContactSet(a, b rulingSide) ([2]proofarith.DyV3, bool) {
	switch {
	case a.curved && b.curved:
		var lo, hi proofarith.DyV3
		for i := range 3 {
			lo[i] = dyMax(dyMin(a.ends[0][i], a.ends[1][i]), dyMin(b.ends[0][i], b.ends[1][i]))
			hi[i] = dyMin(dyMax(a.ends[0][i], a.ends[1][i]), dyMax(b.ends[0][i], b.ends[1][i]))
			if proofarith.DyCmp(lo[i], hi[i]) > 0 {
				return [2]proofarith.DyV3{}, false
			}
		}
		if rulingEndsEqual([2]proofarith.DyV3{lo, lo}, [2]proofarith.DyV3{hi, hi}) {
			return [2]proofarith.DyV3{}, false
		}
		return orderedRulingEnds([2]proofarith.DyV3{lo, hi}), true
	case a.curved:
		return orderedRulingEnds(a.ends), true
	case b.curved:
		return orderedRulingEnds(b.ends), true
	default:
		return [2]proofarith.DyV3{}, false
	}
}

func rulingEndsEqual(a, b [2]proofarith.DyV3) bool {
	for i := range a {
		for k := range 3 {
			if proofarith.DyCmp(a[i][k], b[i][k]) != 0 {
				return false
			}
		}
	}
	return true
}

// rulingNormal reads each side's Face.NormalAt at the witness and publishes
// the tighter ball as the A-to-B normal. A cylinder reading taken at a
// rounded witness also charges the radial tilt to the true contact point:
// two radial vectors whose difference is at most e, the longer of length r,
// have unit directions at most 2e/r apart. The exact separating normal must
// lie in both balls; a ball that misses it refuses the entry.
func rulingNormal(a, b rulingSide, witness VecMeasurement,
	outward proofarith.DyV3) (VecMeasurement, units.Value, bool) {
	normalA, okA := rulingSideNormal(a, witness, outward)
	inward := proofarith.DvSub(proofarith.DyV3{}, outward)
	normalB, okB := rulingSideNormal(b, witness, inward)
	if !okA || !okB {
		return VecMeasurement{}, units.Value{}, false
	}
	normal := normalA
	if normalB.Bound.Base() < normalA.Bound.Base() {
		normal = VecMeasurement{Value: normalB.Value.Scale(-1), Bound: normalB.Bound,
			Exactness: normalB.Exactness}
	}
	bound := normal.Bound.Base()
	if bound == 0 {
		return normal, units.Radians(0), true
	}
	// A unit vector within bound of the published one is at most 4·bound
	// radians away while bound stays below a half (orientedBoxNormal's rule).
	if bound >= .5 {
		return VecMeasurement{}, units.Value{}, false
	}
	return normal, units.Radians(upRound(4 * bound)), true
}

// rulingSideNormal is one side's outward normal ball at the witness, checked
// to contain the exact outward normal of the certificate.
func rulingSideNormal(side rulingSide, witness VecMeasurement,
	outward proofarith.DyV3) (VecMeasurement, bool) {
	reading, err := side.face.NormalAt(witness.Value)
	if err != nil {
		return VecMeasurement{}, false
	}
	bound := reading.Bound.Base()
	if side.curved && witness.Bound.Base() > 0 {
		tilt := divUpper(2*witness.Bound.Base(), proofarith.DyFloatDown(side.radius))
		bound = absSumUpper(bound, tilt)
	}
	if !finiteMeasurementValues(bound) {
		return VecMeasurement{}, false
	}
	value, ok := dyVecOf(reading.Value)
	if !ok {
		return VecMeasurement{}, false
	}
	residual := proofarith.DvSub(value, outward)
	ball := proofarith.MustDyOf(bound)
	if proofarith.DyCmp(proofarith.DvDot(residual, residual), proofarith.DyMul(ball, ball)) > 0 {
		return VecMeasurement{}, false
	}
	return VecMeasurement{Value: reading.Value, Bound: units.Scalar(bound), Exactness: exactnessOf(bound)}, true
}
