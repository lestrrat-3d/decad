package contactruling

import (
	vocab "github.com/lestrrat-3d/decad/report"

	"github.com/lestrrat-3d/decad/internal/clearance"
	"github.com/lestrrat-3d/decad/internal/contactround"
	"github.com/lestrrat-3d/decad/internal/measurement"
	pairbox "github.com/lestrrat-3d/decad/internal/pair/box"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// rulingSide is one body's share of a certified ruling contact: its original
// face, the exact ruling interval its own occupied set puts on the
// separating plane, and the cylinder radius that scales its normal tilt.
type rulingSide struct {
	face   *topo.Face
	ends   [2]proofarith.DyV3
	radius proofarith.Dyadic // zero for a plane side
	curved bool
}

// PublishRulingManifold publishes the manifold of
// agent-docs/contact-geometry-design.md §4.5 from the clearance kernel's ruling
// certificate. Each cylinder side must be a full source cylinder, so its
// occupied set meets the separating plane in exactly its tangent ruling; a
// plane side holds the whole certified ruling inside its trim. The complete
// contact set is then one segment, published as its two ends.
func PublishRulingManifold(report *contactReport, ruling *clearance.RulingContact) {
	outward := ruling.Normal
	sideA, reason := rulingSideOf(report.A, ruling.FaceA, outward, ruling.Offset)
	if reason != vocab.ContactNoReason {
		report.Reason = reason
		return
	}
	inward := proofarith.DvSub(proofarith.DyV3{}, outward)
	sideB, reason := rulingSideOf(report.B, ruling.FaceB, inward, proofarith.DyNeg(ruling.Offset))
	if reason != vocab.ContactNoReason {
		report.Reason = reason
		return
	}
	ends, ok := rulingContactSet(sideA, sideB)
	if !ok || !proofarith.DvEqual(ends[0], ruling.Ends[0]) ||
		!proofarith.DvEqual(ends[1], ruling.Ends[1]) {
		report.Reason = vocab.ContactAmbiguousFeature
		return
	}
	var witnesses [2]measurement.VecMeasurement
	for i := range ends {
		witness, ok := pairbox.DyadicPointMeasurement(&ends[i])
		if !ok || witness.Bound.Base() > report.Request.PointResolution.Base() {
			report.Reason = vocab.ContactPointTooCoarse
			return
		}
		witnesses[i] = witness
	}
	points := make([]contactPoint, 0, len(ends))
	for _, witness := range witnesses {
		normal, angle, ok := rulingNormal(sideA, sideB, witness, outward)
		if !ok || angle.Base() > report.Request.NormalResolution.Base() {
			report.Reason = vocab.ContactNoNormalProof
			return
		}
		points = append(points, contactPoint{
			OnA: witness, OnB: witness, Normal: normal, NormalAngle: angle,
			Separation: measurement.Measurement{Value: units.Millimeters(0), Bound: units.Millimeters(0), Exactness: measurement.Exact},
			FaceA:      sideA.face, FaceB: sideB.face,
			FeatureA: contactFeature{Face: sideA.face}, FeatureB: contactFeature{Face: sideB.face},
		})
	}
	report.Manifold = &contactManifold{Points: points}
	report.Reason = vocab.ContactNoReason
}

// rulingSideOf maps one certified carrier back to its body's original face.
// outward is the side's exact outward normal at the contact and offset the
// separating plane's offset along it.
func rulingSideOf(b *topo.Body, carrier *clearance.CFace, outward proofarith.DyV3,
	offset proofarith.Dyadic) (rulingSide, vocab.ContactReason) {
	if carrier.Kind == clearance.CkPlane {
		face, ok := uniquePlaneFace(b, clearance.DyAxisVec(outward), offset)
		if !ok {
			return rulingSide{}, vocab.ContactAmbiguousFeature
		}
		return rulingSide{face: face}, vocab.ContactNoReason
	}
	cylinder, ok := contactround.CylinderAtPose(b, r3.Identity())
	if !ok {
		return rulingSide{}, vocab.ContactPayloadUnsupported
	}
	normalAxis, side, ok := clearance.SignedAxis(clearance.DyAxisVec(outward))
	if !ok || normalAxis == cylinder.Axis {
		return rulingSide{}, vocab.ContactAmbiguousFeature
	}
	var wall *topo.Face
	for _, face := range topobuild.Faces(b) {
		if _, isCylinder := face.Surface.(surfacegeom.Cylinder); isCylinder {
			wall = face
		}
	}
	if wall == nil {
		return rulingSide{}, vocab.ContactAmbiguousFeature
	}
	// The source box is the exact disk-by-interval box: its transverse
	// extents are the axis coordinate plus or minus the radius.
	box := cylinder.Box
	half := proofarith.MustDyOf(.5)
	var point proofarith.DyV3
	var radius proofarith.Dyadic
	for i := range 3 {
		switch i {
		case cylinder.Axis:
			continue
		case normalAxis:
			radius = proofarith.DyMul(proofarith.DySubScalar(box.Hi[i], box.Lo[i]), half)
			point[i] = box.Lo[i]
			if side == 1 {
				point[i] = box.Hi[i]
			}
		default:
			point[i] = proofarith.DyMul(proofarith.DyAdd(box.Lo[i], box.Hi[i]), half)
		}
	}
	if proofarith.DyCmp(proofarith.DvDot(outward, point), offset) != 0 {
		return rulingSide{}, vocab.ContactAmbiguousFeature
	}
	ends := [2]proofarith.DyV3{point, point}
	ends[0][cylinder.Axis], ends[1][cylinder.Axis] = box.Lo[cylinder.Axis], box.Hi[cylinder.Axis]
	return rulingSide{face: wall, ends: ends, radius: radius, curved: true}, vocab.ContactNoReason
}

// uniquePlaneFace finds the one original planar face whose outward normal is
// exactly normal and whose plane lies exactly at offset along it.
func uniquePlaneFace(b *topo.Body, normal r3.Vec, offset proofarith.Dyadic) (*topo.Face, bool) {
	var found *topo.Face
	for _, face := range topobuild.Faces(b) {
		plane, ok := face.Surface.(surfacegeom.Plane)
		if !ok {
			continue
		}
		faceNormal := plane.Frame.N()
		if face.Reversed {
			faceNormal = faceNormal.Scale(-1)
		}
		origin, okOrigin := clearance.DyVecOf(plane.Frame.Origin())
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
			lo[i] = proofarith.DyMax(proofarith.DyMin(a.ends[0][i], a.ends[1][i]), proofarith.DyMin(b.ends[0][i], b.ends[1][i]))
			hi[i] = proofarith.DyMin(proofarith.DyMax(a.ends[0][i], a.ends[1][i]), proofarith.DyMax(b.ends[0][i], b.ends[1][i]))
			if proofarith.DyCmp(lo[i], hi[i]) > 0 {
				return [2]proofarith.DyV3{}, false
			}
		}
		if proofarith.DvEqual(lo, hi) {
			return [2]proofarith.DyV3{}, false
		}
		return clearance.OrderedRulingEnds([2]proofarith.DyV3{lo, hi}), true
	case a.curved:
		return clearance.OrderedRulingEnds(a.ends), true
	case b.curved:
		return clearance.OrderedRulingEnds(b.ends), true
	default:
		return [2]proofarith.DyV3{}, false
	}
}

// rulingNormal reads each side's Face.NormalAt at the witness and publishes
// the tighter ball as the A-to-B normal. A cylinder reading taken at a
// rounded witness also charges the radial tilt to the true contact point:
// two radial vectors whose difference is at most e, the longer of length r,
// have unit directions at most 2e/r apart. The exact separating normal must
// lie in both balls; a ball that misses it refuses the entry.
func rulingNormal(a, b rulingSide, witness measurement.VecMeasurement,
	outward proofarith.DyV3) (measurement.VecMeasurement, units.Value, bool) {
	normalA, okA := rulingSideNormal(a, witness, outward)
	inward := proofarith.DvSub(proofarith.DyV3{}, outward)
	normalB, okB := rulingSideNormal(b, witness, inward)
	if !okA || !okB {
		return measurement.VecMeasurement{}, units.Value{}, false
	}
	normal := normalA
	if normalB.Bound.Base() < normalA.Bound.Base() {
		normal = measurement.VecMeasurement{Value: normalB.Value.Scale(-1), Bound: normalB.Bound,
			Exactness: normalB.Exactness}
	}
	bound := normal.Bound.Base()
	if bound == 0 {
		return normal, units.Radians(0), true
	}
	// A unit vector within bound of the published one is at most 4·bound
	// radians away while bound stays below a half (orientedBoxNormal's rule).
	if bound >= .5 {
		return measurement.VecMeasurement{}, units.Value{}, false
	}
	return normal, units.Radians(proofbound.UpRound(4 * bound)), true
}

// rulingSideNormal is one side's outward normal ball at the witness, checked
// to contain the exact outward normal of the certificate.
func rulingSideNormal(side rulingSide, witness measurement.VecMeasurement,
	outward proofarith.DyV3) (measurement.VecMeasurement, bool) {
	reading, err := topobuild.NormalAt(side.face, witness.Value)
	if err != nil {
		return measurement.VecMeasurement{}, false
	}
	bound := reading.Bound.Base()
	if side.curved && witness.Bound.Base() > 0 {
		tilt := proofbound.DivUpper(2*witness.Bound.Base(), proofarith.DyFloatDown(side.radius))
		bound = proofbound.AbsSumUpper(bound, tilt)
	}
	if !measurement.FiniteValues(bound) {
		return measurement.VecMeasurement{}, false
	}
	value, ok := clearance.DyVecOf(reading.Value)
	if !ok {
		return measurement.VecMeasurement{}, false
	}
	residual := proofarith.DvSub(value, outward)
	ball := proofarith.MustDyOf(bound)
	if proofarith.DyCmp(proofarith.DvDot(residual, residual), proofarith.DyMul(ball, ball)) > 0 {
		return measurement.VecMeasurement{}, false
	}
	return measurement.VecMeasurement{Value: reading.Value, Bound: units.Scalar(bound), Exactness: measurement.ExactnessOf(bound)}, true
}
