package decad

import (
	"github.com/lestrrat-3d/decad/internal/clearance"
	"github.com/lestrrat-3d/decad/internal/pair"
	pairbox "github.com/lestrrat-3d/decad/internal/pair/box"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// orientedSourceBox pairs the internal exact parallelotope with its source
// face identities, which belong to the public body.
type orientedSourceBox struct {
	pairbox.OrientedBox
	faces [3][2]*Face
}

func sourceOrientedBoxAtPose(body *Body, pose r3.Transform) (orientedSourceBox, bool) {
	pp, ok := body.payload.(prismPayload)
	if !ok || !body.solid || body.kind != BodySolid || pp.surfaceResult ||
		pp.sectionDelta != 0 || pp.z0Delta != 0 || pp.z1Delta != 0 ||
		!signedAxisTransform(pp.xform) {
		return orientedSourceBox{}, false
	}
	geometry, placedEdge, ok := pairbox.SourceOrientedBox(pp.profile, pp.frame, pp.z0, pp.z1, pp.xform, pose)
	if !ok {
		return orientedSourceBox{}, false
	}
	box := orientedSourceBox{OrientedBox: geometry}
	// The original planar faces retain their source identities after the read
	// pose. Match them in the body's cardinal placed frame, before rotation.
	faces := body.Faces()
	if len(faces) != 6 {
		return orientedSourceBox{}, false
	}
	for _, face := range faces {
		plane, ok := face.surface.(Plane)
		if !ok || face.normalBound != 0 {
			return orientedSourceBox{}, false
		}
		normal := plane.Frame.N()
		if face.reversed {
			normal = normal.Scale(-1)
		}
		mapped := false
		for axis := range 3 {
			projection := proofarith.DvDot(proofarith.DyVec(normal), placedEdge[axis]).Sign()
			if projection == 0 {
				continue
			}
			side := 0
			if projection > 0 {
				side = 1
			}
			if mapped || box.faces[axis][side] != nil {
				return orientedSourceBox{}, false
			}
			box.faces[axis][side], mapped = face, true
		}
		if !mapped {
			return orientedSourceBox{}, false
		}
	}
	for axis := range 3 {
		if box.faces[axis][0] == nil || box.faces[axis][1] == nil {
			return orientedSourceBox{}, false
		}
	}
	return box, true
}

// orientedBoxRelation keeps the contact report's relation type at the root.
func orientedBoxRelation(a, b orientedSourceBox) (ContactRelation, proofarith.Dyadic, proofarith.Dyadic) {
	relation, gap, normSquared := pairbox.OrientedBoxRelation(a.OrientedBox, b.OrientedBox)
	switch relation {
	case pair.Separated:
		return ContactSeparated, gap, normSquared
	case pair.Touching:
		return ContactTouching, gap, normSquared
	case pair.Overlapping:
		return ContactOverlapping, gap, normSquared
	default:
		return ContactUndecided, gap, normSquared
	}
}

func pairRelationOf(relation ContactRelation) pair.Relation {
	switch relation {
	case ContactSeparated:
		return pair.Separated
	case ContactTouching:
		return pair.Touching
	case ContactOverlapping:
		return pair.Overlapping
	default:
		return pair.Undecided
	}
}

func classifyOrientedSourceBoxes(report *ContactReport, a, b orientedSourceBox) {
	relation, gap, normSquared := orientedBoxRelation(a, b)
	switch relation {
	case ContactTouching:
		report.Relation = relation
		report.Gap = &Measurement{Value: units.Millimeters(0), Bound: units.Millimeters(0), Exactness: Exact}
		report.Reason = ContactNoNormalProof
		publishContainedHorizontalPatch(report, a, b)
		if report.Manifold == nil {
			publishClippedHorizontalPatch(report, a, b)
		}
		if report.Manifold == nil && report.Reason != ContactPointTooCoarse {
			publishOrientedBoxPatch(report, a, b)
		}
		if report.Manifold == nil && report.Reason != ContactPointTooCoarse {
			publishOrientedAxisPatch(report, &a, &b)
		}
	case ContactOverlapping:
		report.Relation, report.Reason = relation, ContactNoNormalProof
		publishContainedHorizontalPatch(report, a, b)
		if report.Manifold == nil {
			publishOrientedAxisPatch(report, &a, &b)
		}
	case ContactSeparated:
		reading, ok := orientedBoxGap(a, b, gap, normSquared)
		if !ok {
			report.Reason = ContactNoGapProof
			return
		}
		report.Relation, report.Gap = relation, &reading
	}
}

// publishContainedHorizontalPatch handles a complete rotating box face
// inside an axis-aligned box face. Exact corner and side-plane comparisons
// certify the four-point patch, including a uniquely shallow penetration.
func publishContainedHorizontalPatch(report *ContactReport, a, b orientedSourceBox) {
	if base, ok := sourceBoxAtPose(report.A, report.PoseA); ok &&
		publishHorizontalPatchOrder(report, base, b, report.B, report.PoseB, true) {
		return
	}
	if base, ok := sourceBoxAtPose(report.B, report.PoseB); ok {
		publishHorizontalPatchOrder(report, base, a, report.A, report.PoseA, false)
	}
}

func publishHorizontalPatchOrder(report *ContactReport, base sourceBoxContactProof,
	rotated orientedSourceBox, body *Body, pose r3.Transform, baseIsA bool) bool {
	basis := pose.Basis()
	if basis.EX.Z != 0 || basis.EY.Z != 0 || basis.EZ != (r3.Vec{Z: 1}) {
		return false
	}
	source, ok := sourceBoxAtPose(body, r3.Identity())
	if !ok {
		return false
	}
	patch, ok := pairbox.ContainedHorizontalPatch(base.axisBox(), rotated.OrientedBox, pairRelationOf(report.Relation))
	if !ok {
		return false
	}
	reading, ok := sourceBoxSignedReading(patch.Separation)
	if !ok {
		return false
	}
	normalZ := 1.0
	if !patch.BaseBelow {
		normalZ = -1
	}
	if !baseIsA {
		normalZ = -normalZ
	}
	faceBase, faceRotated := base.faces[2][patch.BaseSide], source.faces[2][patch.RotatedSide]
	points := make([]ContactPoint, 0, len(patch.Corners))
	for _, corner := range patch.Corners {
		basePoint := corner
		basePoint[2] = patch.SupportZ
		baseReading, okBase := sourceBoxPoint(basePoint)
		rotatedReading, okRotated := sourceBoxPoint(corner)
		if !okBase || !okRotated ||
			baseReading.Bound.Base() > report.Request.PointResolution.Base() ||
			rotatedReading.Bound.Base() > report.Request.PointResolution.Base() {
			return false
		}
		point := ContactPoint{Normal: VecMeasurement{Value: r3.Vec{Z: normalZ},
			Exactness: Exact, Bound: units.Scalar(0)}, NormalAngle: units.Radians(0),
			Separation: reading}
		if baseIsA {
			point.OnA, point.OnB = baseReading, rotatedReading
			point.FaceA, point.FaceB = faceBase, faceRotated
		} else {
			point.OnA, point.OnB = rotatedReading, baseReading
			point.FaceA, point.FaceB = faceRotated, faceBase
		}
		point.FeatureA, point.FeatureB = ContactFeature{Face: point.FaceA}, ContactFeature{Face: point.FaceB}
		points = append(points, point)
	}
	report.Manifold = &ContactManifold{Points: points}
	report.Reason = ContactNoReason
	return true
}

// publishOrientedAxisPatch reduces two opposed axis-normal faces to one
// certified interior witness. It leaves other oriented contacts without a
// manifold, including edge and corner contacts.
func publishOrientedAxisPatch(report *ContactReport, a, b *orientedSourceBox) {
	var chosen *ContactPoint
	best := proofarith.DyZero()
	_, alignedA := sourceBoxAtPose(report.A, report.PoseA)
	_, alignedB := sourceBoxAtPose(report.B, report.PoseB)
	for _, candidate := range pairbox.AxisFaceWitnesses(a.OrientedBox, b.OrientedBox, pairRelationOf(report.Relation)) {
		// A horizontal patch against a signed-axis box needs the complete
		// contained four-point proof above; one interior point cannot replace it.
		if candidate.Axis == 2 && (alignedA || alignedB) {
			continue
		}
		if chosen != nil && proofarith.DyCmp(proofarith.DyAbs(candidate.Separation), proofarith.DyAbs(best)) >= 0 {
			continue
		}
		reading, ok := sourceBoxSignedReading(candidate.Separation)
		if !ok || reading.Bound.Base() > report.Request.PointResolution.Base() {
			continue
		}
		onA, readA := sourceBoxPointAt(&candidate.PointA)
		onB, readB := sourceBoxPointAt(&candidate.PointB)
		if !readA || !readB || onA.Bound.Base() > report.Request.PointResolution.Base() ||
			onB.Bound.Base() > report.Request.PointResolution.Base() {
			continue
		}
		normal := r3.Vec{}
		switch candidate.Axis {
		case 0:
			normal.X = float64(candidate.Sign)
		case 1:
			normal.Y = float64(candidate.Sign)
		case 2:
			normal.Z = float64(candidate.Sign)
		}
		fa, fb := orientedSourceFace(report.A, report.PoseA, candidate.Axis, candidate.SideA),
			orientedSourceFace(report.B, report.PoseB, candidate.Axis, candidate.SideB)
		if fa == nil || fb == nil {
			continue
		}
		point := ContactPoint{
			OnA: onA, OnB: onB,
			Normal:      VecMeasurement{Value: normal, Exactness: Exact, Bound: units.Scalar(0)},
			NormalAngle: units.Radians(0), Separation: reading,
			FaceA: fa, FaceB: fb,
			FeatureA: ContactFeature{Face: fa}, FeatureB: ContactFeature{Face: fb},
		}
		chosen, best = &point, candidate.Separation
	}
	if chosen != nil {
		report.Manifold = &ContactManifold{Points: []ContactPoint{*chosen}}
		report.Reason = ContactNoReason
	}
}

func orientedSourceFace(body *Body, pose r3.Transform, axis, side int) *Face {
	for _, face := range body.Faces() {
		plane, ok := face.surface.(Plane)
		if !ok || face.normalBound != 0 {
			continue
		}
		normal := plane.Frame.N()
		if face.reversed {
			normal = normal.Scale(-1)
		}
		normal = pose.ApplyDir(normal)
		foundAxis, foundSide, ok := clearance.SignedAxis(normal)
		if ok && foundAxis == axis && foundSide == side {
			return face
		}
	}
	return nil
}

// orientedBoxGap publishes the neutral gap enclosure as a public reading.
func orientedBoxGap(a, b orientedSourceBox, gap, normSquared proofarith.Dyadic) (Measurement, bool) {
	reading, ok := pairbox.OrientedBoxGap(a.OrientedBox, b.OrientedBox, gap, normSquared)
	if !ok {
		return Measurement{}, false
	}
	return Measurement{Value: units.Millimeters(reading.ValueMM), Bound: units.Millimeters(reading.BoundMM),
		Exactness: exactnessOf(reading.BoundMM)}, true
}
