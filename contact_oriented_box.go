package decad

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/clearance"
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/pair"
	pairbox "github.com/lestrrat-3d/decad/internal/pair/box"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/reportvocab"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
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
	pp, ok := body.n.Payload.(prismPayload)
	if !ok || !body.n.Solid || body.n.Kind != topo.BodySolid || pp.SurfaceResult ||
		pp.SectionDelta != 0 || pp.Z0Delta != 0 || pp.Z1Delta != 0 ||
		!pairbox.SignedAxisTransform(pp.Xform) {
		return orientedSourceBox{}, false
	}
	geometry, placedEdge, ok := pairbox.SourceOrientedBox(pp.Profile, pp.Frame, pp.Z0, pp.Z1, pp.Xform, pose)
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
		plane, ok := face.n.Surface.(surfacegeom.Plane)
		if !ok || face.n.NormalBound != 0 {
			return orientedSourceBox{}, false
		}
		normal := plane.Frame.N()
		if face.n.Reversed {
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

// orientedBoxRelation maps the box kernel's relation to the contact report's.
func orientedBoxRelation(a, b orientedSourceBox) (reportvocab.ContactRelation, proofarith.Dyadic, proofarith.Dyadic) {
	relation, gap, normSquared := pairbox.OrientedBoxRelation(a.OrientedBox, b.OrientedBox)
	switch relation {
	case pair.Separated:
		return reportvocab.ContactSeparated, gap, normSquared
	case pair.Touching:
		return reportvocab.ContactTouching, gap, normSquared
	case pair.Overlapping:
		return reportvocab.ContactOverlapping, gap, normSquared
	default:
		return reportvocab.ContactUndecided, gap, normSquared
	}
}

func pairRelationOf(relation reportvocab.ContactRelation) pair.Relation {
	switch relation {
	case reportvocab.ContactSeparated:
		return pair.Separated
	case reportvocab.ContactTouching:
		return pair.Touching
	case reportvocab.ContactOverlapping:
		return pair.Overlapping
	default:
		return pair.Undecided
	}
}

func classifyOrientedSourceBoxes(report *reportvocab.ContactReport, a, b orientedSourceBox) {
	relation, gap, normSquared := orientedBoxRelation(a, b)
	switch relation {
	case reportvocab.ContactTouching:
		report.Relation = relation
		report.Gap = &measurement.Measurement{Value: units.Millimeters(0), Bound: units.Millimeters(0), Exactness: measurement.Exact}
		report.Reason = reportvocab.ContactNoNormalProof
		publishContainedHorizontalPatch(report, a, b)
		if report.Manifold == nil {
			publishClippedHorizontalPatch(report, a, b)
		}
		if report.Manifold == nil && report.Reason != reportvocab.ContactPointTooCoarse {
			publishOrientedBoxPatch(report, a, b)
		}
		if report.Manifold == nil && report.Reason != reportvocab.ContactPointTooCoarse {
			publishOrientedAxisPatch(report, &a, &b)
		}
	case reportvocab.ContactOverlapping:
		report.Relation, report.Reason = relation, reportvocab.ContactNoNormalProof
		publishContainedHorizontalPatch(report, a, b)
		if report.Manifold == nil {
			publishOrientedAxisPatch(report, &a, &b)
		}
	case reportvocab.ContactSeparated:
		reading, ok := orientedBoxGap(a, b, gap, normSquared)
		if !ok {
			report.Reason = reportvocab.ContactNoGapProof
			return
		}
		report.Relation, report.Gap = relation, &reading
	}
}

// publishContainedHorizontalPatch handles a complete rotating box face
// inside an axis-aligned box face. Exact corner and side-plane comparisons
// certify the four-point patch, including a uniquely shallow penetration.
func publishContainedHorizontalPatch(report *reportvocab.ContactReport, a, b orientedSourceBox) {
	if base, ok := sourceBoxAtPose(bodyHandle(report.A), report.PoseA); ok &&
		publishHorizontalPatchOrder(report, base, b, bodyHandle(report.B), report.PoseB, true) {
		return
	}
	if base, ok := sourceBoxAtPose(bodyHandle(report.B), report.PoseB); ok {
		publishHorizontalPatchOrder(report, base, a, bodyHandle(report.A), report.PoseA, false)
	}
}

func publishHorizontalPatchOrder(report *reportvocab.ContactReport, base sourceBoxContactProof,
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
	points := make([]reportvocab.ContactPoint, 0, len(patch.Corners))
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
		point := reportvocab.ContactPoint{Normal: measurement.VecMeasurement{Value: r3.Vec{Z: normalZ},
			Exactness: measurement.Exact, Bound: units.Scalar(0)}, NormalAngle: units.Radians(0),
			Separation: reading}
		if baseIsA {
			point.OnA, point.OnB = baseReading, rotatedReading
			point.FaceA, point.FaceB = faceNode(faceBase), faceNode(faceRotated)
		} else {
			point.OnA, point.OnB = rotatedReading, baseReading
			point.FaceA, point.FaceB = faceNode(faceRotated), faceNode(faceBase)
		}
		point.FeatureA, point.FeatureB = reportvocab.ContactFeature{Face: point.FaceA}, reportvocab.ContactFeature{Face: point.FaceB}
		points = append(points, point)
	}
	report.Manifold = &reportvocab.ContactManifold{Points: points}
	report.Reason = reportvocab.ContactNoReason
	return true
}

// publishOrientedAxisPatch reduces two opposed axis-normal faces to one
// certified interior witness. It leaves other oriented contacts without a
// manifold, including edge and corner contacts.
func publishOrientedAxisPatch(report *reportvocab.ContactReport, a, b *orientedSourceBox) {
	var chosen *reportvocab.ContactPoint
	best := proofarith.DyZero()
	_, alignedA := sourceBoxAtPose(bodyHandle(report.A), report.PoseA)
	_, alignedB := sourceBoxAtPose(bodyHandle(report.B), report.PoseB)
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
		fa, fb := orientedSourceFace(bodyHandle(report.A), report.PoseA, candidate.Axis, candidate.SideA),
			orientedSourceFace(bodyHandle(report.B), report.PoseB, candidate.Axis, candidate.SideB)
		if fa == nil || fb == nil {
			continue
		}
		point := reportvocab.ContactPoint{
			OnA: onA, OnB: onB,
			Normal:      measurement.VecMeasurement{Value: normal, Exactness: measurement.Exact, Bound: units.Scalar(0)},
			NormalAngle: units.Radians(0), Separation: reading,
			FaceA: faceNode(fa), FaceB: faceNode(fb),
			FeatureA: reportvocab.ContactFeature{Face: faceNode(fa)}, FeatureB: reportvocab.ContactFeature{Face: faceNode(fb)},
		}
		chosen, best = &point, candidate.Separation
	}
	if chosen != nil {
		report.Manifold = &reportvocab.ContactManifold{Points: []reportvocab.ContactPoint{*chosen}}
		report.Reason = reportvocab.ContactNoReason
	}
}

func orientedSourceFace(body *Body, pose r3.Transform, axis, side int) *Face {
	for _, face := range body.Faces() {
		plane, ok := face.n.Surface.(surfacegeom.Plane)
		if !ok || face.n.NormalBound != 0 {
			continue
		}
		normal := plane.Frame.N()
		if face.n.Reversed {
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

// orientedBoxGap publishes the neutral gap enclosure as a report reading.
func orientedBoxGap(a, b orientedSourceBox, gap, normSquared proofarith.Dyadic) (measurement.Measurement, bool) {
	reading, ok := pairbox.OrientedBoxGap(a.OrientedBox, b.OrientedBox, gap, normSquared)
	if !ok {
		return measurement.Measurement{}, false
	}
	return measurement.Measurement{Value: units.Millimeters(reading.ValueMM), Bound: units.Millimeters(reading.BoundMM),
		Exactness: measurement.ExactnessOf(reading.BoundMM)}, true
}

// publishOrientedBoxPatch handles opposed faces of boxes with parallel source
// edge directions. Their complete contact set is a rectangle in A's exact
// dual basis, even when the read rotation's dyadic entries are slightly skew.
func publishOrientedBoxPatch(report *reportvocab.ContactReport, a, b orientedSourceBox) {
	geometry, failure := pairbox.OrientedFacePatch(a.OrientedBox, b.OrientedBox)
	switch failure {
	case pairbox.OrientedPatchNoNormal:
		report.Reason = reportvocab.ContactNoNormalProof
		return
	case pairbox.OrientedPatchAmbiguous:
		report.Reason = reportvocab.ContactAmbiguousFeature
		return
	}
	faceA := a.faces[geometry.FaceA.Axis][geometry.FaceA.Side]
	faceB := b.faces[geometry.FaceB.Axis][geometry.FaceB.Side]
	if faceA == nil || faceB == nil {
		report.Reason = reportvocab.ContactNoNormalProof
		return
	}
	normal, angle, ok := orientedBoxNormal(geometry.Normal)
	if !ok || angle.Base() > report.Request.NormalResolution.Base() {
		report.Reason = reportvocab.ContactNoNormalProof
		return
	}
	points := make([]reportvocab.ContactPoint, 0, 4)
	for _, corner := range [][2]int{{0, 0}, {1, 0}, {1, 1}, {0, 1}} {
		point := geometry.Point(a.OrientedBox, corner)
		witness, valid := orientedBoxPoint(point)
		onBoth := vecMeasurementToInternal(witness)
		if !valid || witness.Bound.Base() > report.Request.PointResolution.Base() {
			report.Reason = reportvocab.ContactPointTooCoarse
			return
		}
		points = append(points, reportvocab.ContactPoint{OnA: onBoth, OnB: onBoth, Normal: normal,
			NormalAngle: angle, Separation: measurement.Measurement{Value: units.Millimeters(0),
				Bound: units.Millimeters(0), Exactness: measurement.Exact}, FaceA: faceNode(faceA), FaceB: faceNode(faceB),
			FeatureA: reportvocab.ContactFeature{Face: faceNode(faceA)}, FeatureB: reportvocab.ContactFeature{Face: faceNode(faceB)}})
	}
	report.Manifold = &reportvocab.ContactManifold{Points: points}
	report.Reason = reportvocab.ContactNoReason
}

func orientedBoxPoint(point [3]*big.Rat) (VecMeasurement, bool) {
	reading, ok := pairbox.RationalPointReading(point)
	if !ok {
		return VecMeasurement{}, false
	}
	return VecMeasurement{Value: reading.Value, Bound: units.Millimeters(reading.BoundMM),
		Exactness: exactnessOf(reading.BoundMM)}, true
}

func orientedBoxNormal(axis proofarith.DyV3) (measurement.VecMeasurement, units.Value, bool) {
	reading, ok := pairbox.DyadicNormalReading(axis)
	if !ok {
		return measurement.VecMeasurement{}, units.Value{}, false
	}
	return measurement.VecMeasurement{Value: reading.Value, Bound: units.Scalar(reading.Bound),
		Exactness: measurement.ExactnessOf(reading.Bound)}, units.Radians(reading.Angle), true
}
