package contactbox

import (
	"github.com/lestrrat-3d/decad/internal/clearance"
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/pair"
	pairbox "github.com/lestrrat-3d/decad/internal/pair/box"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	vocab "github.com/lestrrat-3d/decad/report"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// OrientedSourceBox pairs the internal exact parallelotope with its source
// face identities, which belong to the public body.
type OrientedSourceBox struct {
	pairbox.OrientedBox
	Faces [3][2]*topo.Face
}

func SourceOrientedBoxAtPose(body *topo.Body, pose r3.Transform) (OrientedSourceBox, bool) {
	pp, ok := body.Payload.(prismrecord.Payload)
	if !ok || !body.Solid || body.Kind != topo.BodySolid || pp.SurfaceResult ||
		pp.SectionDelta != 0 || pp.Z0Delta != 0 || pp.Z1Delta != 0 ||
		!pairbox.SignedAxisTransform(pp.Xform) {
		return OrientedSourceBox{}, false
	}
	geometry, placedEdge, ok := pairbox.SourceOrientedBox(pp.Profile, pp.Frame, pp.Z0, pp.Z1, pp.Xform, pose)
	if !ok {
		return OrientedSourceBox{}, false
	}
	box := OrientedSourceBox{OrientedBox: geometry}
	// The original planar faces retain their source identities after the read
	// pose. Match them in the body's cardinal placed frame, before rotation.
	faces := topobuild.Faces(body)
	if len(faces) != 6 {
		return OrientedSourceBox{}, false
	}
	for _, face := range faces {
		plane, ok := face.Surface.(surfacegeom.Plane)
		if !ok || face.NormalBound != 0 {
			return OrientedSourceBox{}, false
		}
		normal := plane.Frame.N()
		if face.Reversed {
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
			if mapped || box.Faces[axis][side] != nil {
				return OrientedSourceBox{}, false
			}
			box.Faces[axis][side], mapped = face, true
		}
		if !mapped {
			return OrientedSourceBox{}, false
		}
	}
	for axis := range 3 {
		if box.Faces[axis][0] == nil || box.Faces[axis][1] == nil {
			return OrientedSourceBox{}, false
		}
	}
	return box, true
}

// OrientedBoxRelation maps the box kernel's relation to the contact report's.
func OrientedBoxRelation(a, b OrientedSourceBox) (vocab.ContactRelation, proofarith.Dyadic, proofarith.Dyadic) {
	relation, gap, normSquared := pairbox.OrientedBoxRelation(a.OrientedBox, b.OrientedBox)
	switch relation {
	case pair.Separated:
		return vocab.ContactSeparated, gap, normSquared
	case pair.Touching:
		return vocab.ContactTouching, gap, normSquared
	case pair.Overlapping:
		return vocab.ContactOverlapping, gap, normSquared
	default:
		return vocab.ContactUndecided, gap, normSquared
	}
}

func ClassifyOriented(report *contactReport, a, b OrientedSourceBox) {
	relation, gap, normSquared := OrientedBoxRelation(a, b)
	switch relation {
	case vocab.ContactTouching:
		report.Relation = relation
		report.Gap = &measurement.Measurement{Value: units.Millimeters(0), Bound: units.Millimeters(0), Exactness: measurement.Exact}
		report.Reason = vocab.ContactNoNormalProof
		publishContainedHorizontalPatch(report, a, b)
		if report.Manifold == nil {
			PublishClippedHorizontalPatch(report, a, b)
		}
		if report.Manifold == nil && report.Reason != vocab.ContactPointTooCoarse {
			PublishOrientedBoxPatch(report, a, b)
		}
		if report.Manifold == nil && report.Reason != vocab.ContactPointTooCoarse {
			publishOrientedAxisPatch(report, &a, &b)
		}
	case vocab.ContactOverlapping:
		report.Relation, report.Reason = relation, vocab.ContactNoNormalProof
		publishContainedHorizontalPatch(report, a, b)
		if report.Manifold == nil {
			publishOrientedAxisPatch(report, &a, &b)
		}
	case vocab.ContactSeparated:
		reading, ok := orientedBoxGap(a, b, gap, normSquared)
		if !ok {
			report.Reason = vocab.ContactNoGapProof
			return
		}
		report.Relation, report.Gap = relation, &reading
	}
}

// publishContainedHorizontalPatch handles a complete rotating box face
// inside an axis-aligned box face. Exact corner and side-plane comparisons
// certify the four-point patch, including a uniquely shallow penetration.
func publishContainedHorizontalPatch(report *contactReport, a, b OrientedSourceBox) {
	if base, ok := SourceBoxAtPose(report.A, report.PoseA); ok &&
		publishHorizontalPatchOrder(report, base, b, report.B, report.PoseB, true) {
		return
	}
	if base, ok := SourceBoxAtPose(report.B, report.PoseB); ok {
		publishHorizontalPatchOrder(report, base, a, report.A, report.PoseA, false)
	}
}

func publishHorizontalPatchOrder(report *contactReport, base SourceBox,
	rotated OrientedSourceBox, body *topo.Body, pose r3.Transform, baseIsA bool) bool {
	basis := pose.Basis()
	if basis.EX.Z != 0 || basis.EY.Z != 0 || basis.EZ != (r3.Vec{Z: 1}) {
		return false
	}
	source, ok := SourceBoxAtPose(body, r3.Identity())
	if !ok {
		return false
	}
	patch, ok := pairbox.ContainedHorizontalPatch(base.AxisBox(), rotated.OrientedBox, pair.RelationOfContact(report.Relation))
	if !ok {
		return false
	}
	reading, ok := pairbox.SignedMeasurement(patch.Separation)
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
	faceBase, faceRotated := base.Faces[2][patch.BaseSide], source.Faces[2][patch.RotatedSide]
	points := make([]contactPoint, 0, len(patch.Corners))
	for _, corner := range patch.Corners {
		basePoint := corner
		basePoint[2] = patch.SupportZ
		baseReading, okBase := pairbox.DyadicPointMeasurement(&basePoint)
		rotatedReading, okRotated := pairbox.DyadicPointMeasurement(&corner)
		if !okBase || !okRotated ||
			baseReading.Bound.Base() > report.Request.PointResolution.Base() ||
			rotatedReading.Bound.Base() > report.Request.PointResolution.Base() {
			return false
		}
		point := contactPoint{Normal: measurement.VecMeasurement{Value: r3.Vec{Z: normalZ},
			Exactness: measurement.Exact, Bound: units.Scalar(0)}, NormalAngle: units.Radians(0),
			Separation: reading}
		if baseIsA {
			point.OnA, point.OnB = baseReading, rotatedReading
			point.FaceA, point.FaceB = faceBase, faceRotated
		} else {
			point.OnA, point.OnB = rotatedReading, baseReading
			point.FaceA, point.FaceB = faceRotated, faceBase
		}
		point.FeatureA, point.FeatureB = contactFeature{Face: point.FaceA}, contactFeature{Face: point.FaceB}
		points = append(points, point)
	}
	report.Manifold = &contactManifold{Points: points}
	report.Reason = vocab.ContactNoReason
	return true
}

// publishOrientedAxisPatch reduces two opposed axis-normal faces to one
// certified interior witness. It leaves other oriented contacts without a
// manifold, including edge and corner contacts.
func publishOrientedAxisPatch(report *contactReport, a, b *OrientedSourceBox) {
	var chosen *contactPoint
	best := proofarith.DyZero()
	_, alignedA := SourceBoxAtPose(report.A, report.PoseA)
	_, alignedB := SourceBoxAtPose(report.B, report.PoseB)
	for _, candidate := range pairbox.AxisFaceWitnesses(a.OrientedBox, b.OrientedBox, pair.RelationOfContact(report.Relation)) {
		// A horizontal patch against a signed-axis box needs the complete
		// contained four-point proof above; one interior point cannot replace it.
		if candidate.Axis == 2 && (alignedA || alignedB) {
			continue
		}
		if chosen != nil && proofarith.DyCmp(proofarith.DyAbs(candidate.Separation), proofarith.DyAbs(best)) >= 0 {
			continue
		}
		reading, ok := pairbox.SignedMeasurement(candidate.Separation)
		if !ok || reading.Bound.Base() > report.Request.PointResolution.Base() {
			continue
		}
		onA, readA := pairbox.DyadicPointMeasurement(&candidate.PointA)
		onB, readB := pairbox.DyadicPointMeasurement(&candidate.PointB)
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
		point := contactPoint{
			OnA: onA, OnB: onB,
			Normal:      measurement.VecMeasurement{Value: normal, Exactness: measurement.Exact, Bound: units.Scalar(0)},
			NormalAngle: units.Radians(0), Separation: reading,
			FaceA: fa, FaceB: fb,
			FeatureA: contactFeature{Face: fa}, FeatureB: contactFeature{Face: fb},
		}
		chosen, best = &point, candidate.Separation
	}
	if chosen != nil {
		report.Manifold = &contactManifold{Points: []contactPoint{*chosen}}
		report.Reason = vocab.ContactNoReason
	}
}

func orientedSourceFace(body *topo.Body, pose r3.Transform, axis, side int) *topo.Face {
	for _, face := range topobuild.Faces(body) {
		plane, ok := face.Surface.(surfacegeom.Plane)
		if !ok || face.NormalBound != 0 {
			continue
		}
		normal := plane.Frame.N()
		if face.Reversed {
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
func orientedBoxGap(a, b OrientedSourceBox, gap, normSquared proofarith.Dyadic) (measurement.Measurement, bool) {
	reading, ok := pairbox.OrientedBoxGap(a.OrientedBox, b.OrientedBox, gap, normSquared)
	if !ok {
		return measurement.Measurement{}, false
	}
	return measurement.Measurement{Value: units.Millimeters(reading.ValueMM), Bound: units.Millimeters(reading.BoundMM),
		Exactness: measurement.ExactnessOf(reading.BoundMM)}, true
}

// PublishOrientedBoxPatch handles opposed faces of boxes with parallel source
// edge directions. Their complete contact set is a rectangle in A's exact
// dual basis, even when the read rotation's dyadic entries are slightly skew.
func PublishOrientedBoxPatch(report *contactReport, a, b OrientedSourceBox) {
	geometry, failure := pairbox.OrientedFacePatch(a.OrientedBox, b.OrientedBox)
	switch failure {
	case pairbox.OrientedPatchNoNormal:
		report.Reason = vocab.ContactNoNormalProof
		return
	case pairbox.OrientedPatchAmbiguous:
		report.Reason = vocab.ContactAmbiguousFeature
		return
	}
	faceA := a.Faces[geometry.FaceA.Axis][geometry.FaceA.Side]
	faceB := b.Faces[geometry.FaceB.Axis][geometry.FaceB.Side]
	if faceA == nil || faceB == nil {
		report.Reason = vocab.ContactNoNormalProof
		return
	}
	normal, angle, ok := pairbox.DyadicNormalMeasurement(geometry.Normal)
	if !ok || angle.Base() > report.Request.NormalResolution.Base() {
		report.Reason = vocab.ContactNoNormalProof
		return
	}
	points := make([]contactPoint, 0, 4)
	for _, corner := range [][2]int{{0, 0}, {1, 0}, {1, 1}, {0, 1}} {
		point := geometry.Point(a.OrientedBox, corner)
		onBoth, valid := pairbox.PointMeasurement(point)
		if !valid || onBoth.Bound.Base() > report.Request.PointResolution.Base() {
			report.Reason = vocab.ContactPointTooCoarse
			return
		}
		points = append(points, contactPoint{OnA: onBoth, OnB: onBoth, Normal: normal,
			NormalAngle: angle, Separation: measurement.Measurement{Value: units.Millimeters(0),
				Bound: units.Millimeters(0), Exactness: measurement.Exact}, FaceA: faceA, FaceB: faceB,
			FeatureA: contactFeature{Face: faceA}, FeatureB: contactFeature{Face: faceB}})
	}
	report.Manifold = &contactManifold{Points: points}
	report.Reason = vocab.ContactNoReason
}
