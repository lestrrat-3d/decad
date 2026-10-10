package decad

import (
	"github.com/lestrrat-3d/decad/internal/clearance"
	pairbox "github.com/lestrrat-3d/decad/internal/pair/box"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// sourceCylinderContactProof encloses a complete source cylinder from either
// a full-circle prism or a full revolve of an axis-incident rectangle. Its
// box is an outer bound; the strict face corridor makes axial support exact.
type sourceCylinderContactProof struct {
	box   sourceBoxContactProof
	axis  int
	faces [2]*Face
	wall  *Face
}

func sourceCylinderAtPose(b *Body, pose r3.Transform) (sourceCylinderContactProof, bool) {
	if cylinder, ok := sourceRevolvedCylinderAtPose(b, pose); ok {
		return cylinder, true
	}
	pp, ok := b.payload.(prismPayload)
	if !ok || !b.solid || b.kind != BodySolid || pp.surfaceResult ||
		pp.sectionDelta != 0 || pp.z0Delta != 0 || pp.z1Delta != 0 ||
		len(pp.profile.Holes) != 0 || len(pp.profile.Outer.Segments) != 1 ||
		!pairbox.CardinalBasis(pp.frame.U(), pp.frame.V(), pp.frame.N()) ||
		!signedAxisTransform(pp.xform) || !signedAxisTransform(pose) ||
		!proofbound.FiniteVec(pp.frame.Origin()) || !finiteMeasurementValues(pp.z0, pp.z1) ||
		pp.z0 >= pp.z1 {
		return sourceCylinderContactProof{}, false
	}
	circle, ok := pp.profile.Outer.Segments[0].(circleSeg)
	if !ok || !circle.CCW || circle.TStart != 0 || circle.TEnd != 1 ||
		circle.Radius.Kind() != units.Length ||
		!finiteMeasurementValues(circle.Center.U, circle.Center.V, circle.Radius.Base()) ||
		circle.Radius.Base() <= 0 {
		return sourceCylinderContactProof{}, false
	}
	axis, _, ok := clearance.SignedAxis(pose.ApplyDir(pp.xform.ApplyDir(pp.frame.N())))
	if !ok {
		return sourceCylinderContactProof{}, false
	}
	faces := b.Faces()
	if len(faces) != 3 || len(b.Edges()) != 2 {
		return sourceCylinderContactProof{}, false
	}
	planes, walls := 0, 0
	var endFaces [2]*Face
	var wallFace *Face
	for _, face := range faces {
		if face.normalBound != 0 {
			return sourceCylinderContactProof{}, false
		}
		switch surface := face.surface.(type) {
		case Plane:
			planes++
			normal := surface.Frame.N()
			if face.reversed {
				normal = normal.Scale(-1)
			}
			faceAxis, side, valid := clearance.SignedAxis(pose.ApplyDir(normal))
			if !valid || endFaces[side] != nil || faceAxis != axis {
				return sourceCylinderContactProof{}, false
			}
			endFaces[side] = face
		case Cylinder:
			walls++
			wallFace = face
		default:
			return sourceCylinderContactProof{}, false
		}
	}
	if planes != 2 || walls != 1 || endFaces[0] == nil || endFaces[1] == nil {
		return sourceCylinderContactProof{}, false
	}
	numeric := pairbox.SourcePrismCylinderBox(pp.frame, pp.xform, pose, circle.Center,
		pp.z0, pp.z1, circle.Radius.Base(), axis)
	box := sourceBoxContactProof{lo: numeric.Lo, hi: numeric.Hi}
	return sourceCylinderContactProof{box: box, axis: axis, faces: endFaces, wall: wallFace}, true
}

// sourceRevolvedCylinderAtPose reads the full recorded meridian, not the
// body's outer bounds. A rectangle with one edge exactly on a cardinal axis
// sweeps one complete disk at every level of its axial interval.
func sourceRevolvedCylinderAtPose(b *Body, pose r3.Transform) (sourceCylinderContactProof, bool) {
	rp, ok := b.payload.(revolvePayload)
	if !ok || !b.solid || b.kind != BodySolid || rp.surfaceResult || !rp.full ||
		rp.sectionDelta != 0 || !pairbox.RectangularProfile(rp.profile) ||
		!pairbox.CardinalBasis(rp.frame.U(), rp.frame.V(), rp.frame.N()) ||
		!signedAxisTransform(rp.xform) || !signedAxisTransform(pose) ||
		!proofbound.FiniteVec(rp.frame.Origin()) ||
		rp.ax.aUBound != 0 || rp.ax.aVBound != 0 ||
		rp.ax.dUBound != 0 || rp.ax.dVBound != 0 {
		return sourceCylinderContactProof{}, false
	}
	if _, _, ok := clearance.SignedAxis(r3.Vec{X: rp.ax.dU, Y: rp.ax.dV}); !ok {
		return sourceCylinderContactProof{}, false
	}
	if !finiteMeasurementValues(rp.ax.aU, rp.ax.aV) {
		return sourceCylinderContactProof{}, false
	}
	axis, _, ok := clearance.SignedAxis(pose.ApplyDir(rp.xform.ApplyDir(rp.basis().W)))
	if !ok {
		return sourceCylinderContactProof{}, false
	}
	numeric, ok := pairbox.SourceRevolvedCylinderBox(rp.frame, rp.xform, pose,
		Point2{U: rp.ax.aU, V: rp.ax.aV}, Point2{U: rp.ax.dU, V: rp.ax.dV},
		rp.profile.Outer.Segments, axis)
	if !ok {
		return sourceCylinderContactProof{}, false
	}
	box := sourceBoxContactProof{lo: numeric.Lo, hi: numeric.Hi}
	faces := b.Faces()
	if len(faces) != 3 || len(b.Edges()) != 2 {
		return sourceCylinderContactProof{}, false
	}
	planes, walls := 0, 0
	var endFaces [2]*Face
	for _, face := range faces {
		if face.normalBound != 0 {
			return sourceCylinderContactProof{}, false
		}
		switch surface := face.surface.(type) {
		case Plane:
			planes++
			normal := surface.Frame.N()
			if face.reversed {
				normal = normal.Scale(-1)
			}
			faceAxis, side, valid := clearance.SignedAxis(pose.ApplyDir(normal))
			if !valid || faceAxis != axis || endFaces[side] != nil {
				return sourceCylinderContactProof{}, false
			}
			endFaces[side] = face
		case Cylinder:
			walls++
		default:
			return sourceCylinderContactProof{}, false
		}
	}
	if planes != 2 || walls != 1 || endFaces[0] == nil || endFaces[1] == nil {
		return sourceCylinderContactProof{}, false
	}
	return sourceCylinderContactProof{box: box, axis: axis, faces: endFaces}, true
}

// The complete cylinder lies inside the two projected box-face intervals.
// Its axial disks or full circular sidewall supply exact support at the
// selected face; a sidewall touch has a complete axial line contact set.
func classifySourceCylinderBox(report *ContactReport, cylinder sourceCylinderContactProof,
	box sourceBoxContactProof, cylinderFirst bool) {
	axis, side, signedGap, ok := pairbox.CylinderFaceCorridor(
		cylinder.box.axisBox(), box.axisBox(), cylinder.axis, cylinder.wall != nil)
	if !ok {
		report.Reason = ContactNoGapProof
		return
	}
	if signedGap.Sign() > 0 {
		var gaps [3]proofarith.Dyadic
		gaps[axis] = signedGap
		gap, ok := sourceBoxGap(gaps)
		if !ok {
			report.Reason = ContactNoGapProof
			return
		}
		report.Relation, report.Gap = ContactSeparated, &gap
		return
	}
	if cylinder.faces[0] == nil || cylinder.faces[1] == nil {
		report.Reason = ContactNoGapProof
		return
	}
	if signedGap.IsZero() {
		report.Relation = ContactTouching
		gap := Measurement{Value: units.Millimeters(0), Bound: units.Millimeters(0), Exactness: Exact}
		report.Gap = &gap
	} else {
		report.Relation = ContactOverlapping
	}
	boxFace, cylinderFace := box.faces[axis][side], cylinder.faces[1-side]
	if axis != cylinder.axis {
		cylinderFace = cylinder.wall
	}
	if boxFace == nil || cylinderFace == nil {
		report.Reason = ContactNoNormalProof
		return
	}
	boxReading, cylinderReading, signedReading, ok := pairbox.CylinderFacePoint(
		cylinder.box.axisBox(), box.axisBox(), axis, side, signedGap,
		report.Request.PointResolution.Base())
	if !ok {
		report.Reason = ContactPointTooCoarse
		return
	}
	sign := signIfBoxSide(side)
	boxWitness, cylinderWitness := sourceBoxPointMeasurement(boxReading), sourceBoxPointMeasurement(cylinderReading)
	point := ContactPoint{OnA: boxWitness, OnB: cylinderWitness,
		FaceA: boxFace, FaceB: cylinderFace,
		FeatureA: ContactFeature{Face: boxFace}, FeatureB: ContactFeature{Face: cylinderFace},
		NormalAngle: units.Radians(0), Separation: sourceBoxScalar(signedReading)}
	if cylinderFirst {
		point.OnA, point.OnB = point.OnB, point.OnA
		point.FaceA, point.FaceB = point.FaceB, point.FaceA
		point.FeatureA, point.FeatureB = point.FeatureB, point.FeatureA
		sign = -sign
	}
	var normal r3.Vec
	switch axis {
	case 0:
		normal.X = sign
	case 1:
		normal.Y = sign
	case 2:
		normal.Z = sign
	}
	point.Normal = VecMeasurement{Value: normal, Bound: units.Scalar(0), Exactness: Exact}
	report.Manifold = &ContactManifold{Points: []ContactPoint{point}}
}
