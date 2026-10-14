package decad

import (
	"github.com/lestrrat-3d/decad/internal/clearance"
	"github.com/lestrrat-3d/decad/internal/measurement"
	pairbox "github.com/lestrrat-3d/decad/internal/pair/box"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
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
	pp, ok := b.n.Payload.(prismPayload)
	if !ok || !b.n.Solid || b.n.Kind != topo.BodySolid || pp.SurfaceResult ||
		pp.SectionDelta != 0 || pp.Z0Delta != 0 || pp.Z1Delta != 0 ||
		len(pp.Profile.Holes) != 0 || len(pp.Profile.Outer.Segments) != 1 ||
		!pairbox.CardinalBasis(pp.Frame.U(), pp.Frame.V(), pp.Frame.N()) ||
		!signedAxisTransform(pp.Xform) || !signedAxisTransform(pose) ||
		!proofbound.FiniteVec(pp.Frame.Origin()) || !measurement.FiniteValues(pp.Z0, pp.Z1) ||
		pp.Z0 >= pp.Z1 {
		return sourceCylinderContactProof{}, false
	}
	circle, ok := pp.Profile.Outer.Segments[0].(circleSeg)
	if !ok || !circle.CCW || circle.TStart != 0 || circle.TEnd != 1 ||
		circle.Radius.Kind() != units.Length ||
		!measurement.FiniteValues(circle.Center.U, circle.Center.V, circle.Radius.Base()) ||
		circle.Radius.Base() <= 0 {
		return sourceCylinderContactProof{}, false
	}
	axis, _, ok := clearance.SignedAxis(pose.ApplyDir(pp.Xform.ApplyDir(pp.Frame.N())))
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
		if face.n.NormalBound != 0 {
			return sourceCylinderContactProof{}, false
		}
		switch surface := face.n.Surface.(type) {
		case surfacegeom.Plane:
			planes++
			normal := surface.Frame.N()
			if face.n.Reversed {
				normal = normal.Scale(-1)
			}
			faceAxis, side, valid := clearance.SignedAxis(pose.ApplyDir(normal))
			if !valid || endFaces[side] != nil || faceAxis != axis {
				return sourceCylinderContactProof{}, false
			}
			endFaces[side] = face
		case surfacegeom.Cylinder:
			walls++
			wallFace = face
		default:
			return sourceCylinderContactProof{}, false
		}
	}
	if planes != 2 || walls != 1 || endFaces[0] == nil || endFaces[1] == nil {
		return sourceCylinderContactProof{}, false
	}
	numeric := pairbox.SourcePrismCylinderBox(pp.Frame, pp.Xform, pose, circle.Center,
		pp.Z0, pp.Z1, circle.Radius.Base(), axis)
	box := sourceBoxContactProof{lo: numeric.Lo, hi: numeric.Hi}
	return sourceCylinderContactProof{box: box, axis: axis, faces: endFaces, wall: wallFace}, true
}

// sourceRevolvedCylinderAtPose reads the full recorded meridian, not the
// body's outer bounds. A rectangle with one edge exactly on a cardinal axis
// sweeps one complete disk at every level of its axial interval.
func sourceRevolvedCylinderAtPose(b *Body, pose r3.Transform) (sourceCylinderContactProof, bool) {
	rp, ok := b.n.Payload.(revolvePayload)
	if !ok || !b.n.Solid || b.n.Kind != topo.BodySolid || rp.surfaceResult || !rp.full ||
		rp.sectionDelta != 0 || !pairbox.RectangularProfile(rp.profile) ||
		!pairbox.CardinalBasis(rp.frame.U(), rp.frame.V(), rp.frame.N()) ||
		!signedAxisTransform(rp.xform) || !signedAxisTransform(pose) ||
		!proofbound.FiniteVec(rp.frame.Origin()) ||
		rp.ax.AUBound != 0 || rp.ax.AVBound != 0 ||
		rp.ax.DUBound != 0 || rp.ax.DVBound != 0 {
		return sourceCylinderContactProof{}, false
	}
	if _, _, ok := clearance.SignedAxis(r3.Vec{X: rp.ax.DU, Y: rp.ax.DV}); !ok {
		return sourceCylinderContactProof{}, false
	}
	if !measurement.FiniteValues(rp.ax.AU, rp.ax.AV) {
		return sourceCylinderContactProof{}, false
	}
	axis, _, ok := clearance.SignedAxis(pose.ApplyDir(rp.xform.ApplyDir(rp.basis().W)))
	if !ok {
		return sourceCylinderContactProof{}, false
	}
	numeric, ok := pairbox.SourceRevolvedCylinderBox(rp.frame, rp.xform, pose,
		planePoint{U: rp.ax.AU, V: rp.ax.AV}, planePoint{U: rp.ax.DU, V: rp.ax.DV},
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
		if face.n.NormalBound != 0 {
			return sourceCylinderContactProof{}, false
		}
		switch surface := face.n.Surface.(type) {
		case surfacegeom.Plane:
			planes++
			normal := surface.Frame.N()
			if face.n.Reversed {
				normal = normal.Scale(-1)
			}
			faceAxis, side, valid := clearance.SignedAxis(pose.ApplyDir(normal))
			if !valid || faceAxis != axis || endFaces[side] != nil {
				return sourceCylinderContactProof{}, false
			}
			endFaces[side] = face
		case surfacegeom.Cylinder:
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
