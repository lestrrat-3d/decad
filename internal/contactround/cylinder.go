package contactround

import (
	"github.com/lestrrat-3d/decad/internal/clearance"
	"github.com/lestrrat-3d/decad/internal/contactbox"
	"github.com/lestrrat-3d/decad/internal/measurement"
	pairbox "github.com/lestrrat-3d/decad/internal/pair/box"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/reportvocab"
	"github.com/lestrrat-3d/decad/internal/revolveplan"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// CylinderProof encloses a complete source cylinder from either
// a full-circle prism or a full revolve of an axis-incident rectangle. Its
// box is an outer bound; the strict face corridor makes axial support exact.
type CylinderProof struct {
	Box   contactbox.SourceBox
	Axis  int
	Faces [2]*topo.Face
	Wall  *topo.Face
}

func CylinderAtPose(b *topo.Body, pose r3.Transform) (CylinderProof, bool) {
	if cylinder, ok := RevolvedCylinderAtPose(b, pose); ok {
		return cylinder, true
	}
	pp, ok := b.Payload.(prismrecord.Payload)
	if !ok || !b.Solid || b.Kind != topo.BodySolid || pp.SurfaceResult ||
		pp.SectionDelta != 0 || pp.Z0Delta != 0 || pp.Z1Delta != 0 ||
		len(pp.Profile.Holes) != 0 || len(pp.Profile.Outer.Segments) != 1 ||
		!pairbox.CardinalBasis(pp.Frame.U(), pp.Frame.V(), pp.Frame.N()) ||
		!pairbox.SignedAxisTransform(pp.Xform) || !pairbox.SignedAxisTransform(pose) ||
		!proofbound.FiniteVec(pp.Frame.Origin()) || !measurement.FiniteValues(pp.Z0, pp.Z1) ||
		pp.Z0 >= pp.Z1 {
		return CylinderProof{}, false
	}
	circle, ok := pp.Profile.Outer.Segments[0].(sectionrecord.CircleSeg)
	if !ok || !circle.CCW || circle.TStart != 0 || circle.TEnd != 1 ||
		circle.Radius.Kind() != units.Length ||
		!measurement.FiniteValues(circle.Center.U, circle.Center.V, circle.Radius.Base()) ||
		circle.Radius.Base() <= 0 {
		return CylinderProof{}, false
	}
	axis, _, ok := clearance.SignedAxis(pose.ApplyDir(pp.Xform.ApplyDir(pp.Frame.N())))
	if !ok {
		return CylinderProof{}, false
	}
	faces := topobuild.Faces(b)
	if len(faces) != 3 || len(topobuild.Edges(b)) != 2 {
		return CylinderProof{}, false
	}
	planes, walls := 0, 0
	var endFaces [2]*topo.Face
	var wallFace *topo.Face
	for _, face := range faces {
		if face.NormalBound != 0 {
			return CylinderProof{}, false
		}
		switch surface := face.Surface.(type) {
		case surfacegeom.Plane:
			planes++
			normal := surface.Frame.N()
			if face.Reversed {
				normal = normal.Scale(-1)
			}
			faceAxis, side, valid := clearance.SignedAxis(pose.ApplyDir(normal))
			if !valid || endFaces[side] != nil || faceAxis != axis {
				return CylinderProof{}, false
			}
			endFaces[side] = face
		case surfacegeom.Cylinder:
			walls++
			wallFace = face
		default:
			return CylinderProof{}, false
		}
	}
	if planes != 2 || walls != 1 || endFaces[0] == nil || endFaces[1] == nil {
		return CylinderProof{}, false
	}
	numeric := pairbox.SourcePrismCylinderBox(pp.Frame, pp.Xform, pose, circle.Center,
		pp.Z0, pp.Z1, circle.Radius.Base(), axis)
	box := contactbox.SourceBox{Lo: numeric.Lo, Hi: numeric.Hi}
	return CylinderProof{Box: box, Axis: axis, Faces: endFaces, Wall: wallFace}, true
}

// RevolvedCylinderAtPose reads the full recorded meridian, not the
// body's outer bounds. A rectangle with one edge exactly on a cardinal axis
// sweeps one complete disk at every level of its axial interval.
func RevolvedCylinderAtPose(b *topo.Body, pose r3.Transform) (CylinderProof, bool) {
	rp, ok := b.Payload.(revolveplan.Payload)
	if !ok || !b.Solid || b.Kind != topo.BodySolid || rp.SurfaceResult || !rp.Full ||
		rp.SectionDelta != 0 || !pairbox.RectangularProfile(rp.Profile) ||
		!pairbox.CardinalBasis(rp.Frame.U(), rp.Frame.V(), rp.Frame.N()) ||
		!pairbox.SignedAxisTransform(rp.Xform) || !pairbox.SignedAxisTransform(pose) ||
		!proofbound.FiniteVec(rp.Frame.Origin()) ||
		rp.Ax.AUBound != 0 || rp.Ax.AVBound != 0 ||
		rp.Ax.DUBound != 0 || rp.Ax.DVBound != 0 {
		return CylinderProof{}, false
	}
	if _, _, ok := clearance.SignedAxis(r3.Vec{X: rp.Ax.DU, Y: rp.Ax.DV}); !ok {
		return CylinderProof{}, false
	}
	if !measurement.FiniteValues(rp.Ax.AU, rp.Ax.AV) {
		return CylinderProof{}, false
	}
	axis, _, ok := clearance.SignedAxis(pose.ApplyDir(rp.Xform.ApplyDir(rp.Basis().W)))
	if !ok {
		return CylinderProof{}, false
	}
	numeric, ok := pairbox.SourceRevolvedCylinderBox(rp.Frame, rp.Xform, pose,
		sectionrecord.Point2{U: rp.Ax.AU, V: rp.Ax.AV}, sectionrecord.Point2{U: rp.Ax.DU, V: rp.Ax.DV},
		rp.Profile.Outer.Segments, axis)
	if !ok {
		return CylinderProof{}, false
	}
	box := contactbox.SourceBox{Lo: numeric.Lo, Hi: numeric.Hi}
	faces := topobuild.Faces(b)
	if len(faces) != 3 || len(topobuild.Edges(b)) != 2 {
		return CylinderProof{}, false
	}
	planes, walls := 0, 0
	var endFaces [2]*topo.Face
	for _, face := range faces {
		if face.NormalBound != 0 {
			return CylinderProof{}, false
		}
		switch surface := face.Surface.(type) {
		case surfacegeom.Plane:
			planes++
			normal := surface.Frame.N()
			if face.Reversed {
				normal = normal.Scale(-1)
			}
			faceAxis, side, valid := clearance.SignedAxis(pose.ApplyDir(normal))
			if !valid || faceAxis != axis || endFaces[side] != nil {
				return CylinderProof{}, false
			}
			endFaces[side] = face
		case surfacegeom.Cylinder:
			walls++
		default:
			return CylinderProof{}, false
		}
	}
	if planes != 2 || walls != 1 || endFaces[0] == nil || endFaces[1] == nil {
		return CylinderProof{}, false
	}
	return CylinderProof{Box: box, Axis: axis, Faces: endFaces}, true
}

// The complete cylinder lies inside the two projected box-face intervals.
// Its axial disks or full circular sidewall supply exact support at the
// selected face; a sidewall touch has a complete axial line contact set.
func ClassifyCylinderBox(report *reportvocab.ContactReport, cylinder CylinderProof,
	box contactbox.SourceBox, cylinderFirst bool) {
	axis, side, signedGap, ok := pairbox.CylinderFaceCorridor(
		cylinder.Box.AxisBox(), box.AxisBox(), cylinder.Axis, cylinder.Wall != nil)
	if !ok {
		report.Reason = reportvocab.ContactNoGapProof
		return
	}
	if signedGap.Sign() > 0 {
		var gaps [3]proofarith.Dyadic
		gaps[axis] = signedGap
		gap, ok := pairbox.GapMeasurement(gaps)
		if !ok {
			report.Reason = reportvocab.ContactNoGapProof
			return
		}
		report.Relation, report.Gap = reportvocab.ContactSeparated, &gap
		return
	}
	if cylinder.Faces[0] == nil || cylinder.Faces[1] == nil {
		report.Reason = reportvocab.ContactNoGapProof
		return
	}
	if signedGap.IsZero() {
		report.Relation = reportvocab.ContactTouching
		gap := measurement.Measurement{Value: units.Millimeters(0), Bound: units.Millimeters(0), Exactness: measurement.Exact}
		report.Gap = &gap
	} else {
		report.Relation = reportvocab.ContactOverlapping
	}
	boxFace, cylinderFace := box.Faces[axis][side], cylinder.Faces[1-side]
	if axis != cylinder.Axis {
		cylinderFace = cylinder.Wall
	}
	if boxFace == nil || cylinderFace == nil {
		report.Reason = reportvocab.ContactNoNormalProof
		return
	}
	boxReading, cylinderReading, signedReading, ok := pairbox.CylinderFacePoint(
		cylinder.Box.AxisBox(), box.AxisBox(), axis, side, signedGap,
		report.Request.PointResolution.Base())
	if !ok {
		report.Reason = reportvocab.ContactPointTooCoarse
		return
	}
	sign := signIfBoxSide(side)
	boxWitness, cylinderWitness := boxReading.Measurement(), cylinderReading.Measurement()
	point := reportvocab.ContactPoint{OnA: boxWitness, OnB: cylinderWitness,
		FaceA: boxFace, FaceB: cylinderFace,
		FeatureA: reportvocab.ContactFeature{Face: boxFace}, FeatureB: reportvocab.ContactFeature{Face: cylinderFace},
		NormalAngle: units.Radians(0), Separation: signedReading.Measurement()}
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
	point.Normal = measurement.VecMeasurement{Value: normal, Bound: units.Scalar(0), Exactness: measurement.Exact}
	report.Manifold = &reportvocab.ContactManifold{Points: []reportvocab.ContactPoint{point}}
}
