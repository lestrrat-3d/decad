package decad

import (
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
}

func sourceCylinderAtPose(b *Body, pose r3.Transform) (sourceCylinderContactProof, bool) {
	if cylinder, ok := sourceRevolvedCylinderAtPose(b, pose); ok {
		return cylinder, true
	}
	pp, ok := b.payload.(prismPayload)
	if !ok || !b.solid || b.kind != BodySolid || pp.surfaceResult ||
		pp.sectionDelta != 0 || pp.z0Delta != 0 || pp.z1Delta != 0 ||
		len(pp.profile.Holes) != 0 || len(pp.profile.Outer.Segments) != 1 ||
		!cardinalBasis(pp.frame.U(), pp.frame.V(), pp.frame.N()) ||
		!signedAxisTransform(pp.xform) || !signedAxisTransform(pose) ||
		!finiteVec(pp.frame.Origin()) || !finiteMeasurementValues(pp.z0, pp.z1) ||
		pp.z0 >= pp.z1 {
		return sourceCylinderContactProof{}, false
	}
	circle, ok := pp.profile.Outer.Segments[0].(CircleSeg)
	if !ok || !circle.CCW || circle.TStart != 0 || circle.TEnd != 1 ||
		circle.Radius.Kind() != units.Length ||
		!finiteMeasurementValues(circle.Center.U, circle.Center.V, circle.Radius.Base()) ||
		circle.Radius.Base() <= 0 {
		return sourceCylinderContactProof{}, false
	}
	axis, _, ok := signedAxis(pose.ApplyDir(pp.xform.ApplyDir(pp.frame.N())))
	if !ok {
		return sourceCylinderContactProof{}, false
	}
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
			faceAxis, side, valid := signedAxis(pose.ApplyDir(normal))
			if !valid || endFaces[side] != nil || faceAxis != axis {
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
	center := dvAdd(dyVec(pp.frame.Origin()), dvAdd(
		dyScaleVec(dyVec(pp.frame.U()), mustDyOf(circle.Center.U)),
		dyScaleVec(dyVec(pp.frame.V()), mustDyOf(circle.Center.V))))
	low := dvAdd(center, dyScaleVec(dyVec(pp.frame.N()), mustDyOf(pp.z0)))
	high := dvAdd(center, dyScaleVec(dyVec(pp.frame.N()), mustDyOf(pp.z1)))
	low = exactContactTransform(pose, exactContactTransform(pp.xform, low))
	high = exactContactTransform(pose, exactContactTransform(pp.xform, high))
	radius := mustDyOf(circle.Radius.Base())
	var box sourceBoxContactProof
	for i := range 3 {
		box.lo[i], box.hi[i] = dyMin(low[i], high[i]), dyMax(low[i], high[i])
		if i != axis {
			box.lo[i] = dySubScalar(box.lo[i], radius)
			box.hi[i] = dyAdd(box.hi[i], radius)
		}
	}
	return sourceCylinderContactProof{box: box, axis: axis, faces: endFaces}, true
}

// sourceRevolvedCylinderAtPose reads the full recorded meridian, not the
// body's outer bounds. A rectangle with one edge exactly on a cardinal axis
// sweeps one complete disk at every level of its axial interval.
func sourceRevolvedCylinderAtPose(b *Body, pose r3.Transform) (sourceCylinderContactProof, bool) {
	rp, ok := b.payload.(revolvePayload)
	if !ok || !b.solid || b.kind != BodySolid || rp.surfaceResult || !rp.full ||
		rp.sectionDelta != 0 || !rectangularProfile(rp.profile) ||
		!cardinalBasis(rp.frame.U(), rp.frame.V(), rp.frame.N()) ||
		!signedAxisTransform(rp.xform) || !signedAxisTransform(pose) ||
		!finiteVec(rp.frame.Origin()) ||
		rp.ax.aUBound != 0 || rp.ax.aVBound != 0 ||
		rp.ax.dUBound != 0 || rp.ax.dVBound != 0 {
		return sourceCylinderContactProof{}, false
	}
	if _, _, ok := signedAxis(r3.Vec{X: rp.ax.dU, Y: rp.ax.dV}); !ok {
		return sourceCylinderContactProof{}, false
	}
	if !finiteMeasurementValues(rp.ax.aU, rp.ax.aV) {
		return sourceCylinderContactProof{}, false
	}
	var zlo, zhi, rhoLo, rhoHi dyadic
	for i, seg := range rp.profile.Outer.Segments {
		line, ok := seg.(LineSeg)
		if !ok || !finiteMeasurementValues(line.Start.U, line.Start.V) {
			return sourceCylinderContactProof{}, false
		}
		du := dySubScalar(mustDyOf(line.Start.U), mustDyOf(rp.ax.aU))
		dv := dySubScalar(mustDyOf(line.Start.V), mustDyOf(rp.ax.aV))
		z := dyAdd(dyMul(du, mustDyOf(rp.ax.dU)), dyMul(dv, mustDyOf(rp.ax.dV)))
		rho := dySubScalar(dyMul(dv, mustDyOf(rp.ax.dU)), dyMul(du, mustDyOf(rp.ax.dV)))
		if i == 0 {
			zlo, zhi, rhoLo, rhoHi = z, z, rho, rho
		} else {
			zlo, zhi = dyMin(zlo, z), dyMax(zhi, z)
			rhoLo, rhoHi = dyMin(rhoLo, rho), dyMax(rhoHi, rho)
		}
	}
	if !rhoLo.isZero() || rhoHi.sign() <= 0 || dyCmp(zlo, zhi) >= 0 {
		return sourceCylinderContactProof{}, false
	}
	faces := b.Faces()
	if len(faces) != 3 || len(b.Edges()) != 2 {
		return sourceCylinderContactProof{}, false
	}
	planes, walls := 0, 0
	for _, face := range faces {
		if face.normalBound != 0 {
			return sourceCylinderContactProof{}, false
		}
		switch face.surface.(type) {
		case Plane:
			planes++
		case Cylinder:
			walls++
		default:
			return sourceCylinderContactProof{}, false
		}
	}
	if planes != 2 || walls != 1 {
		return sourceCylinderContactProof{}, false
	}
	anchor := dvAdd(dyVec(rp.frame.Origin()), dvAdd(
		dyScaleVec(dyVec(rp.frame.U()), mustDyOf(rp.ax.aU)),
		dyScaleVec(dyVec(rp.frame.V()), mustDyOf(rp.ax.aV))))
	w := dvAdd(dyScaleVec(dyVec(rp.frame.U()), mustDyOf(rp.ax.dU)),
		dyScaleVec(dyVec(rp.frame.V()), mustDyOf(rp.ax.dV)))
	low := exactContactTransform(pose, exactContactTransform(rp.xform, dvAdd(anchor, dyScaleVec(w, zlo))))
	high := exactContactTransform(pose, exactContactTransform(rp.xform, dvAdd(anchor, dyScaleVec(w, zhi))))
	axis, _, ok := signedAxis(pose.ApplyDir(rp.xform.ApplyDir(rp.basis().w)))
	if !ok {
		return sourceCylinderContactProof{}, false
	}
	var box sourceBoxContactProof
	for i := range 3 {
		box.lo[i], box.hi[i] = dyMin(low[i], high[i]), dyMax(low[i], high[i])
		if i != axis {
			box.lo[i] = dySubScalar(box.lo[i], rhoHi)
			box.hi[i] = dyAdd(box.hi[i], rhoHi)
		}
	}
	return sourceCylinderContactProof{box: box, axis: axis}, true
}

// The disk-inside-face corridor makes axial support comparisons exact for
// separation. The extruded source's original end faces also certify planar
// touch and shallow crossing of the opposed faces.
func classifySourceCylinderBox(report *ContactReport, cylinder sourceCylinderContactProof,
	box sourceBoxContactProof, cylinderFirst bool) {
	if !cylinderInsideBoxFace(cylinder.box, box, cylinder.axis) {
		report.Reason = ContactNoGapProof
		return
	}
	axis := cylinder.axis
	var side int
	var signedGap dyadic
	switch {
	case dyCmp(cylinder.box.lo[axis], box.hi[axis]) >= 0:
		side, signedGap = 1, dySubScalar(cylinder.box.lo[axis], box.hi[axis])
	case dyCmp(cylinder.box.hi[axis], box.lo[axis]) <= 0:
		side, signedGap = 0, dySubScalar(box.lo[axis], cylinder.box.hi[axis])
	case dyCmp(cylinder.box.lo[axis], box.lo[axis]) > 0 &&
		dyCmp(cylinder.box.hi[axis], box.hi[axis]) > 0:
		side, signedGap = 1, dySubScalar(cylinder.box.lo[axis], box.hi[axis])
	case dyCmp(cylinder.box.hi[axis], box.hi[axis]) < 0 &&
		dyCmp(cylinder.box.lo[axis], box.lo[axis]) < 0:
		side, signedGap = 0, dySubScalar(box.lo[axis], cylinder.box.hi[axis])
	default:
		report.Reason = ContactAmbiguousFeature
		return
	}
	if signedGap.sign() > 0 {
		var gaps [3]dyadic
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
	if signedGap.isZero() {
		report.Relation = ContactTouching
		gap := Measurement{Value: units.Millimeters(0), Bound: units.Millimeters(0), Exactness: Exact}
		report.Gap = &gap
	} else {
		report.Relation = ContactOverlapping
	}
	boxFace, cylinderFace := box.faces[axis][side], cylinder.faces[1-side]
	if boxFace == nil || cylinderFace == nil {
		report.Reason = ContactNoNormalProof
		return
	}
	var boxPoint, cylinderPoint dyV3
	for i := range 3 {
		if i == axis {
			if side == 1 {
				boxPoint[i], cylinderPoint[i] = box.hi[i], cylinder.box.lo[i]
			} else {
				boxPoint[i], cylinderPoint[i] = box.lo[i], cylinder.box.hi[i]
			}
			continue
		}
		center := dyMul(dyAdd(cylinder.box.lo[i], cylinder.box.hi[i]), mustDyOf(.5))
		boxPoint[i], cylinderPoint[i] = center, center
	}
	boxWitness, okBox := sourceBoxPointAt(&boxPoint)
	cylinderWitness, okCylinder := sourceBoxPointAt(&cylinderPoint)
	if !okBox || !okCylinder ||
		boxWitness.Bound.Base() > report.Request.PointResolution.Base() ||
		cylinderWitness.Bound.Base() > report.Request.PointResolution.Base() {
		report.Reason = ContactPointTooCoarse
		return
	}
	separation, ok := sourceBoxSignedReading(signedGap)
	if !ok {
		report.Reason = ContactPointTooCoarse
		return
	}
	sign := signIfBoxSide(side)
	point := ContactPoint{OnA: boxWitness, OnB: cylinderWitness,
		FaceA: boxFace, FaceB: cylinderFace,
		FeatureA: ContactFeature{Face: boxFace}, FeatureB: ContactFeature{Face: cylinderFace},
		NormalAngle: units.Radians(0), Separation: separation}
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

func cylinderInsideBoxFace(cylinder, box sourceBoxContactProof, axial int) bool {
	for axis := range 3 {
		if axis == axial {
			continue
		}
		if dyCmp(cylinder.lo[axis], box.lo[axis]) <= 0 ||
			dyCmp(cylinder.hi[axis], box.hi[axis]) >= 0 {
			return false
		}
	}
	return true
}
