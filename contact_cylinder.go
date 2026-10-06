package decad

import (
	"github.com/lestrrat-3d/decad/internal/clearance"
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
		!cardinalBasis(pp.frame.U(), pp.frame.V(), pp.frame.N()) ||
		!signedAxisTransform(pp.xform) || !signedAxisTransform(pose) ||
		!proofbound.FiniteVec(pp.frame.Origin()) || !finiteMeasurementValues(pp.z0, pp.z1) ||
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
	center := proofarith.DvAdd(proofarith.DyVec(pp.frame.Origin()), proofarith.DvAdd(
		dyScaleVec(proofarith.DyVec(pp.frame.U()), proofarith.MustDyOf(circle.Center.U)),
		dyScaleVec(proofarith.DyVec(pp.frame.V()), proofarith.MustDyOf(circle.Center.V))))
	low := proofarith.DvAdd(center, dyScaleVec(proofarith.DyVec(pp.frame.N()), proofarith.MustDyOf(pp.z0)))
	high := proofarith.DvAdd(center, dyScaleVec(proofarith.DyVec(pp.frame.N()), proofarith.MustDyOf(pp.z1)))
	low = exactContactTransform(pose, exactContactTransform(pp.xform, low))
	high = exactContactTransform(pose, exactContactTransform(pp.xform, high))
	radius := proofarith.MustDyOf(circle.Radius.Base())
	var box sourceBoxContactProof
	for i := range 3 {
		box.lo[i], box.hi[i] = dyMin(low[i], high[i]), dyMax(low[i], high[i])
		if i != axis {
			box.lo[i] = proofarith.DySubScalar(box.lo[i], radius)
			box.hi[i] = proofarith.DyAdd(box.hi[i], radius)
		}
	}
	return sourceCylinderContactProof{box: box, axis: axis, faces: endFaces, wall: wallFace}, true
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
	var zlo, zhi, rhoLo, rhoHi proofarith.Dyadic
	for i, seg := range rp.profile.Outer.Segments {
		line, ok := seg.(LineSeg)
		if !ok || !finiteMeasurementValues(line.Start.U, line.Start.V) {
			return sourceCylinderContactProof{}, false
		}
		du := proofarith.DySubScalar(proofarith.MustDyOf(line.Start.U), proofarith.MustDyOf(rp.ax.aU))
		dv := proofarith.DySubScalar(proofarith.MustDyOf(line.Start.V), proofarith.MustDyOf(rp.ax.aV))
		z := proofarith.DyAdd(proofarith.DyMul(du, proofarith.MustDyOf(rp.ax.dU)), proofarith.DyMul(dv, proofarith.MustDyOf(rp.ax.dV)))
		rho := proofarith.DySubScalar(proofarith.DyMul(dv, proofarith.MustDyOf(rp.ax.dU)), proofarith.DyMul(du, proofarith.MustDyOf(rp.ax.dV)))
		if i == 0 {
			zlo, zhi, rhoLo, rhoHi = z, z, rho, rho
		} else {
			zlo, zhi = dyMin(zlo, z), dyMax(zhi, z)
			rhoLo, rhoHi = dyMin(rhoLo, rho), dyMax(rhoHi, rho)
		}
	}
	if !rhoLo.IsZero() || rhoHi.Sign() <= 0 || proofarith.DyCmp(zlo, zhi) >= 0 {
		return sourceCylinderContactProof{}, false
	}
	anchor := proofarith.DvAdd(proofarith.DyVec(rp.frame.Origin()), proofarith.DvAdd(
		dyScaleVec(proofarith.DyVec(rp.frame.U()), proofarith.MustDyOf(rp.ax.aU)),
		dyScaleVec(proofarith.DyVec(rp.frame.V()), proofarith.MustDyOf(rp.ax.aV))))
	w := proofarith.DvAdd(dyScaleVec(proofarith.DyVec(rp.frame.U()), proofarith.MustDyOf(rp.ax.dU)),
		dyScaleVec(proofarith.DyVec(rp.frame.V()), proofarith.MustDyOf(rp.ax.dV)))
	low := exactContactTransform(pose, exactContactTransform(rp.xform, proofarith.DvAdd(anchor, dyScaleVec(w, zlo))))
	high := exactContactTransform(pose, exactContactTransform(rp.xform, proofarith.DvAdd(anchor, dyScaleVec(w, zhi))))
	axis, _, ok := clearance.SignedAxis(pose.ApplyDir(rp.xform.ApplyDir(rp.basis().W)))
	if !ok {
		return sourceCylinderContactProof{}, false
	}
	var box sourceBoxContactProof
	for i := range 3 {
		box.lo[i], box.hi[i] = dyMin(low[i], high[i]), dyMax(low[i], high[i])
		if i != axis {
			box.lo[i] = proofarith.DySubScalar(box.lo[i], rhoHi)
			box.hi[i] = proofarith.DyAdd(box.hi[i], rhoHi)
		}
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
	axis, side, signedGap, ok := sourceCylinderBoxFace(cylinder, box)
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
	var boxPoint, cylinderPoint proofarith.DyV3
	for i := range 3 {
		if i == axis {
			if side == 1 {
				boxPoint[i], cylinderPoint[i] = box.hi[i], cylinder.box.lo[i]
			} else {
				boxPoint[i], cylinderPoint[i] = box.lo[i], cylinder.box.hi[i]
			}
			continue
		}
		center := proofarith.DyMul(proofarith.DyAdd(cylinder.box.lo[i], cylinder.box.hi[i]), proofarith.MustDyOf(.5))
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

// sourceCylinderBoxFace selects one complete box-face corridor. A circular
// sidewall supports a transverse axis at the same exact extremum as its outer
// box; the other transverse coordinate and the full axial interval stay
// strictly within the source-box face.
func sourceCylinderBoxFace(cylinder sourceCylinderContactProof,
	box sourceBoxContactProof) (int, int, proofarith.Dyadic, bool) {
	selected := -1
	var selectedSide int
	var selectedGap proofarith.Dyadic
	for axis := range 3 {
		if axis != cylinder.axis && (cylinder.wall == nil || cylinder.axis != 2) {
			continue
		}
		if !cylinderInsideBoxFace(cylinder.box, box, axis) {
			continue
		}
		var side int
		var gap proofarith.Dyadic
		switch {
		case proofarith.DyCmp(cylinder.box.lo[axis], box.hi[axis]) >= 0:
			side, gap = 1, proofarith.DySubScalar(cylinder.box.lo[axis], box.hi[axis])
		case proofarith.DyCmp(cylinder.box.hi[axis], box.lo[axis]) <= 0:
			side, gap = 0, proofarith.DySubScalar(box.lo[axis], cylinder.box.hi[axis])
		case proofarith.DyCmp(cylinder.box.lo[axis], box.lo[axis]) > 0 &&
			proofarith.DyCmp(cylinder.box.hi[axis], box.hi[axis]) > 0:
			side, gap = 1, proofarith.DySubScalar(cylinder.box.lo[axis], box.hi[axis])
		case proofarith.DyCmp(cylinder.box.hi[axis], box.hi[axis]) < 0 &&
			proofarith.DyCmp(cylinder.box.lo[axis], box.lo[axis]) < 0:
			side, gap = 0, proofarith.DySubScalar(box.lo[axis], cylinder.box.hi[axis])
		default:
			continue
		}
		if selected >= 0 {
			return 0, 0, proofarith.Dyadic{}, false
		}
		selected, selectedSide, selectedGap = axis, side, gap
	}
	return selected, selectedSide, selectedGap, selected >= 0
}

func cylinderInsideBoxFace(cylinder, box sourceBoxContactProof, normalAxis int) bool {
	for axis := range 3 {
		if axis == normalAxis {
			continue
		}
		if proofarith.DyCmp(cylinder.lo[axis], box.lo[axis]) <= 0 ||
			proofarith.DyCmp(cylinder.hi[axis], box.hi[axis]) >= 0 {
			return false
		}
	}
	return true
}
