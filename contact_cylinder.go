package decad

import (
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// sourceCylinderContactProof encloses the complete circular prism from its
// recorded full-circle section and exact axial limits. Its box is an outer
// bound only; it never establishes cylinder touch or overlap.
type sourceCylinderContactProof struct {
	box  sourceBoxContactProof
	axis int
}

func sourceCylinderAtPose(b *Body, pose r3.Transform) (sourceCylinderContactProof, bool) {
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
	center := dvAdd(dyVec(pp.frame.Origin()), dvAdd(
		dyScaleVec(dyVec(pp.frame.U()), mustDyOf(circle.Center.U)),
		dyScaleVec(dyVec(pp.frame.V()), mustDyOf(circle.Center.V))))
	low := dvAdd(center, dyScaleVec(dyVec(pp.frame.N()), mustDyOf(pp.z0)))
	high := dvAdd(center, dyScaleVec(dyVec(pp.frame.N()), mustDyOf(pp.z1)))
	low = exactContactTransform(pose, exactContactTransform(pp.xform, low))
	high = exactContactTransform(pose, exactContactTransform(pp.xform, high))
	axis, _, ok := signedAxis(pose.ApplyDir(pp.xform.ApplyDir(pp.frame.N())))
	if !ok {
		return sourceCylinderContactProof{}, false
	}
	radius := mustDyOf(circle.Radius.Base())
	var box sourceBoxContactProof
	for i := range 3 {
		box.lo[i], box.hi[i] = dyMin(low[i], high[i]), dyMax(low[i], high[i])
		if i != axis {
			box.lo[i] = dySubScalar(box.lo[i], radius)
			box.hi[i] = dyAdd(box.hi[i], radius)
		}
	}
	return sourceCylinderContactProof{box: box, axis: axis}, true
}

// A disjoint outer box proves cylinder separation. Its other relations make
// no statement about the curved occupied set.
func classifySourceCylinderBox(report *ContactReport, cylinder sourceCylinderContactProof,
	box sourceBoxContactProof) {
	var gaps [3]dyadic
	if !cylinderInsideBoxFace(cylinder.box, box, cylinder.axis) {
		report.Reason = ContactNoGapProof
		return
	}
	axis := cylinder.axis
	switch {
	case dyCmp(cylinder.box.hi[axis], box.lo[axis]) < 0:
		gaps[axis] = dySubScalar(box.lo[axis], cylinder.box.hi[axis])
	case dyCmp(box.hi[axis], cylinder.box.lo[axis]) < 0:
		gaps[axis] = dySubScalar(cylinder.box.lo[axis], box.hi[axis])
	default:
		report.Reason = ContactNoGapProof
		return
	}
	gap, ok := sourceBoxGap(gaps)
	if !ok {
		report.Reason = ContactNoGapProof
		return
	}
	report.Relation, report.Gap = ContactSeparated, &gap
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
