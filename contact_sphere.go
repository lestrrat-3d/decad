package decad

import (
	"math/big"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// sourceSphereContactProof records the exact ball made by a full revolution
// of one semicircle and its on-axis diameter. A tagged spherical face alone
// cannot establish the occupied set: the axis resolver admits small snaps.
type sourceSphereContactProof struct {
	center dyV3
	radius dyadic
	face   *Face
}

func sourceSphereAtPose(b *Body, pose r3.Transform) (sourceSphereContactProof, bool) {
	proof, ok := sourceSphereRecord(b)
	if !ok {
		return sourceSphereContactProof{}, false
	}
	rp, ok := b.payload.(revolvePayload)
	if !ok || !signedAxisTransform(rp.xform) || !pose.IsValid() || pose.IsReflection() {
		return sourceSphereContactProof{}, false
	}
	center := exactContactTransform(rp.xform, proof.center)
	if !signedAxisTransform(pose) {
		// A read rotation cannot move a ball centered at the query origin.
		// Other centers need an exact rotation of their offset before admission.
		for _, component := range center {
			if component.sign() != 0 {
				return sourceSphereContactProof{}, false
			}
		}
		proof.center = dyVec(pose.Translation())
		return proof, true
	}
	proof.center = exactContactTransform(pose, center)
	return proof, true
}

// sourceSphereRecord proves the unplaced occupied solid is one complete ball.
// A rigid placement cannot change its radius or isotropic centroidal inertia.
func sourceSphereRecord(b *Body) (sourceSphereContactProof, bool) {
	rp, ok := b.payload.(revolvePayload)
	if !ok || !b.solid || b.kind != BodySolid || rp.surfaceResult || !rp.full ||
		rp.sectionDelta != 0 || len(rp.profile.Holes) != 0 ||
		len(rp.profile.Outer.Segments) != 2 ||
		!finiteVec(rp.frame.Origin()) ||
		!cardinalBasis(rp.frame.U(), rp.frame.V(), rp.frame.N()) ||
		rp.ax.dU != 1 || rp.ax.dV != 0 ||
		rp.ax.aUBound != 0 || rp.ax.aVBound != 0 ||
		rp.ax.dUBound != 0 || rp.ax.dVBound != 0 {
		return sourceSphereContactProof{}, false
	}
	var arc ArcSeg
	var line LineSeg
	arcSeen, lineSeen := false, false
	for _, segment := range rp.profile.Outer.Segments {
		switch s := segment.(type) {
		case ArcSeg:
			arc, arcSeen = s, true
		case LineSeg:
			line, lineSeen = s, true
		default:
			return sourceSphereContactProof{}, false
		}
	}
	sameEnds := line.Start == arc.Start && line.End == arc.End ||
		line.Start == arc.End && line.End == arc.Start
	if !arcSeen || !lineSeen || arc.TStart != 0 || arc.TEnd != 1 ||
		line.TStart != 0 || line.TEnd != 1 || arc.Center.V != rp.ax.aV ||
		arc.Start.V != arc.Center.V || arc.End.V != arc.Center.V ||
		arc.Start.U <= arc.Center.U || arc.End.U >= arc.Center.U ||
		!sameEnds ||
		!finiteMeasurementValues(arc.Center.U, arc.Center.V, arc.Start.U, arc.End.U) {
		return sourceSphereContactProof{}, false
	}
	radius := dySubScalar(mustDyOf(arc.Start.U), mustDyOf(arc.Center.U))
	if dyCmp(radius, dySubScalar(mustDyOf(arc.Center.U), mustDyOf(arc.End.U))) != 0 {
		return sourceSphereContactProof{}, false
	}
	faces := b.Faces()
	if len(faces) != 1 || faces[0].normalBound != 0 || faces[0].reversed ||
		len(faces[0].loops) != 0 || len(b.Edges()) != 0 {
		return sourceSphereContactProof{}, false
	}
	if _, ok := faces[0].surface.(Sphere); !ok {
		return sourceSphereContactProof{}, false
	}
	local := dvAdd(dyVec(rp.frame.Origin()),
		dvAdd(dyScaleVec(dyVec(rp.frame.U()), mustDyOf(arc.Center.U)),
			dyScaleVec(dyVec(rp.frame.V()), mustDyOf(arc.Center.V))))
	return sourceSphereContactProof{
		center: local,
		radius: radius, face: faces[0],
	}, true
}

// classifySourceSphereBox uses squared rational distance to the complete box.
// Its face witness is published only when one box support is active and the
// sphere's projected disk stays inside the other two face intervals.
func classifySourceSphereBox(report *ContactReport, sphere sourceSphereContactProof,
	box sourceBoxContactProof, sphereFirst bool) {
	var nearest dyV3
	distance2 := dyZero()
	outsideAxis, outsideSide, outsideCount := 0, 0, 0
	for i := range 3 {
		nearest[i] = dyMax(box.lo[i], dyMin(sphere.center[i], box.hi[i]))
		d := dySubScalar(sphere.center[i], nearest[i])
		distance2 = dyAdd(distance2, dyMul(d, d))
		if d.sign() != 0 {
			outsideAxis, outsideCount = i, outsideCount+1
			if d.sign() > 0 {
				outsideSide = 1
			}
		}
	}
	r2 := dyMul(sphere.radius, sphere.radius)
	switch dyCmp(distance2, r2) {
	case 1:
		lo := ratFloatDown(new(big.Rat).Sub(floatRat(dySqrtDown(distance2)), sphere.radius.rat()))
		hi := ratFloatUp(new(big.Rat).Sub(floatRat(dySqrtUp(distance2)), sphere.radius.rat()))
		if !finiteMeasurementValues(lo, hi) || lo <= 0 || hi < lo {
			report.Reason = ContactNoGapProof
			return
		}
		value := lo + (hi-lo)/2
		left := new(big.Rat).Sub(floatRat(value), floatRat(lo))
		right := new(big.Rat).Sub(floatRat(hi), floatRat(value))
		if right.Cmp(left) > 0 {
			left = right
		}
		bound := ratFloatUp(left)
		if !finiteMeasurementValues(value, bound) ||
			new(big.Rat).Sub(floatRat(value), floatRat(bound)).Sign() <= 0 {
			report.Reason = ContactNoGapProof
			return
		}
		gap := Measurement{Value: units.Millimeters(value), Bound: units.Millimeters(bound),
			Exactness: exactnessOf(bound)}
		report.Relation, report.Gap = ContactSeparated, &gap
		return
	case 0:
		report.Relation = ContactTouching
		gap := Measurement{Value: units.Millimeters(0), Bound: units.Millimeters(0), Exactness: Exact}
		report.Gap = &gap
	default:
		report.Relation = ContactOverlapping
	}
	if outsideCount != 1 {
		report.Reason = ContactAmbiguousFeature
		return
	}
	axis := outsideAxis
	for i := range 3 {
		if i == axis {
			continue
		}
		if dyCmp(dySubScalar(sphere.center[i], sphere.radius), box.lo[i]) <= 0 ||
			dyCmp(dyAdd(sphere.center[i], sphere.radius), box.hi[i]) >= 0 {
			report.Reason = ContactAmbiguousFeature
			return
		}
	}
	if report.Relation == ContactOverlapping {
		opposite := box.lo[axis]
		if outsideSide == 0 {
			opposite = box.hi[axis]
		}
		if dyCmp(dyAbs(dySubScalar(sphere.center[axis], opposite)), sphere.radius) <= 0 {
			report.Reason = ContactAmbiguousFeature
			return
		}
	}
	face := box.faces[axis][outsideSide]
	if face == nil {
		report.Reason = ContactNoNormalProof
		return
	}
	// The box-to-sphere direction is the outward support normal of the box.
	normal := r3.Vec{}
	sign := -1.0
	if outsideSide == 1 {
		sign = 1
	}
	if sphereFirst {
		sign = -sign
	}
	switch axis {
	case 0:
		normal.X = sign
	case 1:
		normal.Y = sign
	case 2:
		normal.Z = sign
	}
	witnessSphere := sphere.center
	witnessSphere[axis] = dySubScalar(sphere.center[axis],
		dyMul(mustDyOf(signIfBoxSide(outsideSide)), sphere.radius))
	witnessBox := nearest
	var pA, pB dyV3
	var featureA, featureB ContactFeature
	if sphereFirst {
		pA, pB = witnessSphere, witnessBox
		featureA, featureB = ContactFeature{Face: sphere.face}, ContactFeature{Face: face}
	} else {
		pA, pB = witnessBox, witnessSphere
		featureA, featureB = ContactFeature{Face: face}, ContactFeature{Face: sphere.face}
	}
	onA, okA := sourceBoxPoint(pA)
	onB, okB := sourceBoxPoint(pB)
	if !okA || !okB || onA.Bound.Base() > report.Request.PointResolution.Base() ||
		onB.Bound.Base() > report.Request.PointResolution.Base() {
		report.Reason = ContactPointTooCoarse
		return
	}
	distance := dyAbs(dySubScalar(sphere.center[axis], nearest[axis]))
	sep, ok := sourceBoxSignedReading(dySubScalar(distance, sphere.radius))
	if !ok {
		report.Reason = ContactPointTooCoarse
		return
	}
	report.Manifold = &ContactManifold{Points: []ContactPoint{{
		OnA: onA, OnB: onB, Normal: VecMeasurement{
			Value: normal, Bound: units.Scalar(0), Exactness: Exact},
		NormalAngle: units.Radians(0), Separation: sep,
		FaceA: featureA.Face, FaceB: featureB.Face, FeatureA: featureA, FeatureB: featureB,
	}}}
}

func signIfBoxSide(side int) float64 {
	if side == 1 {
		return 1
	}
	return -1
}

func translatedSphere(s sourceSphereContactProof, delta [3]dyadic, f *big.Rat) (sourceSphereContactProof, bool) {
	fraction, ok := dyOfRat(f)
	if !ok {
		return sourceSphereContactProof{}, false
	}
	for i := range 3 {
		s.center[i] = dyAdd(s.center[i], dyMul(delta[i], fraction))
	}
	return s, true
}
