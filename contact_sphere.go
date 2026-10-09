package decad

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/pair"
	"github.com/lestrrat-3d/decad/internal/pair/box"
	"github.com/lestrrat-3d/decad/internal/proofbound"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// sourceSphereContactProof records the exact ball made by a full revolution
// of one semicircle and its on-axis diameter. A tagged spherical face alone
// cannot establish the occupied set: the axis resolver admits small snaps.
type sourceSphereContactProof struct {
	center proofarith.DyV3
	radius proofarith.Dyadic
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
			if component.Sign() != 0 {
				return sourceSphereContactProof{}, false
			}
		}
		proof.center = proofarith.DyVec(pose.Translation())
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
		!proofbound.FiniteVec(rp.frame.Origin()) ||
		!cardinalBasis(rp.frame.U(), rp.frame.V(), rp.frame.N()) ||
		rp.ax.dU != 1 || rp.ax.dV != 0 ||
		rp.ax.aUBound != 0 || rp.ax.aVBound != 0 ||
		rp.ax.dUBound != 0 || rp.ax.dVBound != 0 {
		return sourceSphereContactProof{}, false
	}
	var arc arcSeg
	var line lineSeg
	arcSeen, lineSeen := false, false
	for _, segment := range rp.profile.Outer.Segments {
		switch s := segment.(type) {
		case arcSeg:
			arc, arcSeen = s, true
		case lineSeg:
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
	radius := proofarith.DySubScalar(proofarith.MustDyOf(arc.Start.U), proofarith.MustDyOf(arc.Center.U))
	if proofarith.DyCmp(radius, proofarith.DySubScalar(proofarith.MustDyOf(arc.Center.U), proofarith.MustDyOf(arc.End.U))) != 0 {
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
	local := proofarith.DvAdd(proofarith.DyVec(rp.frame.Origin()),
		proofarith.DvAdd(dyScaleVec(proofarith.DyVec(rp.frame.U()), proofarith.MustDyOf(arc.Center.U)),
			dyScaleVec(proofarith.DyVec(rp.frame.V()), proofarith.MustDyOf(arc.Center.V))))
	return sourceSphereContactProof{
		center: local,
		radius: radius, face: faces[0],
	}, true
}

// classifySourceSphereBox binds the neutral complete-ball result to the
// source faces and public contact report.
func classifySourceSphereBox(report *ContactReport, sphere sourceSphereContactProof,
	boxProof sourceBoxContactProof, sphereFirst bool) {
	result := box.ClassifyAxisSphere(
		box.AxisSphere{Center: sphere.center, Radius: sphere.radius},
		boxProof.axisBox(), report.Request.PointResolution.Base(),
	)
	switch result.Relation {
	case pair.Separated:
		report.Relation = ContactSeparated
	case pair.Touching:
		report.Relation = ContactTouching
	case pair.Overlapping:
		report.Relation = ContactOverlapping
	}
	report.Reason = sourceBoxReason(result.Reason)
	if result.Gap != nil {
		gap := sourceBoxScalar(*result.Gap)
		report.Gap = &gap
	}
	if result.Witness == nil {
		return
	}
	witness := result.Witness
	face := boxProof.faces[witness.Face.Axis][witness.Face.Side]
	if face == nil {
		report.Reason = ContactNoNormalProof
		return
	}
	sign := witness.NormalSign
	if sphereFirst {
		sign = -sign
	}
	normal := r3.Vec{}
	switch witness.NormalAxis {
	case 0:
		normal.X = float64(sign)
	case 1:
		normal.Y = float64(sign)
	case 2:
		normal.Z = float64(sign)
	}
	onSphere := sourceBoxPointMeasurement(witness.SpherePoint)
	onBox := sourceBoxPointMeasurement(witness.BoxPoint)
	featureSphere, featureBox := ContactFeature{Face: sphere.face}, ContactFeature{Face: face}
	onA, onB := onBox, onSphere
	featureA, featureB := featureBox, featureSphere
	if sphereFirst {
		onA, onB = onSphere, onBox
		featureA, featureB = featureSphere, featureBox
	}
	report.Manifold = &ContactManifold{Points: []ContactPoint{{
		OnA: onA, OnB: onB, Normal: VecMeasurement{
			Value: normal, Bound: units.Scalar(0), Exactness: Exact},
		NormalAngle: units.Radians(0), Separation: sourceBoxScalar(witness.Separation),
		FaceA: featureA.Face, FaceB: featureB.Face, FeatureA: featureA, FeatureB: featureB,
	}}}
}

func signIfBoxSide(side int) float64 {
	if side == 1 {
		return 1
	}
	return -1
}

func translatedSphere(s sourceSphereContactProof, delta [3]proofarith.Dyadic, f *big.Rat) (sourceSphereContactProof, bool) {
	fraction, ok := proofarith.DyOfRat(f)
	if !ok {
		return sourceSphereContactProof{}, false
	}
	for i := range 3 {
		s.center[i] = proofarith.DyAdd(s.center[i], proofarith.DyMul(delta[i], fraction))
	}
	return s, true
}
