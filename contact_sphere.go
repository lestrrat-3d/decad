package decad

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/pair"
	"github.com/lestrrat-3d/decad/internal/pair/box"
	"github.com/lestrrat-3d/decad/internal/pair/sphere"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/reportvocab"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
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
	rp, ok := b.n.Payload.(revolvePayload)
	if !ok || !box.SignedAxisTransform(rp.Xform) || !pose.IsValid() || pose.IsReflection() {
		return sourceSphereContactProof{}, false
	}
	center := proofarith.DvTransform(rp.Xform, proof.center)
	if !box.SignedAxisTransform(pose) {
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
	proof.center = proofarith.DvTransform(pose, center)
	return proof, true
}

// sourceSphereRecord proves the unplaced occupied solid is one complete ball.
// A rigid placement cannot change its radius or isotropic centroidal inertia.
func sourceSphereRecord(b *Body) (sourceSphereContactProof, bool) {
	rp, ok := b.n.Payload.(revolvePayload)
	if !ok || !b.n.Solid || b.n.Kind != topo.BodySolid || rp.SurfaceResult || !rp.Full ||
		rp.SectionDelta != 0 || len(rp.Profile.Holes) != 0 ||
		len(rp.Profile.Outer.Segments) != 2 ||
		!proofbound.FiniteVec(rp.Frame.Origin()) ||
		!box.CardinalBasis(rp.Frame.U(), rp.Frame.V(), rp.Frame.N()) ||
		rp.Ax.DU != 1 || rp.Ax.DV != 0 ||
		rp.Ax.AUBound != 0 || rp.Ax.AVBound != 0 ||
		rp.Ax.DUBound != 0 || rp.Ax.DVBound != 0 {
		return sourceSphereContactProof{}, false
	}
	var arc arcSeg
	var line lineSeg
	arcSeen, lineSeen := false, false
	for _, segment := range rp.Profile.Outer.Segments {
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
		line.TStart != 0 || line.TEnd != 1 || arc.Center.V != rp.Ax.AV ||
		arc.Start.V != arc.Center.V || arc.End.V != arc.Center.V ||
		arc.Start.U <= arc.Center.U || arc.End.U >= arc.Center.U ||
		!sameEnds ||
		!measurement.FiniteValues(arc.Center.U, arc.Center.V, arc.Start.U, arc.End.U) {
		return sourceSphereContactProof{}, false
	}
	radius := proofarith.DySubScalar(proofarith.MustDyOf(arc.Start.U), proofarith.MustDyOf(arc.Center.U))
	if proofarith.DyCmp(radius, proofarith.DySubScalar(proofarith.MustDyOf(arc.Center.U), proofarith.MustDyOf(arc.End.U))) != 0 {
		return sourceSphereContactProof{}, false
	}
	faces := b.Faces()
	if len(faces) != 1 || faces[0].n.NormalBound != 0 || faces[0].n.Reversed ||
		len(faces[0].n.Loops) != 0 || len(b.Edges()) != 0 {
		return sourceSphereContactProof{}, false
	}
	if _, ok := faces[0].n.Surface.(surfacegeom.Sphere); !ok {
		return sourceSphereContactProof{}, false
	}
	local := proofarith.DvAdd(proofarith.DyVec(rp.Frame.Origin()),
		proofarith.DvAdd(proofarith.DvScale(proofarith.DyVec(rp.Frame.U()), proofarith.MustDyOf(arc.Center.U)),
			proofarith.DvScale(proofarith.DyVec(rp.Frame.V()), proofarith.MustDyOf(arc.Center.V))))
	return sourceSphereContactProof{
		center: local,
		radius: radius, face: faces[0],
	}, true
}

// classifySourceSphereBox binds the neutral complete-ball result to the
// source faces and the contact report.
func classifySourceSphereBox(report *reportvocab.ContactReport, sphere sourceSphereContactProof,
	boxProof sourceBoxContactProof, sphereFirst bool) {
	result := box.ClassifyAxisSphere(
		box.AxisSphere{Center: sphere.center, Radius: sphere.radius},
		boxProof.axisBox(), report.Request.PointResolution.Base(),
	)
	switch result.Relation {
	case pair.Separated:
		report.Relation = reportvocab.ContactSeparated
	case pair.Touching:
		report.Relation = reportvocab.ContactTouching
	case pair.Overlapping:
		report.Relation = reportvocab.ContactOverlapping
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
		report.Reason = reportvocab.ContactNoNormalProof
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
	featureSphere, featureBox := reportvocab.ContactFeature{Face: faceNode(sphere.face)}, reportvocab.ContactFeature{Face: faceNode(face)}
	onA, onB := onBox, onSphere
	featureA, featureB := featureBox, featureSphere
	if sphereFirst {
		onA, onB = onSphere, onBox
		featureA, featureB = featureSphere, featureBox
	}
	report.Manifold = &reportvocab.ContactManifold{Points: []reportvocab.ContactPoint{{
		OnA: onA, OnB: onB, Normal: measurement.VecMeasurement{
			Value: normal, Bound: units.Scalar(0), Exactness: measurement.Exact},
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

// classifySourceSpherePair binds the neutral complete-ball result to the
// original source faces and the contact report.
func classifySourceSpherePair(report *reportvocab.ContactReport, a, b sourceSphereContactProof) {
	result := sphere.Classify(
		sphere.Ball{Center: a.center, Radius: a.radius},
		sphere.Ball{Center: b.center, Radius: b.radius},
		report.Request.PointResolution.Base(), report.Request.NormalResolution.Base(),
	)
	switch result.Relation {
	case pair.Separated:
		report.Relation = reportvocab.ContactSeparated
	case pair.Touching:
		report.Relation = reportvocab.ContactTouching
	case pair.Overlapping:
		report.Relation = reportvocab.ContactOverlapping
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
	normal := measurement.VecMeasurement{Value: witness.Normal.Value,
		Bound:     units.Scalar(witness.Normal.Bound),
		Exactness: measurement.ExactnessOf(witness.Normal.Bound)}
	report.Manifold = &reportvocab.ContactManifold{Points: []reportvocab.ContactPoint{{
		OnA: sourceBoxPointMeasurement(witness.OnA), OnB: sourceBoxPointMeasurement(witness.OnB),
		Normal: normal, NormalAngle: units.Radians(witness.Normal.Angle),
		Separation: sourceBoxScalar(witness.Separation),
		FaceA:      faceNode(a.face), FaceB: faceNode(b.face),
		FeatureA: reportvocab.ContactFeature{Face: faceNode(a.face)}, FeatureB: reportvocab.ContactFeature{Face: faceNode(b.face)},
	}}}
}
