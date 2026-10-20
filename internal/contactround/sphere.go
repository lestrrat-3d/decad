package contactround

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/contactbox"
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/pair"
	pairbox "github.com/lestrrat-3d/decad/internal/pair/box"
	"github.com/lestrrat-3d/decad/internal/pair/sphere"
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

// SphereProof records the exact ball made by a full revolution
// of one semicircle and its on-axis diameter. A tagged spherical face alone
// cannot establish the occupied set: the axis resolver admits small snaps.
type SphereProof struct {
	Center proofarith.DyV3
	Radius proofarith.Dyadic
	Face   *topo.Face
}

func SphereAtPose(b *topo.Body, pose r3.Transform) (SphereProof, bool) {
	proof, ok := SphereRecord(b)
	if !ok {
		return SphereProof{}, false
	}
	rp, ok := b.Payload.(revolveplan.Payload)
	if !ok || !pairbox.SignedAxisTransform(rp.Xform) || !pose.IsValid() || pose.IsReflection() {
		return SphereProof{}, false
	}
	center := proofarith.DvTransform(rp.Xform, proof.Center)
	if !pairbox.SignedAxisTransform(pose) {
		// A read rotation cannot move a ball centered at the query origin.
		// Other centers need an exact rotation of their offset before admission.
		for _, component := range center {
			if component.Sign() != 0 {
				return SphereProof{}, false
			}
		}
		proof.Center = proofarith.DyVec(pose.Translation())
		return proof, true
	}
	proof.Center = proofarith.DvTransform(pose, center)
	return proof, true
}

// SphereRecord proves the unplaced occupied solid is one complete ball.
// A rigid placement cannot change its radius or isotropic centroidal inertia.
func SphereRecord(b *topo.Body) (SphereProof, bool) {
	rp, ok := b.Payload.(revolveplan.Payload)
	if !ok || !b.Solid || b.Kind != topo.BodySolid || rp.SurfaceResult || !rp.Full ||
		rp.SectionDelta != 0 || len(rp.Profile.Holes) != 0 ||
		len(rp.Profile.Outer.Segments) != 2 ||
		!proofbound.FiniteVec(rp.Frame.Origin()) ||
		!pairbox.CardinalBasis(rp.Frame.U(), rp.Frame.V(), rp.Frame.N()) ||
		rp.Ax.DU != 1 || rp.Ax.DV != 0 ||
		rp.Ax.AUBound != 0 || rp.Ax.AVBound != 0 ||
		rp.Ax.DUBound != 0 || rp.Ax.DVBound != 0 {
		return SphereProof{}, false
	}
	var arc sectionrecord.ArcSeg
	var line sectionrecord.LineSeg
	arcSeen, lineSeen := false, false
	for _, segment := range rp.Profile.Outer.Segments {
		switch s := segment.(type) {
		case sectionrecord.ArcSeg:
			arc, arcSeen = s, true
		case sectionrecord.LineSeg:
			line, lineSeen = s, true
		default:
			return SphereProof{}, false
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
		return SphereProof{}, false
	}
	radius := proofarith.DySubScalar(proofarith.MustDyOf(arc.Start.U), proofarith.MustDyOf(arc.Center.U))
	if proofarith.DyCmp(radius, proofarith.DySubScalar(proofarith.MustDyOf(arc.Center.U), proofarith.MustDyOf(arc.End.U))) != 0 {
		return SphereProof{}, false
	}
	faces := topobuild.Faces(b)
	if len(faces) != 1 || faces[0].NormalBound != 0 || faces[0].Reversed ||
		len(faces[0].Loops) != 0 || len(topobuild.Edges(b)) != 0 {
		return SphereProof{}, false
	}
	if _, ok := faces[0].Surface.(surfacegeom.Sphere); !ok {
		return SphereProof{}, false
	}
	local := proofarith.DvAdd(proofarith.DyVec(rp.Frame.Origin()),
		proofarith.DvAdd(proofarith.DvScale(proofarith.DyVec(rp.Frame.U()), proofarith.MustDyOf(arc.Center.U)),
			proofarith.DvScale(proofarith.DyVec(rp.Frame.V()), proofarith.MustDyOf(arc.Center.V))))
	return SphereProof{
		Center: local,
		Radius: radius, Face: faces[0],
	}, true
}

// ClassifySphereBox binds the neutral complete-ball result to the
// source faces and the contact report.
func ClassifySphereBox(report *reportvocab.ContactReport, sphere SphereProof,
	boxProof contactbox.SourceBox, sphereFirst bool) {
	result := pairbox.ClassifyAxisSphere(
		pairbox.AxisSphere{Center: sphere.Center, Radius: sphere.Radius},
		boxProof.AxisBox(), report.Request.PointResolution.Base(),
	)
	switch result.Relation {
	case pair.Separated:
		report.Relation = reportvocab.ContactSeparated
	case pair.Touching:
		report.Relation = reportvocab.ContactTouching
	case pair.Overlapping:
		report.Relation = reportvocab.ContactOverlapping
	}
	report.Reason = reportvocab.ContactReasonOf(result.Reason)
	if result.Gap != nil {
		gap := result.Gap.Measurement()
		report.Gap = &gap
	}
	if result.Witness == nil {
		return
	}
	witness := result.Witness
	face := boxProof.Faces[witness.Face.Axis][witness.Face.Side]
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
	onSphere := witness.SpherePoint.Measurement()
	onBox := witness.BoxPoint.Measurement()
	featureSphere, featureBox := reportvocab.ContactFeature{Face: sphere.Face}, reportvocab.ContactFeature{Face: face}
	onA, onB := onBox, onSphere
	featureA, featureB := featureBox, featureSphere
	if sphereFirst {
		onA, onB = onSphere, onBox
		featureA, featureB = featureSphere, featureBox
	}
	report.Manifold = &reportvocab.ContactManifold{Points: []reportvocab.ContactPoint{{
		OnA: onA, OnB: onB, Normal: measurement.VecMeasurement{
			Value: normal, Bound: units.Scalar(0), Exactness: measurement.Exact},
		NormalAngle: units.Radians(0), Separation: witness.Separation.Measurement(),
		FaceA: featureA.Face, FaceB: featureB.Face, FeatureA: featureA, FeatureB: featureB,
	}}}
}

func signIfBoxSide(side int) float64 {
	if side == 1 {
		return 1
	}
	return -1
}

func TranslatedSphere(s SphereProof, delta [3]proofarith.Dyadic, f *big.Rat) (SphereProof, bool) {
	fraction, ok := proofarith.DyOfRat(f)
	if !ok {
		return SphereProof{}, false
	}
	for i := range 3 {
		s.Center[i] = proofarith.DyAdd(s.Center[i], proofarith.DyMul(delta[i], fraction))
	}
	return s, true
}

// ClassifySpherePair binds the neutral complete-ball result to the
// original source faces and the contact report.
func ClassifySpherePair(report *reportvocab.ContactReport, a, b SphereProof) {
	result := sphere.Classify(
		sphere.Ball{Center: a.Center, Radius: a.Radius},
		sphere.Ball{Center: b.Center, Radius: b.Radius},
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
	report.Reason = reportvocab.ContactReasonOf(result.Reason)
	if result.Gap != nil {
		gap := result.Gap.Measurement()
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
		OnA: witness.OnA.Measurement(), OnB: witness.OnB.Measurement(),
		Normal: normal, NormalAngle: units.Radians(witness.Normal.Angle),
		Separation: witness.Separation.Measurement(),
		FaceA:      a.Face, FaceB: b.Face,
		FeatureA: reportvocab.ContactFeature{Face: a.Face}, FeatureB: reportvocab.ContactFeature{Face: b.Face},
	}}}
}

// ClassifySphereOrientedBox maps the box kernel's bounded response to
// the original source faces and the contact report's measurements.
func ClassifySphereOrientedBox(report *reportvocab.ContactReport, sphere SphereProof,
	box contactbox.OrientedSourceBox, sphereFirst bool) {
	result := pairbox.ClassifyOrientedSphere(
		pairbox.AxisSphere{Center: sphere.Center, Radius: sphere.Radius}, box.OrientedBox, sphereFirst,
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
	report.Reason = reportvocab.ContactReasonOf(result.Reason)
	if result.Gap != nil {
		gap := result.Gap.Measurement()
		report.Gap = &gap
	}
	if result.Witness == nil {
		return
	}
	witness := result.Witness
	face := box.Faces[witness.Face.Axis][witness.Face.Side]
	if face == nil {
		report.Reason = reportvocab.ContactNoNormalProof
		return
	}
	normal := measurement.VecMeasurement{Value: witness.Normal.Value,
		Bound: units.Scalar(witness.Normal.Bound), Exactness: measurement.ExactnessOf(witness.Normal.Bound)}
	point := reportvocab.ContactPoint{Normal: normal, NormalAngle: units.Radians(witness.Normal.Angle),
		Separation: witness.Separation.Measurement()}
	boxPoint := witness.BoxPoint.Measurement()
	spherePoint := witness.SpherePoint.Measurement()
	if sphereFirst {
		point.OnA, point.OnB = spherePoint, boxPoint
		point.FeatureA, point.FeatureB = reportvocab.ContactFeature{Face: sphere.Face}, reportvocab.ContactFeature{Face: face}
	} else {
		point.OnA, point.OnB = boxPoint, spherePoint
		point.FeatureA, point.FeatureB = reportvocab.ContactFeature{Face: face}, reportvocab.ContactFeature{Face: sphere.Face}
	}
	point.FaceA, point.FaceB = point.FeatureA.Face, point.FeatureB.Face
	report.Manifold = &reportvocab.ContactManifold{Points: []reportvocab.ContactPoint{point}}
}
