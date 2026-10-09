package decad

import (
	"github.com/lestrrat-3d/decad/internal/pair"
	pairbox "github.com/lestrrat-3d/decad/internal/pair/box"
	"github.com/lestrrat-3d/units"
)

// classifySourceSphereOrientedBox maps the box kernel's bounded response to
// the original source faces and public measurements.
func classifySourceSphereOrientedBox(report *ContactReport, sphere sourceSphereContactProof,
	box orientedSourceBox, sphereFirst bool) {
	result := pairbox.ClassifyOrientedSphere(
		pairbox.AxisSphere{Center: sphere.center, Radius: sphere.radius}, box.pairBox(), sphereFirst,
		report.Request.PointResolution.Base(), report.Request.NormalResolution.Base(),
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
	face := box.faces[witness.Face.Axis][witness.Face.Side]
	if face == nil {
		report.Reason = ContactNoNormalProof
		return
	}
	normal := VecMeasurement{Value: witness.Normal.Value,
		Bound: units.Scalar(witness.Normal.Bound), Exactness: exactnessOf(witness.Normal.Bound)}
	point := ContactPoint{Normal: normal, NormalAngle: units.Radians(witness.Normal.Angle),
		Separation: sourceBoxScalar(witness.Separation)}
	boxPoint := sourceBoxPointMeasurement(witness.BoxPoint)
	spherePoint := sourceBoxPointMeasurement(witness.SpherePoint)
	if sphereFirst {
		point.OnA, point.OnB = spherePoint, boxPoint
		point.FeatureA, point.FeatureB = ContactFeature{Face: sphere.face}, ContactFeature{Face: face}
	} else {
		point.OnA, point.OnB = boxPoint, spherePoint
		point.FeatureA, point.FeatureB = ContactFeature{Face: face}, ContactFeature{Face: sphere.face}
	}
	point.FaceA, point.FaceB = point.FeatureA.Face, point.FeatureB.Face
	report.Manifold = &ContactManifold{Points: []ContactPoint{point}}
}
