package decad

import (
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/pair"
	"github.com/lestrrat-3d/decad/internal/pair/sphere"
	"github.com/lestrrat-3d/decad/internal/reportvocab"
	"github.com/lestrrat-3d/units"
)

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
