package decad

import (
	"github.com/lestrrat-3d/decad/internal/pair"
	"github.com/lestrrat-3d/decad/internal/pair/sphere"
	"github.com/lestrrat-3d/units"
)

// classifySourceSpherePair binds the neutral complete-ball result to the
// original source faces and the public contact report.
func classifySourceSpherePair(report *ContactReport, a, b sourceSphereContactProof) {
	result := sphere.Classify(
		sphere.Ball{Center: a.center, Radius: a.radius},
		sphere.Ball{Center: b.center, Radius: b.radius},
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
	normal := VecMeasurement{Value: witness.Normal.Value,
		Bound:     units.Scalar(witness.Normal.Bound),
		Exactness: exactnessOf(witness.Normal.Bound)}
	report.Manifold = &ContactManifold{Points: []ContactPoint{{
		OnA: sourceBoxPointMeasurement(witness.OnA), OnB: sourceBoxPointMeasurement(witness.OnB),
		Normal: normal, NormalAngle: units.Radians(witness.Normal.Angle),
		Separation: sourceBoxScalar(witness.Separation),
		FaceA:      a.face, FaceB: b.face,
		FeatureA: ContactFeature{Face: a.face}, FeatureB: ContactFeature{Face: b.face},
	}}}
}
