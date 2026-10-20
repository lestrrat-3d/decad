package reportvocab

import (
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/pair"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// The contact report vocabulary below is the internal form of the root
// package's ContactRequest, ContactReason, ContactFeature, ContactPoint,
// ContactManifold and ContactReport (agent-docs/multibody-dynamics-design.md §9,
// §10). Contact producers fill it with topology nodes and internal
// measurements; the root package converts a finished report to its public
// type once, at the public boundary.

// ContactRequest states the maximum position and normal error a manifold may
// publish, and the support band and held chord of the approximate paths.
type ContactRequest struct {
	PointResolution  units.Value
	NormalResolution units.Value
	SupportBand      units.Value
	HeldChord        units.Value
}

// ContactReason explains an undecided relation or an absent manifold. Its
// constants match the root ContactReason's in value and order.
type ContactReason int

const (
	ContactNoReason ContactReason = iota
	ContactPayloadUnsupported
	ContactNoGapProof
	ContactNoNormalProof
	ContactAmbiguousFeature
	ContactPointTooCoarse
	ContactNonConvex
)

// ContactReasonOf names a pair kernel's missing certificate as a contact
// reason; pair.NoReason, and any reason it does not name, is ContactNoReason.
func ContactReasonOf(reason pair.Reason) ContactReason {
	switch reason {
	case pair.NoGapProof:
		return ContactNoGapProof
	case pair.AmbiguousFeature:
		return ContactAmbiguousFeature
	case pair.PointTooCoarse:
		return ContactPointTooCoarse
	case pair.PayloadUnsupported:
		return ContactPayloadUnsupported
	case pair.NoNormalProof:
		return ContactNoNormalProof
	default:
		return ContactNoReason
	}
}

// ContactFeature names an original topological feature by its node.
type ContactFeature struct {
	Face   *topo.Face
	Edge   *topo.Edge
	Vertex *topo.Vertex
}

// ContactPoint bounds two boundary witnesses and their A-to-B normal.
type ContactPoint struct {
	OnA, OnB           measurement.VecMeasurement
	Normal             measurement.VecMeasurement
	NormalAngle        units.Value
	Separation         measurement.Measurement
	FaceA, FaceB       *topo.Face
	FeatureA, FeatureB ContactFeature
}

// ContactManifold is a deterministic reduction of the complete certified
// contact patch.
type ContactManifold struct {
	Points []ContactPoint
}

// ContactReport is a pair result at the two query poses.
type ContactReport struct {
	A, B     *topo.Body
	PoseA    r3.Transform
	PoseB    r3.Transform
	Request  ContactRequest
	Relation ContactRelation
	Gap      *measurement.Measurement
	Overlap  *measurement.Measurement
	Manifold *ContactManifold
	Reason   ContactReason
}
