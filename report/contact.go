package report

import (
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// ContactRequest states the maximum position and normal error a manifold may publish.
// SupportBand and HeldChord are nonnegative lengths; zero disables their
// respective approximate contact paths.
type ContactRequest struct {
	PointResolution  units.Value
	NormalResolution units.Value
	SupportBand      units.Value
	HeldChord        units.Value
}

// ContactRelation is the proven relation of the two complete occupied sets.
type ContactRelation int

const (
	ContactUndecided ContactRelation = iota
	ContactSeparated
	ContactTouching
	ContactOverlapping
	// ContactBand is published for a body whose held boundary carries a
	// positive displacement (agent-docs/multibody-dynamics-design.md §10.4), or for
	// an exact planar pair apart by at most the request's SupportBand
	// (§10.5): the interiors are disjoint except possibly within a band of
	// width Gap.Bound around Gap.Value, which is zero. A displaced pair's
	// manifold Separation intervals carry the same band; an exact pair's
	// manifold is its support set, each point at its exact height.
	ContactBand
)

// ContactReason explains an undecided relation or an absent manifold.
type ContactReason int

const (
	ContactNoReason ContactReason = iota
	ContactPayloadUnsupported
	ContactNoGapProof
	ContactNoNormalProof
	ContactAmbiguousFeature
	ContactPointTooCoarse
	// ContactNonConvex withholds a manifold because neither body carries the
	// convexity certificate of agent-docs/multibody-dynamics-design.md §9.2: an
	// overlap, or a touch whose contacts no single face of one body holds
	// with the other body's corners inside it (§10.5).
	ContactNonConvex
)

// ContactFeatureOf names an original topological feature by its face, edge
// and vertex. F, E and V are the face, edge and vertex reference types.
type ContactFeatureOf[F, E, V any] struct {
	Face   F
	Edge   E
	Vertex V
}

// ContactPointOf bounds two boundary witnesses and their A-to-B normal.
type ContactPointOf[F, E, V any] struct {
	OnA, OnB           VecMeasurement
	Normal             VecMeasurement
	NormalAngle        units.Value
	Separation         Measurement
	FaceA, FaceB       F
	FeatureA, FeatureB ContactFeatureOf[F, E, V]
}

// ContactManifoldOf is a deterministic reduction of the complete certified
// contact patch.
type ContactManifoldOf[F, E, V any] struct {
	Points []ContactPointOf[F, E, V]
}

// ContactReportOf is a read-only pair result at the two caller-supplied
// poses. B, F, E and V are the body, face, edge and vertex reference types.
type ContactReportOf[B, F, E, V any] struct {
	A, B     B
	PoseA    r3.Transform
	PoseB    r3.Transform
	Request  ContactRequest
	Relation ContactRelation
	Gap      *Measurement
	Overlap  *Measurement
	Manifold *ContactManifoldOf[F, E, V]
	Reason   ContactReason
}
