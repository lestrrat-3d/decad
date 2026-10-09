package reportvocab

import (
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// ContactRequest states the maximum position and normal error a manifold may
// publish. SupportBand is a nonnegative Length; when positive, an exact planar
// pair also publishes every vertex of the resting body that lies within it
// above a support plane, and an exact planar pair apart by at most it is
// ContactBand (docs/multibody-dynamics-design.md §10.5). The zero Value is a
// zero band, which publishes the exact contact set alone.
//
// HeldChord is a nonnegative Length: the chord tolerance a solid with a curved
// face and no exact contact family of its own (a general revolve, a curved
// cap-loop chamfer, a cup or sweep over a curved section) is tessellated at
// for its held mesh, whose Bound is then the displacement δ the pair charges
// (§10.4). The zero Value admits no such body: it is left undecided rather
// than chorded at a width the caller never stated.
type ContactRequest struct {
	PointResolution  units.Value
	NormalResolution units.Value
	SupportBand      units.Value
	HeldChord        units.Value
}

// ContactFeature names an original topological feature. The first contact
// stage publishes face features; later stages may use edge and vertex fields.
type ContactFeature[FaceT, EdgeT, VertexT comparable] struct {
	Face   FaceT
	Edge   EdgeT
	Vertex VertexT
}

// ContactPoint bounds two boundary witnesses and their A-to-B normal.
// Separation is the signed B-minus-A distance along that normal.
type ContactPoint[FaceT, EdgeT, VertexT comparable] struct {
	OnA, OnB           measurement.VecMeasurement
	Normal             measurement.VecMeasurement
	NormalAngle        units.Value
	Separation         measurement.Measurement
	FaceA, FaceB       FaceT
	FeatureA, FeatureB ContactFeature[FaceT, EdgeT, VertexT]
}

// ContactManifold is a deterministic reduction of the complete certified
// contact patch. Its points are immutable once returned.
type ContactManifold[FaceT, EdgeT, VertexT comparable] struct {
	Points []ContactPoint[FaceT, EdgeT, VertexT]
}

// ContactReport is a read-only pair result at the two caller-supplied poses.
type ContactReport[BodyT, FaceT, EdgeT, VertexT comparable, RelationT, ReasonT any] struct {
	A, B     BodyT
	PoseA    r3.Transform
	PoseB    r3.Transform
	Request  ContactRequest
	Relation RelationT
	Gap      *measurement.Measurement
	Overlap  *measurement.Measurement
	Manifold *ContactManifold[FaceT, EdgeT, VertexT]
	Reason   ReasonT
}
