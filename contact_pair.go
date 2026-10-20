package decad

import (
	"context"
	"fmt"
	"math"
	"slices"

	"github.com/lestrrat-3d/decad/internal/contactpair"
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/reportvocab"
	"github.com/lestrrat-3d/decad/internal/topo"
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

// validateSupportBand admits the zero Value or a finite nonnegative Length.
func validateSupportBand(v units.Value) error {
	return validateNonnegativeLength(v, "support band")
}

// validateHeldChord admits the zero Value or a finite nonnegative Length.
func validateHeldChord(v units.Value) error {
	return validateNonnegativeLength(v, "held chord")
}

// validateNonnegativeLength admits the zero Value or a finite nonnegative
// Length, naming the field in its error.
func validateNonnegativeLength(v units.Value, name string) error {
	if v == (units.Value{}) {
		return nil
	}
	if v.Kind() != units.Length {
		return fmt.Errorf("%w: %s must be a Length", ErrUnitKind, name)
	}
	if !measurement.FiniteValues(v.Base()) {
		return fmt.Errorf("%w: %s is non-finite", ErrNotFinite, name)
	}
	if v.Base() < 0 {
		return fmt.Errorf("%w: %s must be nonnegative", ErrDegenerate, name)
	}
	return nil
}

// ContactRelation is the proven relation of the two complete occupied sets.
type ContactRelation int

const (
	ContactUndecided ContactRelation = iota
	ContactSeparated
	ContactTouching
	ContactOverlapping
	// ContactBand is published for a body whose held boundary carries a
	// positive displacement (docs/multibody-dynamics-design.md §10.4), or for
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
	// convexity certificate of docs/multibody-dynamics-design.md §9.2: an
	// overlap, or a touch whose contacts no single face of one body holds
	// with the other body's corners inside it (§10.5).
	ContactNonConvex
)

// ContactFeature names an original topological feature.
type ContactFeature struct {
	Face   *Face
	Edge   *Edge
	Vertex *Vertex
}

// ContactPoint bounds two boundary witnesses and their A-to-B normal.
type ContactPoint struct {
	OnA, OnB           VecMeasurement
	Normal             VecMeasurement
	NormalAngle        units.Value
	Separation         Measurement
	FaceA, FaceB       *Face
	FeatureA, FeatureB ContactFeature
}

// ContactManifold is a deterministic reduction of the complete certified contact patch.
type ContactManifold struct {
	Points []ContactPoint
}

// ContactReport is a read-only pair result at the two caller-supplied poses.
type ContactReport struct {
	A, B     *Body
	PoseA    r3.Transform
	PoseB    r3.Transform
	Request  ContactRequest
	Relation ContactRelation
	Gap      *Measurement
	Overlap  *Measurement
	Manifold *ContactManifold
	Reason   ContactReason
}

// ContactPair proves the relation of two live solids at poses applied after
// their recorded placements. Bodies and the document are not changed.
// Source rectangular prisms have face manifolds at signed-axis poses and
// co-oriented oblique face touches. Opposed axis-normal oriented faces can
// publish a common interior witness. Other valid poses have relation proofs.
// Source semicircle spheres against boxes or each other also have relation
// and point-manifold proofs at signed-axis poses, or at proper rotating poses
// when the sphere center is the query origin. Sphere-pair center lines
// may be off-axis when their normal and witnesses meet the requested bounds.
// A full source cylinder, extruded or revolved, can prove an axial gap and
// disk-face contact against a containing box face. A vertical full circular
// source prism can also prove a horizontal sidewall gap and line contact.
// A faceted solid with an exact lower support certificate can prove lower
// support face contact or an axial gap
// against a source-box floor that strictly contains its support footprint.
// An exact source box's lower face survives a positive-bound Union when the
// other operand's certified lower bound is strictly above it. Other
// positive-bound faceted solids can prove a strict axial gap with their
// boundary displacement charged, but publish no contact manifold.
// An exactly orthogonal rotated source box can give a sphere a bounded point
// on one interior face.
// Two exact planar solids — prisms over whole LineSeg sections and
// zero-bound faceted Booleans — receive an exact relation proof at any pose
// with a positive determinant. When one is convex, a touch publishes its
// clipped face patches, edges and vertices inside a face, and edge
// crossings, and two convex bodies that overlap slightly publish the patch
// at depth. When neither body is convex, a touch whose every contact lies
// on one flat face of one body, the other body wholly on or in front of its
// plane, publishes the other body's corners strictly inside that face with
// its normal; any other touching or overlapping pair names
// ContactNonConvex. A positive-bound faceted Boolean, a displaced
// stitched solid, and every other solid without an exact contact family of
// its own (a cap-loop chamfer, a cup, a loft, a sweep, a general revolve) are
// admitted through their held meshes with the mesh's boundary displacement δ
// charged: an all-planar body is read at a chord that chords nothing, and a
// body with a curved face at the request's HeldChord, which must be positive.
// A held touch, or a held gap or depth within the summed δ, is ContactBand
// with Gap [−2δ, 2δ]. Under a positive
// SupportBand an exact planar touch or shallow face overlap also publishes the
// support set: every vertex of the resting body within the band above a face
// plane of the other, its foot strictly inside that face, at its exact
// height; and an exact planar pair apart by at most the band is ContactBand
// with Gap [0 ± g], g its gap's upper end, publishing that set
// (docs/multibody-dynamics-design.md §10.5).
// At identity query poses, the analytic clearance kernel can prove relations
// for other solids. Only its ruling touches publish a manifold: the two ends
// of a full source cylinder's ruling on a planar face or another cylinder.
// At other poses a full source cylinder lying along a signed-axis face of an
// exact planar body proves a touch or gap at a signed-axis pose, and at any
// other pose a gap, or a ContactBand whose width charges the pose basis's
// rounding, each with the two lowest rim points as its manifold.
// Both bodies must be non-nil, distinct, live members of d.
func (d *Document) ContactPair(ctx context.Context, a, b *Body, poseA, poseB r3.Transform,
	req ContactRequest) (*ContactReport, error) {
	if a == b && a != nil {
		return nil, fmt.Errorf("%w: contact requires distinct bodies", ErrDegenerate)
	}
	if err := d.requireLive(a); err != nil {
		return nil, err
	}
	if err := d.requireLive(b); err != nil {
		return nil, err
	}
	if !a.n.Solid || a.n.Kind != topo.BodySolid || !b.n.Solid || b.n.Kind != topo.BodySolid {
		return nil, ErrNotSolid
	}
	if !poseA.IsValid() || !poseB.IsValid() {
		return nil, fmt.Errorf("%w: contact pose is invalid", ErrDegenerate)
	}
	if req.PointResolution.Kind() != units.Length || req.NormalResolution.Kind() != units.Angle {
		return nil, fmt.Errorf("%w: contact resolutions must be Length and Angle", ErrUnitKind)
	}
	if !measurement.FiniteValues(req.PointResolution.Base(), req.NormalResolution.Base()) {
		return nil, fmt.Errorf("%w: contact resolution is non-finite", ErrNotFinite)
	}
	if req.PointResolution.Base() <= 0 || req.NormalResolution.Base() <= 0 {
		return nil, fmt.Errorf("%w: contact resolutions must be positive", ErrDegenerate)
	}
	if err := validateSupportBand(req.SupportBand); err != nil {
		return nil, err
	}
	if err := validateHeldChord(req.HeldChord); err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	key := newPairReportKey(b, poseA, poseB, req)
	if report, ok := a.pairReports.Load(key); ok {
		return cloneContactReport(report), nil
	}
	found, err := contactpair.Classify(ctx, verifyAllMesher{}, bodyNode(a), bodyNode(b), poseA, poseB, contactRequestToInternal(req))
	if err != nil {
		return nil, err
	}
	report := contactReportFromInternal(found)
	a.pairReports.Store(key, report, pairReportMemoCap)
	return cloneContactReport(report), nil
}

// Each public contact enum holds the same constants, in the same order, as
// its reportvocab twin; a constant that drifts fails to compile here.
var (
	_ = [1]struct{}{}[int(ContactUndecided)-int(reportvocab.ContactUndecided)]
	_ = [1]struct{}{}[int(ContactSeparated)-int(reportvocab.ContactSeparated)]
	_ = [1]struct{}{}[int(ContactTouching)-int(reportvocab.ContactTouching)]
	_ = [1]struct{}{}[int(ContactOverlapping)-int(reportvocab.ContactOverlapping)]
	_ = [1]struct{}{}[int(ContactBand)-int(reportvocab.ContactBand)]
	_ = [1]struct{}{}[int(ContactNoReason)-int(reportvocab.ContactNoReason)]
	_ = [1]struct{}{}[int(ContactPayloadUnsupported)-int(reportvocab.ContactPayloadUnsupported)]
	_ = [1]struct{}{}[int(ContactNoGapProof)-int(reportvocab.ContactNoGapProof)]
	_ = [1]struct{}{}[int(ContactNoNormalProof)-int(reportvocab.ContactNoNormalProof)]
	_ = [1]struct{}{}[int(ContactAmbiguousFeature)-int(reportvocab.ContactAmbiguousFeature)]
	_ = [1]struct{}{}[int(ContactPointTooCoarse)-int(reportvocab.ContactPointTooCoarse)]
	_ = [1]struct{}{}[int(ContactNonConvex)-int(reportvocab.ContactNonConvex)]
)

// contactRequestToInternal gives a public request its reportvocab form.
func contactRequestToInternal(req ContactRequest) reportvocab.ContactRequest {
	return reportvocab.ContactRequest(req)
}

// contactRequestFromInternal gives an internal request its public form.
func contactRequestFromInternal(req reportvocab.ContactRequest) ContactRequest {
	return ContactRequest(req)
}

// contactFeatureFromInternal maps a feature's nodes to their handles; a nil
// node stays a nil handle.
func contactFeatureFromInternal(f reportvocab.ContactFeature) ContactFeature {
	return ContactFeature{Face: faceHandle(f.Face), Edge: edgeHandle(f.Edge), Vertex: vertexHandle(f.Vertex)}
}

// contactFeatureToInternal maps a feature's handles to their nodes; a nil
// handle stays a nil node.
func contactFeatureToInternal(f ContactFeature) reportvocab.ContactFeature {
	return reportvocab.ContactFeature{Face: faceNode(f.Face), Edge: edgeNode(f.Edge), Vertex: vertexNode(f.Vertex)}
}

// contactPointFromInternal gives an internal point its public form.
func contactPointFromInternal(p reportvocab.ContactPoint) ContactPoint {
	return ContactPoint{
		OnA: vecMeasurementFromInternal(p.OnA), OnB: vecMeasurementFromInternal(p.OnB),
		Normal: vecMeasurementFromInternal(p.Normal), NormalAngle: p.NormalAngle,
		Separation: measurementFromInternal(p.Separation),
		FaceA:      faceHandle(p.FaceA), FaceB: faceHandle(p.FaceB),
		FeatureA: contactFeatureFromInternal(p.FeatureA), FeatureB: contactFeatureFromInternal(p.FeatureB),
	}
}

// contactManifoldFromInternal gives an internal manifold a fresh public
// form: a nil manifold stays nil, and nil Points stay nil while empty
// non-nil Points stay empty and non-nil.
func contactManifoldFromInternal(m *reportvocab.ContactManifold) *ContactManifold {
	if m == nil {
		return nil
	}
	if m.Points == nil {
		return &ContactManifold{}
	}
	points := make([]ContactPoint, len(m.Points))
	for i := range m.Points {
		points[i] = contactPointFromInternal(m.Points[i])
	}
	return &ContactManifold{Points: points}
}

// contactReportFromInternal gives a finished internal report a fresh public
// form. Topology nodes map to their handles, and Gap, Overlap and Manifold
// keep their nil or non-nil state.
func contactReportFromInternal(r *reportvocab.ContactReport) *ContactReport {
	return &ContactReport{
		A: bodyHandle(r.A), B: bodyHandle(r.B), PoseA: r.PoseA, PoseB: r.PoseB,
		Request:  contactRequestFromInternal(r.Request),
		Relation: ContactRelation(r.Relation),
		Gap:      measurementPtrFromInternal(r.Gap), Overlap: measurementPtrFromInternal(r.Overlap),
		Manifold: contactManifoldFromInternal(r.Manifold),
		Reason:   ContactReason(r.Reason),
	}
}

// pairReportMemoCap bounds one body's completed contact reports.
const pairReportMemoCap = 1024

// pairReportKey preserves pose bits, including the sign of zero.
type pairReportKey struct {
	b     *Body
	poses [24]uint64
	req   ContactRequest
}

func newPairReportKey(b *Body, poseA, poseB r3.Transform, req ContactRequest) pairReportKey {
	key := pairReportKey{b: b, req: req}
	i := 0
	for _, pose := range [2]r3.Transform{poseA, poseB} {
		basis := pose.Basis()
		for _, v := range [4]r3.Vec{basis.EX, basis.EY, basis.EZ, pose.Translation()} {
			key.poses[i] = math.Float64bits(v.X)
			key.poses[i+1] = math.Float64bits(v.Y)
			key.poses[i+2] = math.Float64bits(v.Z)
			i += 3
		}
	}
	return key
}

func cloneContactReport(r *ContactReport) *ContactReport {
	out := *r
	if r.Gap != nil {
		gap := *r.Gap
		out.Gap = &gap
	}
	if r.Overlap != nil {
		overlap := *r.Overlap
		out.Overlap = &overlap
	}
	if r.Manifold != nil {
		out.Manifold = &ContactManifold{Points: slices.Clone(r.Manifold.Points)}
	}
	return &out
}
