package decad

import (
	"context"
	"fmt"
	"math"
	"slices"

	"github.com/lestrrat-3d/decad/report"

	"github.com/lestrrat-3d/decad/internal/contactpair"
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// ContactRequest states the maximum position and normal error a manifold may publish.
// SupportBand and HeldChord are nonnegative lengths; zero disables their
// respective approximate contact paths.
type ContactRequest = report.ContactRequest

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
type ContactRelation = report.ContactRelation

const (
	ContactUndecided   = report.ContactUndecided
	ContactSeparated   = report.ContactSeparated
	ContactTouching    = report.ContactTouching
	ContactOverlapping = report.ContactOverlapping
	ContactBand        = report.ContactBand
)

const (
	ContactNoReason           = report.ContactNoReason
	ContactPayloadUnsupported = report.ContactPayloadUnsupported
	ContactNoGapProof         = report.ContactNoGapProof
	ContactNoNormalProof      = report.ContactNoNormalProof
	ContactAmbiguousFeature   = report.ContactAmbiguousFeature
	ContactPointTooCoarse     = report.ContactPointTooCoarse
	ContactNonConvex          = report.ContactNonConvex
)

// ContactReason explains an undecided relation or an absent manifold.
type ContactReason = report.ContactReason

type (
	// ContactFeature names an original topological feature.
	ContactFeature = report.ContactFeatureOf[*Face, *Edge, *Vertex]
	// ContactPoint bounds two boundary witnesses and their A-to-B normal.
	ContactPoint = report.ContactPointOf[*Face, *Edge, *Vertex]
	// ContactManifold is a deterministic reduction of the complete certified contact patch.
	ContactManifold = report.ContactManifoldOf[*Face, *Edge, *Vertex]
	// ContactReport is a read-only pair result at the two caller-supplied poses.
	ContactReport = report.ContactReportOf[*Body, *Face, *Edge, *Vertex]
)

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
	found, err := contactpair.Classify(ctx, verifyAllMesher{}, bodyNode(a), bodyNode(b), poseA, poseB, req)
	if err != nil {
		return nil, err
	}
	report := contactReportFromInternal(found)
	a.pairReports.Store(key, report, pairReportMemoCap)
	return cloneContactReport(report), nil
}

// The contact vocabulary with topology nodes as the face, edge, vertex and
// body references, as the internal kernels produce it.
type (
	nodeContactFeature  = report.ContactFeatureOf[*topo.Face, *topo.Edge, *topo.Vertex]
	nodeContactManifold = report.ContactManifoldOf[*topo.Face, *topo.Edge, *topo.Vertex]
	nodeContactReport   = report.ContactReportOf[*topo.Body, *topo.Face, *topo.Edge, *topo.Vertex]
)

// handleLift maps topology nodes to their root handles; a nil node stays a
// nil handle.
var handleLift = report.Lift[*topo.Body, *topo.Face, *topo.Edge, *topo.Vertex, *Body, *Face, *Edge, *Vertex]{
	Body: bodyHandle, Face: faceHandle, Edge: edgeHandle, Vertex: vertexHandle,
}

// nodeLift is handleLift's inverse.
var nodeLift = report.Lift[*Body, *Face, *Edge, *Vertex, *topo.Body, *topo.Face, *topo.Edge, *topo.Vertex]{
	Body: bodyNode, Face: faceNode, Edge: edgeNode, Vertex: vertexNode,
}

func contactFeatureToInternal(f ContactFeature) nodeContactFeature {
	return nodeLift.Feature(f)
}
func contactManifoldFromInternal(m *nodeContactManifold) *ContactManifold {
	return handleLift.Manifold(m)
}
func contactReportFromInternal(r *nodeContactReport) *ContactReport {
	return handleLift.Report(r)
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
