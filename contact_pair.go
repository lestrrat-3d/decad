package decad

import (
	"context"
	"fmt"
	"math"
	"slices"
	"sync"

	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/pair"
	"github.com/lestrrat-3d/decad/internal/pair/box"
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
		return report, nil
	}
	found, err := classifyContactPair(ctx, a, b, poseA, poseB, req)
	if err != nil {
		return nil, err
	}
	report := contactReportFromInternal(found)
	a.pairReports.Store(key, report)
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

// classifyContactPair is ContactPair's proof for validated, distinct, live
// solids and a validated request, before the memo.
func classifyContactPair(ctx context.Context, a, b *Body, poseA, poseB r3.Transform,
	req ContactRequest) (*reportvocab.ContactReport, error) {
	report := &reportvocab.ContactReport{A: bodyNode(a), B: bodyNode(b), PoseA: poseA, PoseB: poseB,
		Request: contactRequestToInternal(req)}
	boxA, okA := sourceBoxAtPose(a, poseA)
	boxB, okB := sourceBoxAtPose(b, poseB)
	if !okA && !okB {
		sphereA, sphereOKA := sourceSphereAtPose(a, poseA)
		sphereB, sphereOKB := sourceSphereAtPose(b, poseB)
		if sphereOKA && sphereOKB {
			classifySourceSpherePair(report, sphereA, sphereB)
			return report, nil
		}
	}
	if okA && !okB {
		if _, faceted := b.n.Payload.(facetedPayload); faceted {
			if err := classifyFacetedFloorBox(ctx, report, b, poseB, boxA, false); err != nil {
				return nil, err
			}
			if err := classifyPlanarFallback(ctx, report); err != nil {
				return nil, err
			}
			return report, nil
		}
		if sphere, ok := sourceSphereAtPose(b, poseB); ok {
			classifySourceSphereBox(report, sphere, boxA, false)
			return report, nil
		}
		if cylinder, ok := sourceCylinderAtPose(b, poseB); ok {
			classifySourceCylinderBox(report, cylinder, boxA, false)
			if err := classifyUndecidedCylinder(ctx, report); err != nil {
				return nil, err
			}
			return report, nil
		}
	}
	if okB && !okA {
		if _, faceted := a.n.Payload.(facetedPayload); faceted {
			if err := classifyFacetedFloorBox(ctx, report, a, poseA, boxB, true); err != nil {
				return nil, err
			}
			if err := classifyPlanarFallback(ctx, report); err != nil {
				return nil, err
			}
			return report, nil
		}
		if sphere, ok := sourceSphereAtPose(a, poseA); ok {
			classifySourceSphereBox(report, sphere, boxB, true)
			return report, nil
		}
		if cylinder, ok := sourceCylinderAtPose(a, poseA); ok {
			classifySourceCylinderBox(report, cylinder, boxB, true)
			if err := classifyUndecidedCylinder(ctx, report); err != nil {
				return nil, err
			}
			return report, nil
		}
	}
	if !okA || !okB {
		orientedA, orientedOKA := sourceOrientedBoxAtPose(a, poseA)
		orientedB, orientedOKB := sourceOrientedBoxAtPose(b, poseB)
		if orientedOKA && !orientedOKB {
			if sphere, ok := sourceSphereAtPose(b, poseB); ok {
				classifySourceSphereOrientedBox(report, sphere, orientedA, false)
				return report, nil
			}
		}
		if orientedOKB && !orientedOKA {
			if sphere, ok := sourceSphereAtPose(a, poseA); ok {
				classifySourceSphereOrientedBox(report, sphere, orientedB, true)
				return report, nil
			}
		}
		if orientedOKA && orientedOKB {
			classifyOrientedSourceBoxes(report, orientedA, orientedB)
			if err := classifyPlanarManifold(ctx, report); err != nil {
				return nil, err
			}
			if err := classifyPlanarSupportBand(ctx, report); err != nil {
				return nil, err
			}
			return report, nil
		}
		planar, err := classifyExactPlanarPair(ctx, report)
		if err != nil {
			return nil, err
		}
		if planar && report.Relation != reportvocab.ContactUndecided {
			return report, nil
		}
		identity := poseA == r3.Identity() && poseB == r3.Identity()
		if !planar && !identity {
			placed, err := classifyPlacedRuling(ctx, report)
			if err != nil {
				return nil, err
			}
			if placed {
				return report, nil
			}
		}
		if identity {
			report.Reason = reportvocab.ContactNoReason
			if err := classifyAnalyticContact(ctx, report); err != nil {
				return nil, err
			}
			return report, nil
		}
		if !planar {
			report.Reason = reportvocab.ContactPayloadUnsupported
		}
		return report, nil
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	classifySourceBoxes(report, boxA, boxB)
	return report, nil
}

// classifyPlanarFallback replaces an undecided source-box/faceted report with
// the exact planar relation when both bodies are admitted and it decides one.
func classifyPlanarFallback(ctx context.Context, report *reportvocab.ContactReport) error {
	if report.Relation != reportvocab.ContactUndecided {
		return nil
	}
	trial := reportvocab.ContactReport{A: report.A, B: report.B, PoseA: report.PoseA, PoseB: report.PoseB,
		Request: report.Request}
	planar, err := classifyExactPlanarPair(ctx, &trial)
	if err != nil {
		return err
	}
	if planar && trial.Relation != reportvocab.ContactUndecided {
		*report = trial
	}
	return nil
}

// classifyPlanarManifold completes an oriented source-box report whose box
// patches publish no manifold for a touching or overlapping pair: both boxes
// are exact planar bodies (docs/multibody-dynamics-design.md §9), so §9.3's
// manifold replaces the report when the exact planar relation agrees and
// every published point is an edge or vertex of one box on, in or crossing
// the other, the rows the box patches do not cover. Face pairs stay with the
// box patches (§9.4), and a report the planar path cannot complete stands.
func classifyPlanarManifold(ctx context.Context, report *reportvocab.ContactReport) error {
	if report.Manifold != nil || report.Reason == reportvocab.ContactPointTooCoarse ||
		report.Relation != reportvocab.ContactTouching && report.Relation != reportvocab.ContactOverlapping {
		return nil
	}
	trial := reportvocab.ContactReport{A: report.A, B: report.B, PoseA: report.PoseA, PoseB: report.PoseB,
		Request: report.Request}
	planar, err := classifyExactPlanarPair(ctx, &trial)
	if err != nil {
		return err
	}
	if !planar || trial.Relation != report.Relation || trial.Manifold == nil {
		return nil
	}
	for _, point := range trial.Manifold.Points {
		if point.FeatureA.Face != nil && point.FeatureB.Face != nil {
			return nil
		}
	}
	*report = trial
	return nil
}

// classifyUndecidedCylinder completes a source cylinder report the box
// proofs left undecided: at identity query poses the analytic kernel's
// verdict, elsewhere the placed ruling of classifyPlacedRuling.
func classifyUndecidedCylinder(ctx context.Context, report *reportvocab.ContactReport) error {
	if report.Relation != reportvocab.ContactUndecided {
		return nil
	}
	if report.PoseA == r3.Identity() && report.PoseB == r3.Identity() {
		report.Reason = reportvocab.ContactNoReason
		return classifyAnalyticContact(ctx, report)
	}
	_, err := classifyPlacedRuling(ctx, report)
	return err
}

// classifyPlanarSupportBand completes an oriented source-box report that may
// be separated by at most the request's positive SupportBand: the box patches
// know no band, so the exact planar path decides whether the pair is §10.5's
// ContactBand, and its report replaces this one when it is.
func classifyPlanarSupportBand(ctx context.Context, report *reportvocab.ContactReport) error {
	if supportBandOf(report.Request).Sign() <= 0 || report.Relation != reportvocab.ContactSeparated || report.Gap == nil ||
		report.Gap.Value.Base()-report.Gap.Bound.Base() > report.Request.SupportBand.Base() {
		return nil
	}
	trial := reportvocab.ContactReport{A: report.A, B: report.B, PoseA: report.PoseA, PoseB: report.PoseB,
		Request: report.Request}
	planar, err := classifyExactPlanarPair(ctx, &trial)
	if err != nil {
		return err
	}
	if planar && trial.Relation == reportvocab.ContactBand {
		*report = trial
	}
	return nil
}

// classifyAnalyticContact consumes the clearance kernel's complete-pair
// verdict. Only a touch the kernel proved by a ruling certificate keeps its
// faces and feet, so only that touch can publish a manifold here.
func classifyAnalyticContact(ctx context.Context, report *reportvocab.ContactReport) error {
	res, err := clearancePair(ctx, bodyHandle(report.A), bodyHandle(report.B), false)
	if err != nil {
		return err
	}
	switch res.verdict {
	case reportvocab.PairDisjoint:
		gap := measurementToInternal(pairGapMeasurement(res))
		if !measurement.FiniteValues(gap.Value.Base(), gap.Bound.Base()) ||
			gap.Value.Base()-gap.Bound.Base() <= 0 {
			report.Reason = reportvocab.ContactNoGapProof
			return nil
		}
		report.Relation = reportvocab.ContactSeparated
		report.Gap = &gap
	case reportvocab.PairTouching:
		report.Relation = reportvocab.ContactTouching
		gap := measurement.Measurement{Value: units.Millimeters(0), Exactness: measurement.Exact, Bound: units.Millimeters(0)}
		report.Gap = &gap
		report.Reason = reportvocab.ContactNoNormalProof
		if res.ruling != nil {
			publishRulingManifold(report, res.ruling)
		}
	case reportvocab.PairOverlapping:
		report.Relation = reportvocab.ContactOverlapping
		report.Reason = reportvocab.ContactNoNormalProof
	default:
		report.Reason = reportvocab.ContactNoGapProof
	}
	return nil
}

func classifySourceBoxes(report *reportvocab.ContactReport, a, b sourceBoxContactProof) {
	result := box.ClassifyAxisBoxes(
		box.AxisBox{Lo: a.lo, Hi: a.hi},
		box.AxisBox{Lo: b.lo, Hi: b.hi},
		box.AxisBoxRequest{PointResolutionMM: report.Request.PointResolution.Base()},
	)
	switch result.Relation {
	case pair.Separated:
		report.Relation = reportvocab.ContactSeparated
	case pair.Touching:
		report.Relation = reportvocab.ContactTouching
	case pair.Overlapping:
		report.Relation = reportvocab.ContactOverlapping
	default:
		report.Relation = reportvocab.ContactUndecided
	}
	report.Reason = sourceBoxReason(result.Reason)
	if result.Gap != nil {
		gap := sourceBoxScalar(*result.Gap)
		report.Gap = &gap
	}
	if result.Patch == nil {
		return
	}
	publishAxisBoxPatch(report, a, b, result.Patch)
}

func sourceBoxReason(reason pair.Reason) reportvocab.ContactReason {
	switch reason {
	case pair.NoGapProof:
		return reportvocab.ContactNoGapProof
	case pair.AmbiguousFeature:
		return reportvocab.ContactAmbiguousFeature
	case pair.PointTooCoarse:
		return reportvocab.ContactPointTooCoarse
	case pair.PayloadUnsupported:
		return reportvocab.ContactPayloadUnsupported
	case pair.NoNormalProof:
		return reportvocab.ContactNoNormalProof
	default:
		return reportvocab.ContactNoReason
	}
}

func publishAxisBoxPatch(report *reportvocab.ContactReport, a, b sourceBoxContactProof, patch *box.AxisBoxPatch) {
	faceA := faceNode(a.faces[patch.FaceA.Axis][patch.FaceA.Side])
	faceB := faceNode(b.faces[patch.FaceB.Axis][patch.FaceB.Side])
	var normal r3.Vec
	switch patch.NormalAxis {
	case 0:
		normal.X = float64(patch.NormalSign)
	case 1:
		normal.Y = float64(patch.NormalSign)
	case 2:
		normal.Z = float64(patch.NormalSign)
	}
	separation := sourceBoxScalar(patch.Separation)
	points := make([]reportvocab.ContactPoint, 0, len(patch.Points))
	for _, point := range patch.Points {
		points = append(points, reportvocab.ContactPoint{
			OnA:         sourceBoxPointMeasurement(point.OnA),
			OnB:         sourceBoxPointMeasurement(point.OnB),
			Normal:      measurement.VecMeasurement{Value: normal, Exactness: measurement.Exact, Bound: units.Scalar(0)},
			NormalAngle: units.Radians(0),
			Separation:  separation,
			FaceA:       faceA, FaceB: faceB,
			FeatureA: reportvocab.ContactFeature{Face: faceA},
			FeatureB: reportvocab.ContactFeature{Face: faceB},
		})
	}
	report.Manifold = &reportvocab.ContactManifold{Points: points}
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

// pairReportMemo stores completed reports and returns independent copies.
type pairReportMemo struct {
	mu      sync.Mutex
	entries map[pairReportKey]*ContactReport
	order   []pairReportKey
	next    int
}

func (m *pairReportMemo) Load(key pairReportKey) (*ContactReport, bool) {
	m.mu.Lock()
	report, ok := m.entries[key]
	m.mu.Unlock()
	if !ok {
		return nil, false
	}
	return cloneContactReport(report), true
}

func (m *pairReportMemo) Store(key pairReportKey, report *ContactReport) {
	m.mu.Lock()
	defer m.mu.Unlock()
	if _, ok := m.entries[key]; ok {
		return
	}
	if m.entries == nil {
		m.entries = make(map[pairReportKey]*ContactReport)
	}
	if len(m.order) < pairReportMemoCap {
		m.order = append(m.order, key)
	} else {
		delete(m.entries, m.order[m.next])
		m.order[m.next] = key
		m.next = (m.next + 1) % pairReportMemoCap
	}
	m.entries[key] = report
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
