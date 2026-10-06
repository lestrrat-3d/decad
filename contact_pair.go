package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/pair"
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
	if !finiteMeasurementValues(v.Base()) {
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

// ContactFeature names an original topological feature. The first contact
// stage publishes face features; later stages may use edge and vertex fields.
type ContactFeature struct {
	Face   *Face
	Edge   *Edge
	Vertex *Vertex
}

// ContactPoint bounds two boundary witnesses and their A-to-B normal.
// Separation is the signed B-minus-A distance along that normal.
type ContactPoint struct {
	OnA, OnB           VecMeasurement
	Normal             VecMeasurement
	NormalAngle        units.Value
	Separation         Measurement
	FaceA, FaceB       *Face
	FeatureA, FeatureB ContactFeature
}

// ContactManifold is a deterministic reduction of the complete certified
// contact patch. Its points are immutable once returned.
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
	if !a.solid || a.kind != BodySolid || !b.solid || b.kind != BodySolid {
		return nil, ErrNotSolid
	}
	if !poseA.IsValid() || !poseB.IsValid() {
		return nil, fmt.Errorf("%w: contact pose is invalid", ErrDegenerate)
	}
	if req.PointResolution.Kind() != units.Length || req.NormalResolution.Kind() != units.Angle {
		return nil, fmt.Errorf("%w: contact resolutions must be Length and Angle", ErrUnitKind)
	}
	if !finiteMeasurementValues(req.PointResolution.Base(), req.NormalResolution.Base()) {
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
	if report, ok := a.pairReports.load(key); ok {
		return report, nil
	}
	report, err := classifyContactPair(ctx, a, b, poseA, poseB, req)
	if err != nil {
		return nil, err
	}
	a.pairReports.store(key, report)
	return cloneContactReport(report), nil
}

// classifyContactPair is ContactPair's proof for validated, distinct, live
// solids and a validated request, before the memo.
func classifyContactPair(ctx context.Context, a, b *Body, poseA, poseB r3.Transform,
	req ContactRequest) (*ContactReport, error) {
	report := &ContactReport{A: a, B: b, PoseA: poseA, PoseB: poseB, Request: req}
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
		if _, faceted := b.payload.(facetedPayload); faceted {
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
		if _, faceted := a.payload.(facetedPayload); faceted {
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
		if planar && report.Relation != ContactUndecided {
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
			report.Reason = ContactNoReason
			if err := classifyAnalyticContact(ctx, report); err != nil {
				return nil, err
			}
			return report, nil
		}
		if !planar {
			report.Reason = ContactPayloadUnsupported
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
func classifyPlanarFallback(ctx context.Context, report *ContactReport) error {
	if report.Relation != ContactUndecided {
		return nil
	}
	trial := ContactReport{A: report.A, B: report.B, PoseA: report.PoseA, PoseB: report.PoseB,
		Request: report.Request}
	planar, err := classifyExactPlanarPair(ctx, &trial)
	if err != nil {
		return err
	}
	if planar && trial.Relation != ContactUndecided {
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
func classifyPlanarManifold(ctx context.Context, report *ContactReport) error {
	if report.Manifold != nil || report.Reason == ContactPointTooCoarse ||
		report.Relation != ContactTouching && report.Relation != ContactOverlapping {
		return nil
	}
	trial := ContactReport{A: report.A, B: report.B, PoseA: report.PoseA, PoseB: report.PoseB,
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
func classifyUndecidedCylinder(ctx context.Context, report *ContactReport) error {
	if report.Relation != ContactUndecided {
		return nil
	}
	if report.PoseA == r3.Identity() && report.PoseB == r3.Identity() {
		report.Reason = ContactNoReason
		return classifyAnalyticContact(ctx, report)
	}
	_, err := classifyPlacedRuling(ctx, report)
	return err
}

// classifyPlanarSupportBand completes an oriented source-box report that may
// be separated by at most the request's positive SupportBand: the box patches
// know no band, so the exact planar path decides whether the pair is §10.5's
// ContactBand, and its report replaces this one when it is.
func classifyPlanarSupportBand(ctx context.Context, report *ContactReport) error {
	if supportBandOf(report.Request).Sign() <= 0 || report.Relation != ContactSeparated || report.Gap == nil ||
		report.Gap.Value.Base()-report.Gap.Bound.Base() > report.Request.SupportBand.Base() {
		return nil
	}
	trial := ContactReport{A: report.A, B: report.B, PoseA: report.PoseA, PoseB: report.PoseB,
		Request: report.Request}
	planar, err := classifyExactPlanarPair(ctx, &trial)
	if err != nil {
		return err
	}
	if planar && trial.Relation == ContactBand {
		*report = trial
	}
	return nil
}

// classifyAnalyticContact consumes the clearance kernel's complete-pair
// verdict. Only a touch the kernel proved by a ruling certificate keeps its
// faces and feet, so only that touch can publish a manifold here.
func classifyAnalyticContact(ctx context.Context, report *ContactReport) error {
	res, err := clearancePair(ctx, report.A, report.B, false)
	if err != nil {
		return err
	}
	switch res.verdict {
	case pairDisjoint:
		gap := pairGapMeasurement(res)
		if !finiteMeasurementValues(gap.Value.Base(), gap.Bound.Base()) ||
			gap.Value.Base()-gap.Bound.Base() <= 0 {
			report.Reason = ContactNoGapProof
			return nil
		}
		report.Relation = ContactSeparated
		report.Gap = &gap
	case pairTouching:
		report.Relation = ContactTouching
		gap := Measurement{Value: units.Millimeters(0), Exactness: Exact, Bound: units.Millimeters(0)}
		report.Gap = &gap
		report.Reason = ContactNoNormalProof
		if res.ruling != nil {
			publishRulingManifold(report, res.ruling)
		}
	case pairOverlapping:
		report.Relation = ContactOverlapping
		report.Reason = ContactNoNormalProof
	default:
		report.Reason = ContactNoGapProof
	}
	return nil
}

func classifySourceBoxes(report *ContactReport, a, b sourceBoxContactProof) {
	result := pair.ClassifyAxisBoxes(
		pair.AxisBox{Lo: a.lo, Hi: a.hi},
		pair.AxisBox{Lo: b.lo, Hi: b.hi},
		pair.AxisBoxRequest{PointResolutionMM: report.Request.PointResolution.Base()},
	)
	switch result.Relation {
	case pair.Separated:
		report.Relation = ContactSeparated
	case pair.Touching:
		report.Relation = ContactTouching
	case pair.Overlapping:
		report.Relation = ContactOverlapping
	default:
		report.Relation = ContactUndecided
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

func sourceBoxReason(reason pair.Reason) ContactReason {
	switch reason {
	case pair.NoGapProof:
		return ContactNoGapProof
	case pair.AmbiguousFeature:
		return ContactAmbiguousFeature
	case pair.PointTooCoarse:
		return ContactPointTooCoarse
	case pair.PayloadUnsupported:
		return ContactPayloadUnsupported
	default:
		return ContactNoReason
	}
}

func publishAxisBoxPatch(report *ContactReport, a, b sourceBoxContactProof, patch *pair.AxisBoxPatch) {
	faceA := a.faces[patch.FaceA.Axis][patch.FaceA.Side]
	faceB := b.faces[patch.FaceB.Axis][patch.FaceB.Side]
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
	points := make([]ContactPoint, 0, len(patch.Points))
	for _, point := range patch.Points {
		points = append(points, ContactPoint{
			OnA:         sourceBoxPointMeasurement(point.OnA),
			OnB:         sourceBoxPointMeasurement(point.OnB),
			Normal:      VecMeasurement{Value: normal, Exactness: Exact, Bound: units.Scalar(0)},
			NormalAngle: units.Radians(0),
			Separation:  separation,
			FaceA:       faceA, FaceB: faceB,
			FeatureA: ContactFeature{Face: faceA},
			FeatureB: ContactFeature{Face: faceB},
		})
	}
	report.Manifold = &ContactManifold{Points: points}
}
