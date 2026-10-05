package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/pair"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// ContactRequest states the maximum position and normal error a manifold may publish.
type ContactRequest struct {
	PointResolution  units.Value
	NormalResolution units.Value
}

// ContactRelation is the proven relation of the two complete occupied sets.
type ContactRelation int

const (
	ContactUndecided ContactRelation = iota
	ContactSeparated
	ContactTouching
	ContactOverlapping
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
	// convexity certificate of docs/multibody-dynamics-design.md §9.2.
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
// with a positive determinant, without a contact manifold. A touching or
// overlapping pair names ContactNonConvex when neither body is convex.
// At identity query poses, the analytic clearance kernel can prove relations
// for other solids. Only its ruling touches publish a manifold: the two ends
// of a full source cylinder's ruling on a planar face or another cylinder.
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
	if err := ctx.Err(); err != nil {
		return nil, err
	}
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
			if report.Relation == ContactUndecided && poseA == r3.Identity() && poseB == r3.Identity() {
				report.Reason = ContactNoReason
				if err := classifyAnalyticContact(ctx, report); err != nil {
					return nil, err
				}
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
			if report.Relation == ContactUndecided && poseA == r3.Identity() && poseB == r3.Identity() {
				report.Reason = ContactNoReason
				if err := classifyAnalyticContact(ctx, report); err != nil {
					return nil, err
				}
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
			return report, nil
		}
		planar, err := classifyExactPlanarPair(ctx, report)
		if err != nil {
			return nil, err
		}
		if planar && report.Relation != ContactUndecided {
			return report, nil
		}
		if poseA == r3.Identity() && poseB == r3.Identity() {
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
