package decad

import (
	"context"
	"fmt"
	"math"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
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
// A zero-bound faceted solid, or its translation-only placed copy with an
// exact source mesh, can prove lower support face contact or an axial gap
// against a source-box floor that strictly contains its support footprint.
// Other positive-bound faceted solids can prove a strict axial gap with their
// boundary displacement charged, but publish no contact manifold.
// An exactly orthogonal rotated source box can give a sphere a bounded point
// on one interior face.
// At identity query poses, the analytic clearance kernel can prove relations
// for other solids without a contact manifold.
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
		if poseA == r3.Identity() && poseB == r3.Identity() {
			if err := classifyAnalyticContact(ctx, report); err != nil {
				return nil, err
			}
			return report, nil
		}
		report.Reason = ContactPayloadUnsupported
		return report, nil
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	classifySourceBoxes(report, boxA, boxB)
	return report, nil
}

// classifyAnalyticContact consumes only the clearance kernel's complete-pair
// verdict. Its candidate intervals do not retain admitted source witnesses or
// normals, so even a proved touch or overlap cannot publish a manifold here.
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
	case pairOverlapping:
		report.Relation = ContactOverlapping
		report.Reason = ContactNoNormalProof
	default:
		report.Reason = ContactNoGapProof
	}
	return nil
}

func classifySourceBoxes(report *ContactReport, a, b sourceBoxContactProof) {
	var gaps [3]proofarith.Dyadic
	touchAxes := 0
	overlaps := true
	for i := range 3 {
		switch {
		case proofarith.DyCmp(a.hi[i], b.lo[i]) < 0:
			gaps[i] = proofarith.DySubScalar(b.lo[i], a.hi[i])
			overlaps = false
		case proofarith.DyCmp(b.hi[i], a.lo[i]) < 0:
			gaps[i] = proofarith.DySubScalar(a.lo[i], b.hi[i])
			overlaps = false
		case proofarith.DyCmp(a.hi[i], b.lo[i]) == 0 || proofarith.DyCmp(b.hi[i], a.lo[i]) == 0:
			touchAxes++
			overlaps = false
		}
	}
	for _, gap := range gaps {
		if gap.Sign() > 0 {
			m, ok := sourceBoxGap(gaps)
			if !ok {
				report.Reason = ContactNoGapProof
				return
			}
			report.Relation = ContactSeparated
			report.Gap = &m
			return
		}
	}
	if touchAxes > 0 {
		report.Relation = ContactTouching
		gap := Measurement{Value: units.Millimeters(0), Exactness: Exact, Bound: units.Millimeters(0)}
		report.Gap = &gap
		if touchAxes != 1 {
			report.Reason = ContactAmbiguousFeature
			return
		}
		for i := range 3 {
			if proofarith.DyCmp(a.hi[i], b.lo[i]) == 0 {
				publishSourceBoxPatch(report, a, b, i, 1, proofarith.DyZero())
				return
			}
			if proofarith.DyCmp(b.hi[i], a.lo[i]) == 0 {
				publishSourceBoxPatch(report, a, b, i, -1, proofarith.DyZero())
				return
			}
		}
	}
	if !overlaps {
		report.Reason = ContactPayloadUnsupported
		return
	}
	report.Relation = ContactOverlapping
	// Six directed translations move B across one of A's support planes.
	// A unique smallest depth and two crossing faces are required for response.
	axis, sign, depth, unique := sourceBoxTranslation(a, b)
	if !unique {
		report.Reason = ContactAmbiguousFeature
		return
	}
	if sign > 0 {
		if proofarith.DyCmp(b.lo[axis], a.lo[axis]) <= 0 || proofarith.DyCmp(b.hi[axis], a.hi[axis]) <= 0 {
			report.Reason = ContactAmbiguousFeature
			return
		}
	} else if proofarith.DyCmp(b.lo[axis], a.lo[axis]) >= 0 || proofarith.DyCmp(b.hi[axis], a.hi[axis]) >= 0 {
		report.Reason = ContactAmbiguousFeature
		return
	}
	publishSourceBoxPatch(report, a, b, axis, sign, proofarith.DyNeg(depth))
}

func sourceBoxTranslation(a, b sourceBoxContactProof) (int, int, proofarith.Dyadic, bool) {
	var best proofarith.Dyadic
	axis, sign, ties := 0, 0, false
	for i := range 3 {
		for _, candidate := range []struct {
			value proofarith.Dyadic
			sign  int
		}{
			{proofarith.DySubScalar(a.hi[i], b.lo[i]), 1},
			{proofarith.DySubScalar(b.hi[i], a.lo[i]), -1},
		} {
			if candidate.value.Sign() <= 0 {
				return 0, 0, proofarith.Dyadic{}, false
			}
			cmp := proofarith.DyCmp(candidate.value, best)
			if sign == 0 || cmp < 0 {
				axis, sign, best, ties = i, candidate.sign, candidate.value, false
			} else if cmp == 0 {
				ties = true
			}
		}
	}
	return axis, sign, best, !ties
}

func sourceBoxGap(gaps [3]proofarith.Dyadic) (Measurement, bool) {
	positive := 0
	var only proofarith.Dyadic
	squared := proofarith.Dyadic{}
	for _, gap := range gaps {
		if gap.Sign() > 0 {
			positive++
			only = gap
			squared = proofarith.DyAdd(squared, proofarith.DyMul(gap, gap))
		}
	}
	if positive == 1 {
		v, exact := only.Float64()
		if !finiteMeasurementValues(v) {
			return Measurement{}, false
		}
		bound := proofarith.DyadicFloatError(only, v)
		return Measurement{Value: units.Millimeters(v), Exactness: exactnessOf(bound),
			Bound: units.Millimeters(bound)}, exact || bound < v
	}
	lo, hi := proofarith.DySqrtDown(squared), proofarith.DySqrtUp(squared)
	if !finiteMeasurementValues(lo, hi) || lo <= 0 {
		return Measurement{}, false
	}
	v := lo + (hi-lo)/2
	bound := provenUpRound(math.Max(v-lo, hi-v))
	return Measurement{Value: units.Millimeters(v), Exactness: exactnessOf(bound),
		Bound: units.Millimeters(bound)}, finiteMeasurementValues(v, bound)
}
