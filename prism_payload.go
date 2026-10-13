package decad

import (
	"context"
	"math"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/massmoment"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismrecord"

	"github.com/lestrrat-3d/decad/internal/proofbound"

	"github.com/lestrrat-3d/r3"
)

// This file is prismPayload — the record a straight extrude evaluates from —
// and the coordinate readings taken directly off it: a point in world space,
// the proven bound on that point, and the profile-wide coordinate envelopes
// every later bound is charged against.
//
// The payload holds a RECORDED section and two RECORDED sweep levels, each
// with its own proven displacement from what the construction denotes
// (sectionDelta, z0Delta/z1Delta). Every reader here composes the
// displacement its own answer depends on rather than reading the record as
// exact. See docs/evaluator-design.md §5 and docs/prism-boolean-design.md §7.

// prismPayload is the evaluator's own record of a prism body. Its plain data —
// section, frame, sweep levels and their displacements, placement, blend roles,
// surface flag and walk cache — is prismrecord.Payload, embedded by value so a
// copy of the wrapper copies the record exactly as before. The wrapper adds what
// needs the root package: transform and placed (featurePayload), and the
// readings that satisfy root interfaces (axialDelta, extentAlong). See
// docs/evaluator-design.md §8.
type prismPayload struct {
	prismrecord.Payload
}

// Z0Scalar and Z1Scalar are the sweep levels as bounded readings — the recorded
// float beside its own axial displacement. Every measurement derived from a
// level integrates these rather than the bare float, so a level the evaluator
// computed can never publish itself as the level it denotes.
func (pp prismPayload) Z0Scalar() proofbound.BoundedScalar {
	return proofbound.MeasuredScalar(pp.Z0, pp.Z0Delta)
}

func (pp prismPayload) Z1Scalar() proofbound.BoundedScalar {
	return proofbound.MeasuredScalar(pp.Z1, pp.Z1Delta)
}

// axialDelta is the larger of the two ends' displacements: the figure a reading
// that cannot attribute its error to one particular end takes.
func (pp prismPayload) axialDelta() float64 { return math.Max(pp.Z0Delta, pp.Z1Delta) }

// Point lifts a plane-local (u, v) at height z into placed world space.
func (pp prismPayload) Point(u, v, z float64) r3.Vec {
	local := pp.Frame.ToWorldUV(u, v)
	n := pp.Frame.N()
	return pp.Xform.Apply(local.Add(n.Scale(z)))
}

// prismPointBound carries plane-local coordinate error through the
// frame/placement and charges the float operations that evaluate both maps.
// The source error is a ball of radius Radius3D of the largest coordinate
// bound, and the lift moves it by the held linear map
// L = placement basis · [U V N] (massmoment.PrismRotation), which stretches
// it by at most prismLiftFactor; the exact rounding pp.point commits is
// charged beside it.
func prismPointBound(pp prismPayload, u, v, z proofbound.BoundedScalar) float64 {
	return prismPointBoundWith(pp, prismLiftFactor(pp), u, v, z)
}

// prismPointBoundWith is prismPointBound with the lift factor read once by a
// caller that lifts many points through the same payload.
func prismPointBoundWith(pp prismPayload, factor float64, u, v, z proofbound.BoundedScalar) float64 {
	source := proofbound.Radius3D(max(u.Bound, v.Bound, z.Bound))
	held := pp.Point(u.Value, v.Value, z.Value)
	round := exactPrismPointRound(pp, u.Value, v.Value, z.Value, held)
	return proofbound.AbsSumUpper(proofbound.ProductUpper(factor, source), round)
}

// prismLiftFactor bounds how far pp's held lift stretches a plane-local
// displacement: 1 + e, rounded up, with e the orthonormality defect of the
// held linear map L, the entrywise absolute sum of LᵀL − I
// (massmoment.MapChargeOf's Stretch). Every eigenvalue of LᵀL lies in
// [1 − e, 1 + e], so |L·d| ≤ √(1 + e)·|d| ≤ (1 + e)·|d|. An exactly
// orthonormal L, an axis-aligned frame without placement among them, answers
// exactly 1. A map that is not finite, or whose defect reaches 1/2, answers
// +Inf.
func prismLiftFactor(pp prismPayload) float64 {
	l, err := massmoment.PrismRotation(pp.Frame, pp.Xform)
	if err != nil {
		return math.Inf(1)
	}
	charge, err := massmoment.MapChargeOf(l)
	if err != nil {
		return math.Inf(1)
	}
	if charge.Stretch == 0 {
		return 1
	}
	return proofbound.AbsSumUpper(1, charge.Stretch)
}

// exactPrismPointRound is proofbound.ExactFrameLiftRound read through pp's own
// frame and accumulated placement: the exact rounding pp.point committed
// lifting (u, v, z) to held.
func exactPrismPointRound(pp prismPayload, u, v, z float64, held r3.Vec) float64 {
	return proofbound.ExactFrameLiftRound(pp.Frame, pp.Xform, u, v, z, held)
}

// CircleCurveBound is an Edge's curveBound for a rim circle or arc this
// payload lifts from the recorded centre (cu, cv) at level z, held with
// centre at center, its axis along axis, and the walk's radius, which sits within
// radiusBound of the radius the record denotes. The denoted curve is the
// recorded circle carried through L = B·[U V N] (massmoment.PrismRotation),
// displaced by the level's own zDelta along L·N, by the section's own
// sectionDelta in the plane, and by centerDelta, how far the denoted centre
// sits from (cu, cv) in the plane. Its distance from the held circle is at most
// the centre's exact lift rounding, massmoment.CircleImageGap's in-plane and
// tilt terms, and those two displacements stretched by L. It answers +Inf,
// and ok false, when that bound is not below half the radius, where radial
// projection onto the held circle stops being continuous.
func (pp prismPayload) CircleCurveBound(cu, cv, z, zDelta, centerDelta, radius, radiusBound float64, center, axis r3.Vec) (float64, bool) {
	l, err := massmoment.PrismRotation(pp.Frame, pp.Xform)
	if err != nil {
		return math.Inf(1), false
	}
	charge, err := massmoment.MapChargeOf(l)
	if err != nil {
		return math.Inf(1), false
	}
	displaced := proofbound.AbsSumUpper(zDelta, proofbound.ProductUpper(2, pp.SectionDelta), centerDelta)
	bound := proofbound.AbsSumUpper(
		exactPrismPointRound(pp, cu, cv, z, center),
		massmoment.CircleImageGap(l, charge.Stretch, axis, radius, radiusBound),
		proofbound.ProductUpper(proofbound.AbsSumUpper(1, charge.Stretch), displaced),
	)
	if proofbound.IsNonFinite(bound) || !(proofbound.ProductUpper(2, bound) < radius) {
		return math.Inf(1), false
	}
	return bound, true
}

// LiftedVertex lifts a plane-local (u, v) at height z through pp.point and
// returns the held point beside the exact rounding that lift committed
// (exactPrismPointRound). Every analytic vertex a prism-shaped build places
// takes its frame and placement charge from here, one vertex at a time, so
// a lift that is exact for the coordinates at hand charges nothing and a lift
// that rounds — a far sketch-plane origin, a tilted frame, a placement —
// charges exactly what it rounded (docs/evaluator-design.md §8).
func (pp prismPayload) LiftedVertex(u, v, z float64) (r3.Vec, float64) {
	held := pp.Point(u, v, z)
	return held, exactPrismPointRound(pp, u, v, z, held)
}

func vecL1(v r3.Vec) float64 {
	return proofbound.AbsSumUpper(v.X, v.Y, v.Z)
}

// prismCentroidGeometryBound is a second, formula-independent proof. A solid's
// centroid is a convex combination of its material points, so it lies within
// the outer prism. The L1 envelope below bounds every such point through the
// frame and rigid placement, and therefore bounds the distance from held.
//
// It reads coordUpper through momentinput.CoordinateEnvelope, never
// momentinput.CoordinateUpper: this proof needs a coordinate MAGNITUDE envelope,
// never a placed cap frame, and every walk kind states one — a free-form
// span's own convex-hull envelope (freeform.FreeformControlExtent) included — so the
// analytic-only refusal CoordinateUpper carries for its OTHER callers
// (internal/capband/coordinate_upper.go, revolve.go) would refuse a centroid this build must
// publish for a section this same build just proved buildable.
//
// walks is the profile's pre-resolved segment walks, or nil; same contract as
// CoordinateEnvelope's own.
func prismCentroidGeometryBound(pp prismPayload, profile profileRecord, held r3.Vec, work *freeform.FreeformWork, walks *momentinput.ProfileWalks) (float64, error) {
	coordUpper, err := momentinput.CoordinateEnvelope(profile, work, walks)
	if err != nil {
		return 0, err
	}
	zUpper := math.Max(math.Abs(pp.Z0), math.Abs(pp.Z1))
	frameUpper := proofbound.AbsSumUpper(
		vecL1(pp.Frame.Origin()),
		proofbound.ProductUpper(vecL1(pp.Frame.U()), coordUpper),
		proofbound.ProductUpper(vecL1(pp.Frame.V()), coordUpper),
		proofbound.ProductUpper(vecL1(pp.Frame.N()), zUpper),
	)
	// A rigid map has each output coordinate bounded by the input L1 norm.
	placedUpper := proofbound.AbsSumUpper(proofbound.ProductUpper(3, frameUpper), vecL1(pp.Xform.Translation()))
	return proofbound.AbsSumUpper(vecL1(held), placedUpper), nil
}

// Dir places a plane-local direction (du, dv, dz in frame coordinates) into
// world space.
func (pp prismPayload) Dir(du, dv, dz float64) r3.Vec {
	world := pp.Frame.U().Scale(du).Add(pp.Frame.V().Scale(dv)).Add(pp.Frame.N().Scale(dz))
	return pp.Xform.ApplyDir(world)
}

// Reflected reports whether the accumulated placement flips handedness — a
// reflected solid's face normals and arc senses invert with it, and every
// orientation decision below corrects for it.
func (pp prismPayload) Reflected() bool { return pp.Xform.IsReflection() }

// transform is the accumulated rigid placement.
func (pp prismPayload) transform() r3.Transform { return pp.Xform }

// placed re-evaluates the same record under the composed motion. It is a
// re-evaluation path: no moments preflight has run on this record within the
// call, so the build opens the record's one counter itself.
//
// The motion is the ONLY thing it changes. Everything the payload carries about
// the section — the record, its displacement, the blend roles and the walk
// resolution — travels unchanged, which is what lets the build read the walks
// back instead of bracketing every free-form arc a second time. That counter is
// still charged what the resolution cost, so a record near its ceiling refuses
// here exactly as it did on the way in (docs/spline-design.md §5.2).
func (pp prismPayload) placed(ctx context.Context, d *Document, ref producerID, composed r3.Transform) (*Body, error) {
	pp.Xform = composed
	return evalPrismContext(ctx, d, ref, pp, freeform.NewFreeformWork())
}
