package prismrecord

import (
	"context"
	"math"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/massmoment"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismplacement"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// This file holds the coordinate readings taken directly off the record: a
// point in world space, the proven bound on that point, and the profile-wide
// coordinate envelopes every later bound is charged against.
//
// The record holds a RECORDED section and two RECORDED sweep levels, each with
// its own proven displacement from what the construction denotes
// (SectionDelta, Z0Delta/Z1Delta). Every reader here composes the displacement
// its own answer depends on rather than reading the record as exact. See
// agent-docs/evaluator-design.md §5 and agent-docs/prism-boolean-design.md §7.

// Z0Scalar and Z1Scalar are the sweep levels as bounded readings — the recorded
// float beside its own axial displacement. Every measurement derived from a
// level integrates these rather than the bare float, so a level the evaluator
// computed can never publish itself as the level it denotes.
func (pp Payload) Z0Scalar() proofbound.BoundedScalar {
	return proofbound.MeasuredScalar(pp.Z0, pp.Z0Delta)
}

// Z1Scalar is Z0Scalar for the far level.
func (pp Payload) Z1Scalar() proofbound.BoundedScalar {
	return proofbound.MeasuredScalar(pp.Z1, pp.Z1Delta)
}

// Point lifts a plane-local (u, v) at height z into placed world space.
func (pp Payload) Point(u, v, z float64) r3.Vec {
	local := pp.Frame.ToWorldUV(u, v)
	n := pp.Frame.N()
	return pp.Xform.Apply(local.Add(n.Scale(z)))
}

// PointBound carries plane-local coordinate error through the
// frame/placement and charges the float operations that evaluate both maps.
// The source error is a ball of radius Radius3D of the largest coordinate
// bound, and the lift moves it by the held linear map
// L = placement basis · [U V N] (massmoment.PrismRotation), which stretches
// it by at most LiftFactor; the exact rounding pp.Point commits is
// charged beside it.
func (pp Payload) PointBound(u, v, z proofbound.BoundedScalar) float64 {
	return pp.PointBoundWith(pp.LiftFactor(), u, v, z)
}

// PointBoundWith is PointBound with the lift factor read once by a
// caller that lifts many points through the same payload.
func (pp Payload) PointBoundWith(factor float64, u, v, z proofbound.BoundedScalar) float64 {
	source := proofbound.Radius3D(max(u.Bound, v.Bound, z.Bound))
	held := pp.Point(u.Value, v.Value, z.Value)
	round := pp.PointRound(u.Value, v.Value, z.Value, held)
	return proofbound.AbsSumUpper(proofbound.ProductUpper(factor, source), round)
}

// LiftFactor bounds how far pp's held lift stretches a plane-local
// displacement: 1 + e, rounded up, with e the orthonormality defect of the
// held linear map L, the entrywise absolute sum of LᵀL − I
// (massmoment.MapChargeOf's Stretch). Every eigenvalue of LᵀL lies in
// [1 − e, 1 + e], so |L·d| ≤ √(1 + e)·|d| ≤ (1 + e)·|d|. An exactly
// orthonormal L, an axis-aligned frame without placement among them, answers
// exactly 1. A map that is not finite, or whose defect reaches 1/2, answers
// +Inf.
func (pp Payload) LiftFactor() float64 {
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

// PointRound is proofbound.ExactFrameLiftRound read through pp's own
// frame and accumulated placement: the exact rounding pp.Point committed
// lifting (u, v, z) to held.
func (pp Payload) PointRound(u, v, z float64, held r3.Vec) float64 {
	return proofbound.ExactFrameLiftRound(pp.Frame, pp.Xform, u, v, z, held)
}

// CircleCurveBound is an Edge's curveBound for a rim circle or arc this
// payload lifts from the recorded centre (cu, cv) at level z, held with
// centre at center, its axis along axis, and the walk's radius, which sits within
// radiusBound of the radius the record denotes. The denoted curve is the
// recorded circle carried through L = B·[U V N] (massmoment.PrismRotation),
// displaced by the level's own zDelta along L·N, by the section's own
// SectionDelta in the plane, and by centerDelta, how far the denoted centre
// sits from (cu, cv) in the plane. Its distance from the held circle is at most
// the centre's exact lift rounding, massmoment.CircleImageGap's in-plane and
// tilt terms, and those two displacements stretched by L. It answers +Inf,
// and ok false, when that bound is not below half the radius, where radial
// projection onto the held circle stops being continuous.
func (pp Payload) CircleCurveBound(cu, cv, z, zDelta, centerDelta, radius, radiusBound float64, center, axis r3.Vec) (float64, bool) {
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
		pp.PointRound(cu, cv, z, center),
		massmoment.CircleImageGap(l, charge.Stretch, axis, radius, radiusBound),
		proofbound.ProductUpper(proofbound.AbsSumUpper(1, charge.Stretch), displaced),
	)
	if proofbound.IsNonFinite(bound) || !(proofbound.ProductUpper(2, bound) < radius) {
		return math.Inf(1), false
	}
	return bound, true
}

// LiftedVertex lifts a plane-local (u, v) at height z through pp.Point and
// returns the held point beside the exact rounding that lift committed
// (PointRound). Every analytic vertex a prism-shaped build places
// takes its frame and placement charge from here, one vertex at a time, so
// a lift that is exact for the coordinates at hand charges nothing and a lift
// that rounds — a far sketch-plane origin, a tilted frame, a placement —
// charges exactly what it rounded (agent-docs/evaluator-design.md §8).
func (pp Payload) LiftedVertex(u, v, z float64) (r3.Vec, float64) {
	held := pp.Point(u, v, z)
	return held, pp.PointRound(u, v, z, held)
}

// LiftedVertexReplay charges the exact frame lift before calculating the
// same held point and bound that LiftedVertex returns.
func (pp Payload) LiftedVertexReplay(ctx context.Context, u, v, z float64,
	budget *proofbound.ReplayBudget) (r3.Vec, float64, error) {
	return proofbound.ExactFrameLiftRoundReplay(ctx, pp.Frame, pp.Xform, u, v, z, budget)
}

func vecL1(v r3.Vec) float64 {
	return proofbound.AbsSumUpper(v.X, v.Y, v.Z)
}

// CentroidGeometryBound is a second, formula-independent proof. A solid's
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
func (pp Payload) CentroidGeometryBound(profile momentinput.Profile, held r3.Vec, work *freeform.FreeformWork, walks *momentinput.ProfileWalks) (float64, error) {
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
func (pp Payload) Dir(du, dv, dz float64) r3.Vec {
	world := pp.Frame.U().Scale(du).Add(pp.Frame.V().Scale(dv)).Add(pp.Frame.N().Scale(dz))
	return pp.Xform.ApplyDir(world)
}

// Reflected reports whether the accumulated placement flips handedness — a
// reflected solid's face normals and arc senses invert with it, and every
// orientation decision below corrects for it.
func (pp Payload) Reflected() bool { return pp.Xform.IsReflection() }

// PlacementOperand is the record's frame, placement and sweep levels, with
// each level's displacement and denotation, as the prismplacement operand the
// analytic prism gates compare.
func (pp Payload) PlacementOperand() prismplacement.Operand {
	return prismplacement.Operand{Frame: pp.Frame, Xform: pp.Xform, Z0: pp.Z0, Z1: pp.Z1,
		Z0Delta: pp.Z0Delta, Z1Delta: pp.Z1Delta, Z0Denoted: pp.Z0Denoted, Z1Denoted: pp.Z1Denoted}
}
