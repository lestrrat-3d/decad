package decad

import (
	"context"
	"math"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/massmoment"
	"github.com/lestrrat-3d/decad/internal/momentinput"

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

// prismPayload is the evaluator's own record of a prism body: the recorded
// region, the plane frame it lifts through, the signed sweep interval, and
// the accumulated rigid placement. Every measurement and the whole topology
// derive from it, which is what makes Placed exact: it re-evaluates the same
// payload under the composed motion (docs/evaluator-design.md §8).
//
// A prism-derived modify body (a filleted or chamfered prism, modify §2) also
// carries the blend-role descriptors of its own rewritten section: blendSegs
// names, per loop, the (loop, segment) indices whose side face is a blend wall,
// and blendKind is "fillet" or "chamfer". They are part of the re-evaluable
// record so a copy or placement re-mints its own blend roles from its own record
// (the modify §9 role rule) — a plain extrude leaves them empty (no-op).
//
// surfaceResult is WithSurfaceResult's own flag (docs/surface-design.md §4):
// true when the build must omit its closing caps and publish a sheet instead
// of a solid. It is part of the re-evaluable record for the same reason the
// blend descriptors are, so Placed, Duplicate and PlacedCopy reproduce the
// sheet with no further code — a plain solid extrude leaves it false.
//
// sectionDelta is the proven upper bound on how far any recorded boundary
// coordinate of the section sits from the section this payload's construction
// DENOTES (docs/prism-boolean-design.md §7). It is zero for every payload a
// caller draws directly — a plain extrude, a placement and every modify rewrite
// record their own coordinates, so the record IS the section they denote — and
// the analytic prism boolean is the one construction that sets it, to the
// displacement its coordinate re-expression commits. Being a payload field it
// re-evaluates with the payload, so a placement or copy keeps it, and every
// measurement evalPrism publishes composes it (never Exact while it is nonzero).
//
// z0Delta and z1Delta are the AXIAL twin of that displacement, one per end: each
// bounds how far the sweep level recorded beside it sits from the level this
// payload's construction denotes. A level the caller stated is its own denotation
// and carries zero — a Distance in millimetres records the number it was given —
// while a COMPUTED level carries the computation's own proven rounding: a ToFace
// or ThroughAll stop resolves its level by float arithmetic over another body's
// face (stops.go), a magnitude in a non-base unit rounds in its rescale to
// millimetres (magnitudeInBounded), and a chamfered end pulls its level in by the
// setback (capblend_moments.go). The two displacements are tracked apart and
// neither ever stands in for the other — sectionDelta moves a boundary coordinate
// IN the plane, these move a level ALONG the normal — while a reading both of them
// displace, a side vertex or the box, sums the two into its own bound. Every
// level-derived reading takes these: the sweep
// height and the volume, wall area and centroid built on it, the box, the side
// vertices and the vertical edge lengths. Being payload fields they re-evaluate
// with the payload, so a placement or copy keeps them.
//
// walks is the plane-local walk resolution of THIS payload's own profile,
// published by the evaluation that built the body and carried through every
// rigid re-evaluation of it (placed). It is a pure cache: it holds nothing the
// record does not already determine, nothing placement-dependent, and every
// read of it is guarded by momentinput.ProfileWalks.reusable, so a payload whose profile
// differs by one float bit resolves afresh. A payload a caller draws directly —
// a plain extrude, a modify rewrite, a boolean result — leaves it nil and
// resolves as before. See docs/evaluator-design.md §8.
type prismPayload struct {
	profile       ProfileRecord
	frame         r3.Frame
	z0, z1        float64
	z0Delta       float64
	z1Delta       float64
	xform         r3.Transform
	blendSegs     []map[int]struct{}
	blendKind     string
	sectionDelta  float64
	walks         *momentinput.ProfileWalks
	surfaceResult bool
}

// z0Scalar and z1Scalar are the sweep levels as bounded readings — the recorded
// float beside its own axial displacement. Every measurement derived from a
// level integrates these rather than the bare float, so a level the evaluator
// computed can never publish itself as the level it denotes.
func (pp prismPayload) z0Scalar() proofbound.BoundedScalar {
	return proofbound.MeasuredScalar(pp.z0, pp.z0Delta)
}

func (pp prismPayload) z1Scalar() proofbound.BoundedScalar {
	return proofbound.MeasuredScalar(pp.z1, pp.z1Delta)
}

// axialDelta is the larger of the two ends' displacements: the figure a reading
// that cannot attribute its error to one particular end takes.
func (pp prismPayload) axialDelta() float64 { return math.Max(pp.z0Delta, pp.z1Delta) }

// point lifts a plane-local (u, v) at height z into placed world space.
func (pp prismPayload) point(u, v, z float64) r3.Vec {
	local := pp.frame.ToWorldUV(u, v)
	n := pp.frame.N()
	return pp.xform.Apply(local.Add(n.Scale(z)))
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
	held := pp.point(u.Value, v.Value, z.Value)
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
	l, err := massmoment.PrismRotation(pp.frame, pp.xform)
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
	return proofbound.ExactFrameLiftRound(pp.frame, pp.xform, u, v, z, held)
}

// circleCurveBound is an Edge's curveBound for a rim circle or arc this
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
func (pp prismPayload) circleCurveBound(cu, cv, z, zDelta, centerDelta, radius, radiusBound float64, center, axis r3.Vec) (float64, bool) {
	l, err := massmoment.PrismRotation(pp.frame, pp.xform)
	if err != nil {
		return math.Inf(1), false
	}
	charge, err := massmoment.MapChargeOf(l)
	if err != nil {
		return math.Inf(1), false
	}
	displaced := proofbound.AbsSumUpper(zDelta, proofbound.ProductUpper(2, pp.sectionDelta), centerDelta)
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

// liftedVertex lifts a plane-local (u, v) at height z through pp.point and
// returns the held point beside the exact rounding that lift committed
// (exactPrismPointRound). Every analytic vertex a prism-shaped build places
// takes its frame and placement charge from here, one vertex at a time, so
// a lift that is exact for the coordinates at hand charges nothing and a lift
// that rounds — a far sketch-plane origin, a tilted frame, a placement —
// charges exactly what it rounded (docs/evaluator-design.md §8).
func (pp prismPayload) liftedVertex(u, v, z float64) (r3.Vec, float64) {
	held := pp.point(u, v, z)
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
func prismCentroidGeometryBound(pp prismPayload, profile ProfileRecord, held r3.Vec, work *freeform.FreeformWork, walks *momentinput.ProfileWalks) (float64, error) {
	coordUpper, err := momentinput.CoordinateEnvelope(profile, work, walks)
	if err != nil {
		return 0, err
	}
	zUpper := math.Max(math.Abs(pp.z0), math.Abs(pp.z1))
	frameUpper := proofbound.AbsSumUpper(
		vecL1(pp.frame.Origin()),
		proofbound.ProductUpper(vecL1(pp.frame.U()), coordUpper),
		proofbound.ProductUpper(vecL1(pp.frame.V()), coordUpper),
		proofbound.ProductUpper(vecL1(pp.frame.N()), zUpper),
	)
	// A rigid map has each output coordinate bounded by the input L1 norm.
	placedUpper := proofbound.AbsSumUpper(proofbound.ProductUpper(3, frameUpper), vecL1(pp.xform.Translation()))
	return proofbound.AbsSumUpper(vecL1(held), placedUpper), nil
}

// dir places a plane-local direction (du, dv, dz in frame coordinates) into
// world space.
func (pp prismPayload) dir(du, dv, dz float64) r3.Vec {
	world := pp.frame.U().Scale(du).Add(pp.frame.V().Scale(dv)).Add(pp.frame.N().Scale(dz))
	return pp.xform.ApplyDir(world)
}

// reflected reports whether the accumulated placement flips handedness — a
// reflected solid's face normals and arc senses invert with it, and every
// orientation decision below corrects for it.
func (pp prismPayload) reflected() bool { return pp.xform.IsReflection() }

// transform is the accumulated rigid placement.
func (pp prismPayload) transform() r3.Transform { return pp.xform }

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
	pp.xform = composed
	return evalPrismContext(ctx, d, ref, pp, freeform.NewFreeformWork())
}
