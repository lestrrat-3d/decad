package decad

import (
	"context"
	"math"
	"math/big"

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
type prismPayload struct {
	profile      ProfileRecord
	frame        r3.Frame
	z0, z1       float64
	z0Delta      float64
	z1Delta      float64
	xform        r3.Transform
	blendSegs    []map[int]struct{}
	blendKind    string
	sectionDelta float64
}

// z0Scalar and z1Scalar are the sweep levels as bounded readings — the recorded
// float beside its own axial displacement. Every measurement derived from a
// level integrates these rather than the bare float, so a level the evaluator
// computed can never publish itself as the level it denotes.
func (pp prismPayload) z0Scalar() boundedScalar { return measuredScalar(pp.z0, pp.z0Delta) }

func (pp prismPayload) z1Scalar() boundedScalar { return measuredScalar(pp.z1, pp.z1Delta) }

// axialDelta is the larger of the two ends' displacements: the figure a reading
// that cannot attribute its error to one particular end takes.
func (pp prismPayload) axialDelta() float64 { return math.Max(pp.z0Delta, pp.z1Delta) }

// point lifts a plane-local (u, v) at height z into placed world space.
func (pp prismPayload) point(u, v, z float64) r3.Vec {
	local := pp.frame.ToWorldUV(u, v)
	n := pp.frame.N()
	return pp.xform.Apply(local.Add(n.Scale(z)))
}

// prismPointBound carries plane-local coordinate error through the isometric
// frame/placement and charges the float operations that evaluate both maps.
func prismPointBound(pp prismPayload, u, v, z boundedScalar) float64 {
	source := radius3D(max(u.bound, v.bound, z.bound))
	held := pp.point(u.value, v.value, z.value)
	round := exactPrismPointRound(pp, u.value, v.value, z.value, held)
	// Frame and transform constructors enforce near-orthonormal maps. A factor
	// four safely carries the source ball through both held linear maps.
	return absSumUpper(productUpper(4, source), round)
}

func vecMaxAbs(v r3.Vec) float64 {
	return max(math.Abs(v.X), math.Abs(v.Y), math.Abs(v.Z))
}

func exactPrismPointRound(pp prismPayload, u, v, z float64, held r3.Vec) float64 {
	ratVec := func(x, y, z *big.Rat) [3]*big.Rat { return [3]*big.Rat{x, y, z} }
	ratOfVec := func(value r3.Vec) [3]*big.Rat {
		return ratVec(floatRat(value.X), floatRat(value.Y), floatRat(value.Z))
	}
	origin := ratOfVec(pp.frame.Origin())
	fu, fv, fn := ratOfVec(pp.frame.U()), ratOfVec(pp.frame.V()), ratOfVec(pp.frame.N())
	ru, rv, rz := floatRat(u), floatRat(v), floatRat(z)
	if ru == nil || rv == nil || rz == nil {
		return math.Inf(1)
	}
	local := [3]*big.Rat{}
	for i := range local {
		if origin[i] == nil || fu[i] == nil || fv[i] == nil || fn[i] == nil {
			return math.Inf(1)
		}
		local[i] = ratAdd(
			origin[i],
			ratMul(fu[i], ru),
			ratMul(fv[i], rv),
			ratMul(fn[i], rz),
		)
	}
	basis := pp.xform.Basis()
	ex, ey, ez := ratOfVec(basis.EX), ratOfVec(basis.EY), ratOfVec(basis.EZ)
	translation := ratOfVec(pp.xform.Translation())
	exact := [3]*big.Rat{}
	for i := range exact {
		if ex[i] == nil || ey[i] == nil || ez[i] == nil || translation[i] == nil {
			return math.Inf(1)
		}
		exact[i] = ratAdd(
			ratMul(ex[i], local[0]),
			ratMul(ey[i], local[1]),
			ratMul(ez[i], local[2]),
			translation[i],
		)
	}
	perCoord := max(
		rationalFloatError(exact[0], held.X),
		rationalFloatError(exact[1], held.Y),
		rationalFloatError(exact[2], held.Z),
	)
	return radius3D(perCoord)
}

func vecL1(v r3.Vec) float64 {
	return absSumUpper(v.X, v.Y, v.Z)
}

// walks is the profile's pre-resolved segment walks (docs/spline-design.md
// §5.2, this file's profileWalks doc comment), or nil to resolve each segment
// through walkOf as before. A non-nil walks that was not resolved from THIS
// profile — the recorded segments compared, not their count — is a plumbing
// bug and refuses rather than silently resolving anyway.
func profileCoordinateUpper(profile ProfileRecord, work *freeformWork, walks *profileWalks) (float64, error) {
	if walks != nil && !walks.matches(profile) {
		return 0, errResolvedWalksMismatch
	}
	upper := 0.0
	for li, loop := range append([]LoopRecord{profile.Outer}, profile.Holes...) {
		for si, seg := range loop.Segments {
			w, err := resolveOrRead(seg, work, walks, li, si)
			if err != nil {
				return 0, err
			}
			if err := requireAnalyticWalk(w, "a placed cap frame"); err != nil {
				return 0, err
			}
			upper = math.Max(upper, w.coordUpper)
		}
	}
	return upper, nil
}

// profileCoordinateEnvelope is profileCoordinateUpper without the analytic
// requirement: every walk kind, free-form included, states its own coordUpper
// (walkOf's per-kind construction — freeformControlExtent for a free-form
// span), so a caller that only needs a coordinate MAGNITUDE envelope — never a
// placed cap frame, which genuinely cannot represent a free-form wall — reads
// it directly rather than refusing on a section the caller's own reading
// already handles: extentBoundedAlong's boundary-extreme scan, and
// prismCentroidGeometryBound's convex-combination proof, each state their own
// account of a free-form span and need only the envelope beside it.
//
// walks is profileCoordinateUpper's own optional pre-resolved set, same
// contract: nil resolves as before, a non-matching non-nil set refuses.
func profileCoordinateEnvelope(profile ProfileRecord, work *freeformWork, walks *profileWalks) (float64, error) {
	if walks != nil && !walks.matches(profile) {
		return 0, errResolvedWalksMismatch
	}
	upper := 0.0
	for li, loop := range append([]LoopRecord{profile.Outer}, profile.Holes...) {
		for si, seg := range loop.Segments {
			w, err := resolveOrRead(seg, work, walks, li, si)
			if err != nil {
				return 0, err
			}
			upper = math.Max(upper, w.coordUpper)
		}
	}
	return upper, nil
}

// resolveOrRead is the shared "read the pre-resolved walk, or resolve one"
// step every profileWalks-aware consumer in this file uses: walks non-nil
// (and already checked against the profile by the caller) reads
// walks.at(loopIndex, segIndex); walks nil calls walkOf, exactly as every
// consumer did before profileWalks existed.
func resolveOrRead(seg CurveSegment, work *freeformWork, walks *profileWalks, loopIndex, segIndex int) (segmentWalk, error) {
	if walks != nil {
		return walks.at(loopIndex, segIndex), nil
	}
	return walkOf(seg, work)
}

// prismCentroidGeometryBound is a second, formula-independent proof. A solid's
// centroid is a convex combination of its material points, so it lies within
// the outer prism. The L1 envelope below bounds every such point through the
// frame and rigid placement, and therefore bounds the distance from held.
//
// It reads coordUpper through profileCoordinateEnvelope, never
// profileCoordinateUpper: this proof needs a coordinate MAGNITUDE envelope,
// never a placed cap frame, and every walk kind states one — a free-form
// span's own convex-hull envelope (freeformControlExtent) included — so the
// analytic-only refusal profileCoordinateUpper carries for its OTHER callers
// (capblend_centroid.go, revolve.go) would refuse a centroid this build must
// publish for a section this same build just proved buildable.
//
// walks is the profile's pre-resolved segment walks, or nil; same contract as
// profileCoordinateEnvelope's own.
func prismCentroidGeometryBound(pp prismPayload, profile ProfileRecord, held r3.Vec, work *freeformWork, walks *profileWalks) (float64, error) {
	coordUpper, err := profileCoordinateEnvelope(profile, work, walks)
	if err != nil {
		return 0, err
	}
	zUpper := math.Max(math.Abs(pp.z0), math.Abs(pp.z1))
	frameUpper := absSumUpper(
		vecL1(pp.frame.Origin()),
		productUpper(vecL1(pp.frame.U()), coordUpper),
		productUpper(vecL1(pp.frame.V()), coordUpper),
		productUpper(vecL1(pp.frame.N()), zUpper),
	)
	// A rigid map has each output coordinate bounded by the input L1 norm.
	placedUpper := absSumUpper(productUpper(3, frameUpper), vecL1(pp.xform.Translation()))
	return absSumUpper(vecL1(held), placedUpper), nil
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
func (pp prismPayload) placed(ctx context.Context, d *Document, ref StepRef, composed r3.Transform) (*Body, error) {
	pp.xform = composed
	return evalPrismContext(ctx, d, ref, pp, newFreeformWork())
}
