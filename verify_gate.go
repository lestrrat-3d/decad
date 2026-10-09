package decad

import (
	"context"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/diameter"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/tolerance"
	"github.com/lestrrat-3d/r3"
)

// This file proves the reference diameter Verify's tolerance gate is
// anchored on, per body and per payload.
//
// bodyGateDiameter is the entry point and states the order it tries: a
// payload that can state its own diameter does, a free-form prism section
// resolves one through freeformSectionGateDiameter, and fallbackGateDiameter
// is the last resort over the body's own vertices. A payload with no
// provable diameter returns false rather than a guess, which withholds the
// gate instead of anchoring it on a number nothing proved. See
// docs/verification-design.md §3.

// bodyGateDiameter returns the body's own diameter, never a document scale or
// a bounds-box diagonal — and returns it as a value proven to be at or below
// that diameter. Point witnesses publish through diameter.PointsWithBudget,
// and every arm charges each point it reads the gap a proof states between it
// and a point of the body; a chain revolve's circular edge uses its certified
// length interval.
// A Faceted body's cached value covers every held
// payload vertex, including vertices absent from the B-rep boundary loops. The
// analytic carrier model is built through the shared work budget (§7.2), so a
// cancelled Verify observes cancellation during the build instead of waiting for
// the whole model to finish. newBodyGeomBudget's payload switch is the
// clearance kernel's own — it only covers the payloads whose carrier model is
// an exact restatement of the shipped boundary, because clearance.go and
// interference.go trust that model for containment and contact proofs, not
// only for a diameter. A miss there is not necessarily a body with no usable
// diameter: a free-form-walled prismPayload — the one shipped payload whose
// side face the clearance kernel's exact model has no arm for at all — can read
// its diameter through freeformSectionGateDiameter's own witness set of
// analytic vertices and free-form span endpoints instead, tried first because
// its zero sectionDelta would otherwise read as "no arm" below. That arm
// WITHHOLDS its answer on each of the paths its own doc comment lists, and a
// body it withholds from falls through exactly like any other miss. Every miss
// reaches fallbackGateDiameter, which covers the payloads the exact model does
// not (cup, and a prismPayload whose own sectionDelta is nonzero) with a bound
// that is sound for THIS gate without being eligible for that stronger trust,
// and answers for nothing else — so a free-form-walled prismPayload whose own
// arm declined ends with no gate diameter at all.
//
// The exact model decides only which arm reads the body. Its carrier
// witnesses (CFace.Wit) are float samples whose gaps (clearance.Witness) are
// proven to the carriers, not to the body, and a placed body's carrier
// maximum reads above its own diameter, so no reading here takes them. A prism reads station witnesses along its walls
// (stationGateDiameter), each held within a proven gap of a point of the
// body, and shrinks their maximum by twice the widest gap plus its
// axialDelta. fallbackGateDiameter applies the same reading to the prisms it
// reads, over each one's own displacement. Every revolve, solid or sheet and
// whatever its section displacement, sweeps its meridian stations to the
// angles its farthest pair sits at (revolveGateDiameter) and compares each
// held point exactly against the point it denotes, the sweep angle's own
// displacement included. A cap-loop chamfer reads capBlendGateDiameter.
//
// A loftPayload reads its OWN held vertex-set diameter (diameter.PointsContext),
// never an envelope: the boundary is a polyhedron, and a convex-hull diameter
// is realized at vertices, so the vertex set's own maximum IS AT OR BELOW the
// body's true diameter — the strongest arm in this function, ahead of the
// exact carrier model that does not yet cover this payload class. For an
// unplaced LineSeg-only loft the claim is the stronger one, IS the true
// diameter, because every held vertex is then exact (docs/loft-design.md
// §5) — which is a claim about that payload's own published delta, never
// about a station's KIND, since a station can be a recorded coordinate and
// still sit off the point the record denotes (docs/loft-design.md §5.2's
// arc-end radial residual); a same-kind circular pairing's own interior stations are held on the
// true recorded curve but are themselves COMPUTED (a10-plan.md Part 3 PR 6),
// so the held maximum is still a chorded (equal-or-fewer, never additional)
// vertex set's own diameter over a set that sits ON the true boundary — at
// or below it, whether or not the body is placed. That weaker claim is what
// this arm always publishes, and it still fails safe: understating a
// diameter can only turn a passing reading into a false Suspect, never a
// false Sound.
//
// This arm's zero-subtraction fast path reads payload.delta == 0 exactly
// (loft_build.go's exact identity-transform-and-no-computed-station
// comparison, never a tolerance) and then reports the shared reader's
// answer UNCHANGED: no subtraction and no rounding of its own. What that
// answer is, is the reader's to state — the largest float64 at or below the
// held diameter, since the reader publishes every witness maximum rounded
// toward zero (diameter.PointsWithBudget) — so this arm publishes the
// tightest lower bound a float64 can carry on a diameter that is itself
// already at or below the true one. Subtracting a zero allowance on top of
// it would move that reading in exchange for nothing, which is why
// capBlendPayload.extentBoundedAlong's own `outward` helper (capblend.go)
// keeps a zero-displacement candidate untouched too. delta == 0 no longer
// implies the body is unplaced (a10-plan.md Part 3 PR 6): a curved pair
// chorded at ONE station (m = 1, docs/loft-design.md §12's m = 1 case) has
// no interior computed station either, so an UNPLACED body with such a pair
// can reach this same fast path with delta == 0 and an unshrunk reference.
// What makes that sound is the published ZERO and nothing else: at delta == 0
// every held vertex sits exactly at the point the record denotes for it, so
// the vertex set lies ON the true boundary and its maximum cannot exceed the
// true diameter. Being a RECORDED coordinate is not that premise and never
// stands in for it — an untrimmed ArcSeg's t == 1 end is recorded verbatim
// and still sits its own arc-end radial residual off the denoted curve,
// outward as easily as inward (docs/loft-design.md §5.2). Such a build
// publishes a positive delta and takes the shrink below, which is exactly how
// this arm sees the difference.
//
// A PLACED loft's held vertices are no
// longer provably exact (§12 PR 2a): the true diameter can differ from the
// held one by up to 2*delta (each of the two farthest points can sit up to
// delta from its true position), so this arm shrinks the held reading by
// 2*delta before reporting it, understating rather than overstating —
// tightening the gate can only turn a passing reading into a false Suspect,
// never a false Sound, the identical reasoning fallbackGateDiameter already
// carries. That direction is the SUBTRACTION's to lose: 2*delta is exact (a
// power-of-two scaling), so the difference is the one rounding here, and
// round-to-nearest can land it ABOVE the exact d - 2*delta — a reference
// larger than the one proven, which loosens the very gate this arm exists to
// tighten. freeform.DownRound (internal/freeform/spline_length.go, proofbound.UpRound's mirror) steps it back
// toward zero, so the published reference is at or below the exact shrunken
// value for every input rather than only for the ones whose subtraction
// happens to round down. A shrink that collapses to non-positive leaves the
// body with no usable diameter, exactly like any other unusable magnitude
// here.
func bodyGateDiameter(ctx context.Context, body *Body) (float64, bool, error) {
	if body == nil {
		return 0, false, nil
	}
	if payload, ok := body.payload.(facetedPayload); ok {
		return payload.diameter, tolerance.UsableMagnitude(payload.diameter), nil
	}
	if payload, ok := body.payload.(loftPayload); ok {
		d, ok, err := diameter.PointsContext(ctx, payload.verts)
		if err != nil || !ok {
			return d, ok, err
		}
		if payload.delta == 0 {
			return d, true, nil
		}
		d, ok = diameter.LowerForDisplacement(d, payload.delta)
		return d, ok, nil
	}
	if payload, ok := body.payload.(mitredSweepPayload); ok {
		// A mitred sweep's boundary is a polyhedron over its held vertex
		// table, each vertex within delta of the exact one, so the loft arm's
		// reading and shrink apply unchanged (docs/sweep-design.md §16.6).
		d, ok, err := diameter.PointsContext(ctx, payload.verts)
		if err != nil || !ok {
			return d, ok, err
		}
		d, ok = diameter.LowerForDisplacement(d, payload.delta)
		return d, ok, nil
	}
	if payload, ok := body.payload.(coilPayload); ok {
		// A coil's held shell is a polyhedron over its vertex table, each
		// vertex within delta of a true point, so the mitred arm's reading
		// and shrink apply unchanged (docs/helix-design.md §7).
		d, ok, err := diameter.PointsContext(ctx, payload.verts)
		if err != nil || !ok {
			return d, ok, err
		}
		d, ok = diameter.LowerForDisplacement(d, payload.delta)
		return d, ok, nil
	}
	if payload, ok := body.payload.(stitchPayload); ok {
		// Modelled on the loft arm immediately above: a stitched body's
		// boundary is a polyhedron over its own shared vertex table exactly
		// as a loft's is, so the same held-vertex-set diameter is a
		// certified LOWER bound on the body's true diameter, tightened by
		// the placement's own proven displacement.
		d, ok, err := diameter.PointsContext(ctx, payload.verts)
		if err != nil || !ok {
			return d, ok, err
		}
		if payload.delta == 0 {
			return d, true, nil
		}
		d, ok = diameter.LowerForDisplacement(d, payload.delta)
		return d, ok, nil
	}
	if payload, ok := body.payload.(chainPayload); ok {
		budget := proofbound.NewWorkBudget(ctx)
		g, ok, err := chainGatePoints(ctx, budget, body, payload)
		if err != nil || !ok {
			return 0, false, err
		}
		d, ok, err := g.Diameter(budget)
		return d, ok && d > 0, err
	}
	if payload, ok := body.payload.(draftPayload); ok {
		return draftGateDiameter(ctx, body, payload)
	}
	if _, ok := body.payload.(chainLoftPayload); ok {
		return chainVertexGateDiameter(ctx, body, 0)
	}
	if payload, ok := body.payload.(chainRevolvePayload); ok {
		return chainRevolveEdgeGateDiameter(ctx, body, payload.sectionDelta)
	}
	if payload, ok := body.payload.(brepPayload); ok {
		return brepGateDiameter(ctx, body, payload)
	}
	if _, ok := body.payload.(patchPayload); ok {
		// A patch has no exact carrier model of its own (it is a single flat
		// face, not a prism or a revolve), and no witness set gateWitnessPrism
		// reads either. Its own box is a proven LOWER bound on its diameter
		// instead: the largest single-axis extent of the body's own box is at
		// or below the true diameter (a diameter realizes as SOME pair's
		// distance, whose spread along at least one axis cannot exceed that
		// axis's own box extent), and diameter.LowerForDisplacement shrinks it
		// by the box's own proven Bound so the box's uncertainty can only
		// tighten the gate, never loosen it (docs/surface-design.md §5.1,
		// verification §3).
		box := body.bounds
		d := math.Max(box.Max.X-box.Min.X, math.Max(box.Max.Y-box.Min.Y, box.Max.Z-box.Min.Z))
		d, ok := diameter.LowerForDisplacement(d, box.Bound.Base())
		return d, ok, nil
	}
	if _, ok := body.payload.(bodyPatchPayload); ok {
		// Body.Patch's own payload reuses the patchPayload arm immediately
		// above verbatim: its Bounds is the rebuilt receiver's own box
		// (docs/surface-design.md §5.2), so the same box-based lower bound
		// applies for the same reason.
		box := body.bounds
		d := math.Max(box.Max.X-box.Min.X, math.Max(box.Max.Y-box.Min.Y, box.Max.Z-box.Min.Z))
		d, ok := diameter.LowerForDisplacement(d, box.Bound.Base())
		return d, ok, nil
	}
	budget := proofbound.NewWorkBudget(ctx)
	switch payload := body.payload.(type) {
	case revolvePayload:
		// Every revolve reads its meridian stations, solid or sheet, with or
		// without a section displacement: each point carries its own gap,
		// sectionDelta included, so the exact model's coverage decides nothing.
		return revolveGateDiameter(budget, payload)
	case capBlendPayload:
		return capBlendGateDiameter(ctx, budget, body, payload)
	}
	_, ok, err := newBodyGeomBudget(budget, body)
	if err != nil {
		return 0, false, err
	}
	if ok {
		if payload, isPrism := body.payload.(prismPayload); isPrism {
			return stationGateDiameter(budget, []prismPayload{payload}, payload.axialDelta())
		}
		return 0, false, nil
	}
	if payload, isPrism := body.payload.(prismPayload); isPrism {
		if d, ok, err := freeformSectionGateDiameter(ctx, payload); ok || err != nil {
			return d, ok, err
		}
	}
	return fallbackGateDiameter(budget, body)
}

// chainVertexGateDiameter reads only actual boundary vertices. Each builder
// publishes the displacement of its held vertex in Position().Bound; the
// supplied extra allowance covers section displacement and any endpoint
// rounding that the topology does not charge. The exact-down point-set reader
// and two-sided shrink keep the result below the denoted body's diameter.
func chainVertexGateDiameter(ctx context.Context, body *Body, extraAllow float64) (float64, bool, error) {
	if err := ctx.Err(); err != nil {
		return 0, false, err
	}
	vertices := body.Vertices()
	points := make([]r3.Vec, 0, len(vertices))
	maxBound := 0.0
	for _, vertex := range vertices {
		if err := ctx.Err(); err != nil {
			return 0, false, err
		}
		position := vertex.Position()
		bound := position.Bound.Base()
		if !proofbound.FiniteVec(position.Value) || !tolerance.UsableMagnitude(bound) {
			return 0, false, nil
		}
		points = append(points, position.Value)
		maxBound = math.Max(maxBound, bound)
	}
	d, ok, err := diameter.PointsContext(ctx, points)
	if err != nil || !ok {
		return d, ok, err
	}
	d, ok = diameter.LowerForDisplacement(d, proofbound.AbsSumUpper(maxBound, extraAllow))
	return d, ok && d > 0, nil
}

// A full-turn chain can expose only one seam vertex even though its circular
// rim has positive diameter. Every swept circle or connected circular arc of
// at most one turn has diameter at least its arc length divided by π: up to a
// half turn the endpoint chord proves it, and past a half turn the arc
// contains antipodal points. The edge's published length bound already
// includes its radius and angular uncertainty. Rational arithmetic rounds
// the resulting reference down exactly once.
func chainRevolveEdgeGateDiameter(ctx context.Context, body *Body, sectionDelta float64) (float64, bool, error) {
	if err := ctx.Err(); err != nil {
		return 0, false, err
	}
	for _, edge := range body.Edges() {
		if err := ctx.Err(); err != nil {
			return 0, false, err
		}
		switch edge.Curve().(type) {
		case Circle3, Arc3:
		default:
			continue
		}
		length, err := edge.Length()
		if err != nil {
			return 0, false, nil //nolint:nilerr // unbounded edge length withholds the reference
		}
		value, bound := length.Value.Base(), length.Bound.Base()
		if !tolerance.UsableMagnitude(value) || !tolerance.UsableMagnitude(bound) || value <= bound {
			continue
		}
		lengthLow := new(big.Rat).Sub(new(big.Rat).SetFloat64(value), new(big.Rat).SetFloat64(bound))
		denominator := new(big.Rat).SetFloat64(proofbound.TwoPiUpper())
		diameterLow := new(big.Rat).Quo(lengthLow.Mul(lengthLow, big.NewRat(2, 1)), denominator)
		d, ok := diameter.LowerForDisplacement(proofbound.RatFloatDown(diameterLow), sectionDelta)
		if ok && d > 0 {
			return d, true, nil
		}
	}
	return 0, false, nil
}

// freeformSectionGateDiameter is bodyGateDiameter's arm for a free-form-walled
// prismPayload (docs/verification-design.md §3): the clearance kernel's exact
// carrier model has no arm for a NURBSSurface side face any more than it has
// one for a displaced section, and gateWitnessPrism gives this payload none
// either, because its own sectionDelta reads zero — the one value that arm
// treats as "the section is already its own denotation, read newBodyGeomBudget
// instead". A free-form wall is not read through either model, so this
// function builds its own reference.
//
// It reports a certified LOWER bound on the body's own diameter: the maximum
// distance over a finite set of points KNOWN TO LIE ON the body — every
// analytic walk's own two endpoints, and every free-form span's own two
// endpoints (docs/spline-design.md §5.1's exact-rational Bézier conversion,
// never the recorded control net, and for a FitSplineSeg never the raw
// recorded Fit points, which are neither the converted chain's own ends nor a
// hull the curve stays inside) — at both cap heights. A Bézier interpolates
// its end control points exactly, so every span endpoint is a real point of
// the curve itself, and every distance the maximum ranges over is therefore
// realized between two real body points.
//
// That is the witness set's own half of the claim, and it is only half: a
// maximum over real body points is at or below the true diameter as a
// QUANTITY, while what this arm publishes is a float64. The other half belongs
// to the shared reader — diameter.PointsWithBudget computes the winning pair's
// distance over exact rationals and rounds it toward zero — so the published
// number is at or below that maximum too. Composed, the reading can only
// UNDERSTATE the true diameter and never overstate it, exactly the direction
// §3 requires; the displacement subtracted below only widens the
// understatement further.
//
// The displacement subtracted from that maximum composes three terms, none of
// them a certificate claim: the section's own sectionDelta (zero for a
// free-form wall in practice, since the analytic prism-boolean reduction
// never admits one — docs/prism-boolean-design.md's G4 — but read here rather
// than assumed), the payload's own axialDelta, and the widest per-witness
// gap: the endpoint bound of whichever walk produced it — an analytic walk's
// recorded-coordinate bound (zero for a whole segment, nonzero for a trimmed
// one) or a free-form span's own conversion rounding — carried through the
// frame and placement by prismPointBound, together with the lift's own
// rounding.
// Composing the widest witness bound as one uniform displacement, rather than
// a bound per point, is the same convention diameter.LowerForDisplacement's
// other callers already use: every witness pair is presumed to move by up to
// that much, so the subtracted amount is twice the WORST one, never a mix.
//
// This arm PUBLISHES only when its own witness conversion and the shared reader
// both succeed; otherwise it withholds the diameter outright and never
// substitutes a weaker one. This comment owns the complete list of the paths it
// withholds on — docs/verification-design.md §3 states the contract and points
// here rather than keeping a second copy:
//
//   - the profile carries no free-form segment at all, so this arm has nothing
//     to read the exact carrier model or gateWitnessPrism's own
//     displaced-section arm does not already read;
//   - a recorded segment normalizeSegment refuses, or walkOf refuses (an
//     R-table sentinel), so the section never becomes a walk at all;
//   - a free-form walk holds an empty Bézier span, or a span endpoint with no
//     finite float form (point2Of), so no witness can be placed on that curve;
//   - a witness's own endpoint bound cannot be derived (proofbound.WalkEndBoundAllow's
//     +Inf), since an absent bound must never read as a small one — this covers
//     an analytic walk's own two endpoints and a free-form span's alike;
//   - the shared reader declines the witness maximum (diameter.PointsWithBudget
//     answering ok=false: an empty set, a pair distance that is not a usable
//     magnitude, or a winning pair with no exact rational form);
//   - the displacement subtraction collapses the reading to non-positive
//     (diameter.LowerForDisplacement).
//
// A withheld answer is not rescued downstream. bodyGateDiameter falls through to
// fallbackGateDiameter, whose gateWitnessPrism has no arm for a prismPayload
// whose sectionDelta is zero, so the body ends with NO gate diameter and its
// bounded readings read Suspect. That is the sound direction to fail in — an
// absent reference admits nothing — but it is a real outcome of this arm, not
// one the arm's existence rules out.
//
// Every phase of this arm is cancellable, because neither of its two phases is
// bounded by a work counter of its own: the segment loop polls ctx before each
// segment, and the witness maximum polls it through diameter.PointsContext,
// the same reader the loftPayload arm above uses. That second poll is the one
// that matters for cost — the witness count grows with the profile's segment
// count (four points per segment), and the maximum is quadratic in it, so an unpolled scan is by
// far the longest thing a cancelled Verify could be left waiting on here. The
// resulting error is returned AS an error: cancellation is never folded into
// this arm's structural (0, false, nil) answer, which states only that the
// recorded section gives this arm nothing to read.
func freeformSectionGateDiameter(ctx context.Context, pp prismPayload) (float64, bool, error) {
	factor := prismLiftFactor(pp)
	lift := func(u, v, z float64, bound proofbound.WalkEndBound) (r3.Vec, float64) {
		gap := prismPointBoundWith(pp, factor, proofbound.MeasuredScalar(u, bound.U), proofbound.MeasuredScalar(v, bound.V),
			proofbound.MeasuredScalar(z, 0))
		return pp.point(u, v, z), gap
	}
	return diameter.FreeformSection(ctx, pp.profile.Outer, pp.profile.Holes,
		pp.z0, pp.z1, pp.sectionDelta, lift, pp.axialDelta)
}

// fallbackGateDiameter is bodyGateDiameter's fallback for a payload whose true
// boundary the clearance kernel's exact carrier model does not cover
// (cupPayload, capBlendPayload, and a prismPayload carrying a section
// displacement — verification design §3's "usable finite, non-negative body
// diameter", never a box diagonal, document scale or zero). A free-form-walled
// prismPayload misses that same exact carrier too, and bodyGateDiameter tries
// freeformSectionGateDiameter for it ahead of this function — but when that arm
// withholds its diameter the body DOES reach here, and this function has no arm
// for it either: its own sectionDelta is zero, the one value gateWitnessPrism
// below reads as "no arm at all" for a prismPayload. Such a body ends with no
// gate diameter and its bounded readings read Suspect.
//
// Each payload earns a witness prism for its own reason, and gateWitnessPrism's
// doc comment states each arm separately rather than pooling them behind one
// justification: a cap-loop chamfer never cuts past the receiver's own recorded
// walls, a cup's shell can ADD material (an outward shell), and a displaced
// section is no containing shape at all — it is the body's own recorded
// boundary, read within a proven displacement of the one it denotes. So what
// each arm proves has to be checked against the geometry it actually returns,
// not against "the receiver" as a stand-in for all three. For the two envelope
// arms the returned prism is a SHAPE that provably contains the true body, so
// the reduction itself can only overstate the true diameter, never understate
// it; the displacement subtracted below is what turns any of the three into the
// lower bound §3 requires.
//
// What this function reports is a reading of that geometry through
// stationGateDiameter: the station witnesses prismStationWitnesses places
// along every wall of every witness prism at both of its levels, each one
// charged with its own proven gap. A station is a point of the body only
// where the witness prism's walls run the full height between its two levels
// on the body. That holds for a cup's outer region, a stacked prism's outer
// runs, a displaced section and a sweep's witness prism (the straight prism
// itself, or the start section at one level). A cap-loop chamfer cuts the
// receiver's walls back at the cap levels, so its witness prisms
// (capBlendWitnessPrisms) stop each chamfered loop at its band's side level;
// bodyGateDiameter reads such a body through capBlendGateDiameter, which
// adds its cap circles and vertices to those stations.
//
// For an all-line section the reading reaches the farthest pair, which is
// realized at vertices. A circular wall that sweeps past 180 degrees holds a
// station opposite its start, so its own diameter 2R is read up to rounding
// and the stations' allowance. Between two different walls the farthest pair
// can fall between stations, but every point of a wall of radius R lies
// within 2R*sin(3.75 degrees) of one. An understated D tightens Ref and can
// turn a passing reading into a false Suspect, never a false Sound
// (verification design §3). The reading is built from the payload's own
// frame/xform, the same map a shipped prism's diameter is read through, so it
// carries none of the pose-dependence verification design §4 excludes an
// axis-aligned box for.
//
// A witness prism whose payload carries a displacement has the same
// held-witness issue as the exact prism path: each witness sits within that
// displacement of the point the payload denotes, so the held maximum can
// overstate the denoted body's diameter by twice it. The fallback shrinks the
// held witness maximum by that amount before it becomes a lower-bound
// reference, which is why gateWitnessPrisms hands back a displacement beside
// the prisms to read.
func fallbackGateDiameter(budget *proofbound.WorkBudget, body *Body) (float64, bool, error) {
	witnesses, displacement, ok := gateWitnessPrisms(body.payload)
	if !ok {
		return 0, false, nil
	}
	return stationGateDiameter(budget, witnesses, displacement)
}

// draftGateDiameter is bodyGateDiameter's arm for a draft body
// (docs/draft-design.md Table DD row DD6). The body has no exact carrier
// model here and no single witness prism: its far section is not the near
// one. It reads the stations sectionStations places on the near section P at
// the near level and on the far section Q, read from the payload's own
// far-section record, at the far level. Both are the boundaries of the
// body's two caps, so every station is a point of the body. A near station
// carries its own gap and the near level's displacement; a far station also
// carries farDelta, the displacement of the recorded far contour from the one
// the sweep and taper denote. Where the stations cannot be read, the arm
// reads the body's vertices with their published bounds instead.
func draftGateDiameter(ctx context.Context, body *Body, dp draftPayload) (float64, bool, error) {
	prisms, displacement := draftCapPrisms(dp)
	d, ok, err := stationGateDiameter(proofbound.NewWorkBudget(ctx), prisms, displacement)
	if err != nil || ok {
		return d, ok, err
	}
	return chainVertexGateDiameter(ctx, body, 0)
}

// draftCapPrisms is the near section at the near level and the recorded far
// section at the far level, each as a prism of zero height, beside the
// displacement their stations carry: the levels' and farDelta.
func draftCapPrisms(dp draftPayload) ([]prismPayload, float64) {
	nearZ, _, farZ, _ := dp.levels()
	level := func(profile profileRecord, z float64) prismPayload {
		return prismPayload{profile: profile, frame: dp.frame, z0: z, z1: z, xform: dp.xform}
	}
	return []prismPayload{level(dp.profile, nearZ), level(dp.far, farZ)}, proofbound.AbsSumUpper(dp.axialDelta(), dp.farDelta)
}

// gateWitnessPrism builds the straight prism fallbackGateDiameter reads its
// witnesses off, beside the displacement each of those witnesses can carry
// from the point of the denoted body it stands for. ok is false for every
// payload with no arm here, including a revolvePayload (already exact through
// newBodyGeomBudget, which is why it never reaches this fallback) and an
// analytic-walled prismPayload whose section is its own denotation (the same
// reason). A free-form-walled prismPayload's own section is its denotation
// too, so this switch answers false for it exactly as it does for the analytic
// case: bodyGateDiameter routes a free-form-walled prismPayload through
// freeformSectionGateDiameter before fallbackGateDiameter, and calls this
// function only when that arm has already declined — at which point this false
// answer is what leaves the body with no gate diameter at all.
//
// The three arms read different geometry and earn a witness for different
// reasons. A capBlendPayload reads one witness prism per loop instead
// (capBlendWitnessPrisms, through gateWitnessPrisms).
//
// cupPayload reads its view's outer, the cup's own outer region — the receiver's unmodified section
// for an INWARD shell, but the wider OFFSET (expanded) region for an OUTWARD
// one, since an outward shell adds material and cupPayloadFor
// (shell_cup.go) always assigns the wider of the two profiles to outer
// regardless of sense. Either way the whole cup body — walls, floor and
// cavity alike — sits inside pl.outer's own full-height prism: the cavity
// never reaches farther than the outer region, the same containment
// cupPayload.extentAlong already relies on. Its outer walls run the cup's
// full height, so every station on them is a point of the body. The
// receiver's own section is its own denotation, because every modify op
// refuses a receiver carrying a section displacement (fillet.go's
// requireExactSection), so an inward cup's witnesses carry only the axial
// displacement. An outward cup's outer region is the OFFSET one, recorded
// within the cup's offsetDelta of the region it denotes (shell_cup.go), so
// its witnesses carry that displacement beside the axial one, composed the
// way the displaced-prism arm below composes its own.
//
// A prismPayload whose own sectionDelta is nonzero (docs/prism-boolean-design.md
// §7's re-expressed or cut section — every analytic Union whose merge cut a
// wall, plus any placed prism pair) reads its OWN recorded section, and is not
// a containing shape at all: the denoted section may sit either side of the
// recorded one. It does not need to be. What this gate needs is a lower bound
// on the body's own diameter, and §7 proves every recorded boundary point sits
// within sectionDelta of the section the payload denotes, while each recorded
// level sits within axialDelta of the level it denotes. Those two displacements
// are perpendicular — one moves a coordinate IN the plane, the other moves a
// level ALONG the normal — so their sum is an upper bound on how far a lifted
// witness sits from the denoted body point below it, and diameter.LowerForDisplacement
// turns the held maximum into the lower bound the gate wants. The copy zeroes
// sectionDelta because addPrismFaces (clearance_geom.go) refuses a displaced
// section outright: it builds the clearance kernel's certificate carriers,
// which have to be exact statements about a boundary, and a witness set for a
// diameter is neither a certificate nor a carrier.
func gateWitnessPrism(payload featurePayload) (prismPayload, float64, bool) {
	switch pl := payload.(type) {
	case cupPayload:
		cup := pl.view()
		witness := cup.outerPrism()
		witness.profile = cup.outer
		displacement := proofbound.AbsSumUpper(witness.sectionDelta, witness.axialDelta())
		witness.sectionDelta = 0
		return witness, displacement, true
	case prismPayload:
		if pl.sectionDelta == 0 {
			return prismPayload{}, 0, false
		}
		displacement := proofbound.AbsSumUpper(pl.sectionDelta, pl.axialDelta())
		witness := pl
		witness.sectionDelta = 0
		return witness, displacement, true
	case sweepPayload:
		witness := pl.prism
		displacement := proofbound.AbsSumUpper(witness.sectionDelta, witness.axialDelta())
		witness.sectionDelta = 0
		return witness, displacement, true
	default:
		return prismPayload{}, 0, false
	}
}

// gateWitnessPrisms is gateWitnessPrism's reading for every payload. A stacked
// prism reads one witness prism per outer run (stackedPrismPayload.outerRuns)
// over that run's own interval. Every run's outer loop is the outer wall of
// the slabs it spans, so every witness is a point of the body. The first
// slab's outer swept over the whole stack would hold points a union-built
// stack does not reach, and would overstate its diameter. A capBlendPayload
// reads capBlendWitnessPrisms.
func gateWitnessPrisms(payload featurePayload) ([]prismPayload, float64, bool) {
	if pl, ok := payload.(capBlendPayload); ok {
		return capBlendWitnessPrisms(pl), pl.axialDelta(), true
	}
	if pl, ok := payload.(stackedPrismPayload); ok {
		runs := pl.outerRuns()
		for i := range runs {
			runs[i].sectionDelta = 0
		}
		return runs, proofbound.AbsSumUpper(pl.sectionDelta, pl.axialDelta()), true
	}
	witness, displacement, ok := gateWitnessPrism(payload)
	if !ok {
		return nil, 0, false
	}
	return []prismPayload{witness}, displacement, true
}

// capBlendWitnessPrisms lists one witness prism per loop of a cap-loop
// chamfer's receiver section, each over the interval its loop's own wall runs
// on the body. A chamfer band cuts its loop's wall back from the cap level to
// the side level, the cap level moved ds into the material
// (docs/modify-reach-design.md §8.3.1), and leaves the wall between the two
// side levels untouched. A loop chamfered on the start cap therefore starts
// at z0 + ds, one chamfered on the end cap stops at z1 - ds, and an
// unchamfered loop runs from z0 to z1. Those are the float sums the build
// places each band's side level at, and capBlendPayload.axialDelta bounds how
// far any of them sits from the level it denotes. The receiver's walls at the
// cap levels are not points of the body wherever a band cuts them, and a
// reading from them overstates the body's diameter: a 10 mm cube chamfered 2
// mm around its end cap is sqrt(264) across, against the receiver's
// sqrt(300).
func capBlendWitnessPrisms(pl capBlendPayload) []prismPayload {
	loops := pl.loops()
	out := make([]prismPayload, 0, len(loops))
	for li, loop := range loops {
		witness := pl.prismLike(pl.z0, pl.z1)
		witness.profile = profileRecord{Outer: loop}
		if pl.startLoops[li] {
			witness.z0 = pl.z0 + pl.start.ds
		}
		if pl.endLoops[li] {
			witness.z1 = pl.z1 - pl.end.ds
		}
		out = append(out, witness)
	}
	return out
}
