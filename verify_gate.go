package decad

import (
	"context"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/diameter"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/survey2d"
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
// that diameter. Point witnesses publish through pointSetDiameterWithBudget;
// a chain revolve's circular edge uses its certified length interval.
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
// not (cup, cap-loop chamfer, and a prismPayload whose own sectionDelta is
// nonzero) with a bound that is sound for THIS gate without being eligible
// for that stronger trust, and answers for nothing else — so a free-form-walled
// prismPayload whose own arm declined ends with no gate diameter at all.
//
// A prism with nonzero z0Delta or z1Delta keeps the same carrier model, but
// each held witness can move by axialDelta. The maximum held pair distance can
// therefore overstate the denoted body's diameter by twice that displacement.
// This function shrinks it toward zero before using it as a reference, so the
// result can only tighten the gate. fallbackGateDiameter applies the same
// correction to the prisms it reads, over each one's own displacement. Both
// also read station witnesses along every prism wall (stationGateDiameter),
// because the carrier witnesses miss a circular wall's farthest pair. A
// revolve takes the same shrink over its own angular displacement
// (revolvePayload.angularDelta, docs/evaluator-design.md §6), scaled by the
// radial envelope every witness can carry it at, since a held cap witness
// moves along a circle of that radius rather than along a straight axis.
//
// A loftPayload reads its OWN held vertex-set diameter (pointSetDiameterContext),
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
// toward zero (pointSetDiameterWithBudget) — so this arm publishes the
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
		d, ok, err := pointSetDiameterContext(ctx, payload.verts)
		if err != nil || !ok {
			return d, ok, err
		}
		if payload.delta == 0 {
			return d, true, nil
		}
		d, ok = lowerDiameterForDisplacement(d, payload.delta)
		return d, ok, nil
	}
	if payload, ok := body.payload.(mitredSweepPayload); ok {
		// A mitred sweep's boundary is a polyhedron over its held vertex
		// table, each vertex within delta of the exact one, so the loft arm's
		// reading and shrink apply unchanged (docs/sweep-design.md §16.6).
		d, ok, err := pointSetDiameterContext(ctx, payload.verts)
		if err != nil || !ok {
			return d, ok, err
		}
		d, ok = lowerDiameterForDisplacement(d, payload.delta)
		return d, ok, nil
	}
	if payload, ok := body.payload.(coilPayload); ok {
		// A coil's held shell is a polyhedron over its vertex table, each
		// vertex within delta of a true point, so the mitred arm's reading
		// and shrink apply unchanged (docs/helix-design.md §7).
		d, ok, err := pointSetDiameterContext(ctx, payload.verts)
		if err != nil || !ok {
			return d, ok, err
		}
		d, ok = lowerDiameterForDisplacement(d, payload.delta)
		return d, ok, nil
	}
	if payload, ok := body.payload.(stitchPayload); ok {
		// Modelled on the loft arm immediately above: a stitched body's
		// boundary is a polyhedron over its own shared vertex table exactly
		// as a loft's is, so the same held-vertex-set diameter is a
		// certified LOWER bound on the body's true diameter, tightened by
		// the placement's own proven displacement.
		d, ok, err := pointSetDiameterContext(ctx, payload.verts)
		if err != nil || !ok {
			return d, ok, err
		}
		if payload.delta == 0 {
			return d, true, nil
		}
		d, ok = lowerDiameterForDisplacement(d, payload.delta)
		return d, ok, nil
	}
	if payload, ok := body.payload.(chainPayload); ok {
		endpointAllow, ok, err := chainWalkEndpointAllow(ctx, payload.chains)
		if err != nil || !ok {
			return 0, false, err
		}
		return chainVertexGateDiameter(ctx, body, proofbound.AbsSumUpper(payload.sectionDelta, endpointAllow))
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
		// axis's own box extent), and lowerDiameterForDisplacement shrinks it
		// by the box's own proven Bound so the box's uncertainty can only
		// tighten the gate, never loosen it (docs/surface-design.md §5.1,
		// verification §3).
		box := body.bounds
		d := math.Max(box.Max.X-box.Min.X, math.Max(box.Max.Y-box.Min.Y, box.Max.Z-box.Min.Z))
		d, ok := lowerDiameterForDisplacement(d, box.Bound.Base())
		return d, ok, nil
	}
	if _, ok := body.payload.(bodyPatchPayload); ok {
		// Body.Patch's own payload reuses the patchPayload arm immediately
		// above verbatim: its Bounds is the rebuilt receiver's own box
		// (docs/surface-design.md §5.2), so the same box-based lower bound
		// applies for the same reason.
		box := body.bounds
		d := math.Max(box.Max.X-box.Min.X, math.Max(box.Max.Y-box.Min.Y, box.Max.Z-box.Min.Z))
		d, ok := lowerDiameterForDisplacement(d, box.Bound.Base())
		return d, ok, nil
	}
	budget := proofbound.NewWorkBudget(ctx)
	geom, ok, err := newBodyGeomBudget(budget, body)
	if err != nil {
		return 0, false, err
	}
	if ok {
		d, ok := pointSetDiameter(geom.supports)
		if !ok {
			return d, false, nil
		}
		if payload, isPrism := body.payload.(prismPayload); isPrism {
			d, ok = lowerDiameterForDisplacement(d, payload.axialDelta())
			return stationGateDiameter(budget, d, ok, geom.supports, []prismPayload{payload}, payload.axialDelta())
		}
		if payload, isRevolve := body.payload.(revolvePayload); isRevolve {
			coordUpper, err := profileCoordinateUpper(payload.profile, freeform.NewFreeformWork(), nil)
			if err != nil {
				return 0, false, err
			}
			rhoUpper := payload.ax.radialUpper(coordUpper)
			d, ok = lowerDiameterForDisplacement(d, proofbound.ProductUpper(rhoUpper, payload.angularDelta()))
		}
		return d, ok, nil
	}
	if payload, isPrism := body.payload.(prismPayload); isPrism {
		if d, ok, err := freeformSectionGateDiameter(ctx, payload); ok || err != nil {
			return d, ok, err
		}
	}
	return fallbackGateDiameter(budget, body)
}

// chainWalkEndpointAllow covers the computed walk endpoint coordinates not
// present in ExtrudeChain's topology-vertex bounds for analytic segments.
// It reads every source segment, including those later coalesced into one
// wall, so the result also covers a coalesced line's last endpoint.
func chainWalkEndpointAllow(ctx context.Context, chains []ChainRecord) (float64, bool, error) {
	work := freeform.NewFreeformWork()
	allow := 0.0
	for _, chain := range chains {
		for _, segment := range chain.Segments {
			if err := ctx.Err(); err != nil {
				return 0, false, err
			}
			walk, err := boundarywalk.WalkOf(segment, work)
			if err != nil {
				return 0, false, nil //nolint:nilerr // structural walk refusal withholds the reference
			}
			for _, bound := range [2]proofbound.WalkEndBound{walk.StartBound, walk.EndBound} {
				endAllow := proofbound.WalkEndBoundAllow(bound)
				if !tolerance.UsableMagnitude(endAllow) {
					return 0, false, nil
				}
				allow = math.Max(allow, endAllow)
			}
			// An ArcSeg's recorded natural end can sit off the radius its
			// denoted circle reads from Start, even when proofbound.WalkEndBound is zero.
			residual := loftmesh.ArcNaturalEndRadialUpper(segment)
			if !tolerance.UsableMagnitude(residual) {
				return 0, false, nil
			}
			allow = math.Max(allow, residual)
		}
	}
	return allow, true, ctx.Err()
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
	d, ok, err := pointSetDiameterContext(ctx, points)
	if err != nil || !ok {
		return d, ok, err
	}
	d, ok = lowerDiameterForDisplacement(d, proofbound.AbsSumUpper(maxBound, extraAllow))
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
		d, ok := lowerDiameterForDisplacement(proofbound.RatFloatDown(diameterLow), sectionDelta)
		if ok && d > 0 {
			return d, true, nil
		}
	}
	return 0, false, nil
}

// lowerDiameterForDisplacement adapts the held witness reading to
// internal/diameter's displacement adjustment.
func lowerDiameterForDisplacement(d, displacement float64) (float64, bool) {
	return diameter.LowerForDisplacement(d, displacement)
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
// to the shared reader — pointSetDiameterWithBudget computes the winning pair's
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
// endpoint bound proofbound.WalkEndBoundAllow reads off whichever walk produced it — an
// analytic walk's recorded-coordinate bound (zero for a whole segment,
// nonzero for a trimmed one) or a free-form span's own conversion rounding.
// Composing the widest witness bound as one uniform displacement, rather than
// a bound per point, is the same convention lowerDiameterForDisplacement's
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
//   - the shared reader declines the witness maximum (pointSetDiameterWithBudget
//     answering ok=false: an empty set, a pair distance that is not a usable
//     magnitude, or a winning pair with no exact rational form);
//   - the displacement subtraction collapses the reading to non-positive
//     (lowerDiameterForDisplacement).
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
// segment, and the witness maximum polls it through pointSetDiameterContext,
// the same reader the loftPayload arm above uses. That second poll is the one
// that matters for cost — the witness count grows with the profile's segment
// count (four points per segment), and the maximum is quadratic in it, so an unpolled scan is by
// far the longest thing a cancelled Verify could be left waiting on here. The
// resulting error is returned AS an error: cancellation is never folded into
// this arm's structural (0, false, nil) answer, which states only that the
// recorded section gives this arm nothing to read.
func freeformSectionGateDiameter(ctx context.Context, pp prismPayload) (float64, bool, error) {
	return diameter.FreeformSection(ctx, pp.profile.Outer, pp.profile.Holes,
		pp.z0, pp.z1, pp.sectionDelta, pp.point, pp.axialDelta)
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
// What this function reports is a reading of that geometry, taken through
// the same two readings the exact prism path above takes. The first is the
// maximum over the carrier witnesses addPrismFaces places on each witness
// prism. The second adds the station witnesses prismStationWitnesses places
// along every wall at both levels, and stationGateDiameter keeps the larger
// of the two. Both publish through pointSetDiameterWithBudget, which rounds
// the winning pair's exact distance toward zero. A station is a point of the
// prism's own walls only when those walls run the full height, so a
// capBlendPayload reads the carrier witnesses alone: its chamfer cuts the
// receiver's walls back at the cap levels.
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
	g := &bodyGeom{body: body}
	for _, witness := range witnesses {
		ok, err := g.addPrismFaces(budget, witness)
		if err != nil || !ok {
			return 0, false, err
		}
	}
	var pts []r3.Vec
	for _, f := range g.faces {
		if err := budget.Step(); err != nil {
			return 0, false, err
		}
		pts = append(pts, f.Wit...)
	}
	d, ok, err := pointSetDiameterWithBudget(budget, pts)
	if err != nil || !ok {
		return d, ok, err
	}
	d, ok = lowerDiameterForDisplacement(d, displacement)
	if _, chamfered := body.payload.(capBlendPayload); chamfered {
		// The chamfer cuts the receiver's walls back at the cap levels, so a
		// station on the receiver's section at z0 or z1 is not a point of
		// the body.
		return d, ok, nil
	}
	return stationGateDiameter(budget, d, ok, pts, witnesses, displacement)
}

// gateStationStep is the widest angle between two consecutive stations
// prismStationWitnesses places along one circular wall: 15 degrees.
const gateStationStep = math.Pi / 12

// stationGateDiameter raises a witness-maximum reading d (ok says whether it
// exists) with the station witnesses of prisms (prismStationWitnesses). The
// station reading ranges over held, the witnesses d was read from, together
// with every prism's stations, and shrinks the maximum by displacement (the
// figure d was shrunk by) plus the widest station's own allowance. The result
// is the larger of the two readings. Each is a lower bound on the denoted
// body's diameter on its own, so the larger one is too. Taking the larger one
// leaves d unchanged wherever the stations add nothing, and a section the
// stations cannot read keeps d.
//
// Every prism handed in must have walls that run the full height from z0 to
// z1 on the body, so that each station is a point of the body (within its
// allowance and displacement).
func stationGateDiameter(budget *proofbound.WorkBudget, d float64, ok bool, held []r3.Vec,
	prisms []prismPayload, displacement float64,
) (float64, bool, error) {
	work := freeform.NewFreeformWork()
	pts := append([]r3.Vec{}, held...)
	allow := 0.0
	for _, pp := range prisms {
		stations, stationAllow, read, err := prismStationWitnesses(budget, pp, work)
		if err != nil {
			return 0, false, err
		}
		if !read {
			return d, ok, nil
		}
		pts = append(pts, stations...)
		allow = math.Max(allow, stationAllow)
	}
	sd, sok, err := pointSetDiameterWithBudget(budget, pts)
	if err != nil {
		return 0, false, err
	}
	if !sok {
		return d, ok, nil
	}
	sd, sok = lowerDiameterForDisplacement(sd, proofbound.AbsSumUpper(displacement, allow))
	if !sok || (ok && sd <= d) {
		return d, ok, nil
	}
	return sd, true, nil
}

// prismStationWitnesses lists points on pp's walls at both z0 and z1 for the
// gate diameter alone. They never join a carrier's witnesses (CFace.Wit),
// which the clearance search reads.
//
// The carrier witnesses addPrismFaces places give a circular wall only two
// angles: th0, and the mid-angle at mid-height. With the arc's end vertex,
// that is three angles. A wall that sweeps past 180 degrees holds points
// opposite each other that none of the three reach. An arc closed by its
// chord is worst at a 240 degree sweep: the three samples sit
// 2R*sin(120 degrees) apart while the wall's diameter is 2R, so the carrier
// witnesses alone understate the diameter by a factor of 2/sqrt(3).
//
// prismStationWitnesses adds the following for every circular wall:
//
//   - stations at evenly spaced fractions k/n of the recorded parameter range,
//     with n even and at least 2, and consecutive stations at most
//     gateStationStep apart. k = 0 and k = n are the two ends, and k = n/2 is
//     the mid-angle;
//   - when the sweep exceeds 180 degrees, the two points opposite the start
//     and the end.
//
// For a line wall it adds the two walk ends. Every station is a point the
// record denotes, at a rational parameter inside the recorded range. Its held
// (u, v) comes from math.Sincos at the matching angle, so it carries the gap
// boundarywalk.CircularPointBound proves against that denoted point. A line
// end carries its own walk-end bound. allow is the widest of those gaps,
// carried through the frame and placement by prismPointBound together with
// the lift's own rounding.
//
// Every point of a circular wall lies within gateStationStep/2 of a station
// in angle. A wall that sweeps past 180 degrees also holds the station
// opposite its start, so the wall's own diameter 2R is read up to rounding
// and allow.
//
// read is false, with no error, for a free-form wall, a segment the record
// cannot normalize or walk, and a station whose gap the proof cannot state.
// Cancellation through budget returns the error.
func prismStationWitnesses(budget *proofbound.WorkBudget, pp prismPayload, work *freeform.FreeformWork) ([]r3.Vec, float64, bool, error) {
	var pts []r3.Vec
	allow := 0.0
	add := func(u, v float64, bound proofbound.WalkEndBound) bool {
		if !proofbound.FiniteVec(r3.NewVec(u, v, 0)) || proofbound.IsNonFinite(bound.U) || proofbound.IsNonFinite(bound.V) {
			return false
		}
		for _, z := range [2]float64{pp.z0, pp.z1} {
			a := prismPointBound(pp, proofbound.MeasuredScalar(u, bound.U), proofbound.MeasuredScalar(v, bound.V),
				proofbound.MeasuredScalar(z, 0))
			if !tolerance.UsableMagnitude(a) {
				return false
			}
			allow = math.Max(allow, a)
			pts = append(pts, pp.point(u, v, z))
		}
		return true
	}
	for _, loop := range append([]LoopRecord{pp.profile.Outer}, pp.profile.Holes...) {
		for _, recorded := range loop.Segments {
			if err := budget.Step(); err != nil {
				return nil, 0, false, err
			}
			seg, err := normalizeSegment(recorded)
			if err != nil {
				return nil, 0, false, nil //nolint:nilerr // structural refusal withholds the station reading
			}
			w, err := boundarywalk.WalkOf(seg, work)
			if err != nil {
				return nil, 0, false, nil //nolint:nilerr // structural refusal withholds the station reading
			}
			switch w.Kind {
			case survey2d.WalkLine:
				if !add(w.StartU, w.StartV, w.StartBound) || !add(w.EndU, w.EndV, w.EndBound) {
					return nil, 0, false, nil
				}
				continue
			case survey2d.WalkCircular:
			default:
				return nil, 0, false, nil
			}
			start, span, ok := loftmesh.CircularSegmentRange(seg)
			if !ok {
				return nil, 0, false, nil
			}
			sweep := math.Abs(w.Th1 - w.Th0)
			n := max(2, int(math.Ceil(sweep/gateStationStep)))
			n += n % 2
			fracs := make([]*big.Rat, 0, n+3)
			for k := 0; k <= n; k++ {
				fracs = append(fracs, big.NewRat(int64(k), int64(n)))
			}
			if sweep > math.Pi {
				opposite := math.Pi / sweep
				fracs = append(fracs, new(big.Rat).SetFloat64(opposite), new(big.Rat).SetFloat64(1-opposite))
			}
			for _, frac := range fracs {
				if err := budget.Step(); err != nil {
					return nil, 0, false, err
				}
				f, _ := frac.Float64()
				sin, cos := math.Sincos(w.Th0 + f*(w.Th1-w.Th0))
				u, v := w.CU+w.Radius*cos, w.CV+w.Radius*sin
				t := new(big.Rat).Add(start, new(big.Rat).Mul(frac, span))
				if !add(u, v, boundarywalk.CircularPointBound(seg, t, u, v)) {
					return nil, 0, false, nil
				}
			}
		}
	}
	return pts, allow, true, nil
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
// reasons.
//
// capBlendPayload reads pl.profile, the receiver's own unrewritten section on
// its unchanged interval: a cap-loop chamfer only ever cuts along a chord
// whose feet sit on the receiver's own recorded walls, so it can never place
// a point beyond the receiver's own extruded envelope. cupPayload reads its
// view's outer, the cup's own outer region — the receiver's unmodified section
// for an INWARD shell, but the wider OFFSET (expanded) region for an OUTWARD
// one, since an outward shell adds material and cupPayloadFor
// (shell_cup.go) always assigns the wider of the two profiles to outer
// regardless of sense. Either way the whole cup body — walls, floor and
// cavity alike — sits inside pl.outer's own full-height prism: the cavity
// never reaches farther than the outer region, the same containment
// cupPayload.extentAlong already relies on. Both are CONTAINING shapes, so
// each can only overstate the true diameter as a shape. The receiver's own
// section is its own denotation, because every modify op refuses a receiver
// carrying a section displacement (fillet.go's requireExactSection), so a
// cap blend's and an inward cup's witnesses carry only the axial
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
// witness sits from the denoted body point below it, and lowerDiameterForDisplacement
// turns the held maximum into the lower bound the gate wants. The copy zeroes
// sectionDelta because addPrismFaces (clearance_geom.go) refuses a displaced
// section outright: it builds the clearance kernel's certificate carriers,
// which have to be exact statements about a boundary, and a witness set for a
// diameter is neither a certificate nor a carrier.
func gateWitnessPrism(payload featurePayload) (prismPayload, float64, bool) {
	switch pl := payload.(type) {
	case capBlendPayload:
		witness := prismPayload{
			profile: pl.profile,
			frame:   pl.frame,
			z0:      pl.z0,
			z1:      pl.z1,
			z0Delta: pl.z0Delta,
			z1Delta: pl.z1Delta,
			xform:   pl.xform,
		}
		return witness, witness.axialDelta(), true
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
// stack does not reach, and would overstate its diameter.
func gateWitnessPrisms(payload featurePayload) ([]prismPayload, float64, bool) {
	if pl, ok := payload.(stackedPrismPayload); ok {
		runs, err := pl.outerRuns()
		if err != nil {
			return nil, 0, false
		}
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

// brepGateDiameter is bodyGateDiameter's arm for a brepPayload
// (docs/general-boolean-design.md §4.5): gateWitnessPrism's reader taken over
// every face's own prism view instead of one witness prism. Every body vertex
// is a witness, and every swept face adds the witnesses addPrismFaces takes
// off a prism wall — a circular wall's mid-angle point at mid-height and its
// start at z0, a straight wall's quad midpoint — so a whole circle still
// yields an antipodal pair. The witnesses are read off the recorded body the
// way addPrismFaces reads a prism's, and the recorded body lies within the
// largest section displacement plus the largest level displacement of the body
// the record denotes, so the held maximum is shrunk by that sum
// (lowerDiameterForDisplacement), as the fallback's stacked and
// displaced-prism arms shrink theirs.
func brepGateDiameter(ctx context.Context, body *Body, bp brepPayload) (float64, bool, error) {
	var witnesses []r3.Vec
	for _, v := range body.Vertices() {
		witnesses = append(witnesses, v.position)
	}
	work := freeform.NewFreeformWork()
	for _, f := range bp.faces {
		if f.planar() {
			continue
		}
		if err := ctx.Err(); err != nil {
			return 0, false, err
		}
		w, err := boundarywalk.WalkOf(f.wall, work)
		if err != nil {
			return 0, false, err
		}
		pp := f.view(bp.xform)
		mid := (pp.z0 + pp.z1) / 2
		if w.IsCircular() {
			th := (w.Th0 + w.Th1) / 2
			witnesses = append(witnesses,
				pp.point(w.CU+w.Radius*math.Cos(th), w.CV+w.Radius*math.Sin(th), mid),
				pp.point(w.CU+w.Radius*math.Cos(w.Th0), w.CV+w.Radius*math.Sin(w.Th0), pp.z0))
			continue
		}
		witnesses = append(witnesses, pp.point((w.StartU+w.EndU)/2, (w.StartV+w.EndV)/2, mid))
	}
	d, ok, err := pointSetDiameterContext(ctx, witnesses)
	if err != nil || !ok {
		return d, ok, err
	}
	d, ok = lowerDiameterForDisplacement(d, proofbound.AbsSumUpper(bp.sectionDelta(), bp.axialDelta()))
	return d, ok, nil
}

func pointSetDiameter(points []r3.Vec) (float64, bool) {
	d, ok, _ := pointSetDiameterWithBudget(nil, points)
	return d, ok
}

func pointSetDiameterContext(ctx context.Context, points []r3.Vec) (float64, bool, error) {
	return pointSetDiameterWithBudget(proofbound.NewWorkBudget(ctx), points)
}

// pointSetDiameterWithBudget adapts Verify's witness set to internal/diameter.
// See docs/verification-design.md §3 for its lower-bound contract.
func pointSetDiameterWithBudget(budget *proofbound.WorkBudget, points []r3.Vec) (float64, bool, error) {
	return diameter.PointsWithBudget(budget, points)
}
