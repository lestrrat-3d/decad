package decad

import (
	"context"
	"math"
	"math/big"

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
// that diameter, since every arm below publishes through
// pointSetDiameterWithBudget, whose own doc comment owns that rule.
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
// correction to the prisms it reads, over each one's own displacement.
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
// tighten. downRound (spline_length.go, upRound's mirror) steps it back
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
		return payload.diameter, usableMagnitude(payload.diameter), nil
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
	budget := newWorkBudget(ctx)
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

// lowerDiameterForDisplacement turns a held witness diameter into a lower
// bound on the denoted body's diameter. Every witness can move by at most the
// supplied displacement, so their pair distance can shrink by twice that
// amount. The subtraction rounds toward zero because this value only tightens
// the tolerance gate when it remains a lower bound.
func lowerDiameterForDisplacement(d, displacement float64) (float64, bool) {
	if displacement == 0 {
		return d, true
	}
	if !usableMagnitude(d) || !usableMagnitude(displacement) {
		return 0, false
	}
	d = downRound(d - 2*displacement)
	return d, d > 0 && usableMagnitude(d)
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
// endpoint bound walkEndBoundAllow reads off whichever walk produced it — an
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
//   - a witness's own endpoint bound cannot be derived (walkEndBoundAllow's
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
// count (four points per segment, bounded only by recipe_decode.go's own
// MaxSegments ceiling), and the maximum is quadratic in it, so an unpolled scan is by
// far the longest thing a cancelled Verify could be left waiting on here. The
// resulting error is returned AS an error: cancellation is never folded into
// this arm's structural (0, false, nil) answer, which states only that the
// recorded section gives this arm nothing to read.
func freeformSectionGateDiameter(ctx context.Context, pp prismPayload) (float64, bool, error) {
	work := newFreeformWork()
	sawFreeform := false
	ownBound := 0.0
	var pts []r3.Vec

	addWitness := func(u, v float64, bound walkEndBound) bool {
		allow := walkEndBoundAllow(bound)
		if isNonFinite(allow) {
			return false
		}
		ownBound = math.Max(ownBound, allow)
		pts = append(pts, pp.point(u, v, pp.z0), pp.point(u, v, pp.z1))
		return true
	}

	for _, loop := range append([]LoopRecord{pp.profile.Outer}, pp.profile.Holes...) {
		for _, seg := range loop.Segments {
			if err := ctx.Err(); err != nil {
				return 0, false, err
			}
			seg, err := normalizeSegment(seg)
			if err != nil {
				// normalizeSegment reads no ctx and so never observes
				// cancellation; every error it can return is a structural
				// refusal of the recorded segment itself, read here as "no
				// arm" rather than a hard failure.
				return 0, false, nil //nolint:nilerr // structural refusal, not cancellation — see comment above
			}
			w, err := walkOf(seg, work)
			if err != nil {
				// Likewise walkOf: it reads the record's own freeformWork
				// counter, never ctx, so its error is always a build-time
				// refusal (an R-table sentinel) this arm reads as "no arm"
				// rather than propagates.
				return 0, false, nil //nolint:nilerr // structural refusal, not cancellation — see comment above
			}
			if w.kind != walkFreeform {
				if !addWitness(w.startU, w.startV, w.startBound) || !addWitness(w.endU, w.endV, w.endBound) {
					return 0, false, nil
				}
				continue
			}
			sawFreeform = true
			for _, span := range w.spans {
				if len(span) == 0 {
					return 0, false, nil
				}
				for _, cp := range [2]ratPoint{span[0], span[len(span)-1]} {
					held, ok := point2Of(cp)
					if !ok {
						return 0, false, nil
					}
					bound := walkEndBound{
						u: rationalFloatError(cp.u, held.U),
						v: rationalFloatError(cp.v, held.V),
					}
					if !addWitness(held.U, held.V, bound) {
						return 0, false, nil
					}
				}
			}
		}
	}
	if !sawFreeform {
		return 0, false, nil
	}

	d, ok, err := pointSetDiameterContext(ctx, pts)
	if err != nil {
		return 0, false, err
	}
	if !ok {
		return 0, false, nil
	}
	displacement := absSumUpper(pp.sectionDelta, pp.axialDelta(), ownBound)
	d, ok = lowerDiameterForDisplacement(d, displacement)
	return d, ok, nil
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
// What this function actually reports, though, is a reading of that geometry,
// taken through the identical witness maximum a shipped prismPayload already
// reads its own diameter through above (addPrismFaces gives two witnesses
// per circular wall — the mid-angle point at mid-height and th0 at z0 —
// which pointSetDiameterWithBudget maxes pairwise, and region2.samples adds
// each cap arc's own th0 and mid-angle). That reader ranges over the body's
// own farthest pair — whose exact distance it then publishes rounded toward
// zero, so even the best case here is the largest float at or below the
// witness maximum — exactly when a
// circular wall's farthest pair lands on one of those three sampled angles
// (th0, mid-angle, th1) — guaranteed for an all-line section (the diameter
// is realized at vertices, all sampled), for a full circle (the two samples
// are antipodal), and for the arc-plus-chord family at or below 180 degrees
// of sweep (the diameter is realized at the arc endpoints) — but NEVER
// guaranteed by a bound on the sweep alone: an outward cup's own four 90
// degree corner arcs already understate this fallback's own output — read
// 64.922642 against that body's true diameter 65, a ratio of 1.0012 (its
// bounding-box diagonal is 68.738635, which the rounded corners keep it
// well inside of) — and a bare arc-plus-chord section peaks at 240 degrees, where the
// only sampled points are th0, the mid-angle, and th1, mutually
// 2R*sin(120 degrees) apart while the wall's true diameter is 2R — a ratio
// of 2/sqrt(3), about 15.5% (docs/verification-design.md §3 works that
// family's own figure). That understatement is not something this fallback
// introduces: the same reader already returns it for an ordinary shipped
// prismPayload built from the same curved section, so this fallback is no
// weaker than the exact path it stands in for, and the repair belongs to
// that shared reader rather than to this construction. The consequence
// stays inside the one direction this gate is free to err in: an
// understated D tightens Ref and can turn a passing reading into a false
// Suspect, never a false Sound (verification design §3). It stays intrinsic
// to the body's own geometry — built from the payload's own frame/xform, the
// same map a shipped prism's diameter is read through — so it carries none
// of the pose-dependence verification design §4 excludes an axis-aligned box
// for.
//
// A witness prism whose payload carries a displacement has the same
// held-witness issue as the exact prism path: each witness sits within that
// displacement of the point the payload denotes, so the held maximum can
// overstate the denoted body's diameter by twice it. The fallback shrinks the
// held witness maximum by that amount before it becomes a lower-bound
// reference, which is why gateWitnessPrism hands back a displacement beside
// the prism to read.
func fallbackGateDiameter(budget *workBudget, body *Body) (float64, bool, error) {
	witness, displacement, ok := gateWitnessPrism(body.payload)
	if !ok {
		return 0, false, nil
	}
	g := &bodyGeom{body: body}
	ok, err := g.addPrismFaces(budget, witness)
	if err != nil || !ok {
		return 0, false, err
	}
	var pts []r3.Vec
	for _, f := range g.faces {
		if err := budget.step(); err != nil {
			return 0, false, err
		}
		pts = append(pts, f.wit...)
	}
	d, ok, err := pointSetDiameterWithBudget(budget, pts)
	if err != nil || !ok {
		return d, ok, err
	}
	d, ok = lowerDiameterForDisplacement(d, displacement)
	return d, ok, nil
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
// a point beyond the receiver's own extruded envelope. cupPayload reads
// pl.outer, the cup's own outer region — the receiver's unmodified section
// for an INWARD shell, but the wider OFFSET (expanded) region for an OUTWARD
// one, since an outward shell adds material and cupPayloadFor
// (shell_cup.go) always assigns the wider of the two profiles to outer
// regardless of sense. Either way the whole cup body — walls, floor and
// cavity alike — sits inside pl.outer's own full-height prism: the cavity
// never reaches farther than the outer region, the same containment
// cupPayload.extentAlong already relies on. Both are CONTAINING shapes, so
// each can only overstate the true diameter as a shape; both read a section
// that is its own denotation, because every modify op refuses a receiver
// carrying a section displacement (fillet.go's requireExactSection), so the
// only displacement their witnesses carry is the axial one.
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
		witness := pl.outerPrism()
		witness.profile = pl.outer
		return witness, witness.axialDelta(), true
	case prismPayload:
		if pl.sectionDelta == 0 {
			return prismPayload{}, 0, false
		}
		displacement := absSumUpper(pl.sectionDelta, pl.axialDelta())
		witness := pl
		witness.sectionDelta = 0
		return witness, displacement, true
	default:
		return prismPayload{}, 0, false
	}
}

func pointSetDiameter(points []r3.Vec) (float64, bool) {
	d, ok, _ := pointSetDiameterWithBudget(nil, points)
	return d, ok
}

func pointSetDiameterContext(ctx context.Context, points []r3.Vec) (float64, bool, error) {
	return pointSetDiameterWithBudget(newWorkBudget(ctx), points)
}

// pointSetDiameterWithBudget is the ONE witness-maximum reader every gate
// diameter is published through — bodyGateDiameter's exact carrier arm and its
// loft arm, freeformSectionGateDiameter, fallbackGateDiameter, and the cached
// facetedPayload.diameter buildFacetedBody stores (boolean_body.go). What it
// returns is a CERTIFIED value at or below the exact greatest distance between
// two of the supplied points: the float scan only SELECTS a pair, and the
// published number is that pair's own distance computed over exact rationals
// and rounded toward zero (exactPairDistanceDown).
//
// The float scan cannot publish that number itself. Sub rounds each component
// and Len rounds the norm, so points[i].Sub(points[j]).Len() can land ABOVE
// the pair's exact distance — a 6x6x7 box's corner pair reads
// 11.000000000000002 against an exact sqrt(121) = 11 — while every consumer
// here reads the answer as a LOWER bound on the body's own diameter
// (docs/verification-design.md §3: an understated D tightens the gate into a
// false Suspect at worst, an overstated one loosens it into a false Sound).
// Charging that roundoff as an outward allowance is not open to this function
// either: it publishes one number, not an interval, so the charge has to land
// inside the value, which means rounding the value itself toward zero.
//
// Selecting the pair in floats costs nothing here. Whatever pair the scan
// picks, the published number is a REAL pair distance rounded toward zero and
// is therefore at or below the exact maximum; a near-tie the float rounding
// mis-orders changes how TIGHT the answer is, never whether it is a lower
// bound. So the certification rests on the exact arithmetic alone, and the
// scan is left free to cost what it always did.
//
// ok is false for an empty set, for a pair distance that is not a usable
// magnitude, and for a winning pair whose coordinates have no exact rational
// form — an absent answer, never a substitute one.
func pointSetDiameterWithBudget(budget *workBudget, points []r3.Vec) (float64, bool, error) {
	if len(points) == 0 {
		return 0, false, nil
	}
	best := 0.0
	bestI, bestJ := 0, 0
	for i := range points {
		for j := i + 1; j < len(points); j++ {
			if budget != nil {
				if err := budget.step(); err != nil {
					return 0, false, err
				}
			}
			distance := points[i].Sub(points[j]).Len()
			if !usableMagnitude(distance) {
				return 0, false, nil
			}
			if distance > best {
				best, bestI, bestJ = distance, i, j
			}
		}
	}
	if budget != nil {
		if err := budget.err(); err != nil {
			return 0, false, err
		}
	}
	if best == 0 {
		// A single point, or a set whose every pair is coincident: zero is
		// already exact and needs no rounding step.
		return 0, true, nil
	}
	d, ok := exactPairDistanceDown(points[bestI], points[bestJ])
	if !ok {
		return 0, false, nil
	}
	return d, true, nil
}

// exactPairDistanceDown returns the largest float64 at or below the EXACT
// distance between two points. Both coordinates of each axis are float64s and
// so are exact rationals; the difference and its square are exact in that
// arithmetic, and ratSqrtDown (spline_length.go) decides the last step by
// comparing a candidate's exact square against the exact sum rather than by
// trusting the platform's own square root. Nothing in the chain rounds outward,
// so the answer is proven to be at or below the pair's true distance.
//
// ok is false when a coordinate has no rational form (a non-finite one), which
// its callers read as no answer at all.
func exactPairDistanceDown(a, b r3.Vec) (float64, bool) {
	total := new(big.Rat)
	for _, axis := range [3][2]float64{{a.X, b.X}, {a.Y, b.Y}, {a.Z, b.Z}} {
		ra, rb := floatRat(axis[0]), floatRat(axis[1])
		if ra == nil || rb == nil {
			return 0, false
		}
		d := ra.Sub(ra, rb)
		total.Add(total, d.Mul(d, d))
	}
	return ratSqrtDown(total), true
}
