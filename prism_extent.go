package decad

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file answers the extent questions asked OF a finished prism: how far
// the solid reaches along a direction, and the axis-aligned box that contains
// it.
//
// Every answer is a bounded interval, not a float: the coefficients of the
// direction in the payload's own frame round, the section and axial
// displacements move the boundary the interval is read from, and a boundary
// extreme riding a walked endpoint or a computed arc radius carries that
// walk's own bound. A direction whose extreme cannot be bracketed refuses
// rather than publishing the held value. See docs/evaluator-design.md §5.

// extentAlong is the through-all stop's reading of the prism (stops.go): the
// extent interval along an arbitrary world direction g — the lifted linear
// functional point·g = origin·g + u·(U'·g) + v·(V'·g) + z·(N'·g), primes the
// placed directions, extremized over the region boundary and the sweep —
// beside the proven displacement extentBoundedAlong states for its two ends.
// The stop charges that displacement to the level it resolves and decides its
// own in-path test outside it, so a boundary extreme held by a bracket
// (docs/spline-design.md §6.2) still answers rather than refusing.
//
// The section displacement is NOT one of the terms this reading carries — it
// moves a coordinate IN the plane, and the interval is stated over the
// recorded section — so a prism holding one refuses instead
// (docs/prism-boolean-design.md §12). prismBoundsContext reads
// extentBoundedAlong directly and composes both terms into its own outward
// bound.
// The stop and clearance callers hold no preflight counter for this record, so
// the interface forms open the record's own — one per extent reading, never one
// per segment.
func (pp prismPayload) extentAlong(g r3.Vec) (float64, float64, float64, error) {
	if pp.sectionDelta != 0 {
		return 0, 0, 0, fmt.Errorf(`%w: a through-all stop cannot use a prism with a proven section displacement`, ErrUnsupported)
	}
	return pp.extentBoundedAlong(context.Background(), g, newFreeformWork(), nil)
}

func (pp prismPayload) extentAlongContext(ctx context.Context, g r3.Vec) (float64, float64, error) {
	return pp.extentAlongWork(ctx, g, newFreeformWork())
}

// extentAlongWork is extentBoundedAlong's refusing wrapper, mirroring
// revolvePayload.extentAlongWork word for word: the reading for a consumer that
// takes the interval as an exact one and has nowhere to put a displacement.
// clearance.go's payloadExtent is that consumer — its separating-plane
// short-circuit compares two bodies' intervals and simply loses the
// short-circuit where it cannot get an exact one — so a direction whose extreme
// only a bracket holds refuses here rather than publish a held coordinate as the
// one it denotes. Which candidate holds it does not matter and the refusal never
// names a kind: a free-form span's enclosure, a computed arc radius and a walked
// endpoint the record does not state all reach this wrapper the same way,
// through one nonzero bound. A through-all stop does not read through this wrapper:
// it consumes the bounded reading and charges the displacement to its own level
// (stops.go, docs/spline-design.md §6.4).
func (pp prismPayload) extentAlongWork(ctx context.Context, g r3.Vec, work *freeformWork) (float64, float64, error) {
	lo, hi, bound, err := pp.extentBoundedAlong(ctx, g, work, nil)
	if err != nil {
		return 0, 0, err
	}
	if bound != 0 {
		return 0, 0, fmt.Errorf(`%w: this prism's extent along this direction is known only to a proven displacement of %v mm; this reading has no bound to widen`, ErrUnsupported, bound)
	}
	return lo, hi, nil
}

// extentBoundedAlong is the bounded reading itself: the interval AND its
// proven half-width, folded from boundaryExtremesBoundedContext's own
// per-candidate enclosures (docs/spline-design.md §6.2) AND from the frame and
// placement's own rounding (prismPlacementCoeffAllow). The boundary-scan term
// follows the CANDIDATES the extremes are held by, never the section's kind: a
// section whose extremes are all values the record states — straight walls,
// and an arc or circle read where its own recorded endpoint or its exactly
// representable apex wins — carries only zero-width candidates, while a
// trimmed circular endpoint, a computed arc radius or a free-form span's
// enclosure each publish the width their own construction owes. The frame and
// placement term is independent of it and composes outward: a straight-walled
// section under a tilted placement still widens, and an unplaced or
// axis-aligned section still reports zero for this term, which is what keeps
// an ordinary prism's box Exact.
//
// A THIRD term composes outward with both: the reading's own final summation
// base + lo + zlo, charged exactly against the same terms by exactSumRound
// (bounds.go). It is not covered by either of the other two — a pure
// translation leaves every coefficient exactly right and every multiply exact,
// and the addition that follows still rounds — and it is zero exactly where
// that addition is exactly representable, so an unplaced prism's box stays
// Exact.
//
// walks is pp.profile's pre-resolved segment walks, or nil to resolve as
// before through boundaryExtremesBoundedContext and profileCoordinateEnvelope's
// own walkOf calls. prismBoundsContext passes the same *profileWalks to every
// one of its three per-axis calls, so the record's boundary walks resolve
// once for the whole box rather than once per axis (this file's profileWalks
// doc comment).
func (pp prismPayload) extentBoundedAlong(ctx context.Context, g r3.Vec, work *freeformWork, walks *profileWalks) (float64, float64, float64, error) {
	base := pp.xform.Apply(pp.frame.Origin()).Dot(g)
	gu := pp.dir(1, 0, 0).Dot(g)
	gv := pp.dir(0, 1, 0).Dot(g)
	gz := pp.dir(0, 0, 1).Dot(g)
	lo, hi, bound, err := boundaryExtremesBoundedContext(ctx, pp.profile, gu, gv, work, walks)
	if err != nil {
		return 0, 0, 0, err
	}
	zlo := math.Min(pp.z0*gz, pp.z1*gz)
	zhi := math.Max(pp.z0*gz, pp.z1*gz)
	coordUpper, err := profileCoordinateEnvelope(pp.profile, work, walks)
	if err != nil {
		return 0, 0, 0, err
	}
	zUpper := math.Max(math.Abs(pp.z0), math.Abs(pp.z1))
	placeAllow := prismPlacementCoeffAllow(pp, g, base, gu, gv, gz, coordUpper, zUpper)
	// The recombination is charged per ENDPOINT and composed outward: the two
	// ends are summed from different terms and round by different amounts, while
	// the scan's and the placement's terms speak for both ends alike, so the
	// reading publishes the larger of the two per-end totals — the same shape
	// revolvePayload.extentBoundedAlong states for its own per-end composition.
	loEnd, hiEnd := base+lo+zlo, base+hi+zhi
	sumAllow := math.Max(
		exactSumRound(loEnd, base, lo, zlo),
		exactSumRound(hiEnd, base, hi, zhi),
	)
	bound = absSumUpper(bound, placeAllow, sumAllow)
	return loEnd, hiEnd, bound, nil
}

// prismPlacementCoeffAllow bounds how far base/gu/gv/gz — the four scalar
// coefficients extentBoundedAlong lifts the boundary and sweep extremes
// through — can sit from the value the SAME frame-and-placement chain's exact
// arithmetic would give, through exactIsometryDotRound's rational check
// (bounds.go): zero exactly where the frame is axis-aligned and the placement
// is the identity, nonzero only where that isometry's own float evaluation
// genuinely rounds. Each coefficient's own displacement moves the published
// extreme at the rate of the coordinate it multiplies —
// directionalPerturbationAllow's own Lipschitz shape, coordUpper for gu/gv and
// zUpper for gz — while base's displaces the extreme directly, at both ends
// alike, since it is the section's own constant offset under this direction.
// capBlendPayload.extentBoundedAlong reuses this unchanged through
// prismLike's shared frame and placement. A second, independent term
// (prismDecompositionRoundAllow) covers the rounding this reading's own
// DECOMPOSITION gu*u + gv*v + gz*z commits even when every coefficient is
// itself exactly right: multiplying an EXACT but non-trivial coefficient by a
// recorded coordinate still rounds, and grouping the sum this way (the
// boundary scan's own gu*u+gv*v first) is a DIFFERENT float computation than
// applying the frame and placement to the point directly, even though the two
// are equal in exact arithmetic. The recombination with base that FOLLOWS is a
// third term, charged exactly at the call site rather than here
// (exactSumRound), because it rounds for placements this function's own check
// proves exact — a translation is committed there and nowhere else.
func prismPlacementCoeffAllow(pp prismPayload, g r3.Vec, base, gu, gv, gz, coordUpper, zUpper float64) float64 {
	baseRound := exactIsometryDotRound(pp.xform, pp.frame.Origin(), g, true, base)
	guRound := exactIsometryDotRound(pp.xform, pp.frame.U(), g, false, gu)
	gvRound := exactIsometryDotRound(pp.xform, pp.frame.V(), g, false, gv)
	gzRound := exactIsometryDotRound(pp.xform, pp.frame.N(), g, false, gz)
	return absSumUpper(
		baseRound,
		directionalPerturbationAllow(guRound, coordUpper),
		directionalPerturbationAllow(gvRound, coordUpper),
		directionalPerturbationAllow(gzRound, zUpper),
		prismDecompositionRoundAllow(gu, gv, gz, base, coordUpper, zUpper),
	)
}

// prismDecompositionRoundAllow bounds the rounding the MULTIPLY-AND-SUM
// combination base + gu*u + gv*v + gz*z commits, given base/gu/gv/gz
// themselves proven exact against the isometry that produced them
// (prismPlacementCoeffAllow's own exactIsometryDotRound check, above). IEEE
// 754 multiplies exactly by 0, 1 or -1 for ANY operand — those three values
// are the only ones that never round a multiply — so a coefficient outside
// that set can round when it multiplies a recorded coordinate, and this
// reading's own left-to-right summation order (the boundary-extreme scan's
// gu*u+gv*v first, then +base, then +gz*z — a DIFFERENT grouping than
// applying the frame and placement to the point directly) can round again on
// top of that even where every individual multiply happens not to. Both are
// genuinely new roundings a non-axis-permuting frame commits, so the term is
// zero where every coefficient is trivial — and otherwise reuses
// analyticRoundBound's own established "a bounded number of basic ops at a
// magnitude" contract: at most 3 multiplies and 3 additions here, far under
// its 128-operation budget. |base| stays in that envelope because the same
// left-to-right evaluation folds base in, and charging it in both arms is only
// ever wider than the non-trivial arm owes.
//
// The trivial arm's zero speaks for the DECOMPOSITION alone, never for the
// whole endpoint. g is a unit world axis and the frame orthonormal, so
// gu²+gv²+gz² = 1 and an all-trivial reading has exactly one coefficient at
// ±1 with the other two at 0: every multiply is exact and so is the scan's own
// gu*u+gv*v, whatever the coordinates are. What that argument does NOT reach
// is the recombination with base and the sweep level, which rounds for a
// coefficient set this arm calls trivial — a pure translation is exactly that
// case — so extentBoundedAlong charges it separately and exactly through
// exactSumRound (bounds.go), and this term must never be read as covering it.
func prismDecompositionRoundAllow(gu, gv, gz, base, coordUpper, zUpper float64) float64 {
	trivial := func(c float64) bool { return c == 0 || c == 1 || c == -1 }
	if trivial(gu) && trivial(gv) && trivial(gz) {
		return 0
	}
	// The non-trivial arm is loose, and it is loose in the safe direction: one
	// fixture measures both halves of that. A 5x5 mm section swept 1 mm on the
	// UNPLACED frame U=(0.6,0.8,0), V=(-0.8,0.6,0) reports Min=(-4,0,0),
	// Max=(3,7,1), Approximate, bound 3.9790393202565666e-13. That whole figure
	// is this term: divide it by analyticRoundBound's own 256*unitRoundoff and
	// it comes to 14 up to that helper's outward rounding, which is the scale
	// below — |gu|*coordUpper + |gv|*coordUpper = 0.6*10 + 0.8*10, the walk's
	// coordUpper being 10 for that section — while every sibling term answers
	// zero: exactIsometryDotRound on base/gu/gv/gz under the identity xform,
	// exactSumRound on base 0 with levels 0 and 1, and both displacement terms
	// prismBoundsContext composes. A zero bound on that fixture would be
	// unsound rather than tighter, because the extremes it would call exact are
	// not: summing this frame's OWN held entries over the rationals against the
	// section corners {0,5}^2 gives X-min = -5*float64(0.8), X-max =
	// 5*float64(0.6) and Y-max = 5*float64(0.6) + 5*float64(0.8), none of them
	// representable, since float64(0.8) sits above 4/5 and 5*float64(0.8) needs
	// 55 significand bits — it is exactly 4 + 2^-52, a QUARTER of the 2^-50 ulp
	// above 4, which ordinary round-to-nearest returns as 4 with no tie. Each of
	// the three published coordinates therefore misses its true extreme by a
	// representable amount (2.22e-16 in X-min, 1.11e-16 in X-max and Y-max),
	// X-min and Y-max landing INSIDE the true extreme and X-max landing outward
	// of it, a miss this term covers with wide margin.
	scale := absSumUpper(
		productUpper(math.Abs(gu), coordUpper),
		productUpper(math.Abs(gv), coordUpper),
		productUpper(math.Abs(gz), zUpper),
		math.Abs(base),
	)
	return analyticRoundBound(scale)
}

// prismBoundsContext computes the exact axis-aligned bounds of the placed prism:
// for each world axis, the directional extreme of the region boundary under
// the lifted linear functional, plus the sweep's own extreme
// (docs/evaluator-design.md §5).
//
// walks is pp.profile's pre-resolved segment walks, or nil to resolve each
// segment through walkOf as before. Passed straight to all three per-axis
// extentBoundedAlong calls below (this file's profileWalks doc comment), so a
// non-nil walks resolves the record's boundary once for the whole box instead
// of once per axis.
func prismBoundsContext(ctx context.Context, pp prismPayload, work *freeformWork, walks *profileWalks) (Box, error) {
	axes := []r3.Vec{r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1)}
	var minC, maxC [3]float64
	extremeBound := 0.0
	for i, axis := range axes {
		if err := ctx.Err(); err != nil {
			return Box{}, err
		}
		lo, hi, bound, err := pp.extentBoundedAlong(ctx, axis, work, walks)
		if err != nil {
			return Box{}, err
		}
		minC[i] = lo
		maxC[i] = hi
		extremeBound = math.Max(extremeBound, bound)
	}
	// A displaced section displaces every extreme it holds, so the box's own
	// error carries the section displacement itself — δ outward on every face
	// (docs/prism-boolean-design.md §7) — summed with the boundary's own
	// directional-extreme bracket bound (docs/spline-design.md §6.2) and with
	// the frame and placement's own rounding (prismPlacementCoeffAllow) and the
	// endpoint summation's (exactSumRound), both folded into
	// extentBoundedAlong's own returned bound above: the frame and
	// placement ARE isometries in exact arithmetic, but their FLOAT evaluation
	// rounds wherever the frame is not axis-aligned or the placement is not the
	// identity, and adding the resulting terms into one published coordinate
	// rounds again — for a pure translation it is the ONLY rounding there is —
	// so a box that reads either as an exact leaf can miss
	// the true extreme by a representable amount. All three terms are zero for
	// a caller-drawn, unplaced, axis-aligned payload whose extremes are all
	// values its record states, which keeps the ordinary prism's box Exact as
	// before. The bracket's own term is what decides the rest, never the
	// section's kind: a straight-walled section reports zero, an analytic one whose extreme is
	// held by a trimmed circular endpoint or a computed arc radius reports that
	// candidate's own width, and a free-form section whose extremes along
	// these three axes are all held by exactly representable candidate values
	// reports a zero width and stays Exact too (a span monotone along an axis
	// contributes its two exactly interpolated endpoints and nothing else),
	// while an extreme held by an irrational interior root publishes that
	// bracket's width and is Approximate — §6.2's own stated contract
	// consequence. The sum only
	// goes through absSumUpper's own per-term rounding where there are two
	// genuine terms to compose: bumping a lone sectionDelta a second time for
	// an always-zero extremeBound term would grow the box's bound past the
	// single upRound tessellate.go's own mesh bound composes it against.
	//
	// The sweep's own ends enter the same way. Each axis reading takes the
	// levels through zlo/zhi scaled by |gz| ≤ 1 (a unit axis against a placed
	// unit normal), so the larger end displacement bounds the box face either
	// level can move, and it composes with the section's term because the two
	// displace along different axes.
	axial := pp.axialDelta()
	terms := make([]float64, 0, 3)
	if pp.sectionDelta != 0 {
		terms = append(terms, pp.sectionDelta)
	}
	if extremeBound != 0 {
		terms = append(terms, extremeBound)
	}
	if axial != 0 {
		terms = append(terms, axial)
	}
	bound := 0.0
	switch len(terms) {
	case 0:
	case 1:
		bound = terms[0]
	default:
		bound = absSumUpper(terms...)
	}
	return Box{
		Min:       r3.NewVec(minC[0], minC[1], minC[2]),
		Max:       r3.NewVec(maxC[0], maxC[1], maxC[2]),
		Exactness: exactnessOf(bound),
		Bound:     units.Millimeters(bound),
	}, nil
}

// circularExtremeInterval encloses the two EXACT extremes of the functional
// g(u, v) = gu·u + gv·v over the whole circle a circular walk lies on. Writing
// the walk as c + r·(cos θ, sin θ) gives g(θ) = (gu·cU + gv·cV) + r·|(gu, gv)|·
// cos(θ − θ*), so the circle's own minimum and maximum are P ∓ r·|(gu, gv)| —
// an identity in which the angle does not appear at all.
//
// That is why the scan's circular candidate is bounded from here rather than
// from a certified sine and cosine at the candidate's own angle: the angle is a
// SELECTION (does the walk sweep the apex?), while the VALUE the fold publishes
// is this closed form, and reading it this way charges no π-rounding guard for
// an angle that never enters the answer. The three inputs that are not exact
// leaves each enter through the bounded arithmetic: the walk's radius under its
// own proven bound (segmentWalk.radiusBound — an ArcSeg states Start and Center,
// so its radius is a math.Hypot), the direction's magnitude through
// boundedNorm2's certified square-root brackets, and every product and sum
// through boundedMul/boundedAdd's own exact rounding terms. An exactly
// representable apex therefore still reports a zero bound, which is what lets a
// recorded circle's box stay Exact along an axis whose reading the apex holds —
// the walk's own endpoints answer for themselves there
// (segmentWalk.startBound/endBound).
func circularExtremeInterval(w segmentWalk, gu, gv float64) (boundedScalar, boundedScalar) {
	gmag := boundedNorm2(exactScalar(gu), exactScalar(gv))
	centre := boundedAdd(
		boundedMul(exactScalar(gu), exactScalar(w.cU)),
		boundedMul(exactScalar(gv), exactScalar(w.cV)),
	)
	amplitude := boundedMul(measuredScalar(w.radius, w.radiusBound), gmag)
	return boundedSub(centre, amplitude), boundedAdd(centre, amplitude)
}

// boundaryExtremesBoundedContext is the one scan, total over walkKind
// (docs/spline-design.md §6.2): the min and max of g(u, v) = gu·u + gv·v over
// the recorded region's boundary, AND the proven half-width every CANDIDATE's
// own position contributes to that interval.
//
// That half-width is the candidates' POSITIONAL displacement alone. The scan
// evaluates each candidate as the float gu·u + gv·v, and the rounding of that
// multiply-and-sum is the CALLER's to charge, at the coordinate envelope the
// caller's own geometry states: a prism reads it through
// prismDecompositionRoundAllow, a revolve through
// planeDotDecompositionRoundAllow (bounds.go), and a caller that charges
// neither publishes a candidate the record states verbatim — a zero-width one,
// on which this scan reports zero — as if the arithmetic reading it had
// committed nothing.
//
// An ENDPOINT candidate is the walk's own endpoint read through the direction
// the caller holds, which this evaluator reads as an exact leaf throughout (the
// convention survey2d.go's own file comment states). The endpoint itself is an
// exact leaf only where the record STATES it — a line's or an arc's natural
// bounds — and there the candidate has zero width, so an all-straight section's
// reading stays exact. Every other endpoint is one this evaluator computed, and
// the walk states what it is worth (segmentWalk.startBound/endBound);
// pointPerturbationAllow carries that displacement through the functional so
// the candidate enters at the width its own construction owes, never at zero.
// An endpoint whose bound no arithmetic could state refuses the whole scan
// rather than folding an infinity into the accumulators.
//
// An interior CIRCULAR candidate is a second computed reading: its position is
// the walk's radius times a cosine and a sine, and the radius itself is a
// math.Hypot for every ArcSeg, so it enters the fold under the proven enclosure
// circularExtremeInterval derives — the single owner of that term, charged where
// the candidate is produced rather than beside the fold by whichever consumer
// noticed. A free-form walk folds each of its converted spans' own proven
// enclosure (spanExtremeEnclosureContext) and takes no endpoint candidate of its
// own: a span enclosure already covers the span's whole parameter range,
// endpoints included.
//
// The fold is the shipped capBlendPayload.extentBoundedAlong idiom: track the
// lower and upper ends of every candidate contributing to the region minimum
// (loLower/loUpper) and to the region maximum (hiLower/hiUpper) separately, so
// a candidate that loses the extremization contributes nothing to the reported
// bound, and report the midpoint of each composed interval with the larger of
// the two half widths, rounded up — the same convention freeformArcLength
// already uses. The fold is sound because every candidate interval encloses a
// value the true boundary actually attains: the reported minimum's lower end is
// the least of the candidates' lower ends and so never exceeds the truth, and
// its upper end is the least of their upper ends, which the candidate attaining
// the true minimum keeps at or above it.
//
// A span enclosure that convention cannot state in float64 refuses at the
// conversion rather than entering the fold (spline_extreme.go's
// freeformExtremeFloats, Table R row R18), so every number these accumulators
// hold is finite and the only reading left to the empty-region check below is
// a region that genuinely contributed no candidate.
//
// The direction is carried through this scan as the two FLOATS the caller
// holds, and it is gated by requireFiniteDirection, which reads them and
// allocates nothing. The rational lift each span's Bernstein coefficients need
// happens inside spanExtremeEnclosureContext, behind that span's own R7 charge
// — §5.2's rule is that every charge is levied before the work allocates, and
// a rational built here would allocate ahead of every charge this scan makes.
//
// walks is profile's pre-resolved segment walks, or nil to resolve each
// segment through walkOf as before (this file's profileWalks doc comment). A
// non-nil walks that was not resolved from THIS profile's own recorded
// segments refuses.
func boundaryExtremesBoundedContext(ctx context.Context, profile ProfileRecord, gu, gv float64, work *freeformWork, walks *profileWalks) (float64, float64, float64, error) {
	if err := requireFiniteDirection(gu, gv); err != nil {
		return 0, 0, 0, err
	}
	if walks != nil && !walks.matches(profile) {
		return 0, 0, 0, errResolvedWalksMismatch
	}

	loLower, loUpper := math.Inf(1), math.Inf(1)
	hiLower, hiUpper := math.Inf(-1), math.Inf(-1)
	takeLo := func(l, h float64) {
		loLower = math.Min(loLower, l)
		loUpper = math.Min(loUpper, h)
	}
	takeHi := func(l, h float64) {
		hiLower = math.Max(hiLower, l)
		hiUpper = math.Max(hiUpper, h)
	}
	// take folds one candidate's held value under the proven bound its own
	// generator derived. A zero bound enters as the held value twice: widening
	// an exact candidate by a directed rounding would mint an error the
	// arithmetic provably did not commit. A nonzero one is stepped outward with
	// math.Nextafter rather than upRound/downRound, since a directional value
	// can be negative and those two only move a POSITIVE bound toward zero.
	take := func(g, allow float64) {
		if allow == 0 {
			takeLo(g, g)
			takeHi(g, g)
			return
		}
		lo := math.Nextafter(g-allow, math.Inf(-1))
		hi := math.Nextafter(g+allow, math.Inf(1))
		takeLo(lo, hi)
		takeHi(lo, hi)
	}
	takeVertex := func(u, v float64, bound walkEndBound) {
		take(gu*u+gv*v, pointPerturbationAllow(bound, gu, gv))
	}
	// Every span enclosure enters the fold through freeformExtremeFloats
	// (spline_extreme.go), which rounds outward through ratFloatDown/ratFloatUp
	// — never downRound/upRound: a directional value can be negative, and those
	// only ever move a POSITIVE bound toward zero (spline_length.go's
	// arc-length-only convention), the wrong direction for a negative
	// candidate and a spurious one-ulp widening of an exactly representable
	// value either way — and refuses ErrUnsupported for an enclosure the
	// float64 range cannot state, so no infinity ever reaches these
	// accumulators.

	for li, loop := range append([]LoopRecord{profile.Outer}, profile.Holes...) {
		for si, seg := range loop.Segments {
			if err := ctx.Err(); err != nil {
				return 0, 0, 0, err
			}
			w, err := resolveOrRead(seg, work, walks, li, si)
			if err != nil {
				return 0, 0, 0, err
			}
			if w.kind == walkFreeform {
				for _, span := range w.spans {
					minIv, maxIv, err := spanExtremeEnclosureContext(ctx, span, gu, gv, work)
					if err != nil {
						return 0, 0, 0, err
					}
					minLo, minHi, err := freeformExtremeFloats(minIv)
					if err != nil {
						return 0, 0, 0, err
					}
					maxLo, maxHi, err := freeformExtremeFloats(maxIv)
					if err != nil {
						return 0, 0, 0, err
					}
					takeLo(minLo, minHi)
					takeHi(maxLo, maxHi)
				}
				continue
			}
			if !w.startBound.derivable() || !w.endBound.derivable() {
				return 0, 0, 0, fmt.Errorf(`%w: a boundary segment's walked endpoint states no proven displacement, so this scan cannot bound the region's extremes`, ErrUnsupported)
			}
			takeVertex(w.startU, w.startV, w.startBound)
			takeVertex(w.endU, w.endV, w.endBound)
			if !w.isCircular() {
				continue
			}
			// Interior extremes at θ* where the functional's gradient
			// aligns with the radius: θ* = atan2(gv, gu) (+π). The angle
			// SELECTS which of the circle's two extremes the walk sweeps;
			// circularExtremeInterval states what that extreme is worth.
			gmag := math.Hypot(gu, gv)
			if gmag == 0 {
				continue
			}
			minIv, maxIv := circularExtremeInterval(w, gu, gv)
			star := math.Atan2(gv, gu)
			tlo, thi := math.Min(w.th0, w.th1), math.Max(w.th0, w.th1)
			for ci, cand := range [2]float64{star, star + math.Pi} {
				apex := maxIv
				if ci == 1 {
					apex = minIv
				}
				for k := math.Floor((tlo-cand)/(2*math.Pi)) * 2 * math.Pi; cand+k <= thi+1e-12; k += 2 * math.Pi {
					th := cand + k
					if th < tlo-1e-12 {
						continue
					}
					held := gu*(w.cU+w.radius*math.Cos(th)) + gv*(w.cV+w.radius*math.Sin(th))
					take(held, boundedFloatError(apex, held))
				}
			}
		}
	}
	if math.IsInf(loLower, 1) {
		return 0, 0, 0, fmt.Errorf(`%w: the recorded region has no boundary`, ErrDegenerate)
	}
	loMid := loLower + (loUpper-loLower)/2
	hiMid := hiLower + (hiUpper-hiLower)/2
	bound := upRound(math.Max(
		math.Max(loMid-loLower, loUpper-loMid),
		math.Max(hiMid-hiLower, hiUpper-hiMid),
	))
	return loMid, hiMid, bound, nil
}
