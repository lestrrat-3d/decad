package proofbound

import (
	"context"
	"math"
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
)

// This file is the single owner of every proven error bound a faceted
// (boolean-built) measurement reports. NO measurement site computes a bound
// inline: each error mechanism the mesh boolean is subject to has exactly one
// helper here, and every site routes through it. One reader below owns no
// mechanism of its own: CellAllowsOf returns all three of a wall cell's bounds,
// publishing exactly what the three helpers listed below publish.
//
// The mechanisms, and the helper that owns each:
//
//   - the chord displacement δ of a point on an OPERAND's surface → carried in
//     the payload per vertex, composed by the mesh boolean
//     (docs/faceted-vertex-bounds-design.md §3);
//   - the TRIM AMPLIFICATION of a point on the boolean's OWN rim: a rim vertex
//     is the crossing of two chord PLANES, so it is displaced not by δ but by
//     (δA + δB)/sin θ of its own facet pair, θ that pair's crossing angle —
//     unbounded as the surfaces approach tangency → meshbool.RimBound, which
//     the root refuses once the inflated bound stops meaning anything;
//   - float SUMMATION of the reported value itself → SumSlop, a proven bound
//     for a NAIVE loop (never zero for a positive float-computed value, down
//     to the subnormal range, which is what keeps exactnessOf honest);
//   - ACCUMULATION over the N elements of a chain → ChainLengthBound;
//   - RIGID-MOTION rounding → RigidRoundAllow, charged at the INPUT and
//     translation magnitudes, which is where the rounding is actually
//     committed — never at the output's;
//   - the VOLUME a vertex displacement sweeps out → SweptVolumeAllow, charged
//     against PerturbedAreaUpper — the area of the surface the displacement
//     acted ON, which is NOT the area of the mesh that survived it;
//   - an ABSOLUTE upper bound on the AREA of every surface ONE chorded wall
//     cell's chord-to-curve homotopy visits, from the bilinear RULED patch
//     between its four chord corners through to the ruled surface between
//     the two TRUE curves it denotes → CellChordCurveAreaUpper, the area the
//     loft's per-cell wall volume leg multiplies by that cell's own matched
//     departure (docs/loft-gear-bounds-design.md §2) — never a
//     held-facet-area-plus-excess reading, which does not bound a family
//     whose area is not sign-definite relative to any one held facet, and
//     never fed the loft evaluator's own sagitta sectionDelta in place of its
//     own PARAMETER-MATCHED matchedDeltaUpper obligation, a strictly stronger
//     claim the two coincide for only a LINE or an ARC (F1);
//   - the VOLUME between ONE loft wall cell's HELD two flat triangles and the
//     BILINEAR RULED patch a chord-to-curve homotopy's own "chord point"
//     implicitly denotes at that cell → CellTwistVolumeAllow, the exact swept
//     measure |det(a,T,b)|/12 over the cell's side vectors and twist — a mechanism
//     the loft's chord-to-curve wall leg does not speak for, since that
//     leg's own homotopy starts FROM the ruled patch, never from the
//     triangle pair the evaluator actually holds; CellTwistOffsetUpper is
//     the POINTWISE deviation bound |T|/4 used by the facet-departure proof,
//     and CellTwistAreaAllow is the gap between that triangle pair's own area
//     and the ruled patch's, as the minimum of a premise-free linear arm and a
//     cancellation-preserving quadratic arm. No reading of the twist vector
//     is taken in float64;
//   - the AREA a 2D boundary displacement sweeps out → SectionDisplacementArea,
//     the same identity one dimension down: the region a recorded section can
//     move is a tube about its own recorded boundary, with
//     SectionDisplacementLength reading the same displacement as a perimeter;
//   - the DISPLACEMENT a recorded CUT PARAMETER puts on the point it names —
//     the fragment endpoint slides along its own exact carrier by the parameter
//     rounding times the carrier's speed → CutDisplacementAllow;
//   - the DISPLACEMENT a consumed source segment's own WALKED endpoint puts on
//     the point the private scene builds from it, when the segment's recorded
//     range is narrower than its own natural domain →
//     WalkEndpointAllow, charged at the SOURCE operand magnitudes the walk's
//     arithmetic touches — never at the endpoint that arithmetic produced,
//     which a cancelling difference can leave arbitrarily small;
//   - a free-form WALK ENDPOINT's own per-component bound (WalkEndBound,
//     segmentWalk's startBound/endBound and the identical reading over an
//     interior span joint), read as a 3D world-space DISTANCE the point can
//     sit from the curve it denotes → WalkEndBoundAllow, named for the
//     endpoint rather than for its one caller today (verify.go's free-form
//     tolerance-gate diameter arm) so a later reading over the same bound can
//     share it;
//   - the AREA of ONE RULED QUAD whose cap-level chord alone is displaced (the
//     cap-loop chamfer's own band patches, docs/modify-reach-design.md §8.4) →
//     BandPatchAreaAllow, the same two-factor product one patch at a time
//     rather than one whole section;
//   - the AREA of that SAME patch under a displacement of its OTHER directrix
//     — the side level, one rounded float sum, so the whole directrix
//     translates rigidly rather than moving point by point →
//     BandLevelAreaAllow;
//   - the VOLUME gap between a cap-loop chamfer's straight-ruled Cone patch
//     and the curved miter locus it denotes at a non-tangential corner →
//     ChordLocusVolumeAllow, an erosion-monotonicity sandwich between the two
//     shared-window cone-sector fluxes, the built patch's own enclosed flux's
//     reach past that interval (ChordLocusBuiltExcursion), plus each
//     mitered corner's sliver flux (ChordLocusCornerFlux); the MEASURE of the
//     region between the two solids, which the first moment reads, is
//     ChordLocusRegionAllow, a swept-volume term over a homotopy from the
//     wide sector to the built patch (ChordLocusHomotopyAreaUpper) plus a thin
//     shell over the corner wedges (ChordLocusCornerShellUpper);
//   - the FIRST MOMENT a cap-loop chamfer's contour displacement can move →
//     SweptMomentAllow, SweptVolumeAllow's own one-dimension-higher sibling;
//   - the FIRST MOMENT a loft's chorded boundary can move under the same
//     chord-to-curve homotopy → ChordedBoundaryMomentAllow,
//     which multiplies only the two three-dimensional swept measures by their
//     own radii: the wall measure reaches matchedDelta beyond the held envelope,
//     while the twist sweep stays in its four corners' convex hull;
//   - the LENGTH gap between a cap-loop chamfer's straight-ruled corner
//     miter ruling and the curved locus it denotes at a non-tangential
//     corner adjacent to a circular wall → ChordLocusLengthAllow, the
//     range's own width times a proven upper bound on the locus's own speed,
//     minus the held chord;
//   - a per-coordinate maximum read as a 3D DISTANCE → Radius3D;
//   - propagating a proven interval through a SQUARE ROOT (a candidate disk's
//     own centre distance, an Apollonius radius) → BoundedSqrt, which reads
//     the operand's own interval ends through the same rational sqrt brackets
//     (RatSqrtDown/RatSqrtUp) a free-form arc's radius already does, rather
//     than trusting math.Sqrt's accuracy on either end, and proves an exact
//     operand's float root exact through exactFloatSquare's FMA residual;
//   - a linear functional's own extreme over a bounded region moving when its
//     DIRECTION is perturbed (a revolved solid's directional extent, whose swept direction
//     carries the sweep angle's own trig enclosure) →
//     DirectionalPerturbationAllow, charged against the envelope of the very
//     coordinate that direction multiplies — which is the caller's to name,
//     since a revolve's swept radial coefficient multiplies a distance from
//     the axis and not from the profile's own frame origin;
//   - a HELD float measured against a bounded scalar's own proven enclosure of
//     the quantity that float stands for → BoundedFloatError, the bridge every
//     candidate producer crosses when it evaluates a value one way and proves
//     it another (extrude.go's circular boundary-extreme candidate, whose held
//     position runs through math.Cos/math.Sin while its enclosure comes from
//     the angle-free apex identity).
//   - a HELD scalar coefficient a directional reading lifts a boundary or
//     sweep extreme through — a prism or cap-blend box's base/gu/gv/gz, a
//     revolve box's base/wg/c0/c1 — measured against the EXACT rational image
//     of the same frame-and-placement chain the coefficient's own float
//     evaluation ran → ExactIsometryDotRound, RigidRoundAllow's tight
//     companion: zero exactly where that chain's arithmetic commits no
//     rounding for the input at hand (an identity placement, an axis-aligned
//     frame direction), never a worst-case ulp estimate where an exact
//     rational comparison is available instead.
//   - a HELD WORLD POINT an analytic builder lifts from a plane-local
//     (u, v, z) through its own frame and accumulated placement — a prism's,
//     cap-loop chamfer's, brep's or patch's vertex, a clearance carrier's
//     anchor — measured against the EXACT image of the same chain →
//     ExactFrameLiftRound, with ExactRigidRound its placement half. It reads
//     the frame origin as a leaf of the chain, so an axis-aligned plane far
//     from the world origin is charged the rounding its own origin + u sum
//     commits.
//   - a HELD PLANE-LOCAL COORDINATE a world point projects to under a frame —
//     a revolve axis's own anchor (revolve.go's axisInPlane) — measured against
//     the EXACT rational image of the same (p − origin)·axis chain the
//     coordinate's own float evaluation ran → ExactFrameLocalRound,
//     ExactIsometryDotRound's sibling one transform earlier. It charges the
//     ROUNDING that projection commits and nothing else, so a magnitude
//     envelope over the point's own distance from the frame origin — a term
//     that grows with that distance while the projection's true error stays
//     zero — never stands in for it.
//   - a HELD PLANE-LOCAL DOT PRODUCT of a direction against a frame's own axis
//     anchor — the anchor shift a revolve's extreme reading subtracts to move a
//     plane-local extreme into axis coordinates (revolve.go's
//     axisExtremeContext) — measured against the EXACT rational value of the
//     same two products and their sum → ExactPlaneDotRound. Its two products
//     round at the ANCHOR's own magnitude, so the term grows with the anchor
//     while staying the rounding it is.
//   - the same plane-local dot product gu·u + gv·v evaluated at every CANDIDATE
//     the boundary-extreme scan folds (extrude.go's
//     boundaryExtremesBoundedContext), where no single u and v is available to
//     measure against → PlaneDotDecompositionRoundAllow, ExactPlaneDotRound's
//     envelope-scaled sibling: the anchor helper answers for ONE stated point,
//     this one for a whole scan, at the SECTION's own coordinate envelope
//     rather than the anchor's magnitude. A reading that charges only the
//     anchor's dot publishes a section extreme whose own multiply-and-sum
//     rounded at a magnitude nothing else in the reading scales with.
//   - the FINAL SUMMATION a directional extent reading commits when it
//     recombines its own already-held terms into ONE published endpoint — a
//     prism's base + boundary extreme + sweep level, a revolve's or a cap-loop
//     chamfer's base + boundary extreme → ExactSumRound, ExactIsometryDotRound's
//     own companion one step later: that helper proves each COEFFICIENT right,
//     this one charges what ADDING the terms commits, and a placement can leave
//     every coefficient exactly right while the sum still rounds.
//   - the DISPLACEMENT a deliberate SNAP-TO-ZERO puts on the coordinate it
//     overwrites — a revolve walk endpoint's radial coordinate assigned exactly
//     0 within the axis's own contact tolerance (revolve.go's axisFrame.walk) →
//     SnapToZeroAllow, which charges the discarded magnitude itself, because
//     that magnitude IS the error the assignment introduces. A reading that
//     charges only the pre-snap arithmetic's own rounding publishes the assigned
//     zero as Exact and excludes the positive radius it stands for.

const (
	// UnitRoundoff is float64's u = 2⁻⁵³: the relative error a single
	// round-to-nearest operation can commit.
	UnitRoundoff = 1.1102230246251565e-16
)

// UpRound nudges a positive bound to the next representable float64, so the
// bound's own rounding can never land it below the quantity it bounds.
//
// It cannot repair a quantity that ALREADY flushed: a positive product or
// quotient that underflows arrives here as +0, and this helper leaves 0 alone
// deliberately, because nudging it would widen every legitimately EXACT zero
// in the package into a positive bound. A caller holding a +0 it has PROVEN
// positive is therefore holding a flush, not an answer, and must reach it
// through ProvenUpRound — or through ProductUpper and DivUpper, which carry
// that rule for the multiply and the divide that commit the flush.
func UpRound(x float64) float64 {
	if x > 0 {
		return math.Nextafter(x, math.Inf(1))
	}
	return x
}

// ProvenUpRound is UpRound for a value its caller has PROVEN strictly
// positive: it never publishes 0.
//
// float64's own arithmetic rounds a positive result to +0 once that result
// falls below half the smallest subnormal, and a bound of 0 is not a small
// bound — it is the CLAIM that the quantity it bounds is enclosed exactly, so
// the published interval collapses to a point that excludes the truth, and
// exactnessOf reads the same 0 as Exact for a body whose error is merely tiny.
// math.SmallestNonzeroFloat64 is the correct replacement rather than a fudge:
// rounding to +0 proves the exact result lies at or below half the smallest
// subnormal, so the smallest subnormal is itself a valid — and finite — upper
// bound on it.
//
// +Inf is deliberately NOT the answer here. A refusal would propagate to
// consumers that read a positive bound as their own gate
// (CellChordPatchNormalLower's 0 sentinel, ChordedBoundaryMomentAllow's
// matchedDelta > 0 branch) and turn a
// tiny-but-real bound into a refused reading.
//
// A caller whose operands are NOT proven positive keeps UpRound: a zero that
// is honestly zero must stay zero.
func ProvenUpRound(x float64) float64 {
	return proofarith.ProvenUpRound(x)
}

// DivUpper is ProductUpper's division twin: it carries the same "a positive
// quantity must never publish as a proven zero" rule ProvenUpRound states,
// which a bare UpRound(num/den) cannot, the quotient having already flushed
// before UpRound sees it.
//
// num must be a proven UPPER bound on the numerator and den a proven positive
// LOWER bound on the denominator, so that num/den bounds the true quotient.
// A numerator at or below zero is an ABSENT term and answers an honest 0. A
// denominator that is not finite and positive states no scale to divide by —
// including one that has itself overflowed to +Inf — and is a BROKEN caller
// claim: it answers +Inf, never a bound, the same rule
// CellChordCurveAreaUpper's own F5 refusal follows.
func DivUpper(num, den float64) float64 {
	if math.IsNaN(num) || math.IsNaN(den) || den <= 0 || math.IsInf(den, 1) {
		return math.Inf(1)
	}
	if num <= 0 {
		return 0
	}
	// A +Inf numerator needs no arm of its own: +Inf/den is +Inf, which
	// ProvenUpRound passes through as the refusal it already is.
	return ProvenUpRound(num / den)
}

// Radius3D turns a per-coordinate bound into the 3D distance bound its
// consumers read: all three coordinates can be off at once, so the corner sits
// up to √3 times the per-coordinate bound away (core §5.2 — a coordinate's
// error bound is a radius, not an axis extent).
func Radius3D(perCoord float64) float64 {
	return proofarith.Radius3D(perCoord)
}

// HeldDelta is the EXACT difference a − b of two held corners, taken over
// clearance_degen.go's own rational kernel rather than r3's float64 Sub: a
// float64 coordinate is an exact rational, so the difference is exact and
// carries none of the cancellation error the float subtraction commits. Every
// proven bound in this file that needs the LENGTH of a corner difference must
// form it this way and then take DvLenUpper of it — r3.Vec.Len is
// math.Hypot, which rounds to NEAREST and so is not an upper bound on
// anything (nor is Sub), and a final UpRound of the product buys back only
// ~1 ulp of the PRODUCT, never the norms' own inward error, which a
// near-cancelling difference can carry as a fraction of the term rather than
// an ulp of it.
//
// Both corners must already be finite (FiniteVec), which every caller here
// checks before it lifts.
func HeldDelta(a, b r3.Vec) proofarith.DyV3 {
	return proofarith.DvSub(proofarith.DyVec(a), proofarith.DyVec(b))
}

// DvLenUpper is a PROVEN upper bound on |u| for an exactly-represented vector:
// the squared length is exact dyadic arithmetic and DySqrtUp brackets its
// root by exact comparison (f·f ≥ u·u), so the published value encloses the
// true norm whatever the platform's own sqrt does. A zero vector answers
// exactly 0, so a bound that vanishes with its vector still vanishes. A norm past the float64 range answers +Inf, a
// refusal rather than a bound.
func DvLenUpper(u proofarith.DyV3) float64 { return proofarith.DySqrtUp(proofarith.DvDot(u, u)) }

// DvLenAtLeast reports whether claim is PROVABLY at least |u|, decided by
// exact comparison of claim² against u·u rather than against a rounded norm.
// It is the falsifier form a "this claimed length cannot be below its own
// chord" gate needs: exact in BOTH directions, so it neither admits a claim
// that is genuinely short nor refuses one that is exactly tight.
func DvLenAtLeast(claim float64, u proofarith.DyV3) bool {
	if !(claim >= 0) {
		return false
	}
	c, ok := proofarith.DyOf(claim)
	if !ok {
		return false
	}
	return proofarith.DyCmp(proofarith.DyMul(c, c), proofarith.DvDot(u, u)) >= 0
}

// SumSlop is a PROVEN bound on the rounding a NAIVE float64 summation of n
// terms commits, given absSum = Σ|term|. The classic bound is
// (n−1)·u/(1 − (n−1)·u) · Σ|term|, which 2·(n−1)·u·Σ|term| dominates for
// (n−1)·u ≤ ½ — true for any mesh a machine can hold. On top of it each term
// is itself a float evaluation (a cross product, a norm, a square root): a
// handful of ulps, charged as 4·u per term.
//
// Both of those charges are RELATIVE to absSum, and float64's relative error
// model stops holding where a product or quotient lands in the subnormal
// range: there one step's error is ABSOLUTE, up to 2⁻¹⁰⁷⁵ (half the subnormal
// spacing), however small the result. The summation loop is unaffected — a
// sum never rounds on underflow, since every partial sum below 2⁻¹⁰²² is a
// multiple of 2⁻¹⁰⁷⁴ — but a term's own evaluation is, so each term is also
// charged underflowTermUlps·2⁻¹⁰⁷⁴ absolute. Without that charge a subnormal
// absSum rounds both relative charges to +0, and the facet whose float area is
// 2⁻¹⁰⁷⁴ against an exact 1.25·2⁻¹⁰⁷⁴ would publish a zero bound.
//
// It is NEVER zero for a positive FINITE absSum. That is the point: a
// float-computed value is not exactly representable, so it may never reach
// Exact.
//
// A non-finite absSum is the one case it answers 0 for, because a saturated
// scale is no scale and this helper may not invent one. That answer is a
// PRECONDITION on the caller, not a bound: a caller whose absSum can saturate
// independently of the value it speaks for must state its own bound for that
// case, or the term silently vanishes (loft_moments.go's wallBound does, with
// +Inf). Every other caller here passes the held value itself as absSum and
// adds this term to it, so a saturation carries +Inf into the published bound
// on its own.
func SumSlop(n int, absSum float64) float64 {
	if n <= 0 || absSum <= 0 || IsNonFinite(absSum) {
		return 0
	}
	loop := 2 * float64(n-1) * UnitRoundoff * absSum
	terms := 4 * UnitRoundoff * absSum
	underflow := float64(n) * underflowTermUlps * math.SmallestNonzeroFloat64
	return ProvenUpRound(loop + terms + underflow)
}

// underflowTermUlps is the absolute charge, in units of the smallest
// subnormal 2⁻¹⁰⁷⁴, that SumSlop adds per term for the underflow the term's
// own float evaluation can commit.
//
// The costliest term SumSlop's callers sum is a facet's halved cross-product
// norm, b.Sub(a).Cross(c.Sub(a)).Len()/2. Subtraction and square root never
// round on underflow, and r3's Len squares a component only when it is at
// least 2⁻⁵¹¹, whose square is normal. That leaves nine steps whose underflow
// reaches the term additively: the cross product's six products, the final
// multiply of each of Len's two nested math.Hypot calls, and the halving.
// Each is off by at most 2⁻¹⁰⁷⁵ and the norm is 1-Lipschitz, so together they
// move the term by at most 9·2⁻¹⁰⁷⁵ = 4.5·2⁻¹⁰⁷⁴ before the (1 + u) factors
// of the steps that follow, which keep it below 5·2⁻¹⁰⁷⁴. A Hypot's own
// divide and square feed 1 + r² ≥ 1, so their underflow reaches the term only
// as a relative error far below u, inside the 4·u per-term charge. A plain
// length term, Len of a difference, commits only the two Hypot multiplies.
//
// A caller whose term commits more underflow-prone steps than this must
// charge the excess itself.
const underflowTermUlps = 5

// CrossProductUpper bounds every component of a×b and every product the float
// evaluation forms on the way: each component is a difference of two of the
// six products below, so their absolute sum dominates all of them AND the
// resulting vector's own norm. Where a and b are nearly parallel that
// difference cancels, and the cross product's rounding error tracks this sum
// rather than the norm it cancelled to.
func CrossProductUpper(a, b r3.Vec) float64 {
	return AbsSumUpper(
		ProductUpper(math.Abs(a.Y), math.Abs(b.Z)),
		ProductUpper(math.Abs(a.Z), math.Abs(b.Y)),
		ProductUpper(math.Abs(a.Z), math.Abs(b.X)),
		ProductUpper(math.Abs(a.X), math.Abs(b.Z)),
		ProductUpper(math.Abs(a.X), math.Abs(b.Y)),
		ProductUpper(math.Abs(a.Y), math.Abs(b.X)),
	)
}

// FacetAreaTermSlop is a PROVEN bound on how far one facet's float area term,
// b.Sub(a).Cross(c.Sub(a)).Len()/2, sits from the exact area of the triangle
// its three held vertices span. A caller summing such terms adds it per facet
// beside SumSlop, which bounds the summation loop.
//
// SumSlop's own per-term charge is RELATIVE to the term, and that is not
// enough for a sliver. The two edge subtractions and the six products of the
// cross product each round relative to their OWN magnitudes, which reach
// |b−a|·|c−a| while the area can be smaller by any factor: a facet whose three
// vertices are nearly collinear cancels in every component. The edge
// subtractions move each product by at most (2u + u²) of itself, the products
// round by u of themselves, so the cross product is off by at most
// (3u + u²)·CrossProductUpper in the 1-norm, which dominates its 2-norm. The
// norm is 1-Lipschitz, and the component subtractions, the norm's own
// rounding and the halving are relative errors of a value no larger than
// CrossProductUpper. AnalyticRoundBound's 256·u at that scale covers the whole
// chain with room for a norm that carries no tight ulp contract — the same
// charge internal/capband's patch areas take.
//
// Each of those steps can also underflow, where the error is absolute and no
// relative charge speaks for it. The underflowTermUlps charge is added
// unconditionally, so a facet whose float area flushes to exactly zero still
// takes a positive bound.
func FacetAreaTermSlop(a, b, c r3.Vec) float64 {
	env := CrossProductUpper(b.Sub(a), c.Sub(a))
	return AbsSumUpper(AnalyticRoundBound(env), underflowTermUlps*math.SmallestNonzeroFloat64)
}

// ChainLengthBound is the proven bound on a boolean rim's length: the chain
// holds nSegs chords whose two endpoints EACH move by up to delta, so the
// held length can be off by 2·nSegs·delta — plus the float slop of summing
// nSegs square roots. Never zero for a positive float-computed length, even when
// delta is (an all-planar boolean's rim is a float sum of sqrts, and the last
// ulp is not free).
func ChainLengthBound(nSegs int, delta, heldLen float64) float64 {
	if nSegs <= 0 {
		return 0
	}
	return UpRound(2*float64(nSegs)*delta + SumSlop(nSegs, heldLen))
}

// MaxFiniteUlp is 2⁹⁷¹, the spacing between the two largest finite float64s
// and therefore the LARGEST spacing any pair of adjacent finite float64s has.
// Half of it bounds the rounding a single operation whose result is finite can
// commit, whatever that result's magnitude.
const MaxFiniteUlp = 0x1p971

// RigidRoundAllow bounds the rounding a rigid motion commits on one point.
// The rounding happens INSIDE the products and sums — at the magnitude of the
// INPUT coordinate and of the translation — not at the magnitude of the
// result: a body built far from the origin and moved back rounds at the far
// magnitude and would be charged at the near one. Every intermediate stays
// under 2·maxInputAbs + maxTransAbs (an orthonormal row's dot product is at
// most √3·|input|), 16 ulps there cover the products, the two sums and the
// translation, and the consumers read a 3D distance, so ×√3.
//
// That scale is a MAGNITUDE, not a value the motion produces, so it saturates
// on inputs the motion itself handles perfectly well: 2·maxInputAbs overflows
// for any coordinate above MaxFloat64/2, and UlpOf answers +Inf at MaxFloat64
// and NaN past it, since its own math.Nextafter step leaves the finite range.
// The answer must never be NaN. NaN is not a large bound but the ABSENCE of
// one, and it is silent: NaN > 0 is false, so every consumer's own `delta > 0`
// widening is skipped and the term vanishes from the measurements it was
// supposed to widen. So a saturated scale falls back to MaxFiniteUlp, which
// is a PROVEN charge rather than a substitute for one: this helper answers for
// a point whose placed coordinates the caller has already proven finite (a
// non-finite coordinate is that caller's own refusal — docs/loft-design.md
// Table S row S13 for a loft), an operation with a finite result rounds by at
// most half the spacing at that result, and 16·MaxFiniteUlp dominates the six
// products and sums one placed coordinate commits (three products, two sums
// joining them, and the translation's own).
//
// A second call site is the plane-frame lift of a COMPUTED chord station (a
// loft's own curved-pairing reach, docs/loft-design.md §5 — the chord-chain
// subsection lands with the arc design change): the mechanism
// is identical, an orthonormal map (Plane.ToWorldUV, in effect r3's own
// Frame) composed with a translation, so the rounding it commits is bounded
// the same way — at the pre-lift plane-local coordinate's own magnitude and
// the frame origin's, never at the lifted world point's.
func RigidRoundAllow(maxInputAbs, maxTransAbs float64) float64 {
	m := 2*math.Abs(maxInputAbs) + math.Abs(maxTransAbs)
	ulp := UlpOf(m)
	if IsNonFinite(ulp) {
		ulp = MaxFiniteUlp
	}
	return Radius3D(16 * ulp)
}

// ExactFrameLiftRound proves, exactly, how far one held world point sits from
// the point a plane-local (u, v, z) denotes under frame and then xform: the
// construction prismPayload.point evaluates in float64 (frame.ToWorldUV(u, v)
// plus frame.N()·z, then xform.Apply). The denoted point is the same chain
// over dyadic rationals, with the frame's origin, U, V and N, the placement's
// basis and translation, and u, v, z all read as exact leaves, and the answer
// is the 3D distance bound (Radius3D) of the per-coordinate gap between that
// value and held, rounded upward.
//
// held must be the SAME float point the caller's own arithmetic produced, so
// the comparison measures the rounding that arithmetic committed. Every term of
// the chain counts: ToWorldUV's own origin + u·U + v·V sum rounds at the
// origin's magnitude even for an axis-aligned frame (a sketch plane at
// x = 10⁶ + 0.1 rounds every vertex by up to half an ulp at 10⁶), and a
// placement adds its own rounding on top. A body built far away and placed back
// is charged for the far lift, because the far lift is where it rounded.
//
// The answer is zero exactly where the whole chain is exact for the input at
// hand: an axis-aligned frame with an integer origin and integer coordinates
// under the identity placement, for example. A non-finite leaf answers +Inf,
// never 0: an absent bound must never read as a small one.
//
// This is the per-vertex bound every analytic builder's own frame lift takes
// (prismPayload's rim and side vertices, capBlendPayload's cap-level vertices,
// brepPayload's vertices, patchPayload's rim vertices, the clearance kernel's
// prism carriers). Tessellation reads the same helper for its own cap and wall
// vertices, through prism_payload.go's exactPrismPointRound.
func ExactFrameLiftRound(frame r3.Frame, xform r3.Transform, u, v, z float64, held r3.Vec) float64 {
	basis := xform.Basis()
	origin, fu, fv, fn := frame.Origin(), frame.U(), frame.V(), frame.N()
	tr := xform.Translation()
	for _, w := range [...]r3.Vec{origin, fu, fv, fn, basis.EX, basis.EY, basis.EZ, tr} {
		if !FiniteVec(w) {
			return math.Inf(1)
		}
	}
	if IsNonFinite(u) || IsNonFinite(v) || IsNonFinite(z) {
		return math.Inf(1)
	}
	o, du, dv, dn := proofarith.DyVec(origin), proofarith.DyVec(fu), proofarith.DyVec(fv), proofarith.DyVec(fn)
	ru, rv, rz := proofarith.MustDyOf(u), proofarith.MustDyOf(v), proofarith.MustDyOf(z)
	var local proofarith.DyV3
	for i := range local {
		local[i] = proofarith.DyAdd(
			proofarith.DyAdd(o[i], proofarith.DyMul(du[i], ru)),
			proofarith.DyAdd(proofarith.DyMul(dv[i], rv), proofarith.DyMul(dn[i], rz)),
		)
	}
	return ExactRigidRound(basis, tr, local, held)
}

// ExactRigidRound is ExactFrameLiftRound's second half: the Radius3D bound of
// the per-coordinate gap between held and the exact rigid image
// basis·local + translation of an exact pre-placement point. Every caller has
// already proven basis, translation and local finite.
func ExactRigidRound(basis r3.Basis, translation r3.Vec, local proofarith.DyV3, held r3.Vec) float64 {
	ex, ey, ez, t := proofarith.DyVec(basis.EX), proofarith.DyVec(basis.EY), proofarith.DyVec(basis.EZ), proofarith.DyVec(translation)
	h := [3]float64{held.X, held.Y, held.Z}
	perCoord := 0.0
	for i := range h {
		exact := proofarith.DyAdd(
			proofarith.DyAdd(proofarith.DyMul(ex[i], local[0]), proofarith.DyMul(ey[i], local[1])),
			proofarith.DyAdd(proofarith.DyMul(ez[i], local[2]), t[i]),
		)
		perCoord = max(perCoord, proofarith.DyRoundedFloatError(exact, h[i]))
	}
	return Radius3D(perCoord)
}

// DirRoundAllow bounds the rounding a builder's own `dir`-style construction
// commits on one DIRECTION: a plane-local direction of magnitude at most
// maxInputAbs, combined from the payload's own frame vectors and then carried
// through the accumulated placement's rotation (`prismPayload.dir`,
// `revolvePayload`'s equivalent radial/velocity directions) — the same
// two-step map `ExactFrameLiftRound` measures for a POINT, with the origin and
// the translation dropped: a direction carries neither, so neither the frame
// origin's magnitude nor the translation's enters the rounding, and
// `RigidRoundAllow`'s own maxTransAbs term is zero.
//
// It is zero exactly where both the frame is axis-aligned and the placement
// is Identity: an axis-aligned frame's U/V/N combine only 0, 1 and -1
// coefficients, each exact in float64, and `ApplyDir` under Identity changes
// nothing — so under that exemption every direction `dir` builds is bit-exact,
// and the CROSS product of two such directions (a plane's own outward normal,
// `addPrismFaces`/`addRevolveFaces`) is a cross product of standard basis
// vectors, itself exact. Off that exemption, the charge is sound but not
// claimed tight.
func DirRoundAllow(frame r3.Frame, xform r3.Transform, maxInputAbs float64) float64 {
	trivialFrame := frame.U() == r3.NewVec(1, 0, 0) && frame.V() == r3.NewVec(0, 1, 0)
	if trivialFrame && xform == r3.Identity() {
		return 0
	}
	return RigidRoundAllow(maxInputAbs, 0)
}

// PerturbedAreaUpper bounds the total facet area of a mesh whose vertices may
// each sit up to delta from the HELD ones — and of every mesh on the straight
// path between the two, which is what the swept-volume bound integrates over.
//
// Per facet, with held edge vectors u', v' and true ones u = u' + du,
// v = v' + dv (|du|, |dv| ≤ 2·delta, each endpoint moving by up to delta):
// |u × v| ≤ |u' × v'| + 2·delta·(|u'| + |v'|) + 4·delta², so the true area is
// at most the held area plus delta·(|u'| + |v'|) + 2·delta². A facet the weld
// COLLAPSED holds zero area and the correction is the whole of its bound —
// which is the point: it is the only term that speaks for it.
//
// Every per-facet term is a PROVEN upper bound before it is summed:
// PerturbedTriangleAreaUpper forms the cross product and both edge vectors in
// exact dyadic arithmetic and roots them through DySqrtUp, so a thin facet's
// float cross-product error, which scales with the products rather than the
// result, never enters. The loft's volume residual depends on it
// (docs/loft-gear-bounds-design.md §2), so it may not be a float estimate.
// The summation slop of those held terms is SumSlop's. A non-finite vertex
// answers +Inf: no term can be stated for it.
func PerturbedAreaUpper(verts []r3.Vec, tris [][3]int, delta float64) float64 {
	area, _ := PerturbedAreaUpperWithBudget(nil, verts, tris, delta)
	return area
}

// PerturbedTriangleAreaAllow bounds how far ONE triangle's area can move when
// each of its three vertices sits within delta of the held ones: delta·(|u| +
// |v|) + 2·delta², u and v the triangle's own edge vectors b-a and c-a — the
// per-facet correction term PerturbedAreaUpper's own doc comment derives,
// pulled out as its single owner (docs/loft-design.md PR 2a) so a caller that
// needs one triangle's own allowance, rather than a whole mesh's held area
// plus the same correction summed over every facet, does not recompute it
// inline.
//
// It is proven for ONE TRIANGLE's own area and is spent for nothing else: a
// held facet against the facet its displaced vertices denote, and a cap's
// triangulation against the polygon its displaced vertices denote. The AREA
// STEP between two ruled SURFACES is a different quantity of the same shape and
// order, and CellStationShiftAreaAllow charges it from its own inequality.
func PerturbedTriangleAreaAllow(a, b, c r3.Vec, delta float64) float64 {
	u, v := b.Sub(a), c.Sub(a)
	return delta*(u.Len()+v.Len()) + 2*delta*delta
}

// PerturbedTriangleAreaUpper is a PROVEN upper bound on one facet's held area
// plus PerturbedTriangleAreaAllow's correction, |u' × v'|/2 + delta·(|u'| +
// |v'|) + 2·delta², with every norm read from exact dyadic coordinates through
// DySqrtUp and every sum and product rounded outward. A non-finite vertex
// answers +Inf.
func PerturbedTriangleAreaUpper(a, b, c r3.Vec, delta float64) float64 {
	if !FiniteVec(a) || !FiniteVec(b) || !FiniteVec(c) {
		return math.Inf(1)
	}
	u, v := HeldDelta(b, a), HeldDelta(c, a)
	n := proofarith.DvCross(u, v)
	area := proofarith.DySqrtUp(proofarith.DyShift(proofarith.DvDot(n, n), -2))
	allow := AbsSumUpper(
		ProductUpper(delta, AbsSumUpper(DvLenUpper(u), DvLenUpper(v))),
		ProductUpper(2, ProductUpper(delta, delta)),
	)
	return AbsSumUpper(area, allow)
}

func PerturbedAreaUpperContext(
	ctx context.Context,
	verts []r3.Vec,
	tris [][3]int,
	delta float64,
) (float64, error) {
	return PerturbedAreaUpperWithBudget(NewWorkBudget(ctx), verts, tris, delta)
}

func PerturbedAreaUpperWithBudget(
	budget *WorkBudget,
	verts []r3.Vec,
	tris [][3]int,
	delta float64,
) (float64, error) {
	total := 0.0
	for _, t := range tris {
		if budget != nil {
			if err := budget.Step(); err != nil {
				return 0, err
			}
		}
		total += PerturbedTriangleAreaUpper(verts[t[0]], verts[t[1]], verts[t[2]], delta)
	}
	if budget != nil {
		if err := budget.Err(); err != nil {
			return 0, err
		}
	}
	return UpRound(total + SumSlop(len(tris), total)), nil
}

// SweptVolumeAllow bounds the volume between two closed meshes whose vertices
// correspond and differ by at most delta. The signed volume is a polynomial in
// the vertices, so along the straight path from one mesh to the other
// |dV/dt| ≤ delta · A(t) — every boundary point moves at speed at most delta,
// and it can only displace volume at the rate the area it sweeps allows. So
// |V' − V| ≤ delta · sup A(t), and areaUpper must bound the area along the WHOLE
// path (PerturbedAreaUpper does).
//
// The identity holds facet by facet, so it holds whatever the rounding does to
// the mesh's shape — a facet flattened to zero area still answers for the volume
// it swept getting there. What it needs is the area of the surface the motion
// acted ON: charge it against what survived the motion and the collapsed facets'
// own swept volume drops silently out of the bound.
func SweptVolumeAllow(delta, areaUpper float64) float64 {
	if delta <= 0 || areaUpper <= 0 {
		return 0
	}
	return ProductUpper(delta, areaUpper)
}

// CellChordCurveAreaUpper bounds the AREA of EVERY surface ONE loft wall
// cell's chord-to-curve homotopy visits — the bilinear RULED patch between
// the cell's four chord corners at homotopy-time t=0 through to the RULED
// surface between the two TRUE recorded curves the cell's two sides denote
// at t=1, and every intermediate surface between them
// (docs/loft-design.md §5 — the chord-chain subsection lands with the arc
// design change).
//
// This is an ABSOLUTE bound, never a held-facet-area-plus-excess reading:
// Area(patch) − Area(held triangles) is not sign-definite (a cell can hold
// almost no triangle area while its ruled patch carries substantial area —
// vLo=(0,0,0) vHi=(1,0,0) wLo=(0,1,h) wHi=(0,0,h) at small h holds a chord
// facet area of h while its own bilinear patch already carries area 1/3 +
// O(h)), so no fixed held quantity the excess could subtract from bounds it.
//
// The derivation fixes s in [0,1] to parametrize EACH side under ONE SHARED
// parametrization the caller chooses — constant ARC-LENGTH speed for the
// circular arm, the free-form arm's own SPAN-UNIFORM native fraction (the
// identical parameter the loft correspondence itself pairs stations on,
// internal/freeform/spline_sagitta.go's spanMatchedDeltaUpper) — never the side's own native
// curve parameter used unmatched. Nothing below actually depends on which
// parametrization was chosen: the derivation reads exactly two properties of
// it, and nothing more. (1) a per-side tangent-magnitude bound, arcLenUpperA/
// arcLenUpperB, that is at least that side's own chord length UNDER that
// parametrization. (2) matchedDeltaUpper, a bound on |curve(s) − chord(s)| at
// the SAME s, proven under that SAME shared parametrization on BOTH sides.
// Arc length is one such choice — the circular arm's, whose sagitta discharges
// it exactly for the IDEAL chord (loftCircularCellStations' own doc comment),
// leaving the held stations' own displacement for the caller to compose in.
// The free-form arm's
// span-uniform fraction is another, discharged by spanMatchedDeltaUpper. A
// caller adopting a third parametrization may reuse this derivation unchanged
// provided it proves the same two properties under it. With the shared
// parametrization fixed, it defines one (t-independent) homotopy
// a_t(s) = (1−t)·a_chord(s) + t·a_curve(s) for each side,
// X_t(s,r) = (1−r)·a_t(s) + r·b_t(s) the cell's own ruled surface at time t:
//
//   - ∂a_t/∂s = (1−t)·(vHi−vLo) + t·a_curve'(s) is a convex combination (in
//     t, for fixed s) of a CONSTANT vector of magnitude the chord length and
//     a vector of magnitude arcLenUpper (the shared parametrization's own
//     speed bound), so |∂a_t/∂s| <= max(chordLen, arcLenUpper) =
//     arcLenUpper (a chord never exceeds the curve length it subtends, arc or
//     free-form span alike — property (1) above). The same
//     holds for b_t, so eA := max(arcLenUpperA, arcLenUpperB) bounds every
//     ∂X_t/∂s = (1−r)·∂a_t/∂s + r·∂b_t/∂s (itself a convex combination of the
//     two) at every s, r, t.
//   - ∂X_t/∂r = b_t(s) − a_t(s) = (1−t)·[b_chord(s)−a_chord(s)] +
//     t·[b_curve(s)−a_curve(s)], again a convex combination in t. The chord
//     term is linear in s (b_chord(s)−a_chord(s) = (1−s)·(wLo−vLo) +
//     s·(wHi−vHi)), so it is itself bounded by eBBase :=
//     max(|wLo−vLo|,|wHi−vHi|) via the SAME convexity CellTwistVolumeAllow's
//     part (b) uses — and read through the SAME CellSpanUpper that part uses,
//     never r3.Vec.Len, for the reason CellSpanUpper's own doc comment gives.
//     The curve term is within matchedDeltaUpper of the matching chord term
//     AT THE SAME s — never merely somewhere on the
//     chord, which is all a SAGITTA (a SET-distance from the curve to its
//     nearest chord point, decad's own sectionDelta) proves — so triangle
//     inequality gives |b_curve(s)−a_curve(s)| <= eBBase +
//     2·matchedDeltaUpper. eB := eBBase + 2·matchedDeltaUpper therefore
//     bounds |∂X_t/∂r| at every s, r, t (2·matchedDeltaUpper only ever
//     widens the chord case's own exact bound, so one eB serves both
//     endpoints of the t sweep).
//   - |∂X_t/∂s × ∂X_t/∂r| <= |∂X_t/∂s|·|∂X_t/∂r| <= eA·eB pointwise, so
//     Area(X_t) <= eA·eB for every t in [0,1] (the (s,r) domain is the unit
//     square, area 1) — the published bound, ready for the loft's
//     per-cell wall volume leg, matchedDelta · sup_t A(t) at that cell
//     (docs/loft-gear-bounds-design.md §2).
//
// matchedDeltaUpper is a DIFFERENT, STRONGER quantity than the loft
// evaluator's own sectionDelta field (loftPayload.sectionDelta,
// loft_build.go) and the two are never interchangeable. sectionDelta is a
// SET-distance sagitta: every curve point sits within it of SOME chord
// point. matchedDeltaUpper must be a PROVEN bound on |curve(s) − chord(s)|
// at the SAME s under the caller's OWN shared parametrization (above) — a
// PARAMETER-MATCHED bound. The two coincide only when the curve's own
// natural traversal already matches the parametrization the caller chose: a
// straight LINE (trivially, chord(s)=curve(s) identically, both zero) or a
// circular ARC under its own uniform-angle parametrization
// (TestArcMatchedDeltaEqualsSagitta derives that equality analytically and
// checks every step of the derivation over exact rationals, covering every
// cell angle chordCount can produce). For any other curve kind under an
// ARC-LENGTH parametrization the two provably DIFFER, by an amount that can reach the CHORD LENGTH
// itself rather than the sagitta: a curve can hug its chord within an
// arbitrarily small sagitta while packing almost all of its arc length into
// one short span, so its arc-length-matched point sits far from the chord
// point at the same s (TestCellChordCurveAreaUpperRefusesTheSagittaZigzag
// pins the exact counterexample). A free-form span under its OWN
// span-uniform native fraction is the one other case this derivation admits
// today, discharged not by the sagitta but by spanMatchedDeltaUpper
// (internal/freeform/spline_sagitta.go), which is proven under that exact parametrization
// rather than assumed equal to it. A caller that cannot prove the
// parameter-matched bound under whichever shared parametrization it adopted
// must pass +Inf, never the sagitta as a stand-in (F1's own rule: a
// SET-distance may never be silently upgraded into a parameter-matched
// one) — every caller reaching this helper today either proves the bound or
// has none to pass. The loft evaluator proves it in two composed legs
// (docs/loft-design.md §5.2's matchedDelta row): the arm's own matched
// departure for the IDEAL chord — the circular arm's sagitta, or the
// free-form arm's spanMatchedDeltaUpper — plus the displacement of the two
// HELD stations the chord this helper reads actually joins, which is what
// makes the published bound a claim about the corners passed in rather than
// about a chord the build never drew.
//
// arcLenUpperA and arcLenUpperB must each be a PROVEN upper bound on that
// side's own arc length over the cell — never smaller than the corresponding
// chord length, which the derivation's first bullet depends on (a chord
// never exceeds the arc it subtends). That premise is falsified against
// CellSpanUpper of the side's own chord rather than r3.Vec.Len: a raw norm
// on the REFUSING side of the comparison rounds DOWN, so it admits a claim
// that provably sits below the true chord, while the certified upper endpoint
// can only over-refuse by an ulp — the reject-only-safe direction. Even a
// caller whose side is a straight LINE must therefore state its chord through
// CellSpanUpper, since r3.Vec.Len of that same chord is not itself a proven
// upper bound on it. Any of the three non-finite, either arc length claim
// negative or smaller than its own chord, matchedDeltaUpper
// negative, or any of the four corners carrying a non-finite coordinate, is
// a BROKEN caller claim and this helper answers +Inf for it, never 0
// (CutDisplacementAllow's own rule): a claim this derivation's own premises
// falsify must never publish a shrunken bound. Zero stays the answer for a
// wholly degenerate cell (both sides zero length), which legitimately has no
// area for a ruled surface to sweep.
func CellChordCurveAreaUpper(vLo, vHi, wLo, wHi r3.Vec, arcLenUpperA, arcLenUpperB, matchedDeltaUpper float64) float64 {
	if !FiniteVec(vLo) || !FiniteVec(vHi) || !FiniteVec(wLo) || !FiniteVec(wHi) {
		return math.Inf(1)
	}
	if !CellChordClaimsStated(arcLenUpperA, arcLenUpperB, matchedDeltaUpper) {
		return math.Inf(1)
	}
	return CellChordCurveAreaFromSpans(CellSpansOf(vLo, vHi, wLo, wHi), arcLenUpperA, arcLenUpperB, matchedDeltaUpper)
}

// CellChordClaimsStated reports whether the three scalar claims
// CellChordCurveAreaUpper takes are stateable at all: finite and
// non-negative. It is the single owner of that gate, read by every entry
// point into the bound, so no caller can be admitted under one spelling of
// it and refused under another. A claim it rejects is a BROKEN caller claim
// and its own entry point answers +Inf, never 0 (CellChordCurveAreaUpper's
// own doc comment).
func CellChordClaimsStated(arcLenUpperA, arcLenUpperB, matchedDeltaUpper float64) bool {
	if IsNonFinite(arcLenUpperA) || IsNonFinite(arcLenUpperB) || IsNonFinite(matchedDeltaUpper) {
		return false
	}
	return arcLenUpperA >= 0 && arcLenUpperB >= 0 && matchedDeltaUpper >= 0
}

// CellChordCurveAreaFromSpans is CellChordCurveAreaUpper's own derivation
// once that helper's corner-finiteness and scalar-claim gates have passed
// and the cell's four certified spans are in hand. It is the ONE
// implementation of the published area bound: CellChordCurveAreaUpper and
// CellAllowsOf both return exactly what it returns, so which entry point a
// caller takes changes only which computations happen, never the number.
//
// The premise the derivation's first bullet rests on — an arc-length claim
// never below its own side's chord — is falsified here against spans.sideA
// and spans.sideB, the CERTIFIED endpoints CellSpanUpper publishes, for the
// reason CellChordCurveAreaUpper's own doc comment gives.
func CellChordCurveAreaFromSpans(spans CellSpans, arcLenUpperA, arcLenUpperB, matchedDeltaUpper float64) float64 {
	if arcLenUpperA < spans.SideA || arcLenUpperB < spans.SideB {
		return math.Inf(1)
	}
	eA := math.Max(arcLenUpperA, arcLenUpperB)
	if eA <= 0 {
		return 0
	}
	eBBase := math.Max(spans.RungLo, spans.RungHi)
	eB := AbsSumUpper(eBBase, ProductUpper(2, matchedDeltaUpper))
	return ProductUpper(eA, eB)
}

// CellTwistVolumeAllow bounds the swept volume between one loft wall cell's
// held triangle pair and its bilinear ruled patch. Put a=vHi-vLo,
// b=wLo-vLo and T=vLo-vHi-wLo+wHi. On the two parameter triangles the ruled
// patch's displacement from the held surface is respectively
// r(s-1)T and s(r-1)T. It vanishes on the unit square's boundary.
//
// Along the straight homotopy, dotting that displacement with the surface
// normal cancels every term containing T twice. Its absolute Jacobian is
// therefore |det(a,T,b)| times r(1-s) on 0<=r<=s<=1 and s(1-r) on
// 0<=s<=r<=1, independent of the homotopy parameter. Each triangular factor
// integrates to 1/24, so the complete swept measure is exactly
//
//	|det(a,T,b)| / 12.
//
// This parameterized swept measure bounds the occupied symmetric difference
// even if the sweep overlaps itself, so the moment twin may spend it as a
// measure as well as Volume spending it as a magnitude. The determinant is
// formed exactly over big.Rat and only the final quotient rounds outward.
// Non-finite corners answer +Inf before the exact lift.
func CellTwistVolumeAllow(vLo, vHi, wLo, wHi r3.Vec) float64 {
	if !FiniteVec(vLo) || !FiniteVec(vHi) || !FiniteVec(wLo) || !FiniteVec(wHi) {
		return math.Inf(1)
	}
	det := CellTwistVolume(vLo, vHi, wLo, wHi)
	if det.Sign() == 0 {
		return 0
	}
	det.Abs(det)
	return RatFloatUp(det)
}

// CellTwistVolume returns the exact signed volume correction from one held
// wall-cell triangle pair to its bilinear ruled patch. The derivation is
// CellTwistVolumeAllow's; this form preserves the determinant's sign and
// delays all rounding until the complete build has summed its cells.
// Callers must prove every corner finite before entering the rational lift.
func CellTwistVolume(vLo, vHi, wLo, wHi r3.Vec) *big.Rat {
	a := HeldDelta(vHi, vLo)
	b := HeldDelta(wLo, vLo)
	twist := proofarith.DvSub(HeldDelta(vLo, vHi), HeldDelta(wLo, wHi))
	det := proofarith.DvDot(a, proofarith.DvCross(twist, b))
	return new(big.Rat).Quo(det.Rat(), big.NewRat(12, 1))
}

// RatV3 is a triple of exact RATIONALS, for the one family of readings in this
// package whose arithmetic genuinely leaves the dyadic set: the moment
// integrals below divide by (i+1)(j+1) and by factorial denominators, and
// neither is a power of two, so their results are fractions no binary exponent
// can state (dyadic.go's own doc comment owns that boundary). Every OTHER exact
// vector here is a dyV3 — the cross products, dots and determinants these
// integrals are built FROM included, which is why the integrals take dyV3 and
// return big.Rat.
type RatV3 [3]*big.Rat

// CellTwistMoment returns the exact correction to loftMassAccumulator's
// first-moment numerators when one held wall-cell triangle pair is replaced
// by its bilinear patch. The returned components use the accumulator's
// scaling: each is 24 times the corresponding first moment about anchor.
//
// For q=X-anchor, the divergence theorem gives the i-th first moment as
// 1/2 integral(q_i^2 n_i). The bilinear patch has
// q=q0+s*a+r*b+s*r*T and n=(a+r*T)x(b+s*T), so the integrand is a polynomial
// integrated exactly over the unit square. The two held triangles use the
// same identity over their parameter simplices. Their difference, multiplied
// by 12, is therefore the exact correction in loftMassAccumulator's units.
func CellTwistMoment(vLo, vHi, wLo, wHi, anchor r3.Vec) RatV3 {
	qLo := HeldDelta(vLo, anchor)
	qHi := HeldDelta(vHi, anchor)
	qWLo := HeldDelta(wLo, anchor)
	qWHi := HeldDelta(wHi, anchor)
	a := proofarith.DvSub(qHi, qLo)
	b := proofarith.DvSub(qWLo, qLo)
	twist := proofarith.DvSub(proofarith.DvSub(qLo, qHi), proofarith.DvSub(qWLo, qWHi))

	var out RatV3
	for axis := range out {
		patch := BilinearPatchMomentIntegral(qLo, a, b, twist, axis)
		held0 := TriangleMomentIntegral(qLo, qHi, qWHi, axis)
		held1 := TriangleMomentIntegral(qLo, qWHi, qWLo, axis)
		out[axis] = new(big.Rat).Sub(patch, held0)
		out[axis].Sub(out[axis], held1)
		out[axis].Mul(out[axis], big.NewRat(12, 1))
	}
	return out
}

// CellTwistMomentFromVolume reuses the exact signed correction as a planarity
// certificate. Its determinant is zero exactly when the four corners are
// coplanar. In one plane, the bilinear patch and the two held triangles have
// the same oriented boundary, so their signed first-moment fluxes agree even
// for a tapered or self-crossing quadrilateral and any anchor. A nonzero
// determinant keeps the full integration path.
func CellTwistMomentFromVolume(vLo, vHi, wLo, wHi, anchor r3.Vec, signed *big.Rat) RatV3 {
	if signed.Sign() != 0 {
		return CellTwistMoment(vLo, vHi, wLo, wHi, anchor)
	}
	var out RatV3
	for axis := range out {
		out[axis] = new(big.Rat)
	}
	return out
}

type MomentPoly map[[2]int]*big.Rat

func MomentPolyAdd(p MomentPoly, degree [2]int, term *big.Rat) {
	if term.Sign() == 0 {
		return
	}
	if p[degree] == nil {
		p[degree] = new(big.Rat)
	}
	p[degree].Add(p[degree], term)
}

func MomentPolyMul(a, b MomentPoly) MomentPoly {
	out := make(MomentPoly)
	for da, ca := range a {
		for db, cb := range b {
			MomentPolyAdd(out, [2]int{da[0] + db[0], da[1] + db[1]}, new(big.Rat).Mul(ca, cb))
		}
	}
	return out
}

func BilinearPatchMomentIntegral(q0, a, b, twist proofarith.DyV3, axis int) *big.Rat {
	q := MomentPoly{
		{0, 0}: q0[axis].Rat(),
		{1, 0}: a[axis].Rat(),
		{0, 1}: b[axis].Rat(),
		{1, 1}: twist[axis].Rat(),
	}
	n0 := proofarith.DvCross(a, b)
	ns := proofarith.DvCross(a, twist)
	nr := proofarith.DvCross(twist, b)
	n := MomentPoly{
		{0, 0}: n0[axis].Rat(),
		{1, 0}: ns[axis].Rat(),
		{0, 1}: nr[axis].Rat(),
	}
	integrand := MomentPolyMul(MomentPolyMul(q, q), n)
	out := new(big.Rat)
	for degree, coefficient := range integrand {
		den := int64((degree[0] + 1) * (degree[1] + 1))
		out.Add(out, new(big.Rat).Quo(coefficient, big.NewRat(den, 1)))
	}
	return out
}

func TriangleMomentIntegral(q0, q1, q2 proofarith.DyV3, axis int) *big.Rat {
	e1 := proofarith.DvSub(q1, q0)
	e2 := proofarith.DvSub(q2, q0)
	q := MomentPoly{
		{0, 0}: q0[axis].Rat(),
		{1, 0}: e1[axis].Rat(),
		{0, 1}: e2[axis].Rat(),
	}
	q2Poly := MomentPolyMul(q, q)
	n := proofarith.DvCross(e1, e2)[axis].Rat()
	out := new(big.Rat)
	for degree, coefficient := range q2Poly {
		// Integral over s>=0, r>=0, s+r<=1 of s^i*r^j is
		// i!*j!/(i+j+2)!; the polynomial degree here is at most two.
		num := int64(1)
		for i := 2; i <= degree[0]; i++ {
			num *= int64(i)
		}
		for i := 2; i <= degree[1]; i++ {
			num *= int64(i)
		}
		den := int64(1)
		for i := 2; i <= degree[0]+degree[1]+2; i++ {
			den *= int64(i)
		}
		term := new(big.Rat).Mul(coefficient, n)
		term.Mul(term, big.NewRat(num, den))
		out.Add(out, term)
	}
	return out
}

// CellTwistQuarterUpper is the CERTIFIED upper endpoint of |T|/4 for a wall
// cell, the single quantity CellTwistAreaLinearFromSpans and
// CellTwistOffsetUpper both rest on. Nothing about it may be computed in
// float64:
//
//   - T = vLo − vHi − wLo + wHi is a CANCELLING chain. An ordinary
//     parallelogram cell — one whose two rules were themselves built by adding
//     a common offset — drives the float chain to EXACTLY (0,0,0) while the
//     exact T, the addition's own rounding residue, is nonzero, so a computed
//     zero proves nothing and a computed magnitude carries unbounded relative
//     loss. The four corners are float64 and therefore exact rationals, so the
//     chain is evaluated exactly, with no rounding at all, and answers 0 only
//     when T is EXACTLY the zero vector.
//   - |T| is a square root, and r3.Vec.Len is nested math.Hypot, which Go
//     publishes no accuracy contract for and which falls several ulp BELOW the
//     exact norm for a large share of vectors. The magnitude therefore comes
//     from RatSqrtUp of the exact |T|²/16, whose answer is decided by exact
//     rational comparison rather than by any libm's rounding.
//
// The chain runs over the HOMOGENEOUS INTEGER kernel (xpt, internal/proof/exact_point.go),
// which carries the same exact values with no normalisation per operation —
// xhp's own doc comment gives the reason — and materialises one big.Rat at the
// end, for RatSqrtUp alone. A big.Rat is canonical, so the value handed over
// decides RatSqrtUp's answer by itself: the representation the chain took to
// reach it cannot change the endpoint published.
func CellTwistQuarterUpper(vLo, vHi, wLo, wHi r3.Vec) float64 {
	return XtwistQuarterUpper(CellCornersOf(vLo, vHi, wLo, wHi))
}

// XtwistQuarterUpper is CellTwistQuarterUpper's own chain over corners already
// lifted, so a caller reading more than one of a cell's exact quantities lifts
// them once.
func XtwistQuarterUpper(c CellCorners) float64 {
	// T = vLo − vHi − wLo + wHi = (vLo − vHi) − (wLo − wHi).
	t := proofarith.Xsub(proofarith.Xsub(c.VLo, c.VHi), proofarith.Xsub(c.WLo, c.WHi))
	if t.X.Sign() == 0 && t.Y.Sign() == 0 && t.Z.Sign() == 0 {
		return 0
	}
	// |T|²/16 over one shared positive denominator: proofarith.XdotNum's own w·w, times
	// the 16. The whole quotient normalises once, here.
	den := new(big.Int).Mul(new(big.Int).Mul(t.W, t.W), big.NewInt(16))
	return RatSqrtUp(new(big.Rat).SetFrac(proofarith.XdotNum(t, t), den))
}

// CellSpanUpper is the CERTIFIED upper endpoint of |a − b| for two of a wall
// cell's own corners — the edge lengths CellTwistAreaLinearFromSpans reads,
// and the same endpoint CellChordCurveAreaUpper reads for its own eBBase and
// for the chord its arc-length premise is falsified against. It exists for
// the same reason
// CellTwistQuarterUpper does: r3.Vec.Len is nested math.Hypot, which is not
// correctly rounded and can sit several ulp below the exact norm, and a
// trailing one-ulp UpRound cannot recover a multi-ulp shortfall. The
// corners are float64 and hence exact rationals, so the difference and its
// squared norm are exact and RatSqrtUp decides the root by exact comparison —
// carried, like the twist chain, through the homogeneous integer kernel and
// materialised as a big.Rat only for RatSqrtUp itself.
//
// It reads its corners as exact rationals, so a caller must have already
// refused a non-finite one before calling: every entry point that reaches it
// goes through CellCornersOf, and each of those runs FiniteVec first.
func CellSpanUpper(a, b r3.Vec) float64 {
	return XspanUpper(proofarith.XptOf(a), proofarith.XptOf(b))
}

// XspanUpper is CellSpanUpper's own reading over corners already lifted.
func XspanUpper(a, b proofarith.Xpt) float64 {
	d := proofarith.Xsub(a, b)
	return RatSqrtUp(proofarith.XdotRat(d, d))
}

// CellCorners is ONE wall cell's four corners lifted to exact homogeneous
// integer coordinates (proofarith.XptOf, internal/proof/exact_point.go). Every exact quantity a cell
// publishes — its four certified spans and its certified |T|/4 endpoint — is a
// function of these four points and nothing else, and lifting a corner is the
// single most expensive step in each of them, so a caller reading more than one
// of those quantities lifts the cell's corners once and reads them all from the
// same four points.
//
// The lift rounds nothing: a float64 is an exact dyadic rational (proofarith.XptOf's own
// doc comment), so these four points denote the cell's own corners exactly.
type CellCorners struct{ VLo, VHi, WLo, WHi proofarith.Xpt }

// CellCornersOf lifts a cell whose four corners the caller has already proved
// finite, which is the exact lift's own precondition.
func CellCornersOf(vLo, vHi, wLo, wHi r3.Vec) CellCorners {
	return CellCorners{VLo: proofarith.XptOf(vLo), VHi: proofarith.XptOf(vHi), WLo: proofarith.XptOf(wLo), WHi: proofarith.XptOf(wHi)}
}

// CellSpans is ONE wall cell's four certified corner spans — every span any
// bound over that cell reads, and nothing else. CellChordCurveAreaUpper's own
// arc-length premise gate and eBBase, and CellTwistAreaLinearFromSpans' own eA
// and eB, are each one of these four and no other quantity.
//
// It exists so a caller that reads more than one of a cell's bounds certifies
// each span ONCE (CellAllowsOf) instead of once per bound. Each span is an
// exact-arithmetic reading ending in a RatSqrtUp — the price of the certified
// endpoint CellSpanUpper's own doc comment explains — so recertifying the same
// four corners per bound is the dominant cost of a chorded wall, and paying it
// once changes only which computations happen, never what any of them returns.
type CellSpans struct {
	SideA  float64 // |vHi − vLo|, side A's own chord
	SideB  float64 // |wHi − wLo|, side B's own chord
	RungLo float64 // |wLo − vLo|, the cell's own rung at s=0
	RungHi float64 // |wHi − vHi|, the cell's own rung at s=1
}

// CellSpansOf certifies all four spans of a cell whose four corners the caller
// has already proved finite, which is CellCornersOf's own precondition.
func CellSpansOf(vLo, vHi, wLo, wHi r3.Vec) CellSpans {
	return CellCornersOf(vLo, vHi, wLo, wHi).Spans()
}

// spans certifies all four of the cell's spans from its already-lifted corners.
func (c CellCorners) Spans() CellSpans {
	return CellSpans{
		SideA:  XspanUpper(c.VHi, c.VLo),
		SideB:  XspanUpper(c.WHi, c.WLo),
		RungLo: XspanUpper(c.WLo, c.VLo),
		RungHi: XspanUpper(c.WHi, c.VHi),
	}
}

// CellTwistOffsetUpper is the pointwise deviation bound |T|/4 used by the
// facet-departure proof. The symmetric difference between the held triangle
// pair and the ruled patch extends outside every held facet by at most this
// much, for the same T = vLo−vHi−wLo+wHi CellTwistVolumeAllow reads. A caller
// that needs one bound for a WHOLE boundary takes the MAXIMUM of this over
// every wall cell, never a sum — it bounds how far any SINGLE point can sit
// from the held facet at the matching parameter, not an accumulation over
// cells.
//
// Non-finite corners answer +Inf rather than a silently-computed NaN, the
// same guard CellTwistVolumeAllow's own doc comment states for its sibling.
//
// It reads the certified |T|/4 endpoint through CellTwistQuarterUpper, so neither the cancelling T chain nor
// r3.Vec.Len's own missing accuracy contract can put this reading below the
// deviation it claims to dominate.
func CellTwistOffsetUpper(vLo, vHi, wLo, wHi r3.Vec) float64 {
	if !FiniteVec(vLo) || !FiniteVec(vHi) || !FiniteVec(wLo) || !FiniteVec(wHi) {
		return math.Inf(1)
	}
	return CellTwistQuarterUpper(vLo, vHi, wLo, wHi)
}

// CellAllows is every bound ONE wall cell publishes, each field carrying
// exactly what the like-named helper publishes for that same cell.
type CellAllows struct {
	ChordCurveAreaUpper float64 // CellChordCurveAreaUpper
	TwistVolumeAllow    float64 // CellTwistVolumeAllow
	TwistOffsetUpper    float64 // CellTwistOffsetUpper
}

// CellAllowsOf reads all three bounds one wall cell publishes. It shares the
// certified spans and |T|/4 endpoint used by the area and offset readings;
// the exact determinant volume reading has its own rational reduction.
//
// It is a sharing of computation, never a bound of its own: each field is
// produced by the same CellChordCurveAreaFromSpans / CellTwistVolumeAllow /
// XtwistQuarterUpper helpers the individual entry points call, under the same gates in
// the same order, so the three numbers are identical to the three helpers' own
// by construction (TestCellAllowsOfMatchesThePerBoundHelpers pins it over a
// randomized sweep). A broken SCALAR claim refuses the area reading alone: the
// twist readings do not take those operands and are not spoken for by them,
// while a non-finite CORNER is unstateable geometry and refuses all three.
func CellAllowsOf(vLo, vHi, wLo, wHi r3.Vec, arcLenUpperA, arcLenUpperB, matchedDeltaUpper float64) CellAllows {
	if !FiniteVec(vLo) || !FiniteVec(vHi) || !FiniteVec(wLo) || !FiniteVec(wHi) {
		inf := math.Inf(1)
		return CellAllows{ChordCurveAreaUpper: inf, TwistVolumeAllow: inf, TwistOffsetUpper: inf}
	}
	corners := CellCornersOf(vLo, vHi, wLo, wHi)
	spans := corners.Spans()
	twistUpper := XtwistQuarterUpper(corners)

	area := math.Inf(1)
	if CellChordClaimsStated(arcLenUpperA, arcLenUpperB, matchedDeltaUpper) {
		area = CellChordCurveAreaFromSpans(spans, arcLenUpperA, arcLenUpperB, matchedDeltaUpper)
	}
	return CellAllows{
		ChordCurveAreaUpper: area,
		TwistVolumeAllow:    CellTwistVolumeAllow(vLo, vHi, wLo, wHi),
		TwistOffsetUpper:    twistUpper,
	}
}

// CellTwistAreaAllow bounds the area gap between one loft wall cell's held
// triangle pair and its bilinear ruled patch. It publishes the smaller of two
// independently proven bounds: CellTwistAreaLinearFromSpans is the existing
// homotopy bound, and CellTwistAreaQuadraticAllow keeps the first-order
// cancellation shared by the two triangles. The linear arm remains the
// fallback when the quadratic arm cannot state a positive denominator.
//
// Every corner operation in both arms is exact rational arithmetic followed
// by outward rounding. In particular T = vLo-vHi-wLo+wHi is a cancelling
// chain and may not be formed with r3.Vec subtraction. Non-finite corners
// answer +Inf rather than entering the exact lift.
func CellTwistAreaAllow(vLo, vHi, wLo, wHi r3.Vec) float64 {
	if !FiniteVec(vLo) || !FiniteVec(vHi) || !FiniteVec(wLo) || !FiniteVec(wHi) {
		return math.Inf(1)
	}
	corners := CellCornersOf(vLo, vHi, wLo, wHi)
	linear := CellTwistAreaLinearFromSpans(corners.Spans(), XtwistQuarterUpper(corners))
	quadratic := CellTwistAreaQuadraticAllow(vLo, vHi, wLo, wHi)
	return math.Min(linear, quadratic)
}

// CellBilinearArea publishes a value and bound for a wall cell's bilinear
// patch area. Its area-element vector is the affine form
//
//	N(s,r) = N0 + s*A + r*B.
//
// On each square of a fixed dyadic partition, Jensen's inequality makes the
// norm at the square centre a lower bound on the square's average |N|.
// Convexity gives the other direction: N is the bilinear interpolation of its
// four corner values, so the triangle inequality bounds |N| by the same
// interpolation of their norms, whose average is the four corner norms'
// average. Summing those local brackets encloses the bilinear patch's area.
//
// The midpoint of the final rational interval is rounded once as the value;
// the bound is the farther exact endpoint distance from that float. All vector
// arithmetic and summation are exact over big.Rat. RatSqrtDown/RatSqrtUp
// bracket only the irrational norms.
func CellBilinearArea(vLo, vHi, wLo, wHi r3.Vec) (float64, float64) {
	const divisions = 4
	if !FiniteVec(vLo) || !FiniteVec(vHi) || !FiniteVec(wLo) || !FiniteVec(wHi) {
		return 0, math.Inf(1)
	}
	da := HeldDelta(vHi, vLo)
	g := HeldDelta(wLo, vLo)
	twist := proofarith.DvSub(HeldDelta(vLo, vHi), HeldDelta(wLo, wHi))
	n0 := proofarith.DvCross(da, g)
	a := proofarith.DvCross(da, twist)
	b := proofarith.DvCross(twist, g)
	aZero, bZero := proofarith.DvIsZero(a), proofarith.DvIsZero(b)

	at := func(s, r proofarith.Dyadic) proofarith.DyV3 {
		var out proofarith.DyV3
		for k := range out {
			out[k] = proofarith.DyAdd(proofarith.DyAdd(n0[k], proofarith.DyMul(s, a[k])), proofarith.DyMul(r, b[k]))
		}
		return out
	}
	normEnd := func(v proofarith.DyV3, up bool) (proofarith.Dyadic, bool) {
		dot := proofarith.DvDot(v, v)
		if up {
			return proofarith.DyOf(proofarith.DySqrtUp(dot))
		}
		return proofarith.DyOf(proofarith.DySqrtDown(dot))
	}

	// divisions is a power of two, so every quadrature node below — i/4,
	// (2i+1)/8 — and every weight the sums are divided by are binary
	// fractions, and the whole quadrature stays inside the dyadic set
	// (dyadic.go's own doc comment). divShift is that power.
	const divShift = 2 // divisions == 1 << divShift
	integralLo := proofarith.DyZero()
	integralHi := proofarith.DyZero()
	// A zero coefficient makes the exact norm constant along that parameter.
	// Reuse its reading, but add it at every original quadrature position.
	var midNorms [divisions][divisions]struct {
		value proofarith.Dyadic
		ready bool
	}
	var cornerNorms [divisions + 1][divisions + 1]struct {
		value proofarith.Dyadic
		ready bool
	}
	for i := range divisions {
		for j := range divisions {
			midI, midJ := i, j
			if aZero {
				midI = 0
			}
			if bZero {
				midJ = 0
			}
			mid := &midNorms[midI][midJ]
			if !mid.ready {
				sMid := proofarith.DyShift(proofarith.DyInt(int64(2*i+1)), -(divShift + 1))
				rMid := proofarith.DyShift(proofarith.DyInt(int64(2*j+1)), -(divShift + 1))
				lo, ok := normEnd(at(sMid, rMid), false)
				if !ok {
					return 0, math.Inf(1)
				}
				mid.value = lo
				mid.ready = true
			}
			integralLo = proofarith.DyAdd(integralLo, mid.value)

			for _, p := range [][2]int{{i, j}, {i + 1, j}, {i, j + 1}, {i + 1, j + 1}} {
				key := p
				if aZero {
					key[0] = 0
				}
				if bZero {
					key[1] = 0
				}
				corner := &cornerNorms[key[0]][key[1]]
				if !corner.ready {
					node := func(v int) proofarith.Dyadic { return proofarith.DyShift(proofarith.DyInt(int64(v)), -divShift) }
					hi, ok := normEnd(at(node(p[0]), node(p[1])), true)
					if !ok {
						return 0, math.Inf(1)
					}
					corner.value = hi
					corner.ready = true
				}
				integralHi = proofarith.DyAdd(integralHi, corner.value)
			}
		}
	}
	integralLo = proofarith.DyShift(integralLo, -2*divShift)
	integralHi = proofarith.DyShift(integralHi, -(2*divShift + 2))

	mid := proofarith.DyShift(proofarith.DyAdd(integralLo, integralHi), -1)
	value, _ := mid.Float64()
	valueDy, ok := proofarith.DyOf(value)
	if !ok {
		return 0, math.Inf(1)
	}
	dLo := proofarith.DyAbs(proofarith.DySubScalar(valueDy, integralLo))
	dHi := proofarith.DyAbs(proofarith.DySubScalar(integralHi, valueDy))
	if proofarith.DyCmp(dHi, dLo) > 0 {
		dLo = dHi
	}
	return value, proofarith.DyFloatUp(dLo)
}

// CellTwistAreaLinearFromSpans is the premise-free arm. On the two parameter
// triangles, the bilinear-to-flat displacement is a scalar multiple of T, so
// each of its two partial derivatives has norm at most |T|. The product rule
// bounds the area functional's rate by |T|*(eA+eB), where eA bounds the two
// section edges and eB the two rungs. Integrating the homotopy parameter gives
// the same expression for the complete area gap.
//
// twistQuarterUpper is the certified |T|/4. Multiplication by four is exact
// unless it overflows, when ProductUpper returns +Inf.
func CellTwistAreaLinearFromSpans(spans CellSpans, twistQuarterUpper float64) float64 {
	if twistQuarterUpper <= 0 {
		return 0
	}
	eA := math.Max(spans.SideA, spans.SideB)
	eB := math.Max(spans.RungLo, spans.RungHi)
	return ProductUpper(ProductUpper(4, twistQuarterUpper), AbsSumUpper(eA, eB))
}

// CellTwistAreaProjectedAllow is CellTwistAreaQuadraticAllow's arm with the
// remainder read off the part of V normal to U alone. Writing p = Û·V and
// V⊥ = V − p·Û,
//
//	|U+V| = sqrt((|U| + p)² + |V⊥|²),  0 ≤ |U+V| − |U| − p ≤ |V⊥|²/(2(|U| + p)),
//
// the second inequality holding wherever |U| + p > 0. Both averages share
// the linear term, so |Area_ruled − Q| is at most the largest remainder over
// the unit square, which holds both rules' nodes. With W = 2U, over that
// square |p| ≤ (|W·A| + |W·B|)/(2|W|) and |V⊥|² ≤ (|A⊥|² + |B⊥|²)/2, where
// |A⊥|² = |A|² − (W·A)²/|W|². So, with c = |W|² − |W·A| − |W·B| > 0,
//
//	|Area_ruled − Q| ≤ (|A⊥|² + |B⊥|²)·|W| / (2c).
//
// Where A and B lie close to U, as they do on a cell whose twist runs along
// its own surface, this is second order in the twist where the quadratic
// arm's |A|² + |B|² is first. A c at or below zero withdraws the arm with
// +Inf. Every vector and product is exact; |W| is rounded up and the
// quotient rounded up.
func CellTwistAreaProjectedAllow(vLo, vHi, wLo, wHi r3.Vec) float64 {
	if !FiniteVec(vLo) || !FiniteVec(vHi) || !FiniteVec(wLo) || !FiniteVec(wHi) {
		return math.Inf(1)
	}
	da := HeldDelta(vHi, vLo)
	g := HeldDelta(wLo, vLo)
	twist := proofarith.DvSub(HeldDelta(vLo, vHi), HeldDelta(wLo, wHi))
	a := proofarith.DvCross(da, twist)
	b := proofarith.DvCross(twist, g)
	n0 := proofarith.DvCross(da, g)
	var w proofarith.DyV3
	for i := range w {
		w[i] = proofarith.DyAdd(proofarith.DyAdd(proofarith.DyShift(n0[i], 1), a[i]), b[i])
	}
	ww := proofarith.DvDot(w, w).Rat()
	if ww.Sign() <= 0 {
		return math.Inf(1)
	}
	wa := proofarith.DvDot(w, a).Rat()
	wb := proofarith.DvDot(w, b).Rat()
	perp := func(v proofarith.DyV3, wv *big.Rat) *big.Rat {
		out := proofarith.DvDot(v, v).Rat()
		return out.Sub(out, new(big.Rat).Quo(new(big.Rat).Mul(wv, wv), ww))
	}
	num := new(big.Rat).Add(perp(a, wa), perp(b, wb))
	if num.Sign() <= 0 {
		return 0
	}
	c := new(big.Rat).Sub(ww, new(big.Rat).Abs(wa))
	c.Sub(c, new(big.Rat).Abs(wb))
	if c.Sign() <= 0 {
		return math.Inf(1)
	}
	wLen, ok := RatOf(RatSqrtUp(ww))
	if !ok {
		return math.Inf(1)
	}
	num.Mul(num, wLen)
	return RatFloatUp(num.Quo(num, c.Mul(c, big.NewRat(2, 1))))
}

// CellTwistAreaQuadraticAllow is the cancellation-preserving arm. Write the
// bilinear patch's area-element vector as
//
//	N(s,r) = N0 + s*A + r*B,
//	N0 = da cross g, A = da cross T, B = T cross g,
//
// where da=vHi-vLo, g=wLo-vLo and T=vLo-vHi-wLo+wHi. The ruled area is the
// unit-square average of |N|. The held triangle pair is the two-point rule
//
//	Q = (|N(1,0)| + |N(0,1)|) / 2.
//
// The square average and Q have the same mean parameter (1/2,1/2), so the
// constant and first-order terms cancel. Put U=N(1/2,1/2) and
// V=(s-1/2)A+(r-1/2)B. The Euclidean norm obeys
//
//	0 <= |U+V|-|U|-Uhat dot V <= |V|^2/(2|U|).
//
// Applying this remainder bound to both averages and using the triangle
// inequality gives
//
//	|Area_ruled-Q| <= (|A|^2+|B|^2+3|A-B|^2)/(12*|2U|).
//
// Every vector and squared norm is exact over big.Rat. The denominator is
// rounded down before division and the quotient is rounded up. A zero or
// unrepresentable denominator withdraws this arm with +Inf; the linear arm
// then remains available.
func CellTwistAreaQuadraticAllow(vLo, vHi, wLo, wHi r3.Vec) float64 {
	da := HeldDelta(vHi, vLo)
	g := HeldDelta(wLo, vLo)
	twist := proofarith.DvSub(HeldDelta(vLo, vHi), HeldDelta(wLo, wHi))
	a := proofarith.DvCross(da, twist)
	b := proofarith.DvCross(twist, g)
	diff := proofarith.DvSub(a, b)

	numerator := proofarith.DyAdd(proofarith.DvDot(a, a), proofarith.DvDot(b, b))
	numerator = proofarith.DyAdd(numerator, proofarith.DyMul(proofarith.DyInt(3), proofarith.DvDot(diff, diff)))
	if numerator.IsZero() {
		return 0
	}

	n0 := proofarith.DvCross(da, g)
	var centerTwice proofarith.DyV3
	for i := range centerTwice {
		centerTwice[i] = proofarith.DyAdd(proofarith.DyAdd(proofarith.DyShift(n0[i], 1), a[i]), b[i])
	}
	centerLenLower := proofarith.DySqrtDown(proofarith.DvDot(centerTwice, centerTwice))
	centerLenRat, ok := RatOf(centerLenLower)
	if !ok || centerLenLower <= 0 {
		return math.Inf(1)
	}
	// The quotient genuinely leaves the dyadic set — a division by twelve times
	// a bracketed norm is no power of two — so this is where the exact vector
	// arithmetic hands over to a general fraction (dyadic.go's own boundary).
	denominator := new(big.Rat).Mul(big.NewRat(12, 1), centerLenRat)
	return RatFloatUp(new(big.Rat).Quo(numerator.Rat(), denominator))
}

// UniformSpeedTangentEnergyUpper is the per-side TANGENT-DEVIATION ENERGY
// CellChordCurveAreaAllow's own tangentEnergyUpper obligation names: a PROVEN
// upper bound on
//
//	J := integral over s in [0,1] of |curve'(s) - chord|^2 ds
//
// where chord = curve(1) - curve(0) is the cell's own chord VECTOR and the
// derivative is taken under the SHARED parametrization CellChordCurveAreaUpper
// fixes. It is the one quantity that makes an area difference second order
// rather than first: the deviation curve'(s) - chord has MEAN ZERO in s (its
// integral is curve(1) - curve(0) - chord = 0 exactly, which is what makes the
// chord the curve's OWN endpoint chord rather than any nearby segment), so a
// consumer that can bound its size in the mean, and not merely pointwise, gets
// the cancellation the pointwise bound |curve'| + |chord| throws away.
//
// The derivation needs ONE premise beyond that: the shared parametrization has
// CONSTANT SPEED over the cell, |curve'(s)| = L for every s, L the cell's own
// true arc length. Then, writing C for the chord vector and c = |C|,
//
//	integral |curve' - C|^2 ds = integral (|curve'|^2 - 2 curve'.C + |C|^2) ds
//	                           = L^2 - 2 C.C + c^2 = L^2 - c^2,
//
// EXACTLY, using integral curve' ds = C once. L^2 - c^2 increases in L for
// L >= c, so any proven arc-length upper bound arcLenUpper >= L gives
// J <= arcLenUpper^2 - c^2 = (arcLenUpper - c)(arcLenUpper + c), the published
// form — the factored one, never the difference of two squares, so a long cell
// whose two squares cancel keeps its own scale.
//
// chordLower must be a PROVEN LOWER bound on c (the published value decreases
// in c, so an overstated chord would understate the energy), and arcLenUpper a
// PROVEN upper bound on that side's own arc length over the cell. The
// CONSTANT-SPEED premise is the caller's to discharge and this helper cannot
// check it: a parametrization that is not constant speed can pack the same arc
// length into a short span and carry an arbitrarily larger energy, so a caller
// that cannot prove constant speed must pass +Inf and let
// CellChordCurveAreaAllow fall back to its own premise-free arm. The circular
// arm's uniform-ANGLE stations (loftCircularCellStations) are constant speed on
// a circle, which is what discharges it today. A free-form cell does not call
// this helper: its deviation is a polynomial, and
// internal/freeform's SpanTangentEnergyUpper integrates it exactly.
//
// A non-finite or negative operand, or an arcLenUpper below the chord it is
// supposed to subtend, is a BROKEN caller claim and answers +Inf, never 0
// (CutDisplacementAllow's own rule).
func UniformSpeedTangentEnergyUpper(arcLenUpper, chordLower float64) float64 {
	if IsNonFinite(arcLenUpper) || IsNonFinite(chordLower) {
		return math.Inf(1)
	}
	if arcLenUpper < 0 || chordLower < 0 || arcLenUpper < chordLower {
		return math.Inf(1)
	}
	return ProductUpper(UpRound(arcLenUpper-chordLower), UpRound(arcLenUpper+chordLower))
}

// CellChordPatchNormalLower is a PROVEN LOWER bound on |N(s,r)|, the AREA
// ELEMENT of ONE loft wall cell's own BILINEAR chord patch
// X(s,r) = (1-r)*(vLo + s*(vHi-vLo)) + r*(wLo + s*(wHi-wLo)), over the whole
// unit square — or 0 where this reduction proves nothing, which is a REFUSAL
// and never a bound (its one consumer, CellChordCurveAreaAllow, drops its
// sharper arm entirely on a 0 rather than dividing by it).
//
// N = X_s cross X_r = P(r) cross g(s), with P(r) = (1-r)*(vHi-vLo) + r*(wHi-wLo)
// the section chord and g(s) = (1-s)*(wLo-vLo) + s*(wHi-vHi) the rung. Both are
// convex combinations, so N is the BILINEAR convex combination of the cell's
// own FOUR CORNER normals,
//
//	N(s,r) = (1-r)(1-s) N00 + (1-r)s N01 + r(1-s) N10 + rs N11,
//
// N00 = (vHi-vLo) cross (wLo-vLo), N01 = (vHi-vLo) cross (wHi-vHi),
// N10 = (wHi-wLo) cross (wLo-vLo), N11 = (wHi-wLo) cross (wHi-vHi), whose four
// weights are non-negative and sum to 1 at every (s, r).
//
// Fix ANY unit vector m. A convex combination's own component along m is at or
// above the smallest component among the combined vectors, so
// |N(s,r)| >= N(s,r).m >= min over the four corners of Ni.m. When that minimum
// is POSITIVE it is a lower bound on the area element everywhere on the cell,
// and when it is not, the four corner normals do not fit in one open half space
// and this reduction has nothing to say — the cell's own patch may genuinely
// degenerate somewhere inside it, and no positive bound exists to publish.
//
// m is taken as the direction of N00+N01+N10+N11, the choice that maximises the
// smallest component for a tight cluster of corner normals and so refuses least
// often. Every corner normal, their sum, and each of the four dot products is
// formed EXACTLY over clearance_degen.go's own dyV3 kernel (a float64
// coordinate is an exact rational, and
// the differences and cross products above are exact rational arithmetic), so
// the only rounding is the final division: the sum's own length is taken UP
// (RatSqrtUp) and the quotient DOWN (RatFloatDown), which can only shrink the
// published lower bound, never inflate it.
//
// A non-finite corner is a BROKEN caller claim: it answers 0, this helper's own
// refusal, rather than a NaN a later comparison would silently drop — the
// consumer turns that refusal into its own premise-free arm, which is where the
// +Inf discipline is spent.
func CellChordPatchNormalLower(vLo, vHi, wLo, wHi r3.Vec) float64 {
	if !FiniteVec(vLo) || !FiniteVec(vHi) || !FiniteVec(wLo) || !FiniteVec(wHi) {
		return 0
	}
	da := proofarith.DvSub(proofarith.DyVec(vHi), proofarith.DyVec(vLo))
	db := proofarith.DvSub(proofarith.DyVec(wHi), proofarith.DyVec(wLo))
	g := proofarith.DvSub(proofarith.DyVec(wLo), proofarith.DyVec(vLo))
	gp := proofarith.DvSub(proofarith.DyVec(wHi), proofarith.DyVec(vHi))
	corners := [4]proofarith.DyV3{
		proofarith.DvCross(da, g), proofarith.DvCross(da, gp),
		proofarith.DvCross(db, g), proofarith.DvCross(db, gp),
	}
	var sum proofarith.DyV3
	for _, c := range corners {
		sum = proofarith.DvAdd(sum, c)
	}
	sumLen2 := proofarith.DvDot(sum, sum)
	if sumLen2.Sign() <= 0 {
		return 0
	}
	minDot := proofarith.DvDot(corners[0], sum)
	for _, c := range corners[1:] {
		if d := proofarith.DvDot(c, sum); proofarith.DyCmp(d, minDot) < 0 {
			minDot = d
		}
	}
	if minDot.Sign() <= 0 {
		return 0
	}
	lenUp := proofarith.DySqrtUp(sumLen2)
	if IsNonFinite(lenUp) || lenUp <= 0 {
		return 0
	}
	lenRat, ok := RatOf(lenUp)
	if !ok {
		return 0
	}
	// Dividing by a bracketed norm leaves the dyadic set, so the quotient is
	// taken as a general fraction (dyadic.go's own boundary).
	lower := RatFloatDown(new(big.Rat).Quo(minDot.Rat(), lenRat))
	if IsNonFinite(lower) || lower <= 0 {
		return 0
	}
	return lower
}

// CellChordCurveAreaAllow bounds the AREA GAP between ONE loft wall cell's own
// BILINEAR CHORD PATCH — CellTwistVolumeAllow's own X(s,r), the t=0 endpoint of
// CellChordCurveAreaUpper's chord-to-curve homotopy — and the RULED PATCH
// THROUGH THE FOUR STATIONS THIS CALL IS HANDED, the patch ruled between the
// two curves that pass through vLo/vHi and wLo/wHi. It is NOT a gap to the
// patch between the two recorded curves as the record places them: every
// reading below — the two chords, eB, cMax, the twist vector and Nmin — comes
// off those four corner arguments, so the patch they pin is the only one the
// arithmetic supports. A caller that hands HELD corners (every caller today)
// therefore gets the gap at the held stations, and the step from those to the
// stations they DENOTE is a leg of its own, named in the composition below.
//
// It replaces an arc-minus-chord LENGTH excess times a rung length. That shape
// is third order in the cell's own sweep, while the gap it stands for is SECOND
// order wherever the ruling runs anything but square across the section's own
// tangent, so it understates a twisted pairing without bound.
//
// # The three-leg composition at the call site
//
// area() (loft_moments.go) charges one wall cell's whole gap as
//
//	|A_true - T_held| <= ruled leg + twist leg + station-shift leg,
//
// A_true the area of the ruled patch between the two curves the cell's DENOTED
// stations carry and T_held the two flat triangles assembleLoft actually holds:
//
//   - the RULED leg is this helper, |Area(bilinear chord patch) - Area(ruled
//     patch)|, both patches pinned at the corners it is handed;
//   - the TWIST leg is CellTwistAreaAllow's, unchanged, |Area(held triangle
//     pair) - Area(bilinear chord patch)|, pinned at those same corners;
//   - the HELD-TO-DENOTED leg is CellStationShiftAreaAllow's, |Area(ruled patch
//     through the held corners) - Area(ruled patch through the stations they
//     denote)|, at the payload's own delta. It carries the step the two legs
//     above stop short of, from its own inequality rather than by analogy with
//     a triangle's, and TestDisplacedStationCellNeedsTheStationShiftLeg pins
//     both directions: the first two legs alone do NOT cover a displaced-station
//     circular cell, and this leg covers the surface step it is charged for.
//
// # Why no corner reading is widened by delta
//
// The two legs are evaluated at held corners, and no eB, cMax or Nmin here is
// widened by delta before use, because there is no held-for-denoted
// substitution inside this helper to charge. Write abar for the recorded curve
// of side a and p, q for its two corner displacements, |p|,|q| <= delta. The
// curve a(s) = abar(s) + (1-s)p + s q passes through vLo and vHi, and the
// affine correction cancels out of BOTH obligations this helper spends, since
// the chord a0 through the held corners is abar's own chord plus that SAME
// affine term:
//
//	a(s) - a0(s) = abar(s) - abar0(s)      (the chord-relative deviation)
//	a'(s) - da   = abar'(s) - dabar        (the tangent deviation)
//
// So matchedDeltaUpper, tangentEnergyUpper and the mean-zero step they feed all
// hold verbatim for a(s), stated against the chord this helper actually reads,
// with no widening at all; eB, cMax, T and Nmin are read at exactly the corners
// of the two patches they speak for. What the affine correction does move is
// the SURFACE, and |Area(ruled(abar,bbar)) - Area(ruled(a,b))| is the one step
// that crosses from held to denoted. It is the third leg's, and
// CellStationShiftAreaAllow charges exactly that step from its own inequality:
// expanding the same |e x v| + |u x f| + |e x f| product one dimension up over
// the two ruled patches sizes it at 2*delta*(int|X_r| + int|X_s|) + 4*delta^2,
// with both integrals bounded from readings this call site already holds. The
// cell's two triangles charge delta*(|vHi-vLo| + 2*|wHi-vLo| + |wLo-vLo|) +
// 4*delta^2 under the SAME expansion at the cell's own four corners, its
// diagonal counted twice — the same shape at the same order over a DIFFERENT
// quantity, and which one is larger is a fact about the cell rather than an
// identity, so a triangle's allowance is never spent for this surface step.
//
// # Setup
//
// Write a(s) and b(s), s in [0,1], for the two curves through the four corners
// this call is handed, under the SHARED parametrization
// CellChordCurveAreaUpper fixes, a(0)=vLo, a(1)=vHi,
// b(0)=wLo, b(1)=wHi; da = vHi-vLo, db = wHi-wLo for the two chords, ca=|da|,
// cb=|db|; a0(s) = vLo + s*da and b0(s) = wLo + s*db for the two chord
// segments. The two patches are X1(s,r) = (1-r)a(s) + r b(s) and
// X0(s,r) = (1-r)a0(s) + r b0(s), each over the unit square, so
// Area(Xk) = double integral of |Nk|, Nk = dXk/ds cross dXk/dr. Write
//
//	ea(s) = a'(s) - da, eb(s) = b'(s) - db      (MEAN ZERO in s)
//	eps(s,r) = (1-r) ea(s) + r eb(s)
//	g(s) = b0(s) - a0(s) = (1-s) G + s G',  G = wLo-vLo, G' = wHi-vHi
//	T = G' - G                                  (the SAME twist vector
//	                                             CellTwistVolumeAllow reads)
//	P(r) = (1-r) da + r db
//	f(s) = (b(s)-b0(s)) - (a(s)-a0(s)),  |f| <= 2*matchedDeltaUpper
//
// Then N0 = P cross g exactly, N1 = (P + eps) cross (g + f), and
//
//	D := N1 - N0 = eps cross g + P cross f + eps cross f.
//
// eB := max(|G|,|G'|) bounds |g| by convexity, cMax := max(ca,cb) bounds |P|.
//
// # The premise-free arm
//
// |Area(X1) - Area(X0)| <= double integral of ||N1| - |N0|| <= double integral
// of |D| <= (eB + 2*md)*(Ia + Ib)/2 + 2*md*cMax, md the matched delta and
// Ia = integral |ea| ds, Ib likewise. This arm needs nothing but the tangent
// bound |a'| <= arcLenUpperA CellChordCurveAreaUpper already requires, since
// Ia <= arcLenUpperA + ca follows from it directly. It is TIGHT where the cell
// twists hard and loose by a factor of order 1/sweep where it does not, so it
// ships as a ceiling the sharper arm below is taken against, never alone.
//
// # The sharp arm
//
// The premise-free arm throws away the whole point: |N1| - |N0| is a difference
// of LENGTHS, and D is mostly a ROTATION of N0, which changes no length at all.
// Convexity of the Euclidean norm recovers that. For any u != 0 and any v,
//
//	u.v/|u| <= |u+v| - |u| <= u.v/|u| + |v|^2/(2|u|),
//
// the left from |u+v| >= (u+v).u/|u| and the right from |u+v| =
// |u|*sqrt(1+x) <= |u|*(1+x/2) at x = (2u.v + |v|^2)/|u|^2 >= -1. Applied
// pointwise at u = N0, v = D and integrated,
//
//	|Area(X1) - Area(X0)| <= |double integral of N0hat.D| + double integral of
//	                         |D|^2/(2|N0|),
//
// which needs a POSITIVE lower bound Nmin on |N0| over the whole cell —
// CellChordPatchNormalLower's four-corner convex-combination reduction. Where
// that refuses, this arm is dropped and the premise-free arm stands alone.
//
// LINEAR TERM. Split D. The eps cross g part is
// N0hat.(eps cross g) = eps.(g cross N0hat) = eps.W, W(s,r) := g(s) cross
// N0hat(s,r), and for each fixed r the integral of eps over s is ZERO, so
// integral eps.W ds = integral eps.(W(s,r) - W(s0,r)) ds for any s0 — the whole
// cancellation, bought by nothing but the mean-zero property. Hence that part
// is at most oscW * max(Ia,Ib), oscW the s-oscillation of W, and
//
//	dW/ds = T cross N0hat + g cross dN0hat/ds,  dN0/ds = P cross T,
//	|dN0hat/ds| <= |P cross T|/|N0| <= max(|da cross T|,|db cross T|)/Nmin,
//	oscW <= |T| + eB * max(|da cross T|,|db cross T|)/Nmin,
//
// using |P(r) cross T| <= max(|da cross T|,|db cross T|) by convexity. Every
// term of oscW carries a factor T: a PLANAR cell (T = 0, the untwisted
// offset-section case) contributes nothing here at all, which is what keeps a
// shipped untwisted pairing's own Area bound where it was. The two remaining
// parts of D contribute at most 2*md*cMax and 2*md*max(Ia,Ib), so
//
//	LIN = oscW*max(Ia,Ib) + 2*md*(cMax + max(Ia,Ib)).
//
// QUADRATIC TERM. |D| <= beta*|eps| + gamma with beta = eB + 2*md and
// gamma = 2*md*cMax, so |D|^2 <= 2*beta^2*|eps|^2 + 2*gamma^2, and
// |eps|^2 <= (1-r)|ea|^2 + r|eb|^2 by convexity of the square, whose double
// integral is (Ja+Jb)/2 for Ja = integral |ea|^2 ds. Hence
//
//	QUAD = (beta^2 * (Ja+Jb) + 2*gamma^2) / (2*Nmin).
//
// The published value is min(premise-free, LIN + QUAD): both are proven upper
// bounds on the same quantity, so their minimum is one too.
//
// # Obligations
//
// arcLenUpperA/arcLenUpperB are CellChordCurveAreaUpper's own: a proven bound on
// that side's tangent magnitude under the shared parametrization, never below
// the chord it subtends. matchedDeltaUpper is that helper's own PARAMETER-
// MATCHED obligation (F1's rule), never a set-distance sagitta. tangentEnergyA/
// tangentEnergyB are each a proven bound on that side's own integral
// |curve' - chord|^2 ds — UniformSpeedTangentEnergyUpper's J on a constant-speed
// arm, internal/freeform's SpanTangentEnergyUpper on a polynomial span, whose
// energy is an exact rational rounded outward once; a caller with no such proof
// passes +Inf and both Ia and Ja fall back to what the tangent bound alone
// gives, Ia <= arcLen+chord and Ja <= (arcLen+chord)^2, which costs tightness
// and never soundness. Every geometric quantity above is read from the four
// corners this call is handed, the same convention CellChordCurveAreaUpper,
// CellTwistVolumeAllow and CellTwistAreaAllow already use, and the two
// sections above own what that convention does and does not claim: the patch
// is pinned at those corners, and the step to the stations a HELD corner
// denotes is the third leg's, charged once by CellStationShiftAreaAllow and
// never here.
//
// A non-finite corner, a non-finite or negative scalar, a negative energy, or an
// arc-length claim below its own chord is a BROKEN caller claim and answers
// +Inf, never 0 (CellChordCurveAreaUpper's own F5 rule).
func CellChordCurveAreaAllow(
	vLo, vHi, wLo, wHi r3.Vec,
	arcLenUpperA, arcLenUpperB, matchedDeltaUpper, tangentEnergyA, tangentEnergyB float64,
) float64 {
	if !FiniteVec(vLo) || !FiniteVec(vHi) || !FiniteVec(wLo) || !FiniteVec(wHi) {
		return math.Inf(1)
	}
	if IsNonFinite(arcLenUpperA) || IsNonFinite(arcLenUpperB) || IsNonFinite(matchedDeltaUpper) {
		return math.Inf(1)
	}
	if arcLenUpperA < 0 || arcLenUpperB < 0 || matchedDeltaUpper < 0 {
		return math.Inf(1)
	}
	if math.IsNaN(tangentEnergyA) || math.IsNaN(tangentEnergyB) || tangentEnergyA < 0 || tangentEnergyB < 0 {
		return math.Inf(1)
	}
	// Every corner difference, cross product and norm below is formed EXACTLY
	// and rounded OUTWARD, CellTwistAreaAllow's own rule: a float64 Sub/Len
	// pair rounds to NEAREST and so bounds nothing, and the twist vector in
	// particular cancels. The arc-length-versus-chord gate is decided by exact
	// comparison instead (DvLenAtLeast), so an outward-rounded chord can never
	// refuse a caller whose claim is exactly tight.
	da, db := HeldDelta(vHi, vLo), HeldDelta(wHi, wLo)
	if !DvLenAtLeast(arcLenUpperA, da) || !DvLenAtLeast(arcLenUpperB, db) {
		return math.Inf(1)
	}
	ca, cb := DvLenUpper(da), DvLenUpper(db)
	eB := math.Max(DvLenUpper(HeldDelta(wLo, vLo)), DvLenUpper(HeldDelta(wHi, vHi)))
	cMax := math.Max(ca, cb)
	md := matchedDeltaUpper

	// Ia/Ib bound the MEAN deviation and Ja/Jb its ENERGY, each taken against
	// the premise-free reading the tangent bound alone proves, so a caller with
	// no energy proof degrades rather than saturates.
	ia, ja := TangentDeviationUpper(arcLenUpperA, ca, tangentEnergyA)
	ib, jb := TangentDeviationUpper(arcLenUpperB, cb, tangentEnergyB)
	iMax := math.Max(ia, ib)
	beta := AbsSumUpper(eB, ProductUpper(2, md))
	gamma := ProductUpper(ProductUpper(2, md), cMax)

	free := AbsSumUpper(DivUpper(ProductUpper(beta, AbsSumUpper(ia, ib)), 2), gamma)
	if IsNonFinite(free) {
		return math.Inf(1)
	}

	nMin := CellChordPatchNormalLower(vLo, vHi, wLo, wHi)
	if nMin <= 0 {
		return free
	}
	twist := proofarith.DvSub(HeldDelta(vLo, vHi), HeldDelta(wLo, wHi))
	pCrossT := math.Max(DvLenUpper(proofarith.DvCross(da, twist)), DvLenUpper(proofarith.DvCross(db, twist)))
	oscW := AbsSumUpper(DvLenUpper(twist), DivUpper(ProductUpper(eB, pCrossT), nMin))
	lin := AbsSumUpper(
		ProductUpper(oscW, iMax),
		ProductUpper(ProductUpper(2, md), AbsSumUpper(cMax, iMax)),
	)
	quad := DivUpper(AbsSumUpper(
		ProductUpper(ProductUpper(beta, beta), AbsSumUpper(ja, jb)),
		ProductUpper(2, ProductUpper(gamma, gamma)),
	), 2*nMin)
	sharp := AbsSumUpper(lin, quad)
	if IsNonFinite(sharp) {
		return free
	}
	return math.Min(free, sharp)
}

// TangentDeviationUpper turns ONE side's own arc-length bound, held chord and
// (possibly absent) tangent-deviation energy into the two readings
// CellChordCurveAreaAllow spends: a bound on integral |curve' - chord| ds and
// one on integral |curve' - chord|^2 ds, both over s in [0,1].
//
// The energy arm is UniformSpeedTangentEnergyUpper's J, and Cauchy-Schwarz over
// the unit interval turns it into the first: integral |e| <= sqrt(integral
// |e|^2). The premise-free arm reads only |curve'| <= arcLenUpper, which
// CellChordCurveAreaUpper already requires of arcLenUpper, and gives
// integral |e| <= arcLenUpper + chordLen pointwise-then-integrated, hence also
// integral |e|^2 <= (arcLenUpper + chordLen)^2. Each reading is the SMALLER of
// the two arms, so an absent (+Inf) energy costs tightness and never soundness.
func TangentDeviationUpper(arcLenUpper, chordLen, energyUpper float64) (float64, float64) {
	span := AbsSumUpper(arcLenUpper, chordLen)
	freeI, freeJ := span, ProductUpper(span, span)
	if IsNonFinite(energyUpper) || energyUpper < 0 {
		return freeI, freeJ
	}
	fromEnergy := UpRound(math.Sqrt(energyUpper))
	return math.Min(freeI, fromEnergy), math.Min(freeJ, energyUpper)
}

// CellStationShiftAreaAllow bounds the AREA STEP from the ruled patch through
// ONE loft wall cell's HELD corners to the ruled patch through the STATIONS
// those corners denote — the THIRD leg of the three-leg composition
// CellChordCurveAreaAllow's own doc comment states, and the one leg the other
// two are scoped away from, since both of those pin their patches at the
// corners they are handed.
//
// # Setup
//
// Write abar(s), bbar(s), s in [0,1], for the two DENOTED curves under the
// shared parametrization CellChordCurveAreaUpper fixes, and a(s), b(s) for the
// two curves through the HELD corners vLo/vHi and wLo/wHi this call is handed.
// Each held corner sits within delta of the station it denotes, so, writing p
// and q for one side's two corner displacements (|p|,|q| <= delta),
//
//	a(s) = abar(s) + (1-s)p + s q,
//
// CellChordCurveAreaAllow's own affine correction. The two ruled patches are
// Xbar(s,r) = (1-r)abar(s) + r bbar(s) and X(s,r) = (1-r)a(s) + r b(s) over the
// unit square, and this helper bounds |Area(X) - Area(Xbar)|.
//
// # The inequality
//
// D := X - Xbar is BILINEAR in (s,r), its four corner values the four corner
// displacements, so each partial is a convex combination of a DIFFERENCE of two
// of them and |D_s| <= 2*delta, |D_r| <= 2*delta pointwise. Expanding the cross
// product,
//
//	|(Xbar_s + D_s) x (Xbar_r + D_r)| <= |Xbar_s x Xbar_r|
//	    + |D_s||Xbar_r| + |Xbar_s||D_r| + |D_s||D_r|,
//
// and the same expansion with the two patches exchanged bounds the other
// direction, so integrating over the unit square,
//
//	|Area(X) - Area(Xbar)| <= 2*delta*(int|Xbar_r| + int|Xbar_s|) + 4*delta^2,
//
// the SAME |e x v| + |u x f| + |e x f| expansion PerturbedTriangleAreaAllow
// derives for ONE TRIANGLE, one dimension up. That triangle helper is NOT a
// bound on this step and is not spent for it: its own reading is over a
// triangle's two edge vectors, and which of the two expressions is larger is a
// fact about the cell, so the step is charged HERE, from its own inequality.
//
// # The two integrals, from the readings the call site already holds
//
// int|Xbar_s|: Xbar_s = (1-r)abar'(s) + r bbar'(s), so |Xbar_s| is at most
// (1-r)|abar'| + r|bbar'| by convexity and its double integral is
// (int|abar'| + int|bbar'|)/2 <= (arcLenUpperA + arcLenUpperB)/2, each side's
// own arc-length claim being a bound on that side's speed under the shared
// parametrization — CellChordCurveAreaUpper's own obligation, read here for the
// DENOTED curve it actually speaks for.
//
// int|Xbar_r|: Xbar_r = bbar(s) - abar(s), the DENOTED rung, which joins two
// CURVE points rather than the two chord ends eB is read from. Write abar0 and
// bbar0 for the two denoted chords and mdCurve for the PARAMETER-MATCHED
// chord-to-curve departure, so |abar - abar0| <= mdCurve and likewise for b.
// Then
//
//	|bbar(s) - abar(s)| <= |bbar0(s) - abar0(s)| + 2*mdCurve,
//
// and the denoted chord rung is (1-s)Gbar + s Gbar' with Gbar = denoted
// wLo-vLo and Gbar' = denoted wHi-vHi, hence at most max(|Gbar|,|Gbar'|) by
// convexity — the SAME eB convexity reading CellChordCurveAreaAllow forms, one
// station displacement out: each denoted corner sits within delta of its held
// one, so max(|Gbar|,|Gbar'|) <= eB + 2*delta at the held eB. Therefore
//
//	int|Xbar_r| <= eB + 2*delta + 2*mdCurve <= eB + 2*matchedDeltaUpper,
//
// the last step being the caller's obligation below.
//
// # Obligations
//
// matchedDeltaUpper must be the cell's own COMPOSED matched delta —
// loft_build.go's chordCellDeltaUpper, the certified chord-to-curve sagitta
// PLUS the station displacement delta — never the sagitta alone, because the
// final step above spends exactly mdCurve + delta <= matchedDeltaUpper.
// arcLenUpperA/arcLenUpperB are CellChordCurveAreaUpper's own per-side speed
// bounds, and delta is the payload's own proven station displacement.
//
// delta = 0 is a build that holds the stations it denotes, and the step is
// exactly zero. A non-finite corner, or a non-finite or negative scalar, is a
// BROKEN caller claim and answers +Inf, never 0 (CellChordCurveAreaUpper's own
// F5 rule).
func CellStationShiftAreaAllow(
	vLo, vHi, wLo, wHi r3.Vec,
	arcLenUpperA, arcLenUpperB, matchedDeltaUpper, delta float64,
) float64 {
	if !FiniteVec(vLo) || !FiniteVec(vHi) || !FiniteVec(wLo) || !FiniteVec(wHi) {
		return math.Inf(1)
	}
	if IsNonFinite(arcLenUpperA) || IsNonFinite(arcLenUpperB) || IsNonFinite(matchedDeltaUpper) || IsNonFinite(delta) {
		return math.Inf(1)
	}
	if arcLenUpperA < 0 || arcLenUpperB < 0 || matchedDeltaUpper < 0 || delta < 0 {
		return math.Inf(1)
	}
	if delta == 0 {
		return 0
	}
	// eB is formed EXACTLY and rounded OUTWARD, CellChordCurveAreaAllow's own
	// rule for the identical reading: a float64 Sub/Len pair rounds to NEAREST
	// and so bounds nothing.
	eB := math.Max(DvLenUpper(HeldDelta(wLo, vLo)), DvLenUpper(HeldDelta(wHi, vHi)))
	rung := AbsSumUpper(eB, ProductUpper(2, matchedDeltaUpper))
	span := DivUpper(AbsSumUpper(arcLenUpperA, arcLenUpperB), 2)
	return AbsSumUpper(
		ProductUpper(ProductUpper(2, delta), AbsSumUpper(rung, span)),
		ProductUpper(4, ProductUpper(delta, delta)),
	)
}

// SectionDisplacementArea bounds the AREA a recorded 2D section can differ from
// the section its construction denotes, given that every recorded boundary
// coordinate sits within delta of that denoted boundary
// (docs/prism-boolean-design.md §7).
//
// The two regions' symmetric difference lies inside the delta-neighbourhood of
// the recorded boundary: a point in one region and not the other has the two
// boundaries between it and either interior, so it is within delta of the
// recorded one. That neighbourhood is covered by a rectangle 2·delta wide along
// each of the walks — perimeterUpper must be a PROVEN upper bound on their total
// length — plus a disk of radius delta at each of the walks joints, so
// 2·delta·p + n·π·delta² encloses it. The bound is therefore a bound on the
// whole SET displacement, not on the arithmetic that produced the coordinates:
// it stands even where moving the boundary by delta changes which regions the
// construction merged, which coordinate-rounding terms alone cannot cover.
func SectionDisplacementArea(delta float64, walks int, perimeterUpper float64) float64 {
	if delta <= 0 || walks <= 0 {
		return 0
	}
	tube := ProductUpper(ProductUpper(2, delta), perimeterUpper)
	joints := ProductUpper(
		ProductUpper(float64(walks), math.Nextafter(math.Pi, math.Inf(1))),
		ProductUpper(delta, delta),
	)
	return UpRound(tube + joints)
}

// SectionDisplacementLength bounds how far the total LENGTH of a recorded
// section's boundary — walks lines and circular arcs, the class
// docs/prism-boolean-design.md §3.1's G4 admits — can differ from the length of
// the boundary it denotes, given that every recorded coordinate sits within
// delta of that denoted boundary. It is the perimeter's own reading of the same
// displacement SectionDisplacementArea reads as an area.
//
// The per-walk factor is 12·π, which covers both walk kinds. A straight walk's
// two ends each move by at most delta, so its length moves by at most 2·delta
// (ChainLengthBound's own reasoning). A circular walk moves more, because its
// radius and its swept angle both move: its centre and both endpoints sit within
// delta, so the radius moves by at most 2·delta and — while the radius is at
// least 4·delta — each endpoint's angle by at most π·delta/R, giving
// |R'θ' − Rθ| ≤ 2·delta·2π + R'·2π·delta/R' = 6·π·delta; a radius under 4·delta
// leaves both arcs shorter than 12·π·delta outright, so the same figure stands.
// The held sum's own float slop is NOT included: the perimeter this composes
// into already carries it.
func SectionDisplacementLength(delta float64, walks int) float64 {
	if delta <= 0 || walks <= 0 {
		return 0
	}
	perWalk := ProductUpper(12, math.Nextafter(math.Pi, math.Inf(1)))
	return ProductUpper(ProductUpper(float64(walks), perWalk), delta)
}

// CutParamUlps is the allowance, in ulps of the parameter domain [0, 1], that
// a sketch-certified cut parameter is charged against the true parameter of the
// crossing it names (docs/prism-boolean-design.md §7).
//
// It is the ONE number the analytic path's displacement rests on beyond what
// sketch itself certifies, so it is stated here once rather than derived at a
// call site. TExact's own claim (docs/sketch-seam-design.md §1) is that the
// range reproduces the fragment's endpoints "to machine precision", and the
// range comes from one closed-form crossing solve over the two entities' own
// defining data — a short expression whose accumulated relative rounding is a
// handful of ulps. 8 ulps of the WHOLE domain dominates a handful of ulps of any
// t ≤ 1, and it is not a proof that sketch rounds well: it is the quantitative
// reading decad gives the seam's precision claim, and a cut that misses it by
// more is a sketch bug the seam's own falsifier is there to catch.
const CutParamUlps = 8

// CutDisplacementAllow bounds how far the point a recorded cut parameter names
// sits from the true crossing point it denotes, given the carrier's own speed
// |dP/dt| bounded above by tangentUpper.
//
// A cut fragment records the entity's UNCHANGED defining data and a narrowed
// t range (docs/sketch-seam-design.md §1), so the carrier is exactly the
// carrier the construction denotes and the whole displacement sits in the
// parameter: the endpoint slides ALONG that carrier by at most |t − t*| ·
// sup|dP/dt|, which CutParamUlps · ulp(1) · tangentUpper covers. Sliding along
// a carrier is still a boundary coordinate moving, so the answer feeds the
// section displacement every other consumer reads, not a private term.
//
// A non-finite carrier speed answers +Inf rather than a number: an absent bound
// must never read as a small one, and a zero would let the displacement vanish
// silently from every measurement that composes it.
func CutDisplacementAllow(tangentUpper float64) float64 {
	if IsNonFinite(tangentUpper) {
		return math.Inf(1)
	}
	if tangentUpper <= 0 {
		// A carrier that does not move cannot put the point anywhere else,
		// whatever the parameter reads.
		return 0
	}
	return ProductUpper(CutParamUlps*UlpOf(1), tangentUpper)
}

// WalkEndpointAllow bounds the rounding a source segment's own WALKED
// endpoint commits when the record's parameterisation is evaluated at a
// narrowed t rather than read off the segment's own natural bound
// (docs/prism-boolean-design.md §7's δ_walk — the analytic boolean's private
// scene is built from walkOf's own walked geometry, prism_boolean.go's
// buildPrismScene).
//
// operandUpper must be a PROVEN upper bound on the magnitude of every operand
// the walk's OWN arithmetic touches — not on the answer that arithmetic
// produces. The two are different quantities and the difference is the whole
// point: a line's walked endpoint is fl(a + fl(t·fl(b−a))) for the carrier's
// own recorded a and b, and the difference b−a CANCELS, so a fragment sitting
// near the plane origin on a far-reaching carrier rounds by ulps of the
// CARRIER while its own endpoint magnitude is tiny. Charging the endpoint's
// magnitude would under-charge that walk without limit, so callers pass the
// SOURCE envelope: prism_boolean.go's lineWalkOperandUpper for a line (the
// recorded Start/End coordinates, folded together with the walked endpoint so
// the envelope stands whatever the recorded parameter is).
//
// A trimmed line is the only carrier the analytic boolean charges here. A
// trimmed circular one is refused before its scene is built
// (prismProfileHasTrimmedCircularSource), since its rebuilt radius and sweep
// move as well as its endpoints, so walkChargeOf's circular arm — which
// passes segmentWalk.coordUpper, whose |c|+|c|+r+r L1 form bounds the centre
// and radius a cos/sin walk works on — is unreachable through that path. It
// stands so a widening of that refusal meets a charge rather than a silent
// zero.
//
// The proof, per coordinate, with E = operandUpper and u = 2⁻⁵³ the unit
// roundoff. fl(b−a) is off by at most u·|b−a| ≤ 2uE; multiplying by t carries
// that through and rounds once more, at the magnitude of t·(b−a), which is the
// walked endpoint less a and so at most 2E, for another 2uE; the outer sum
// rounds at |a + t(b−a)| ≤ E, for uE. That totals 5uE + O(u²) per coordinate.
// A target that FUSES the multiply and the add — the gc arm64 backend
// compiles lerp2's general arm to a single FMADDD — drops the middle rounding
// and keeps the other two, so it stays under the same total: fusion only ever
// removes a rounding from that sequence, and fl(b−a), the term this
// mechanism's own cancellation makes large, is committed before any fusion
// and survives it unchanged.
// The answer charges 16·ulp(2E) ≥ 16·ulp(1)·E = 32uE per coordinate, better
// than six times that, read as a 3D radius (Radius3D) — the SAME shape
// RigidRoundAllow states for a rigid map's own rounding, which keeps every
// displacement mechanism in this file stated the same way.
//
// A non-finite envelope answers +Inf, never 0: an absent bound must never read
// as a small one (CutDisplacementAllow's own rule, restated here because this
// helper sits right beside it).
func WalkEndpointAllow(operandUpper float64) float64 {
	if IsNonFinite(operandUpper) {
		return math.Inf(1)
	}
	if operandUpper <= 0 {
		return 0
	}
	ulp := UlpOf(2 * operandUpper)
	if IsNonFinite(ulp) {
		return math.Inf(1)
	}
	return Radius3D(16 * ulp)
}

// WalkEndBoundAllow turns a free-form walk endpoint's own PROVEN
// per-component bound (WalkEndBound — segmentWalk's startBound/endBound, or
// the identical reading a caller takes over one interior span joint) into a
// 3D world-space displacement bound. The wider of the two plane-local
// components is read as a per-coordinate bound and carried through the
// payload's orthonormal frame by Radius3D, exactly as any other coordinate
// error is (core §5.2 — a coordinate's error bound is a radius, not an axis
// extent): a frame's U/V/N are pairwise orthogonal unit directions, so a
// bound on each in-plane component is itself a bound on the corresponding
// world-space one, and folding the wider of the two through Radius3D's own
// three-axis shape can only widen the true two-axis figure, never narrow it.
//
// An underivable component answers +Inf, never a small number silently spent
// in its place (CutDisplacementAllow's own rule).
func WalkEndBoundAllow(bound WalkEndBound) float64 {
	if !bound.Derivable() {
		return math.Inf(1)
	}
	return Radius3D(math.Max(math.Abs(bound.U), math.Abs(bound.V)))
}

// BandPatchAreaAllow bounds how far ONE chamfer band patch's own area
// (docs/modify-reach-design.md §8.4) can differ from the area of the ruled
// quad the construction DENOTES, given that its cap-level directrix sits
// within delta of the point it denotes (docs/prism-boolean-design.md §7's
// identity, one ruled patch at a time rather than one whole section). The
// side-level directrix is NOT exact either, but its displacement is a
// different mechanism and has its own helper: BandLevelAreaAllow below, which
// patchAreaOf composes beside this one.
//
// A ruled quad's area is, to first order, its chord length times its slant
// distance, so moving only the cap-level chord changes area two ways at
// once: the chord's own length can change by at most
// SectionDisplacementLength(delta, 1) — the SAME per-walk bound a recorded
// boundary segment's length carries under this displacement, since a single
// chord is exactly what that helper already bounds for one walk — which
// moves area at the rate of the patch's own slant distance; and the slant
// distance can itself change by at most delta, because only its cap-level
// endpoint moves, which moves area at the rate of the chord length it rules
// along. chordUpper and slantUpper must each be a PROVEN upper bound on the
// patch's own held chord length and held slant distance.
func BandPatchAreaAllow(delta, chordUpper, slantUpper float64) float64 {
	if delta <= 0 {
		return 0
	}
	return UpRound(ProductUpper(SectionDisplacementLength(delta, 1), slantUpper) + ProductUpper(chordUpper, delta))
}

// BandLevelAreaAllow is BandPatchAreaAllow's companion on the band's OTHER
// directrix: how far ONE chamfer band patch's own area can differ from the
// area of the patch the construction DENOTES, given that its SIDE-level
// directrix sits within levelDelta of the level it denotes.
//
// The two directrices are displaced by different mechanisms and so are bounded
// by different helpers. The cap-level contour is moved point by point by a
// float offset solve, which is what BandPatchAreaAllow charges. The side level
// is the single float sum capZ + matSign*d (capblend_geom.go's levelDelta), so
// the whole side directrix is translated AXIALLY and RIGIDLY by at most that
// much, its own in-plane shape untouched.
//
// Under such a translation a patch's area moves at the rate of its own two
// directrix lengths, and both patch shapes the band builds give the same
// figure:
//
//   - a Plane patch is two triangles over the quad (sideA, sideB, capB, capA).
//     Moving sideA and sideB together by t leaves (sideB−sideA) alone, so the
//     first triangle's cross product moves by at most |sideB−sideA|·|t|; the
//     second triangle's only moved vertex is sideA, so its cross product moves
//     by at most |capB−capA|·|t| + |t|². Halving each for the triangle areas,
//     the quad's own area moves by at most
//     ½·(|sideB−sideA| + |capB−capA|)·|t| + ½·|t|².
//   - a Cone patch is the frustum sector A = (Δθ/2)·(R0+R1)·√(ΔR²+H²), whose
//     derivative in H is (Δθ/2)·(R0+R1)·H/√(ΔR²+H²) and is therefore bounded
//     by (Δθ/2)·(R0+R1) — half the sum of that patch's own two directrix arc
//     lengths, Δθ·R0 and Δθ·R1.
//
// directrixSumUpper must be a PROVEN upper bound on the sum of the patch's two
// directrix lengths and levelDelta a proven bound on the axial displacement.
// The ½ is dropped and one levelDelta folded in, which dominates both readings
// above.
func BandLevelAreaAllow(levelDelta, directrixSumUpper float64) float64 {
	if levelDelta <= 0 {
		return 0
	}
	return ProductUpper(AbsSumUpper(directrixSumUpper, levelDelta), levelDelta)
}

// ChordLocusVolumeAllow bounds capblend_moments.go's chord-versus-locus
// residual: the flux gap between a cap-loop chamfer's regular-wall Cone patch
// B, the straight-ruled surface the topology builds between its two
// directrices, and the denoted miter-locus surface T
// (docs/modify-reach-design.md §8.3: "at axial fraction s, the denoted miter
// locus is the parallel section offset by s*dc"). B meets T only at s=0 and
// s=1 and chords it in between wherever the cap-level directrix
// (capTh0, capTh1) sweeps a narrower window than the side-level one
// (th0, th1).
//
// The band's volume is a third of the summed fluxes of its closed surface
// about the plane-local origin O, but this term is taken about the arc's axis point
// c at the side level. For any surface X, Flux_O(X) = Flux_c(X) + c·A(X),
// where A(X) is X's vector area, which depends only on X's boundary. The true
// patch and the built one end on different curves at each mitered corner k:
// the true patch on the curved corner-foot locus, the built patch on the
// straight ruling. So A(true) − A(built) is the sum of ±δ_k, the vector area
// of the closed loop the locus and the ruling form, and the patch across the
// ruling carries the opposite sign. This patch's share of that shift,
// split at the corner vertex v_k (at c's level), is |(c − v_k)·δ_k|. A Plane
// neighbour's share is zero: the locus and the ruling both lie in its plane,
// with v_k, so δ_k is normal to it. A Cone neighbour charges its own share
// about its own axis. Both curves are ridden at height (ds/dc)·t for offset
// amount t, so δ_k's horizontal part is (ds/dc)·rot90(W_k) with
// W_k = ∫₀^dc (P(t) − Q(t)) dt, P the locus and Q the ruling, and the share is
// (ds/dc)·|(v_k − c) × W_k|. cornerFlux must be a PROVEN upper bound on the
// sum of this patch's two shares (ChordLocusCornerFlux forms each one). It is
// zero wherever both corners' loci are straight: a reflex foot, a G1 join and
// a whole turn. Without it the about-c bound below covers
// |Flux_c(true) − Flux_c(built)|, but not the about-O difference the band's
// volume reads.
//
// The about-c part reads three fluxes, each a capband.RawFlux value with its
// proven bound, all at the patch's own radii and levels and about c: W, the
// rotationally symmetric cone sector over the UNION of the side window
// (th0, th1) and the cap window (capTh0, capTh1) on both directrices; N, the
// same sector over their INTERSECTION; and B, the built ruled patch itself.
// When both corners trim the cap window inside the side window, W is the side
// sector and N the cap sector. A clockwise-walked patch negates all three, so
// the interval below is the one between N and W in either order.
//
//   - Flux_c(T) lies between N and W. At height z, T is the cone over the
//     window [a(z), b(z)] whose ends are the two corner-foot loci's azimuths
//     about the centre. Each locus runs from the corner, at the side window's
//     end, to the cap-level foot, at the cap window's end, and its azimuth is
//     monotone in the offset amount: for a line meeting the circle,
//     cos(φ − ν) = (α + σ·t)/(R + ρ·t), with ν the line's normal, α its
//     distance from the centre and σ, ρ = ±1, has a t-derivative of constant
//     sign, (σ·R − ρ·α)/(R + ρ·t)²; for two circles the cosine of the angle
//     at this centre, (R1² + d² − R2²)/(2·R1·d), has a derivative whose
//     numerator is ±((R1 ∓ R2)² − d²), constant because R1 ∓ R2 is; and in
//     both cases φ stays on one side of its reference line while the two
//     carriers never touch, which capband.MiterLocusSliverFlux requires
//     before it bounds the corner (a refusal makes cornerFlux +Inf and this
//     term unbounded). A reflex foot and a G1 join run along a straight
//     ruling between the two ends. So a(z) lies between th0 and capTh0 and
//     b(z) between th1 and capTh1, and T's window holds the intersection and
//     lies in the union. Its flux is sandwiched the same way only where the
//     flux density has one sign over the cone, so all three fluxes are taken
//     about the arc's own axis at the side level (capband's
//     chordLocusResidualAllow moves the centre to the origin and the side
//     level to zero). About that point, the cone at angle θ and axial offset z
//     has radius r(z) = R0 + (R1-R0)·z/H, and its flux density per dθ·dz is
//     R0·r(z), never negative. About any other point the density changes sign
//     across the cone and the sandwich fails.
//   - Flux_c(B) lies within ε of the interval between N and W, with
//     ε = ChordLocusBuiltExcursion of the three enclosures: how far B's
//     enclosure reaches past the ends of that interval. ε is proven because
//     each enclosure is, and it needs no claim about where the ruled surface
//     sits. ε is zero wherever B's enclosure lies inside the interval with
//     room for the three bounds.
//
// A number inside an interval and a number within ε of it differ by at most
// the interval's width plus ε, so the about-c part is |W − N| plus both
// reference bounds, plus ε. The bound never reads |W − B| alone. B and W end on different
// rulings at each corner, and the corner triangle between those rulings
// carries a flux of about ½·R0·R1·H·Φ for a corner skew Φ, which no bound on
// how far B's interior points sit from W accounts for.
//
// Every operation rounds outward: the sums and differences, the corner flux
// included, are taken exactly and rounded up once. A flux or bound that does not lift, or a cornerFlux
// that is negative or not finite, answers an unbounded term.
func ChordLocusVolumeAllow(fluxWide, wideBound, fluxNarrow, narrowBound, fluxBuilt, builtBound, cornerFlux float64) float64 {
	if !(cornerFlux >= 0) || IsNonFinite(cornerFlux) {
		return math.Inf(1)
	}
	slack, ok := chordLocusEnvelopeSlack(fluxWide, wideBound, fluxNarrow, narrowBound)
	if !ok {
		return math.Inf(1)
	}
	excursion, ok := chordLocusExcursion(fluxWide, wideBound, fluxNarrow, narrowBound, fluxBuilt, builtBound)
	if !ok {
		return math.Inf(1)
	}
	slack.Add(slack, excursion)
	return RatFloatUp(slack.Add(slack, proofarith.FloatRat(cornerFlux)))
}

// ChordLocusBuiltExcursion is ChordLocusVolumeAllow's ε, rounded up: an
// ε ≥ 0 for which every built flux B within builtBound of fluxBuilt lies in
// [min(W, N) − ε, max(W, N) + ε] for every wide flux W within wideBound of
// fluxWide and every narrow flux N within narrowBound of fluxNarrow. With
// w = fluxWide, n = fluxNarrow, b = fluxBuilt and their bounds wb, nb, bb, it
// is
//
//	max(0, (b + bb) − max(w − wb, n − nb), min(w + wb, n + nb) − (b − bb)).
//
// It is taken exactly. An input that does not lift, or a bound that is
// negative or not finite, answers +Inf.
func ChordLocusBuiltExcursion(fluxWide, wideBound, fluxNarrow, narrowBound, fluxBuilt, builtBound float64) float64 {
	excursion, ok := chordLocusExcursion(fluxWide, wideBound, fluxNarrow, narrowBound, fluxBuilt, builtBound)
	if !ok {
		return math.Inf(1)
	}
	return RatFloatUp(excursion)
}

// ChordLocusRegionAllow bounds three times the VOLUME of the region between
// the solid a cap-loop chamfer's regular-wall Cone patch B bounds and the
// solid the denoted surface T bounds: the measure capband.ChordLocusVolume
// hands to the band's first-moment bound, which moves by at most that volume
// times the largest coordinate the region holds. ChordLocusVolumeAllow bounds
// only the signed flux gap, which can be smaller than the region's measure
// when B lies outside T in one place and inside it in another, so the moment
// term reads this one.
//
// The argument works in cylindrical coordinates (ρ, θ, z) about the arc's
// axis and reads no point on it, so the anchored levels the reference fluxes
// use do not enter. Write r(z) = R0 + (R1 − R0)·z/H for the cone's radius at
// height z above the side level. T is the cone r(z) over the angular window
// [a(z), b(z)] the erosion family gives at that height, with a(z) between θS0
// and θC0 and b(z) between θS1 and θC1, in whichever order: each corner-foot
// locus's azimuth is monotone in the offset amount (ChordLocusVolumeAllow's
// first bullet), so the caller must answer +Inf for a patch whose corner
// could not be bounded, where the carriers may touch and the azimuth turn
// back. Here W is the sector over the side window alone. Ride W and B on one
// parametrisation (u, v) over the unit square: at height v·H,
// W(u, v) has plane-local position ((1−v)·R0 + v·R1)·e^{iθS(u)} and B(u, v)
// has (1−v)·R0·e^{iθS(u)} + v·R1·e^{iθC(u)}, θS and θC linear in u across the
// side and cap windows. H_λ = (1−λ)·W + λ·B is the straight homotopy between
// them.
//
// Across a corner ruling the band's neighbour ends on the locus for T and on
// the ruling for B. Close each patch's difference with the surface σ joining
// the locus point P(z) and the ruling point Q(z) by a straight segment at each
// height z; neighbouring patches carry σ with opposite signs, so the band's
// two solids differ exactly where the sum over patches of the winding numbers
// of the closed surfaces T − B ± σ is nonzero, and the region's volume is at
// most the sum over patches of the volume where each one's winding number is
// nonzero. A Plane neighbour's own T, B and σ all lie in its plane, so its
// winding number is zero almost everywhere. For a Cone patch, the winding
// number at a point is the signed count of crossings of the horizontal ray
// from it away from the axis, so along each such ray it is nonzero only
// between the nearest and the farthest crossing of T, B or σ.
//
// Three facts about the level curves at height z bound those crossings.
// First, H_λ's level curve u ↦ (1−v)·R0·e^{iθS(u)} + v·R1·((1−λ)·e^{iθS(u)} + λ·e^{iθC(u)})
// has azimuth strictly increasing in u, because its cross product with its
// own u-derivative is A²·dS + C²·dC + A·C·(dS + dC)·cos(θC(u) − θS(u)) > 0,
// with A, C ≥ 0 the two coefficients, dS, dC > 0 the window widths and the
// skew below a quarter turn; its ends sit at azimuths between θS0 and θC0 and
// between θC1 and θS1, since a non-negative combination of two unit vectors
// less than a half turn apart points between them. So every H_λ, W at λ = 0
// and B at λ = 1, crosses each ray at an azimuth θ in the middle window
// between those two corner wedges (between max(θS0, θC0) and
// min(θS1, θC1)) exactly once, at a radius f_λ(θ, z)
// continuous in λ. Second, B's radius is at least r(z) − dB, with
// r² − |B|² = 4·v·(1−v)·R0·R1·sin²((θC − θS)/2) ≤ R0·R1·sin²(Φ/2), so
// dB = max(R0, R1)·Φ²/4 for the larger corner skew Φ. Third, σ at height z is
// the segment from P(z), at radius r(z) on T's edge, to Q(z), at radius at
// least r(z) − dB on B's ruling, both inside one corner wedge, whose width is
// at most Φ: every point of the segment has a component of at least
// (r(z) − dB)·cos(Φ/2) along the wedge's bisector, so σ's radius is at least
// r(z) − dσ with dσ = dB + r(z)·2·sin²(Φ/4) ≤ (3/8)·max(R0, R1)·Φ²
// (ChordLocusCornerDeficitUpper). Every crossing's radius is at most r(z): T
// lies on the cone, B and σ inside it by the triangle inequality.
//
// At a middle azimuth only T, at r(z), and B, at f_1(θ, z), cross the ray, so
// the winding number is nonzero only between them. Every radius in that
// stretch is f_λ(θ, z) for some λ in [0, 1] by the intermediate value
// theorem, so the point lies on H_λ. The set the homotopy passes through has
// volume at most ∫∫∫ |∂λH|·|∂uH × ∂vH|, with ∂λH = B − W =
// v·R1·(e^{iθC(u)} − e^{iθS(u)}) of length at most R1·Φ, so at most
// R1·Φ·sup_λ Area(H_λ): SweptVolumeAllow of that displacement against an area
// bound for EVERY H_λ, λ in [0, 1]. homotopyAreaUpper must be such a bound
// (ChordLocusHomotopyAreaUpper forms one); the area of B alone does not
// qualify, since W, at λ = 0, spans the wider side window. At an azimuth in a
// corner wedge every crossing lies in [r(z) − dσ, r(z)], so the region there
// lies in a shell of that thickness over the two corner wedges, whose volume
// cornerShellUpper must bound (ChordLocusCornerShellUpper forms one).
//
// The region therefore has volume at most the swept volume plus the shell's,
// and the bound is exactly that. The corner slivers are part of the shell,
// so no corner flux is charged beside it. A zero skew puts every corner's
// side and cap ends on one ray, so each corner's locus, whose azimuth runs
// between those two ends, is the straight ruling itself, and the built patch
// is the cone sector: the region is empty.
//
// T and B here both run between the same two held levels, H apart. The held
// side level's own displacement from the denoted one moves the whole body,
// slab and band together, and the decad package charges it once per band
// (capblend_moments.go's capBandLevelVolume), so no term here reads it.
//
// radiusUpper must bound both radii, so R1·Φ ≤ radiusUpper·windowSkewMax, and
// windowSkewMax must be a PROVEN upper bound on the larger corner skew, below
// a quarter turn (capband.CornerSkewUpper). The result is three times the
// volume, the flux units capband.ChordLocusVolume divides by 3 once. A skew,
// radius, area or shell that is negative or not finite answers +Inf.
func ChordLocusRegionAllow(radiusUpper, windowSkewMax, homotopyAreaUpper, cornerShellUpper float64) float64 {
	if !(windowSkewMax >= 0) || IsNonFinite(windowSkewMax) {
		return math.Inf(1)
	}
	if windowSkewMax == 0 {
		return 0
	}
	for _, in := range []float64{radiusUpper, homotopyAreaUpper, cornerShellUpper} {
		if !(in >= 0) || IsNonFinite(in) {
			return math.Inf(1)
		}
	}
	swept := SweptVolumeAllow(ProductUpper(radiusUpper, windowSkewMax), homotopyAreaUpper)
	return ProductUpper(3, AbsSumUpper(swept, cornerShellUpper))
}

// ChordLocusCornerDeficitUpper bounds how far inside the cone radius r(z) any
// crossing of a Cone patch's built surface or corner sliver can sit at a
// corner azimuth (ChordLocusRegionAllow's dσ): (3/8)·radiusUpper·Φ², rounded
// up, for radiusUpper a bound on both radii and windowSkewMax the larger
// corner skew Φ. A negative or non-finite input answers +Inf.
func ChordLocusCornerDeficitUpper(radiusUpper, windowSkewMax float64) float64 {
	if !(radiusUpper >= 0) || !(windowSkewMax >= 0) || IsNonFinite(radiusUpper) || IsNonFinite(windowSkewMax) {
		return math.Inf(1)
	}
	return ProductUpper(ProductUpper(radiusUpper, 0.375), ProductUpper(windowSkewMax, windowSkewMax))
}

// ChordLocusCornerShellUpper bounds the volume of ChordLocusRegionAllow's
// corner shell: the points at an azimuth inside either corner wedge, at a
// height in [0, H], whose radius lies within the deficit dσ below r(z). The
// two wedges are at most skewStart + skewEnd wide, and at each height the
// shell's cross-section per unit angle is (r² − (r − dσ)²)/2 ≤ r·dσ, so its
// volume is at most (skewStart + skewEnd)·heightUpper·radiusUpper·dσ, with
// dσ = ChordLocusCornerDeficitUpper(radiusUpper, max(skewStart, skewEnd)).
// Every operation rounds up. A negative or non-finite input answers +Inf.
func ChordLocusCornerShellUpper(radiusUpper, skewStart, skewEnd, heightUpper float64) float64 {
	for _, in := range []float64{radiusUpper, skewStart, skewEnd, heightUpper} {
		if !(in >= 0) || IsNonFinite(in) {
			return math.Inf(1)
		}
	}
	deficit := ChordLocusCornerDeficitUpper(radiusUpper, math.Max(skewStart, skewEnd))
	wedges := AbsSumUpper(skewStart, skewEnd)
	return ProductUpper(ProductUpper(wedges, heightUpper), ProductUpper(radiusUpper, deficit))
}

// ChordLocusHomotopyAreaUpper bounds the area of every surface
// H_λ = (1−λ)·W + λ·B, λ in [0, 1], between a Cone patch's wide reference
// sector W and its built ruled patch B (ChordLocusRegionAllow states the
// parametrisation). With dS and dC the side and cap window widths,
//
//	∂uH = i·((1−v)·R0·dS·e^{iθS} + v·R1·((1−λ)·dS·e^{iθS} + λ·dC·e^{iθC})),
//
// so |∂uH| ≤ max(R0, R1)·max(dS, dC), and ∂vH has in-plane part
// (R1 − R0)·e^{iθS} + λ·R1·(e^{iθC} − e^{iθS}) and axial part H, so
// |∂vH| ≤ |R1 − R0| + R1·Φ + |H|. The area is at most the integral of
// |∂uH|·|∂vH| over the unit square, which is at most the product of the two
// bounds. radiusUpper must bound both radii, windowUpper both window widths,
// radialGapUpper |R1 − R0|, heightUpper |H| and windowSkewMax the larger
// corner skew Φ, each proven. Every operation rounds up. A negative or
// non-finite input answers +Inf.
func ChordLocusHomotopyAreaUpper(radiusUpper, windowUpper, radialGapUpper, heightUpper, windowSkewMax float64) float64 {
	for _, in := range []float64{radiusUpper, windowUpper, radialGapUpper, heightUpper, windowSkewMax} {
		if !(in >= 0) || IsNonFinite(in) {
			return math.Inf(1)
		}
	}
	slant := AbsSumUpper(radialGapUpper, ProductUpper(radiusUpper, windowSkewMax), heightUpper)
	return ProductUpper(ProductUpper(radiusUpper, windowUpper), slant)
}

// chordLocusEnvelopeSlack is |fluxWide − fluxNarrow| + wideBound + narrowBound
// taken exactly: the most the true wide and narrow fluxes can differ by. A
// value that does not lift, or a bound that is negative or not finite,
// answers false.
func chordLocusEnvelopeSlack(fluxWide, wideBound, fluxNarrow, narrowBound float64) (*big.Rat, bool) {
	rs, ok := liftChordLocusFluxes(fluxWide, wideBound, fluxNarrow, narrowBound)
	if !ok {
		return nil, false
	}
	slack := new(big.Rat).Abs(new(big.Rat).Sub(rs[0], rs[2]))
	return slack.Add(slack, rs[1]).Add(slack, rs[3]), true
}

// chordLocusExcursion is ChordLocusBuiltExcursion taken exactly.
func chordLocusExcursion(fluxWide, wideBound, fluxNarrow, narrowBound, fluxBuilt, builtBound float64) (*big.Rat, bool) {
	rs, ok := liftChordLocusFluxes(fluxWide, wideBound, fluxNarrow, narrowBound, fluxBuilt, builtBound)
	if !ok {
		return nil, false
	}
	w, wb, n, nb, b, bb := rs[0], rs[1], rs[2], rs[3], rs[4], rs[5]
	// lowerTop is the least the interval's top end can be, upperBottom the
	// most its bottom end can be.
	lowerTop := ratMax(new(big.Rat).Sub(w, wb), new(big.Rat).Sub(n, nb))
	upperBottom := ratMin(new(big.Rat).Add(w, wb), new(big.Rat).Add(n, nb))
	above := new(big.Rat).Add(b, bb)
	above.Sub(above, lowerTop)
	below := new(big.Rat).Sub(upperBottom, b)
	below.Add(below, bb)
	return ratMax(new(big.Rat), ratMax(above, below)), true
}

func ratMax(a, b *big.Rat) *big.Rat {
	if a.Cmp(b) >= 0 {
		return a
	}
	return b
}

func ratMin(a, b *big.Rat) *big.Rat {
	if a.Cmp(b) <= 0 {
		return a
	}
	return b
}

// liftChordLocusFluxes lifts (value, bound) pairs to exact rationals. A value
// that does not lift, or a bound that is negative or not finite, answers
// false.
func liftChordLocusFluxes(pairs ...float64) ([]*big.Rat, bool) {
	out := make([]*big.Rat, len(pairs))
	for i, x := range pairs {
		if i%2 == 1 && !(x >= 0) {
			return nil, false
		}
		r := proofarith.FloatRat(x)
		if r == nil {
			return nil, false
		}
		out[i] = r
	}
	return out, true
}

// ChordLocusCornerFlux is one mitered corner's share of the chord-locus term
// (ChordLocusVolumeAllow's cornerFlux): (ds/dc)·|(v − c) × W|, rounded up.
// axialUpper must be a proven upper bound on the side setback ds, dcLower a
// proven positive lower bound on the cap setback dc, and momentUpper a proven
// upper bound on the corner's in-plane sliver moment |(v − c) × W| about the
// patch's centre (capcontour.LineCircleLocusSliverMoment and
// capcontour.LocusVelocityHull bound it). A negative or non-finite input, or
// a dcLower that is not positive, answers +Inf. Otherwise a zero moment, the
// moment of a straight locus, charges nothing.
func ChordLocusCornerFlux(axialUpper, dcLower, momentUpper float64) float64 {
	if !(axialUpper >= 0) || !(momentUpper >= 0) || !(dcLower > 0) ||
		IsNonFinite(axialUpper) || IsNonFinite(momentUpper) || IsNonFinite(dcLower) {
		return math.Inf(1)
	}
	if momentUpper == 0 {
		return 0
	}
	return ProductUpper(DivUpper(axialUpper, dcLower), momentUpper)
}

// ChordLocusLengthAllow bounds a cap-blend miter ruling's own chord-versus-
// locus excess (docs/modify-reach-design.md §8.3's boundary bullet): how far
// the denoted corner-foot locus's true length can exceed the built chord it
// is tagged `Line3` as, given a proven upper bound speedUpper on the locus's
// own in-plane speed |dP/dt| over the offset range [0, dc] and the locus's
// own EXACT axial speed |axialSpan|/dc — z is affine in the offset amount by
// construction (a fixed side level and a fixed cap level, ruled linearly),
// so that ratio needs no enclosure, only a division rounded up.
//
// The locus length is at most dc·sqrt(speedUpper² + (axialSpan/dc)²) — the
// range's own width times an upper bound on the 3D speed, Radius2D's own
// √2-scaled bound on the two independently-bounded components — and a chord
// never exceeds the curve it subtends, so chordUpper (a PROVEN upper bound on
// the patch's own held chord) is itself a lower bound on the true locus
// length. The excess is that product minus chordUpper, clamped at zero (a
// negative reading proves nothing — the bound is loose there, not the locus
// short) and rounded up.
func ChordLocusLengthAllow(speedUpper, dc, axialSpan, chordUpper float64) float64 {
	if speedUpper < 0 || dc <= 0 || IsNonFinite(speedUpper) || IsNonFinite(chordUpper) {
		return math.Inf(1)
	}
	rdc, raxial := proofarith.FloatRat(dc), proofarith.FloatRat(axialSpan)
	if rdc == nil || raxial == nil {
		return math.Inf(1)
	}
	zSpeed, exact := new(big.Rat).Quo(new(big.Rat).Abs(raxial), rdc).Float64()
	if !exact {
		zSpeed = math.Nextafter(zSpeed, math.Inf(1))
	}
	if IsNonFinite(zSpeed) {
		return math.Inf(1)
	}
	locusUpper := ProductUpper(dc, Radius2D(speedUpper, zSpeed))
	excess := locusUpper - chordUpper
	if excess <= 0 {
		return 0
	}
	return UpRound(excess)
}

// SweptMomentAllow bounds the FIRST MOMENT a cap contour's own displacement
// can move — SweptVolumeAllow's own sibling one power higher
// (docs/modify-reach-design.md §8.4's fourth reading, beside the band
// volume, the chamfered cap face area, and each band patch's own area). The
// symmetric difference between the band the build holds and the one the
// offset denotes has volume at most SweptVolumeAllow(delta, areaUpper) — that
// identity is unchanged here — and every point of that difference lies within
// coordUpper of the plane-local origin (the fixed point every first-moment
// integral in this file is taken about), so the moment the difference can
// carry is at most that volume times coordUpper: a region of proven volume V
// all of whose points lie within coordUpper of the origin has |∫p dV| <=
// V·coordUpper for any single coordinate p, since |p| <= coordUpper
// pointwise. coordUpper must be a PROVEN upper bound on |u|, |v| and |z| over
// the band's own material — the band lies BETWEEN the original loop and the
// offset cap boundary, so both loops' own envelopes (extrude.go's
// profileCoordinateUpper) are needed, together with max(|z0|, |z1|) — the
// same envelope prismCentroidGeometryBound already forms for the axial
// levels.
func SweptMomentAllow(delta, areaUpper, coordUpper float64) float64 {
	vol := SweptVolumeAllow(delta, areaUpper)
	if vol <= 0 || coordUpper <= 0 {
		return 0
	}
	return ProductUpper(vol, coordUpper)
}

// ChordedBoundaryMomentAllow bounds the first moment of the occupied symmetric
// difference between the held chorded body and the denoted body. Only the wall
// chord-to-curve sweep and the triangle-to-bilinear twist sweep are measures.
// capVolumeUpper is an exact planar signed-volume correction and seamAllow is
// an integration-by-parts residue; neither represents swept 3D material, so
// multiplying the complete signed-volume allowance by a radius charges two
// terms that the moment does not contain.
//
// The wall measure is matchedDelta*wallAreaUpper. Its points can sit
// matchedDelta beyond the held coordinate envelope, so its moment is bounded
// by that measure times coordUpper+matchedDelta. CellTwistVolumeAllow is the
// twist sweep's exact parameterized measure. Every point of its straight
// homotopy is a convex combination of the cell's four held corners, so it
// stays inside coordUpper and needs no maxTwistOffsetUpper widening. Summing
// those two moment bounds covers the complete symmetric difference by the
// two-stage held -> bilinear -> denoted homotopy.
//
// The cap, seam and max-twist arguments remain in this private signature so
// the volume and moment call sites continue to pass one shared proof bundle.
// They are validated as caller claims but do not enter the moment value.
func ChordedBoundaryMomentAllow(matchedDelta, wallAreaUpper, twistVolumeUpper, capVolumeUpper, seamAllow, maxTwistOffsetUpper, coordUpper float64) float64 {
	if IsNonFinite(matchedDelta) || matchedDelta < 0 {
		return math.Inf(1)
	}
	if IsNonFinite(twistVolumeUpper) || twistVolumeUpper < 0 {
		return math.Inf(1)
	}
	if IsNonFinite(capVolumeUpper) || capVolumeUpper < 0 {
		return math.Inf(1)
	}
	if IsNonFinite(seamAllow) || seamAllow < 0 {
		return math.Inf(1)
	}
	if IsNonFinite(maxTwistOffsetUpper) || maxTwistOffsetUpper < 0 {
		return math.Inf(1)
	}
	if IsNonFinite(coordUpper) || coordUpper < 0 {
		return math.Inf(1)
	}
	wallMeasure := 0.0
	if matchedDelta > 0 {
		if IsNonFinite(wallAreaUpper) || wallAreaUpper < 0 {
			return math.Inf(1)
		}
		wallMeasure = ProductUpper(matchedDelta, wallAreaUpper)
	}
	wallMoment := ProductUpper(wallMeasure, AbsSumUpper(coordUpper, matchedDelta))
	twistMoment := ProductUpper(twistVolumeUpper, coordUpper)
	return AbsSumUpper(wallMoment, twistMoment)
}

// ChordedBoundaryMomentResidualAllow is the moment allowance after the exact
// bilinear-patch volume and first-moment corrections have moved the nominal
// centroid off the held triangle surface. The former twist sweep is therefore
// absent; the remaining wall chord-to-curve sweep is unchanged.
func ChordedBoundaryMomentResidualAllow(matchedDelta, wallAreaUpper, capVolumeUpper, seamAllow, maxTwistOffsetUpper, coordUpper float64) float64 {
	return ChordedBoundaryMomentAllow(
		matchedDelta, wallAreaUpper, 0, capVolumeUpper, seamAllow, maxTwistOffsetUpper, coordUpper,
	)
}

// BoundedSqrt propagates a proven bound through a square root: x.bound must
// already prove the true operand lies in [x.value−x.bound, x.value+x.bound]
// (clamped at zero, since every caller's operand is a sum of squares or a
// disk radius). The result brackets the square root of that whole interval
// through RatSqrtDown/RatSqrtUp — the same rational sqrt brackets
// circularLengthInterval reads an ArcSeg's own radius through — evaluated at
// the interval's two ends, never through math.Sqrt's own accuracy on either.
// A non-finite operand bound answers +Inf: an absent bound must never read as
// a small one.
//
// The outward math.Nextafter step on each end is charged only where there is
// rounding for it to cover: x.value ± x.bound is itself a rounded float
// operation, so a genuinely bounded operand's computed ends can each sit an
// ulp inside the interval they stand for and MUST be stepped out. Adding or
// subtracting exactly zero rounds nothing, so a zero-bound operand's ends are
// already the exact interval — the held value twice — and stepping them out
// would invent a rounding error that provably did not occur. The answer is a
// zero bound precisely when the held value is a perfect square of a float64,
// and a genuine directed-rounding bound whenever it is not.
//
// A zero-bound operand whose float root squares back to it exactly is decided
// first, by exactFloatSquare's FMA residual, and publishes that zero bound
// without the rational brackets. Every other operand, including an exact
// square too small for exactFloatSquare's gate, takes the brackets, which
// decide by exact comparison.
func BoundedSqrt(x BoundedScalar) BoundedScalar {
	value := math.Sqrt(math.Max(x.Value, 0))
	if IsNonFinite(x.Bound) {
		return MeasuredScalar(value, math.Inf(1))
	}
	if x.Bound == 0 && proofarith.ExactFloatSquare(value, x.Value) {
		// A zero bound means the true operand IS x.value, and value² equals it
		// exactly, so value is the true root.
		return MeasuredScalar(value, 0)
	}
	lo := math.Max(0, x.Value-x.Bound)
	hi := x.Value + x.Bound
	if x.Bound != 0 {
		lo = math.Nextafter(lo, math.Inf(-1))
		if lo < 0 {
			lo = 0
		}
		hi = math.Nextafter(hi, math.Inf(1))
	}
	loR, hiR := proofarith.FloatRat(lo), proofarith.FloatRat(hi)
	if loR == nil || hiR == nil {
		return MeasuredScalar(value, math.Inf(1))
	}
	sqrtLo := RatSqrtDown(loR)
	sqrtHi := RatSqrtUp(hiR)
	if IsNonFinite(sqrtLo) || IsNonFinite(sqrtHi) {
		return MeasuredScalar(value, math.Inf(1))
	}
	bound := UpRound(math.Max(value-sqrtLo, sqrtHi-value))
	return MeasuredScalar(value, bound)
}

// BoundedNorm2 is BoundedSqrt's own reduction of a 2D length, over two
// components that already carry their own proven bounds. It never routes
// through math.Hypot's undocumented accuracy: the sum of squares composes
// through the bounded arithmetic, and the square root's own rounding comes
// from BoundedSqrt.
func BoundedNorm2(x, y BoundedScalar) BoundedScalar {
	return BoundedSqrt(BoundedAdd(BoundedMul(x, x), BoundedMul(y, y)))
}

// BoundedHypot is BoundedNorm2 over two EXACT leaves — the convention a
// recorded coordinate takes throughout this package (line endpoints and
// normals, arc centres, junction vertices). A radius this evaluator COMPUTED
// is not one of them: it carries its own bound and must be passed through
// BoundedNorm2 instead.
func BoundedHypot(dx, dy float64) BoundedScalar {
	return BoundedNorm2(ExactScalar(dx), ExactScalar(dy))
}

// DirectionalPerturbationAllow bounds how far a linear functional's own
// extreme over a bounded region can move when the functional's direction is
// perturbed by dirBound: an extreme of gu·u + gv·v over a boundary is
// 1-Lipschitz in (gu, gv) against the boundary's own coordinate envelope,
// since |Δgu·u + Δgv·v| <= |(Δgu, Δgv)| · envelopeUpper by Cauchy-Schwarz.
//
// envelopeUpper must be a PROVEN upper bound on the norm of the very
// coordinate the perturbed direction multiplies, measured about the SAME
// origin the functional is written about. Which envelope that is belongs to
// the caller's own geometry, and the two are NOT interchangeable: a section
// read about its plane frame charges the profile's plane-local envelope
// (profileCoordinateUpper), while a revolve's swept radial coefficient
// multiplies the distance from the RESOLVED AXIS and so charges the axis's
// own radial envelope (axisFrame.radialUpper, which folds in the axis
// anchor). Handing this the frame-origin envelope for an axis-referred
// coordinate understates the result without limit as the two origins
// separate.
func DirectionalPerturbationAllow(dirBound, envelopeUpper float64) float64 {
	return ProductUpper(dirBound, envelopeUpper)
}

// PointPerturbationAllow is DirectionalPerturbationAllow's transpose: how far
// the value of the linear functional gu·u + gv·v can move when the POINT it
// reads carries a proven bound on each of its own components, rather than the
// direction carrying one. |gu·Δu + gv·Δv| ≤ |gu|·boundU + |gv|·boundV, summed
// outward.
//
// The two components enter SEPARATELY, each against the direction coefficient
// that actually multiplies it: a direction reading one axis alone charges
// nothing for the other axis's error, which is what keeps a whole circle's
// reading along u exact even though its own held endpoint misses in v
// (WalkEndBound's own comment). boundU and boundV must be PROVEN bounds on the
// point's displacement from the one the record denotes — segmentWalk's
// startBound/endBound, the only producer of those numbers for a walked
// endpoint.
//
// A charged component answers zero exactly where its bound is zero or its
// coefficient is: a coordinate the record states verbatim, read through a
// direction this evaluator reads as an exact leaf, has no width for a directed
// rounding to invent (boundaryExtremesBoundedContext's own convention). A
// non-finite bound answers +Inf, never 0, so an absent bound never reads as a
// small one.
func PointPerturbationAllow(bound WalkEndBound, gu, gv float64) float64 {
	if IsNonFinite(bound.U) || IsNonFinite(bound.V) {
		return math.Inf(1)
	}
	return AbsSumUpper(
		ProductUpper(math.Abs(gu), bound.U),
		ProductUpper(math.Abs(gv), bound.V),
	)
}

// BoundedFloatError is the proven error bound of a HELD float64 against a
// bounded scalar that already encloses the quantity the float stands for:
// |held − true| ≤ |held − value| + bound, the first term measured exactly over
// the rationals (rationalFloatError) and the two summed outward.
//
// It exists because a producer may evaluate a quantity one way and PROVE it
// another, and the two spellings are then different floats. The circular
// boundary-extreme candidate is that shape: its held position runs through
// math.Cos/math.Sin at the candidate's own angle, while its enclosure comes
// from the angle-free apex identity circularExtremeInterval states. Composing
// the gap this way keeps the held reading exactly where it was — no consumer's
// value moves — while the published bound speaks for the truth.
//
// A non-finite operand answers +Inf, never 0: an absent bound must never read
// as a small one (CutDisplacementAllow's own rule).
func BoundedFloatError(bs BoundedScalar, held float64) float64 {
	if IsNonFinite(bs.Value) || IsNonFinite(bs.Bound) || IsNonFinite(held) {
		return math.Inf(1)
	}
	return AbsSumUpper(proofarith.RationalFloatError(proofarith.FloatRat(bs.Value), held), bs.Bound)
}

// ExactIsometryDotRound is the tight companion to RigidRoundAllow: instead of
// a worst-case ulp estimate from an input MAGNITUDE, it proves |held − true|
// exactly, over the rationals, for one scalar coefficient of the form
// xform.Apply(pt).Dot(g) (translate true) or xform.ApplyDir(pt).Dot(g)
// (translate false) — the shape extrude.go's prism box and revolve.go's
// revolve box each lift a boundary or sweep extreme through, reading the
// frame and placement as an exact leaf. It is exactPrismPointRound's own
// rational chain (extrude.go), reduced to the single coefficient a
// directional reading needs rather than a whole placed point, and read
// through a caller-supplied g rather than fixed to a Vec's three components.
//
// held must be the SAME float the caller's own arithmetic produced —
// xform.Apply(pt).Dot(g) or xform.ApplyDir(pt).Dot(g), computed the ordinary
// way — so the comparison measures the rounding that arithmetic actually
// committed, never a different evaluation order's own. Every basis vector,
// pt and g component must itself be an exactly representable float64 (they
// always are here: a Transform's basis and translation, a payload's own
// frame vectors, and g one of the three world unit axes), or the answer is
// +Inf — an absent bound must never read as a small one
// (CutDisplacementAllow's own rule).
func ExactIsometryDotRound(xform r3.Transform, pt, g r3.Vec, translate bool, held float64) float64 {
	ratOfVec := func(v r3.Vec) [3]*big.Rat {
		return [3]*big.Rat{proofarith.FloatRat(v.X), proofarith.FloatRat(v.Y), proofarith.FloatRat(v.Z)}
	}
	anyNil := func(r [3]*big.Rat) bool { return r[0] == nil || r[1] == nil || r[2] == nil }
	ptR, gR := ratOfVec(pt), ratOfVec(g)
	basis := xform.Basis()
	exR, eyR, ezR := ratOfVec(basis.EX), ratOfVec(basis.EY), ratOfVec(basis.EZ)
	if anyNil(ptR) || anyNil(gR) || anyNil(exR) || anyNil(eyR) || anyNil(ezR) {
		return math.Inf(1)
	}
	placed := [3]*big.Rat{}
	for i := range placed {
		placed[i] = RatAdd(RatMul(exR[i], ptR[0]), RatMul(eyR[i], ptR[1]), RatMul(ezR[i], ptR[2]))
	}
	if translate {
		trR := ratOfVec(xform.Translation())
		if anyNil(trR) {
			return math.Inf(1)
		}
		for i := range placed {
			placed[i] = RatAdd(placed[i], trR[i])
		}
	}
	exact := RatAdd(RatMul(placed[0], gR[0]), RatMul(placed[1], gR[1]), RatMul(placed[2], gR[2]))
	return proofarith.RationalFloatError(exact, held)
}

// ExactFrameLocalRound proves, over the rationals, the rounding ONE plane-local
// coordinate of frame.ToLocal(p) commits: |held − (p − frame.Origin())·axis|
// measured exactly, with axis the frame's OWN u or v. It is
// ExactIsometryDotRound one transform earlier — that helper reads a placement's
// basis, this one a frame's — and it is what a plane-local anchor's proven
// bound is: the frame and the world point are exact leaves, so the projection's
// only error is what its own float arithmetic rounded away.
//
// held must be the SAME float the caller's own arithmetic produced —
// frame.ToLocal(p).X against frame.U(), .Y against frame.V() — so the
// comparison measures the rounding that call actually committed. The answer is
// zero exactly where that arithmetic is exact for the input at hand (an
// axis-aligned frame and an exactly representable anchor), and a component no
// rational holds answers +Inf, never 0 (CutDisplacementAllow's own rule).
func ExactFrameLocalRound(frame r3.Frame, p, axis r3.Vec, held float64) float64 {
	ratOfVec := func(v r3.Vec) [3]*big.Rat {
		return [3]*big.Rat{proofarith.FloatRat(v.X), proofarith.FloatRat(v.Y), proofarith.FloatRat(v.Z)}
	}
	anyNil := func(r [3]*big.Rat) bool { return r[0] == nil || r[1] == nil || r[2] == nil }
	pR, axR, oR := ratOfVec(p), ratOfVec(axis), ratOfVec(frame.Origin())
	if anyNil(pR) || anyNil(axR) || anyNil(oR) {
		return math.Inf(1)
	}
	terms := [3]*big.Rat{}
	for i := range terms {
		terms[i] = RatMul(new(big.Rat).Sub(pR[i], oR[i]), axR[i])
	}
	return proofarith.RationalFloatError(RatAdd(terms[0], terms[1], terms[2]), held)
}

// ExactPlaneDotRound proves, over the rationals, the rounding the float64
// evaluation of a plane-local dot product gu·u + gv·v commits: the anchor shift
// a revolve's extreme reading subtracts to carry a plane-local extreme into
// axis coordinates (revolve.go's axisExtremeContext). Both products round at
// the ANCHOR's own magnitude, which the boundary scan's own bound — proven over
// the SECTION's coordinates — says nothing about.
//
// held must be the SAME float the caller's own arithmetic produced, so the
// answer covers however that arithmetic grouped its two products and their sum.
// It speaks for the ROUNDING alone: the u/v operands' own proven uncertainty is
// a different mechanism the caller composes beside it. A component no rational
// holds answers +Inf, never 0 (CutDisplacementAllow's own rule).
func ExactPlaneDotRound(gu, gv, u, v, held float64) float64 {
	guR, gvR, uR, vR := proofarith.FloatRat(gu), proofarith.FloatRat(gv), proofarith.FloatRat(u), proofarith.FloatRat(v)
	if guR == nil || gvR == nil || uR == nil || vR == nil {
		return math.Inf(1)
	}
	return proofarith.RationalFloatError(RatAdd(RatMul(guR, uR), RatMul(gvR, vR)), held)
}

// PlaneDotDecompositionRoundAllow bounds the rounding the boundary-extreme
// scan's OWN evaluation of gu·u + gv·v commits over every candidate it folds
// (extrude.go's boundaryExtremesBoundedContext), given gu and gv themselves
// already proven against the frame, axis and placement chain that produced them
// — the caller's own coefficient terms. It is ExactPlaneDotRound scaled to a
// whole scan: that helper measures ONE stated (u, v) exactly, and the scan
// states none, because the candidate that wins the extremization is not a value
// the scan reports.
//
// The rounding it charges is the SECTION's, and no other term in a revolve's
// extent reading scales with that magnitude: the scan's own published bound
// speaks for each candidate's POSITIONAL displacement (PointPerturbationAllow,
// a circular candidate's enclosure, a free-form span's), which is zero for a
// coordinate the record states verbatim, while the anchor shift's dot rounds at
// the ANCHOR's magnitude alone. A section a million millimetres long under a
// placement-derived direction therefore rounds here and nowhere else.
//
// IEEE 754 multiplies exactly by 0, 1 and -1 for any operand, so a coefficient
// pair drawn from those three values WITH one of the two zero evaluates one
// exact product and adds it to zero: nothing rounds, whatever the coordinates,
// and the term is zero — which is what keeps an unplaced revolve about an
// axis-aligned frame reading its box exactly, however far its section reaches.
// Every other pair can round in each product and again in their sum, and the
// term is AnalyticRoundBound's own budget (two multiplies and one addition, far
// under its 128-operation contract) at the envelope of the very coordinates
// those products multiply.
//
// coordUpper must be a PROVEN upper bound on |u| and |v| over every candidate
// the scan folds — profileCoordinateEnvelope, which reads each walk's own
// coordUpper — measured about the SAME frame origin the scan's candidates are
// written about. extrude.go's prismDecompositionRoundAllow is the prism's own
// spelling of this mechanism, one sweep coordinate wider, and the two are never
// composed: each reading charges its own decomposition exactly once.
func PlaneDotDecompositionRoundAllow(gu, gv, coordUpper float64) float64 {
	trivial := func(c float64) bool { return c == 0 || c == 1 || c == -1 }
	if (gu == 0 || gv == 0) && trivial(gu) && trivial(gv) {
		return 0
	}
	return AnalyticRoundBound(AbsSumUpper(
		ProductUpper(math.Abs(gu), coordUpper),
		ProductUpper(math.Abs(gv), coordUpper),
	))
}

// ExactSumRound proves, in exact dyadic arithmetic, the rounding the FINAL float64
// summation of already-held terms commits: the recombination every directional
// extent reading publishes each of its two endpoints through — a prism's
// base + boundary extreme + sweep level, a revolve's or a cap-loop chamfer's
// base + boundary extreme (extrude.go, revolve.go and capblend.go's
// extentBoundedAlong).
//
// It is ExactIsometryDotRound's companion one step later. That helper proves
// each COEFFICIENT the reading lifts an extreme through exactly right; this one
// charges what ADDING the resulting terms commits, and the two are independent:
// IEEE 754 multiplies exactly by 0, 1 and -1, so an axis-permuting frame under
// an axis-permuting placement leaves every coefficient exactly right and still
// rounds here, because a translation component added to a section coordinate is
// an ordinary float64 addition (10 + 0 is exact, 10 + 0.1 is not). A reading
// that charges only the coefficients therefore publishes a translated body's
// box as Exact with a zero bound while its own endpoint misses the truth.
//
// held must be the SAME float the caller's own arithmetic produced and terms
// the SAME summands it produced it from, in any order: the answer is
// |Σterms − held| measured exactly, so it covers every rounding that summation
// committed however the caller grouped it. It speaks for the summation ALONE —
// each term's own displacement is a different mechanism with its own helper
// above, and the caller composes the two outward, per endpoint.
//
// The answer is zero exactly where the summation is exactly representable, so a
// proven-exact reading — an unplaced body, or a placement whose translation
// lands on a float64 sum — keeps its zero bound and stays Exact. A term no
// rational holds answers +Inf, never 0 (CutDisplacementAllow's own rule).
func ExactSumRound(held float64, terms ...float64) float64 {
	sum := proofarith.DyZero()
	for _, term := range terms {
		d, ok := proofarith.DyOf(term)
		if !ok {
			return math.Inf(1)
		}
		sum = proofarith.DyAdd(sum, d)
	}
	return proofarith.DyRoundedFloatError(sum, held)
}

// SnapToZeroAllow composes the bound a coordinate carries once a deliberate
// snap has overwritten it with exactly 0. bound is what the caller had already
// proven about the pre-snap coordinate and discarded is that coordinate's own
// magnitude, so bound + discarded encloses the same truth about the assigned
// zero: the truth sits within bound of the pre-snap value, which sits exactly
// discarded from 0.
//
// The composition is the whole point of routing through here rather than
// dropping the term. A snap is a decision the caller makes for its own reasons
// — revolve.go's axisFrame.walk snaps so contact classification and vertex
// placement agree exactly — and the assignment is no less an error for being
// deliberate. Charging it leaves the snap's own behaviour untouched while every
// reading that folds the snapped coordinate into a published measurement
// (survey.go's revolveMinRadius) stops claiming an exactness the assignment
// took away.
//
// It answers the caller's own bound unchanged for a discarded magnitude of
// zero, so a coordinate the arithmetic already put exactly on zero keeps
// whatever exactness it arrived with. A discarded magnitude no float can state
// answers +Inf rather than that unchanged bound: a NaN would be the ABSENCE of
// a charge and would silently vanish from every reading it was meant to widen
// (RigidRoundAllow's own rule).
func SnapToZeroAllow(bound, discarded float64) float64 {
	if IsNonFinite(discarded) {
		return math.Inf(1)
	}
	if discarded <= 0 {
		return bound
	}
	return UpRound(bound + discarded)
}
