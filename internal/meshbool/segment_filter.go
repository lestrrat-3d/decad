package meshbool

import (
	"math"

	"github.com/lestrrat-3d/decad/internal/proofbound"

	"github.com/lestrrat-3d/r3"
)

// The mesh boolean uses exact-arithmetic predicates
// (docs/evaluator-design.md §9): sign tests decided by an adaptive float
// filter that falls back to exact rational arithmetic exactly at the
// boundary cases, plus the rational point, segment and parity predicates the
// subdivision and classification passes run on. The exact fallback is
// carried as homogeneous integer coordinates — an integer numerator triple
// over one shared positive denominator, as proof.Xhp and proof.Xpt — never as
// math/big.Rat: every predicate is a homogeneous form of fixed degree in the
// differences, so its sign is invariant under scaling by a positive
// denominator, and the exactness guarantee is unchanged. A point is reduced
// to its canonical form only at vertex emission (proof.Xpt.Key), because welding is
// by exact identity (boolean_mesh.go's StitchFacetsContext) and a homogeneous
// point has many spellings. A sign decided exactly is a topology decision
// that cannot flip (core §2.1), which is what makes the stitched output
// watertight by construction on the tessellated geometry.
const (
	// SegFilterErrCoef covers SegFilter.tooFar's own float evaluation: 64·u,
	// against the magnitude the branch taken carries. tooFar derives it.
	SegFilterErrCoef = 64 * proofbound.UnitRoundoff
	// SegFilterFloor is one absolute term covering every gradual-underflow
	// crumb the same evaluation can commit. Each is at most 2⁻¹⁰⁷⁵ and there
	// are a few dozen of them, so 2⁻¹⁰⁰⁰ dominates them all together. It is an
	// absolute term at the FINAL threshold, so it speaks only for a crumb
	// nothing amplifies afterwards; SegFilterMinNormal is what keeps the
	// interior branch inside that promise.
	SegFilterFloor = 0x1p-1000
	// SegFilterMinNormal is the smallest positive NORMAL float64. tooFar's
	// forward-error bound is a RELATIVE one and holds only over normalized
	// arithmetic; a subnormal intermediate carries absolute error instead, and
	// a later division by a small quantity amplifies that crumb back up to the
	// scale of the answer, which SegFilterFloor does not cover.
	SegFilterMinNormal = 0x1p-1022
	// SegFilterMinDot is the smallest c₁ whose SQUARE is still normal. The
	// interior branch works at the (D·L)² scale, which reaches the subnormal
	// range at the SQUARE ROOT of the coordinate scale that would underflow ww
	// or vv — so the positivity guards on those two say nothing about it.
	SegFilterMinDot = 0x1p-511
)

// SegAdmissionRadius2 is τ², the squared distance from the FLOAT segment beyond
// which a candidate provably cannot be an interior point of the EXACT one. It
// is read once per conforming pass: maxAbs is the largest |coordinate| over
// every vertex of the stitched mesh, and slack is that pass's own grid slack.
//
// DERIVATION. Write a, b, p for the exact rational endpoints and candidate and
// A, B, P for their float64 roundings (proof.Xpt.Vec, round to nearest). Suppose the
// exact predicate WOULD accept p — that is, p = (1−t)·a + t·b for some
// t ∈ (0, 1). Put Q = (1−t)·A + t·B, which is a point OF the float segment
// [A, B]. Then
//
//	dist(P, [A,B]) ≤ |P − Q| ≤ |P − p| + |p − Q|
//	              = |P − p| + |(1−t)·(a − A) + t·(b − B)|
//	              ≤ |P − p| + max(|a − A|, |b − B|)                        (1)
//
// the last step because a convex combination is bounded by its largest term.
// So a candidate whose float point sits FARTHER than the right-hand side of (1)
// from the float segment cannot be an interior point of the exact segment, and
// (1) is the whole of what the threshold has to cover.
//
// Each of those three terms is one coordinate-wise rounding. Round-to-nearest
// gives |fl(x) − x| ≤ ½·ulp(fl(x)), which is at most 2⁻⁵³·|fl(x)| where fl(x)
// is normal and at most 2⁻¹⁰⁷⁵ where it is subnormal or zero — so
// |fl(x) − x| ≤ 2⁻⁵³·|fl(x)| + 2⁻¹⁰⁷⁵ in every case. Reading the three
// coordinates as a vector and applying the triangle inequality,
//
//	|P − p| ≤ 2⁻⁵³·‖P‖ + √3·2⁻¹⁰⁷⁵ ≤ √3·2⁻⁵³·maxAbs + √3·2⁻¹⁰⁷⁵
//
// since ‖P‖ ≤ √3·maxAbs, and the same bound holds for |a − A| and |b − B|
// because maxAbs bounds all three points at once. Substituting into (1),
//
//	dist(P, [A,B]) ≤ 2·√3·2⁻⁵³·maxAbs + 2·√3·2⁻¹⁰⁷⁵ < maxAbs·2⁻⁵⁰ + 2⁻¹⁰⁰⁰
//
// because 2·√3·2⁻⁵³ ≈ 3.85e-16 sits 2.3× below 2⁻⁵⁰ ≈ 8.88e-16, and
// 2·√3·2⁻¹⁰⁷⁵ sits far below 2⁻¹⁰⁰⁰. That 2.3× margin also absorbs the
// rounding of evaluating the bound itself, so no outward rounding is owed on
// the sum.
//
// The pass's own slack is then ADDED to it. The proven requirement is the
// rounding term alone; carrying slack keeps this filter no tighter than the
// grid box that precedes it (ConformOnce), which is that pass's own statement
// of how far a float approximation may sit from its exact point.
//
// τ² underflows to zero only for τ below 2⁻⁵³⁷, and at that scale
// SegFilterFloor already demands a distance above 2⁻⁵⁰⁰ before tooFar will
// reject anything — eleven orders above any τ that small — so a threshold that
// rounds to zero there costs no soundness.
//
// The OTHER end of the range, and the two arguments themselves, are answered by
// returning +Inf, which is the abstain-everywhere threshold: tooFar's left-hand
// side is always finite, so a comparison against +Inf is false for every
// candidate. Two cases earn it, and between them they are every value the
// signature admits that the derivation above does not speak for.
//
//   - A NON-FINITE OR NEGATIVE argument. Both are outside the derivation's
//     terms — maxAbs is a coordinate magnitude and slack a grid width, and
//     neither can be NaN, infinite or below zero and still name what the
//     derivation reads it as. A negative slack is the one that would bite: it
//     can cancel the rounding term instead of widening it, leaving a τ² BELOW
//     the proven requirement, which is a threshold that rejects true hits.
//     NewConformScan cannot produce one (its slack is built from a vector
//     length and a positive cell width), so this is a precondition made
//     total rather than a live defect.
//   - AN OVERFLOWED τ². τ passes 2⁻⁵³⁷'s mirror at about 1.34e154, above which
//     τ·τ saturates. Nothing is proven about a saturated threshold, and +Inf is
//     the reading that costs only the rejections the filter would have made on
//     a mesh whose coordinates run past 1e204.
func SegAdmissionRadius2(slack, maxAbs float64) float64 {
	if !(slack >= 0 && slack <= math.MaxFloat64) || !(maxAbs >= 0 && maxAbs <= math.MaxFloat64) {
		return math.Inf(1)
	}
	tau := slack + (maxAbs*0x1p-50 + SegFilterFloor)
	if tau2 := proofbound.UpRound(tau * tau); !proofbound.IsNonFinite(tau2) {
		return tau2
	}
	return math.Inf(1)
}

// SegFilter is the reject-only float pre-filter in front of the exact
// OnSegmentInterior3 predicate (docs/evaluator-design.md §9). The conforming
// pass hands it every vertex the grid says a facet edge might touch, and a long
// diagonal edge reaches most of the mesh — so without a filter nearly every
// vertex earns a full math/big.Rat cross product to establish what a dozen
// float operations already establish: it is nowhere near the segment.
//
// The filter REJECTS a candidate only when the rejection is proven. It never
// admits one: a candidate it does not reject still goes to the exact predicate,
// which decides it. That direction is the whole safety argument. A filter that
// could ADMIT would be an admission gate on a residual, which this package
// forbids outright; a filter that could reject a true hit would silently drop
// an on-edge vertex and hand back a subdivision that is not conforming, which
// is a WRONG boolean rather than a slow one.
type SegFilter struct {
	// a and b are the float roundings of the segment's exact endpoints, v is
	// the float b − a, and vv is the float v·v.
	A, B, V r3.Vec
	Vv      float64
	// tau2 is τ² from SegAdmissionRadius2.
	Tau2 float64
}

func NewSegFilter(a, b r3.Vec, tau2 float64) SegFilter {
	v := b.Sub(a)
	return SegFilter{A: a, B: b, V: v, Vv: v.Dot(v), Tau2: tau2}
}

// tooFar reports whether p — the float rounding of a candidate's exact
// coordinates — is PROVABLY farther than τ from the float segment [a, b]. A
// false answer means "not proven", never "close enough".
//
// The value is the textbook clamped projection and SegFilterErrCoef covers its
// own float evaluation. With u = 2⁻⁵³, W = |P − A|² (what ww holds) and
// V = |v|², the forward error of each branch is:
//
//   - clamped to A, or to B: the value is a sum of three squares of correctly
//     rounded differences, so it carries RELATIVE error only — three roundings
//     per term and two for the sum give |computed − true| ≤ 5·u·W.
//   - interior: the value is W − c₁²/c₂ with c₁ = w·v and c₂ = V. Each dot
//     product is off by at most 5·u·Σ|term|, and Cauchy–Schwarz bounds
//     Σ|wᵢ·vᵢ| by √(W·V). Propagating that through the square, the division and
//     the final subtraction — and using c₁² ≤ W·V, which cancels the V's —
//     gives |computed − true| ≤ 24·u·W. The cancellation in that subtraction is
//     real, but it is ABSOLUTE error measured against W, and the cases where it
//     swamps the result are exactly the near-the-segment cases the filter must
//     not reject anyway.
//
// The branch is chosen on the COMPUTED c₁, so it can disagree with the true
// projection near c₁ = 0 and c₁ = V. That costs nothing: at either boundary the
// clamped and interior values differ by c₁²/V ≤ (5·u·√(W·V))²/V = 25·u²·W,
// which the budget swallows whole. So SegFilterErrCoef = 64·u is 2.6× the worst
// branch bound, and mag carries the branch's own magnitude.
//
// THAT WHOLE BOUND ASSUMES NORMALIZED ARITHMETIC, which this evaluation leaves
// at both ends of the range. UNDERFLOW reaches the interior branch, immediately
// below; OVERFLOW reaches vv, ww, c₁ and |P−B|², and the enumeration at the end
// of this comment is where it is answered.
//
// W and V are sums of squares of coordinates, but c₁² sits at (D·L)² — the
// SQUARE of the scale W and V sit at — so it reaches the
// subnormal range at the square root of the coordinate scale that would
// underflow either of them. A subnormal c₁² has lost an absolute crumb rather
// than a relative one, and the division by V that follows multiplies that
// crumb by 1/V, which is large exactly when V is small: at the bottom of the
// range the lost crumb comes back as the whole of W, so a candidate exactly ON
// the segment computes d² = W and is REJECTED. SegFilterFloor cannot cover it,
// being an absolute term at the final threshold rather than at the intermediate
// the division amplifies.
//
// Two things answer it, and the branch takes both. It forms the subtrahend as
// (c₁/V)·c₁, so the division comes FIRST and every intermediate sits at the
// scale of the result rather than at (D·L)²; and it ABSTAINS — returns false,
// sending the candidate to the exact predicate that decides it anyway —
// whenever any of those intermediates, or c₁² itself, is not a normal float.
// Abstaining needs no error analysis of its own: it is exactly what the filter
// does for every candidate it cannot prove far away. The rescale is an
// improvement on the arrangement; the abstain is the guarantee.
//
// SATURATION IS TESTED, NOT ARGUED. The bound above is relative and holds only
// over finite normalized arithmetic, so it says nothing once an intermediate
// reaches ±Inf or NaN — and a saturated intermediate does not merely widen the
// answer, it redirects the BRANCH. Inf < Inf is false, so a saturated vv sends
// a projection sitting deep in the interior down the clamp-to-B arm, which then
// computes a perfectly finite |P−B|² and rejects on it. Nothing about that is
// self-announcing: no NaN appears, no guard below trips, and the rejection is
// simply wrong. −Inf and NaN do the same through c₁ > 0 and the clamp-to-A arm.
//
// So every intermediate this evaluation forms is tested for finiteness before
// anything reads it, and the filter abstains on any that is not. The list below
// is complete because it is enumerated from the evaluation's own operations
// rather than from the ways they were expected to fail — each line names one
// operation in the order the code performs it, and the last line closes the
// comparison the operations feed:
//
//   - vv, hence v, hence a and b. The gate at the top, and the one the reported
//     defect needed: a segment longer than about 1.34e154 saturates vv, and a
//     coordinate difference past MaxFloat64 saturates a component of v first.
//     A non-finite a or b arrives here only through this gate, since v is
//     finite in a coordinate only when both endpoints are (Inf − finite is Inf,
//     Inf − Inf is NaN).
//   - ww, hence w, hence p. A saturated |P−A|² is not a distance, and a
//     non-finite candidate coordinate reaches the filter only through it. It
//     cannot arise for a candidate the exact predicate would accept — a point
//     ON the segment has |P−A| ≤ |B−A|, so its ww is within a rounding of a vv
//     the gate above already found finite — which makes this a guard on the
//     branch logic rather than a second soundness route. It costs only
//     rejections of candidates far past a mesh-spanning segment's endpoint.
//   - c₁ = w·v. Behind the two gates above, Cauchy–Schwarz caps |c₁| and each
//     of the dot product's partial sums at √(ww·vv) ≤ MaxFloat64, so a
//     saturation is reachable at most through the last ulp of a value already
//     at MaxFloat64 — no witness for it is in the tests. The guard stands
//     anyway: abstaining costs a candidate one rational predicate, and the two
//     soundness breaks this function has had were both a saturation somebody
//     had argued away.
//   - t, proj, and the c₁² the unscaled arrangement would form. The normality
//     guards inside the interior branch, which UNDERFLOW rather than overflow
//     reaches — t and proj are each bounded by c₁ < vv. They are the round-one
//     guards and the paragraphs above are their derivation.
//   - |P−B|². The guard inside the clamp-to-B branch. It is in the same
//     position as c₁'s: |P−B|² = ww − 2·c₁ + vv, and the branch is only taken
//     for c₁ ≥ vv, so the identity caps it at ww − vv ≤ MaxFloat64 and only the
//     last ulp can carry it over. Guarding it is what keeps the branch from
//     resting on the accident that d2 and mag saturate together and the NaN
//     they make happens to compare false.
//   - the final difference and the comparison. d2 and mag are finite by every
//     line above, and SegFilterErrCoef·mag cannot overflow a finite mag, so the
//     left-hand side is ALWAYS finite. A non-finite τ² therefore never meets a
//     non-finite left-hand side: SegAdmissionRadius2 returns +Inf for every
//     threshold it cannot state, and a finite value is never above +Inf.
func (f SegFilter) TooFar(p r3.Vec) bool {
	if !(f.Vv > 0) || proofbound.IsNonFinite(f.Vv) {
		// The float segment has nothing to project onto — both endpoints
		// rounded to one float, or v underflowed — or it is long enough that a
		// component of v, or vv itself, saturated. Abstaining is always sound.
		return false
	}
	w := p.Sub(f.A)
	ww := w.Dot(w)
	c1 := w.Dot(f.V)
	if proofbound.IsNonFinite(ww) || proofbound.IsNonFinite(c1) {
		// Both feed the branch decision below, so a saturated one does not
		// widen the answer, it picks the wrong formula for it.
		return false
	}
	d2, mag := ww, ww
	if c1 > 0 {
		if c1 < f.Vv {
			// c₁²/V, divided before it is squared. The guard is written so a
			// NaN fails it: every intermediate this branch could form — the
			// two below and the c₁² the unscaled arrangement would form — has
			// to be normal, or the branch has no proven bound and abstains.
			t := c1 / f.Vv
			proj := t * c1
			if !(c1 >= SegFilterMinDot && t >= SegFilterMinNormal && proj >= SegFilterMinNormal) {
				return false
			}
			d2 = ww - proj
		} else {
			e := p.Sub(f.B)
			d2 = e.Dot(e)
			if proofbound.IsNonFinite(d2) {
				return false
			}
			mag = d2
		}
	}
	return d2-(SegFilterErrCoef*mag+SegFilterFloor) > f.Tau2
}
