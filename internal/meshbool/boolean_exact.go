package meshbool

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"

	"github.com/lestrrat-3d/decad/internal/freeform"

	"github.com/lestrrat-3d/decad/internal/proofbound"

	"github.com/lestrrat-3d/r3"
)

// This file is the exact-arithmetic kernel behind the mesh boolean
// (docs/evaluator-design.md §9): sign tests decided by an adaptive float
// filter that falls back to exact rational arithmetic exactly at the
// boundary cases, plus the rational point, segment and parity predicates the
// subdivision and classification passes run on. The exact fallback is
// carried as homogeneous integer coordinates — an integer numerator triple
// over one shared positive denominator, xhp/xpt below — never as
// math/big.Rat: every predicate is a homogeneous form of fixed degree in the
// differences, so its sign is invariant under scaling by a positive
// denominator, and the exactness guarantee is unchanged. A point is reduced
// to its canonical form only at vertex emission (proofbound.Xpt.key), because welding is
// by exact identity (boolean_mesh.go's StitchFacetsContext) and a homogeneous
// point has many spellings. A sign decided exactly is a topology decision
// that cannot flip (core §2.1), which is what makes the stitched output
// watertight by construction on the tessellated geometry.

// Xcross is a × b, exact, stripped the same way as xsub.
func Xcross(a, b proofbound.Xpt) proofbound.Xpt {
	return proofbound.Xpt(proofbound.XhpStripTwosOwned(XhpCross(proofbound.Xhp(a), proofbound.Xhp(b))))
}

// XdotSign is the sign of a·b, decided as a plain integer sign: the shared
// denominator a.w·b.w is always positive, so the numerator's sign IS the
// dot product's sign.
func XdotSign(a, b proofbound.Xpt) int { return proofbound.XdotNum(a, b).Sign() }

// Xlerp is a + t·(b − a) for t = tn/td, exact, with the common power of two
// stripped on return — the growth control that keeps a chain of lerps from
// growing its denominator multiplicatively at every link (measured: 14113
// bits unreduced at lerp depth 6, 462 bits stripped after every step).
func Xlerp(a, b proofbound.Xpt, tn, td *big.Int) proofbound.Xpt {
	return proofbound.Xpt(proofbound.XhpStripTwosOwned(XhpLerp(proofbound.Xhp(a), proofbound.Xhp(b), tn, td)))
}

// OrientNum is the exact value of det[b−a, c−a, d−a] as an integer numerator
// over a positive denominator, formed without ever materialising a big.Rat:
// positive when d lies on the side the counter-clockwise normal of (a, b, c)
// points to.
func OrientNum(a, b, c, d proofbound.Xpt) (num, den *big.Int) {
	ha, hb, hc, hd := proofbound.Xhp(a), proofbound.Xhp(b), proofbound.Xhp(c), proofbound.Xhp(d)
	ba, ca, da := proofbound.XhpSub(hb, ha), proofbound.XhpSub(hc, ha), proofbound.XhpSub(hd, ha)
	cr := XhpCross(ba, ca)
	return proofbound.XhpDotNum(cr, da), new(big.Int).Mul(cr.W, da.W)
}

// OrientSignExact is the exact sign of det[b−a, c−a, d−a], decided as a plain
// integer sign with no big.Rat and no normalisation anywhere in the chain —
// XhpOrientSign's own guarantee, carried through xpt.
func OrientSignExact(a, b, c, d proofbound.Xpt) int {
	return XhpOrientSign(proofbound.Xhp(a), proofbound.Xhp(b), proofbound.Xhp(c), proofbound.Xhp(d))
}

// OrientRat materialises det[b−a, c−a, d−a] as a big.Rat — the one place this
// value pays a normalisation, for the rare caller that needs the value rather
// than the sign.
func OrientRat(a, b, c, d proofbound.Xpt) *big.Rat {
	num, den := OrientNum(a, b, c, d)
	return new(big.Rat).SetFrac(num, den)
}

// OrientSign is the adaptive-precision sign of det[b−a, c−a, d−a] for float
// inputs: a float evaluation whose forward error provably cannot cross zero
// decides the generic case; anything inside the error bound falls back to the
// exact value — the §9 discipline, so a sign is never wrong.
func OrientSign(a, b, c, d r3.Vec) int {
	if sign, certain := OrientSignFloat(a, b, c, d); certain {
		return sign
	}
	return OrientSignExact(proofbound.XptOf(a), proofbound.XptOf(b), proofbound.XptOf(c), proofbound.XptOf(d))
}

// OrientSignPrepared uses an already lifted triangle and its exact normal on
// the uncertain path. xa and xd are the exact lifts of a and d, and n is the
// exact oriented cross product of (b-a) and (c-a), with positive denominator.
func OrientSignPrepared(a, b, c, d r3.Vec, xa, xd, n proofbound.Xpt) int {
	if sign, certain := OrientSignFloat(a, b, c, d); certain {
		return sign
	}
	return XdotSign(n, proofbound.Xsub(xd, xa))
}

// OrientSignFloat gives the same adaptive float decision to both plane-side
// callers. An uncertain sign must be decided by exact integer arithmetic.
func OrientSignFloat(a, b, c, d r3.Vec) (int, bool) {
	bax, bay, baz := b.X-a.X, b.Y-a.Y, b.Z-a.Z
	cax, cay, caz := c.X-a.X, c.Y-a.Y, c.Z-a.Z
	dax, day, daz := d.X-a.X, d.Y-a.Y, d.Z-a.Z
	det := bax*(cay*daz-caz*day) + bay*(caz*dax-cax*daz) + baz*(cax*day-cay*dax)
	perm := math.Abs(bax)*(math.Abs(cay)*math.Abs(daz)+math.Abs(caz)*math.Abs(day)) +
		math.Abs(bay)*(math.Abs(caz)*math.Abs(dax)+math.Abs(cax)*math.Abs(daz)) +
		math.Abs(baz)*(math.Abs(cax)*math.Abs(day)+math.Abs(cay)*math.Abs(dax))
	// The true forward error is bounded by a few ulps of the permanent; 1e-12
	// leaves three decades of margin, so a sign the filter accepts is proven.
	if err := 1e-12 * perm; det > err || det < -err {
		if det > 0 {
			return 1, true
		}
		return -1, true
	}
	return 0, false
}

// OrientSignMixed is the exact plane-side sign of a homogeneous probe against
// a float triangle: positive on the triangle's counter-clockwise-normal side.
func OrientSignMixed(a, b, c r3.Vec, d proofbound.Xpt) int {
	return OrientSignExact(proofbound.XptOf(a), proofbound.XptOf(b), proofbound.XptOf(c), d)
}

// XhpCross is a × b, exact: its numerators over the positive denominator
// a.w·b.w.
func XhpCross(a, b proofbound.Xhp) proofbound.Xhp {
	var term big.Int
	axis := func(a0, b0, a1, b1 *big.Int) *big.Int {
		out := new(big.Int).Mul(a0, b0)
		return out.Sub(out, term.Mul(a1, b1))
	}
	return proofbound.Xhp{
		X: axis(a.Y, b.Z, a.Z, b.Y),
		Y: axis(a.Z, b.X, a.X, b.Z),
		Z: axis(a.X, b.Y, a.Y, b.X),
		W: new(big.Int).Mul(a.W, b.W),
	}
}

// XhpLerp is a + t·(b − a) for t = tn/td, exact. td may arrive negative; a.w
// and b.w are already positive by invariant, so the result's own w — their
// product with td — is renormalised by flipping td's (and tn's) sign first,
// which leaves the value t = tn/td unchanged.
func XhpLerp(a, b proofbound.Xhp, tn, td *big.Int) proofbound.Xhp {
	n, d := tn, td
	if d.Sign() < 0 {
		n = new(big.Int).Neg(n)
		d = new(big.Int).Neg(d)
	}
	// a + t·(b−a) = a·(d−n)/d + b·n/d = [a·(d−n)·b.w + b·n·a.w] / (a.w·b.w·d).
	diff := new(big.Int).Sub(d, n)
	axis := func(av, bv *big.Int) *big.Int {
		term := new(big.Int).Mul(av, diff)
		term.Mul(term, b.W)
		other := new(big.Int).Mul(bv, n)
		other.Mul(other, a.W)
		return term.Add(term, other)
	}
	w := new(big.Int).Mul(a.W, b.W)
	w.Mul(w, d)
	return proofbound.Xhp{X: axis(a.X, b.X), Y: axis(a.Y, b.Y), Z: axis(a.Z, b.Z), W: w}
}

// XhpOrientSign is the exact sign of det[b−a, c−a, d−a], decided as a plain
// integer sign with no big.Rat and no normalisation anywhere in the chain:
// every intermediate xhp carries a positive denominator by construction, so
// the final numerator's sign IS the determinant's sign.
func XhpOrientSign(a, b, c, d proofbound.Xhp) int {
	ba, ca, da := proofbound.XhpSub(b, a), proofbound.XhpSub(c, a), proofbound.XhpSub(d, a)
	return proofbound.XhpDotNum(XhpCross(ba, ca), da).Sign()
}

// XhpRat materialises p's three coordinates as big.Rat — the one place a
// homogeneous point pays a normalisation, and only when a caller genuinely
// needs a rational VALUE rather than a sign.
func XhpRat(p proofbound.Xhp) (x, y, z *big.Rat) {
	return new(big.Rat).SetFrac(p.X, p.W), new(big.Rat).SetFrac(p.Y, p.W), new(big.Rat).SetFrac(p.Z, p.W)
}

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
// A, B, P for their float64 roundings (proofbound.Xpt.vec, round to nearest). Suppose the
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

// FloatInterval is a proven float64 enclosure [lo, hi] of an exact rational
// value: the reject-only pre-filter in front of TriTriClassify's non-coplanar
// arm (TriTriMissesFilter, below) is built entirely on this type, the same
// "outward-rounded float arithmetic, exact fallback for anything it cannot
// prove" shape SegFilter uses for the conforming pass's own segment test. It
// is float64's own enclosure, deliberately apart from proofbound.RatInterval
// (moments.go) and internal/capcontour's Point/Carrier, which enclose in
// big.Rat and serve a different proof (a rational bound, not a float filter).
//
// Every method below returns EITHER a proper enclosure — lo and hi both
// finite, lo ≤ hi, and the true value proven to lie in [lo, hi] — OR the
// sentinel FivAbstain ({-Inf, +Inf}), which encloses every value trivially
// and is this type's uniform "cannot prove anything here" answer. No method
// ever returns a half-finite interval, so a caller need only test one
// sentinel to know whether an answer was reached.
//
// ROUNDING. Each arithmetic method computes its result with plain float64
// operations (round-to-nearest) and then widens the low end down and the high
// end up by exactly one representable value. Round-to-nearest error is at
// most half an ulp of the computed value, so a full ulp of outward widening
// covers it with room to spare — the same margin SegAdmissionRadius2's
// derivation leans on. A binary operation with two interval operands has up
// to four products or quotients at its corners. The result selects their
// extrema before the same one-ulp widening; multiplication uses each
// operand's sign to skip corners that cannot attain an endpoint.
//
// NON-FINITE INPUTS AND OUTPUTS ABSTAIN, NEVER PROPAGATE. Every method checks
// every intermediate it forms for finiteness before trusting it, and returns
// FivAbstain the moment one fails — mirroring SegFilter.tooFar's own
// "saturation is tested, not argued" discipline. A saturated intermediate
// does not merely widen an answer, it can flip which branch a later decision
// takes (Inf compares false against everything finite), so nothing downstream
// may read a value this type has not first proven finite.
type FloatInterval struct{ Lo, Hi float64 }

// FivAbstain is the everywhere-abstaining interval.
var FivAbstain = FloatInterval{Lo: math.Inf(-1), Hi: math.Inf(1)}

// FivNextDown and FivNextUp implement Nextafter for finite inputs. Every
// caller rejects non-finite values before widening them.
func FivNextDown(x float64) float64 {
	if x == 0 {
		return -math.SmallestNonzeroFloat64
	}
	bits := math.Float64bits(x)
	if x > 0 {
		bits--
	} else {
		bits++
	}
	return math.Float64frombits(bits)
}

func FivNextUp(x float64) float64 {
	if x == 0 {
		return math.SmallestNonzeroFloat64
	}
	bits := math.Float64bits(x)
	if x > 0 {
		bits++
	} else {
		bits--
	}
	return math.Float64frombits(bits)
}

// abstains reports whether iv is the abstain sentinel.
func (a FloatInterval) Abstains() bool { return a == FivAbstain }

// contains reports whether x lies within the closed interval.
func (a FloatInterval) Contains(x float64) bool { return a.Lo <= x && x <= a.Hi }

// disjoint reports whether a and b, as closed intervals, share no point.
func (a FloatInterval) Disjoint(b FloatInterval) bool { return a.Hi < b.Lo || b.Hi < a.Lo }

// FivPoint lifts a float64 that is itself an EXACT value (never itself the
// product of a rounding) into a degenerate interval: lo = hi = x needs no
// widening, because there is no rounding error to cover. Every triangle
// vertex coordinate TriTriMissesFilter reads is such a value — a float64 IS
// an exact rational (boolean_exact.go's own opening comment) — so this is the
// leaf constructor for every vertex coordinate the filter touches.
func FivPoint(x float64) FloatInterval {
	if proofbound.IsNonFinite(x) {
		return FivAbstain
	}
	return FloatInterval{Lo: x, Hi: x}
}

// FivRounded lifts a float64 that is itself a CORRECTLY-ROUNDED conversion of
// some exact value — at most half an ulp off truth, the guarantee
// big.Rat.Float64 makes — into an interval with one full ulp of margin on
// each side, which covers that half-ulp with the same spare-half-ulp margin
// every other widening in this type carries. TriTriMissesFilter's na/nb
// arguments are proofbound.Xpt.vec() results, so they are exactly this case.
func FivRounded(x float64) FloatInterval {
	if proofbound.IsNonFinite(x) {
		return FivAbstain
	}
	return FloatInterval{Lo: FivNextDown(x), Hi: FivNextUp(x)}
}

func (a FloatInterval) Add(b FloatInterval) FloatInterval {
	lo, hi := a.Lo+b.Lo, a.Hi+b.Hi
	if proofbound.IsNonFinite(lo) || proofbound.IsNonFinite(hi) {
		return FivAbstain
	}
	return FloatInterval{Lo: FivNextDown(lo), Hi: FivNextUp(hi)}
}

func (a FloatInterval) Sub(b FloatInterval) FloatInterval {
	lo, hi := a.Lo-b.Hi, a.Hi-b.Lo
	if proofbound.IsNonFinite(lo) || proofbound.IsNonFinite(hi) {
		return FivAbstain
	}
	return FloatInterval{Lo: FivNextDown(lo), Hi: FivNextUp(hi)}
}

func (a FloatInterval) Mul(b FloatInterval) FloatInterval {
	var lo, hi float64
	// On sign-definite intervals multiplication is monotone, so only the two
	// corners that attain the range endpoints need to be evaluated.
	switch {
	case a.Lo >= 0 && b.Lo >= 0:
		lo, hi = a.Lo*b.Lo, a.Hi*b.Hi
	case a.Lo >= 0 && b.Hi <= 0:
		lo, hi = a.Hi*b.Lo, a.Lo*b.Hi
	case a.Lo >= 0:
		lo, hi = a.Hi*b.Lo, a.Hi*b.Hi
	case a.Hi <= 0 && b.Lo >= 0:
		lo, hi = a.Lo*b.Hi, a.Hi*b.Lo
	case a.Hi <= 0 && b.Hi <= 0:
		lo, hi = a.Hi*b.Hi, a.Lo*b.Lo
	case a.Hi <= 0:
		lo, hi = a.Lo*b.Hi, a.Lo*b.Lo
	case b.Lo >= 0:
		lo, hi = a.Lo*b.Hi, a.Hi*b.Hi
	case b.Hi <= 0:
		lo, hi = a.Hi*b.Lo, a.Lo*b.Lo
	default:
		lo = math.Min(a.Lo*b.Hi, a.Hi*b.Lo)
		hi = math.Max(a.Lo*b.Lo, a.Hi*b.Hi)
	}
	if proofbound.IsNonFinite(lo) || proofbound.IsNonFinite(hi) {
		return FivAbstain
	}
	return FloatInterval{Lo: FivNextDown(lo), Hi: FivNextUp(hi)}
}

// div guards the one case the other three operations do not have: a
// denominator interval containing zero makes the quotient unbounded (or
// undefined, at zero itself), so it returns FivAbstain outright rather than
// forming a division that could saturate or, worse, land on a finite value
// that means nothing.
func (a FloatInterval) Div(b FloatInterval) FloatInterval {
	if proofbound.IsNonFinite(a.Lo) || proofbound.IsNonFinite(a.Hi) || proofbound.IsNonFinite(b.Lo) || proofbound.IsNonFinite(b.Hi) {
		return FivAbstain
	}
	if b.Lo <= 0 && b.Hi >= 0 {
		return FivAbstain
	}
	q0, q1, q2, q3 := a.Lo/b.Lo, a.Lo/b.Hi, a.Hi/b.Lo, a.Hi/b.Hi
	if proofbound.IsNonFinite(q0) || proofbound.IsNonFinite(q1) || proofbound.IsNonFinite(q2) || proofbound.IsNonFinite(q3) {
		return FivAbstain
	}
	lo := math.Min(math.Min(q0, q1), math.Min(q2, q3))
	hi := math.Max(math.Max(q0, q1), math.Max(q2, q3))
	return FloatInterval{Lo: FivNextDown(lo), Hi: FivNextUp(hi)}
}

// FivVec is a 3-vector of floatIntervals: the interval mirror of xpt, over
// float64 rather than big.Rat.
type FivVec struct{ X, Y, Z FloatInterval }

func FivVecOf(v r3.Vec) FivVec { return FivVec{FivPoint(v.X), FivPoint(v.Y), FivPoint(v.Z)} }

func FivSub(a, b FivVec) FivVec { return FivVec{a.X.Sub(b.X), a.Y.Sub(b.Y), a.Z.Sub(b.Z)} }

func FivDot(a, b FivVec) FloatInterval { return a.X.Mul(b.X).Add(a.Y.Mul(b.Y)).Add(a.Z.Mul(b.Z)) }

func FivCross(a, b FivVec) FivVec {
	return FivVec{
		a.Y.Mul(b.Z).Sub(a.Z.Mul(b.Y)),
		a.Z.Mul(b.X).Sub(a.X.Mul(b.Z)),
		a.X.Mul(b.Y).Sub(a.Y.Mul(b.X)),
	}
}

// TriSpanOnLine is TriTriMissesFilter's per-triangle half: a proven [lo, hi]
// enclosure of the range triangle t occupies when its plane crossings against
// the OTHER triangle o are projected onto dir — the float mirror of
// OrderOnLine over PlaneCrossings' own two cases (a zero-sign vertex sits
// exactly on the crossing; a sign-changing edge crosses it at t). signs is
// the caller's own already-decided sign triple (OrientSign, possibly via its
// exact fallback) for t's vertices against o's plane — this filter never
// re-derives a sign, only asks which of PlaneCrossings' branches it selects,
// exactly as the exact code does.
//
// The bound is deliberately a SUPERSET of the true occupied range: it takes
// every candidate PlaneCrossings would have considered — including one a
// wide interval later drops from contention exactly the way dedupePoints or
// the ≤2-points invariant would — and folds in its own projected interval, so
// the returned span can only be wider than the truth, never narrower. A
// wider span makes the filter LESS likely to prove disjointness, never more:
// the reject-only direction is unaffected by the extra slack. ok is false —
// the uniform abstain signal — the moment any step could not be bounded, or
// no candidate was found at all (which AllOneSide's own gate, run before this
// filter is ever called, has already ruled out for a real non-coplanar pair).
func TriSpanOnLine(t, o [3]FivVec, signs [3]int, dir FivVec) (FloatInterval, bool) {
	lo, hi := math.Inf(1), math.Inf(-1)
	found := false
	planeNormal := FivCross(FivSub(o[1], o[0]), FivSub(o[2], o[0]))
	values := [3]FloatInterval{}
	valueSet := [3]bool{}
	value := func(i int) FloatInterval {
		if !valueSet[i] {
			values[i] = FivDot(planeNormal, FivSub(t[i], o[0]))
			valueSet[i] = true
		}
		return values[i]
	}
	widen := func(p FivVec) bool {
		proj := FivDot(p, dir)
		if proj.Abstains() {
			return false
		}
		lo = math.Min(lo, proj.Lo)
		hi = math.Max(hi, proj.Hi)
		found = true
		return true
	}
	for i := range 3 {
		j := (i + 1) % 3
		if signs[i] == 0 && !widen(t[i]) {
			return FloatInterval{}, false
		}
		if signs[i]*signs[j] >= 0 {
			continue
		}
		vi := value(i)
		vj := value(j)
		frac := vi.Div(vi.Sub(vj))
		if frac.Abstains() {
			return FloatInterval{}, false
		}
		p := FivVec{
			X: t[i].X.Add(frac.Mul(t[j].X.Sub(t[i].X))),
			Y: t[i].Y.Add(frac.Mul(t[j].Y.Sub(t[i].Y))),
			Z: t[i].Z.Add(frac.Mul(t[j].Z.Sub(t[i].Z))),
		}
		if !widen(p) {
			return FloatInterval{}, false
		}
	}
	if !found {
		return FloatInterval{}, false
	}
	return FloatInterval{Lo: lo, Hi: hi}, true
}

// TriTriMissesFilter is the reject-only float pre-filter in front of
// TriTriClassify's non-coplanar rational arm (docs/evaluator-design.md §9),
// dispatched ahead of PlaneCrossings for exactly the reason SegFilter sits
// ahead of OnSegmentInterior3: the circular fixture that motivated it enumerates
// 66,008 facet pairs into that arm, and only 1,860 of them (fu158) have any
// real contact, so almost every call pays a full math/big.Rat cross product to
// establish what a dozen float operations already establish — the pair's
// planes cross too far outside both triangles to meet at all.
//
// It reproduces TriTriClassify's own non-coplanar computation — PlaneCrossings
// per triangle, then the projection onto dir = na × nb that OrderOnLine
// compares — entirely in interval arithmetic (TriSpanOnLine, above), and
// returns true — PROVEN no contact — only when the two triangles' projected
// spans are proven disjoint. Every other outcome, abstention included,
// returns false: the pair still goes to the exact predicate, which decides it
// with no help from this filter. That asymmetry is the whole soundness
// argument, in SegFilter's own words: a filter that could ADMIT would be an
// admission gate on a residual, which this package forbids outright, and a
// filter that could reject a true contact would hand back a non-conforming
// subdivision — a WRONG boolean, not a slow one.
//
// ta, tb are the operands' own float corners, read as exact point intervals
// (FivPoint — a float64 vertex coordinate is exact, never itself a
// rounding). na, nb are proofbound.Xpt.vec() — the correctly-rounded float64 conversion
// of the pair's exact rational normals — read with FivRounded's extra ulp of
// margin for that rounding. sa, sb are the vertex-against-the-other-plane
// sign triples OrientSign already decided; see TriSpanOnLine for how they
// steer the reconstruction.
//
// THE SHALLOW-ANGLE CASE is where this filter is expected to abstain most
// often, and correctly so: when na and nb are nearly parallel, dir = na × nb
// is near zero, so every FivDot projection carries an interval wide relative
// to its own magnitude, and frac's denominator interval is far more likely to
// straddle zero. Both drive TriSpanOnLine to its abstain return, sending the
// pair to the exact predicate — costing one rational classification, never a
// wrong answer.
func TriTriMissesFilter(ta, tb [3]r3.Vec, na, nb r3.Vec, sa, sb [3]int) bool {
	a := [3]FivVec{FivVecOf(ta[0]), FivVecOf(ta[1]), FivVecOf(ta[2])}
	b := [3]FivVec{FivVecOf(tb[0]), FivVecOf(tb[1]), FivVecOf(tb[2])}
	dir := FivCross(
		FivVec{FivRounded(na.X), FivRounded(na.Y), FivRounded(na.Z)},
		FivVec{FivRounded(nb.X), FivRounded(nb.Y), FivRounded(nb.Z)},
	)

	spanA, ok := TriSpanOnLine(a, b, sa, dir)
	if !ok {
		return false
	}
	spanB, ok := TriSpanOnLine(b, a, sb, dir)
	if !ok {
		return false
	}
	return spanA.Disjoint(spanB)
}

// Xp2 is an exact 2D point (a plane projection of an xpt). The cached float
// coordinates only accelerate the conservative sign filter; the rational
// coordinates remain the source of truth whenever the filter cannot decide.
type Xp2 struct {
	U, V        *big.Rat
	Fu, Fv      float64
	FloatFinite bool
	Hu, Hv, Hw  *big.Int
}

func NewXP2(u, v *big.Rat) Xp2 {
	fu, _ := u.Float64()
	fv, _ := v.Float64()
	return Xp2{
		U:  u,
		V:  v,
		Fu: fu,
		Fv: fv,
		FloatFinite: !math.IsNaN(fu) && !math.IsInf(fu, 0) &&
			!math.IsNaN(fv) && !math.IsInf(fv, 0),
	}
}

// NewXP2FromXpt keeps the projected point's homogeneous numerators alongside
// its rational coordinates. The rational values remain the public exact 2D
// representation used by polygon construction, while Cross2x can use the
// homogeneous form to avoid normalising four intermediate differences.
func NewXP2FromXpt(p proofbound.Xpt, u, v int) Xp2 {
	ur, vr := RatCoordOf(p, u), RatCoordOf(p, v)
	fu, _ := ur.Float64()
	fv, _ := vr.Float64()
	return Xp2{
		U:  ur,
		V:  vr,
		Fu: fu,
		Fv: fv,
		Hu: XIntCoordOf(p, u),
		Hv: XIntCoordOf(p, v),
		Hw: p.W,
		FloatFinite: !math.IsNaN(fu) && !math.IsInf(fu, 0) &&
			!math.IsNaN(fv) && !math.IsInf(fv, 0),
	}
}

// key2 is the exact 2D identity.
func (p Xp2) Key2() string { return p.U.RatString() + "|" + p.V.RatString() }

// Cross2x is the exact value of (b − a) × (c − a): positive when a, b, c turn
// counter-clockwise.
//
// The exact result remains available for callers that need more than its sign.
func Cross2x(a, b, c Xp2) *big.Rat {
	if a.Hu != nil && b.Hu != nil && c.Hu != nil {
		baU := new(big.Int).Sub(new(big.Int).Mul(b.Hu, a.Hw), new(big.Int).Mul(a.Hu, b.Hw))
		baV := new(big.Int).Sub(new(big.Int).Mul(b.Hv, a.Hw), new(big.Int).Mul(a.Hv, b.Hw))
		caU := new(big.Int).Sub(new(big.Int).Mul(c.Hu, a.Hw), new(big.Int).Mul(a.Hu, c.Hw))
		caV := new(big.Int).Sub(new(big.Int).Mul(c.Hv, a.Hw), new(big.Int).Mul(a.Hv, c.Hw))
		num := new(big.Int).Sub(new(big.Int).Mul(baU, caV), new(big.Int).Mul(baV, caU))
		den := new(big.Int).Mul(new(big.Int).Mul(a.Hw, a.Hw), new(big.Int).Mul(b.Hw, c.Hw))
		return new(big.Rat).SetFrac(num, den)
	}
	bu := new(big.Rat).Sub(b.U, a.U)
	bv := new(big.Rat).Sub(b.V, a.V)
	cu := new(big.Rat).Sub(c.U, a.U)
	cv := new(big.Rat).Sub(c.V, a.V)
	return new(big.Rat).Sub(new(big.Rat).Mul(bu, cv), new(big.Rat).Mul(bv, cu))
}

// ClipFrac is an exact fraction kept UNNORMALISED: no common factor is ever
// divided out of num and den, so building one costs no GCD. Two of them
// compare through CmpClipFrac, which cross-multiplies instead of normalising.
// The value is exact — nothing here rounds — it simply is not in lowest terms.
//
// Every ClipFrac this package builds carries den > 0, which is what makes the
// cross-multiplied comparison keep its direction.
//
// Neither field is ever used as an arithmetic destination: den commonly
// ALIASES an Xp2's own hw, and a projection cache shares one Xp2 across every
// query of its mesh, so mutating either field would corrupt the cache.
type ClipFrac struct{ Num, Den *big.Int }

// CmpClipFrac compares x against y, both with positive denominators: x is
// below y exactly when x.num·y.den is below y.num·x.den.
func CmpClipFrac(x, y ClipFrac) int {
	left := new(big.Int).Mul(x.Num, y.Den)
	right := new(big.Int).Mul(y.Num, x.Den)
	return left.Cmp(right)
}

// EdgeCross2Fracs is Cross2x(e0, e1, a) and Cross2x(e0, e1, b) as unnormalised
// fractions, both scaled by ONE factor they share.
//
// Its caller (SegTriOverlap2) reads only the ratio −fa/(fb − fa), and that
// ratio does not move when fa and fb are both multiplied by the same nonzero
// constant. So the homogeneous form's e0.hw²·e1.hw — the part of Cross2x's
// denominator that does not depend on the third point — is dropped rather than
// carried, which keeps both integers small. Only the third point's own weight
// survives, and it is the returned denominator.
//
// The homogeneous route needs all four points to carry homogeneous
// coordinates, because the shared factor only cancels when both fractions are
// really scaled by it. When any point lacks them, both fractions come from
// Cross2x's rational value, whose numerator and denominator are already the
// pair (and whose denominator big.Rat keeps positive).
//
// The returned integers are read-only; see ClipFrac.
func EdgeCross2Fracs(e0, e1, a, b Xp2) (ClipFrac, ClipFrac) {
	if e0.Hu == nil || e1.Hu == nil || a.Hu == nil || b.Hu == nil {
		fa, fb := Cross2x(e0, e1, a), Cross2x(e0, e1, b)
		return ClipFrac{Num: fa.Num(), Den: fa.Denom()}, ClipFrac{Num: fb.Num(), Den: fb.Denom()}
	}
	// The edge's own homogeneous difference, computed once for both points.
	baU := new(big.Int).Sub(new(big.Int).Mul(e1.Hu, e0.Hw), new(big.Int).Mul(e0.Hu, e1.Hw))
	baV := new(big.Int).Sub(new(big.Int).Mul(e1.Hv, e0.Hw), new(big.Int).Mul(e0.Hv, e1.Hw))
	return HomCross2Frac(e0, baU, baV, a), HomCross2Frac(e0, baU, baV, b)
}

// HomCross2Frac finishes EdgeCross2Fracs for one point: the cross product's
// homogeneous numerator over that point's own weight, the edge's shared factor
// already dropped.
func HomCross2Frac(e0 Xp2, baU, baV *big.Int, p Xp2) ClipFrac {
	caU := new(big.Int).Sub(new(big.Int).Mul(p.Hu, e0.Hw), new(big.Int).Mul(e0.Hu, p.Hw))
	caV := new(big.Int).Sub(new(big.Int).Mul(p.Hv, e0.Hw), new(big.Int).Mul(e0.Hv, p.Hw))
	num := new(big.Int).Sub(new(big.Int).Mul(baU, caV), new(big.Int).Mul(baV, caU))
	return ClipFrac{Num: num, Den: p.Hw}
}

// Cross2xSign uses a conservative float filter for the common case and keeps
// the exact rational predicate for values close enough to zero that rounding
// could change the answer. The scale uses the original coordinates, not only
// their float differences, so cancellation during subtraction is covered too.
func Cross2xSign(a, b, c Xp2) int {
	au, av, bu, bv, cu, cv := a.Fu, a.Fv, b.Fu, b.Fv, c.Fu, c.Fv
	if a.FloatFinite && b.FloatFinite && c.FloatFinite {
		det := (bu-au)*(cv-av) - (bv-av)*(cu-au)
		scale := (math.Abs(bu)+math.Abs(au))*(math.Abs(cv)+math.Abs(av)) +
			(math.Abs(bv)+math.Abs(av))*(math.Abs(cu)+math.Abs(au))
		err := 1e-12 * scale
		if !math.IsNaN(det) && !math.IsInf(det, 0) && !math.IsInf(err, 0) && err > 0 {
			if det > err {
				return 1
			}
			if det < -err {
				return -1
			}
		}
	}
	return Cross2x(a, b, c).Sign()
}

// PointInTriX reports whether p lies inside or on the closed triangle a, b,
// c, whichever way it is wound — the exact analog of pointInTri.
func PointInTriX(p, a, b, c Xp2) bool {
	d1 := Cross2xSign(a, b, p)
	d2 := Cross2xSign(b, c, p)
	d3 := Cross2xSign(c, a, p)
	hasNeg := d1 < 0 || d2 < 0 || d3 < 0
	hasPos := d1 > 0 || d2 > 0 || d3 > 0
	return !hasNeg || !hasPos
}

// OnSegment2 reports whether p lies on the closed segment (a, b) — collinear
// and within the endpoints. interior additionally excludes the endpoints.
func OnSegment2(a, b, p Xp2) (bool, bool) {
	if Cross2xSign(a, b, p) != 0 {
		return false, false
	}
	// Collinear: order along the dominant axis of the segment.
	du := new(big.Rat).Sub(b.U, a.U)
	dv := new(big.Rat).Sub(b.V, a.V)
	var lo, hi, x *big.Rat
	if du.Sign() != 0 {
		lo, hi, x = a.U, b.U, p.U
	} else if dv.Sign() != 0 {
		lo, hi, x = a.V, b.V, p.V
	} else {
		// A zero-length segment holds only its own point.
		eq := p.U.Cmp(a.U) == 0 && p.V.Cmp(a.V) == 0
		return eq, false
	}
	if lo.Cmp(hi) > 0 {
		lo, hi = hi, lo
	}
	if x.Cmp(lo) < 0 || x.Cmp(hi) > 0 {
		return false, false
	}
	interior := x.Cmp(lo) > 0 && x.Cmp(hi) < 0
	return true, interior
}

// PointInPoly2 reports whether p lies strictly inside the simple polygon —
// exact parity along a +u ray with the half-open rule, so a crossing at a
// shared vertex counts exactly once. A p on the boundary reports onBoundary.
func PointInPoly2(budget *proofbound.WorkBudget, poly []Xp2, p Xp2) (bool, bool, error) {
	n := len(poly)
	for i := range n {
		if err := budget.Step(); err != nil {
			return false, false, err
		}
		on, _ := OnSegment2(poly[i], poly[(i+1)%n], p)
		if on {
			return false, true, nil
		}
	}
	inside := false
	for i := range n {
		if err := budget.Step(); err != nil {
			return false, false, err
		}
		a, b := poly[i], poly[(i+1)%n]
		if (a.V.Cmp(p.V) <= 0) == (b.V.Cmp(p.V) <= 0) {
			continue
		}
		// u of the crossing at height p.v: a.u + (p.v−a.v)·(b.u−a.u)/(b.v−a.v).
		t := new(big.Rat).Quo(new(big.Rat).Sub(p.V, a.V), new(big.Rat).Sub(b.V, a.V))
		u := new(big.Rat).Add(a.U, new(big.Rat).Mul(t, new(big.Rat).Sub(b.U, a.U)))
		if u.Cmp(p.U) > 0 {
			inside = !inside
		}
	}
	return inside, false, nil
}

// PolyArea2Sign is the exact sign of twice the polygon's signed area.
// Reduce after every edge so the integer accumulator cannot grow with the
// number of edges beyond the exact area's denominator.
func PolyArea2Sign(budget *proofbound.WorkBudget, poly []Xp2) (int, error) {
	var num, den big.Int
	den.SetInt64(1)
	var leftDen, rightDen, termDen, left, right, next, part, gcd big.Int
	n := len(poly)
	for i := range n {
		if err := budget.Step(); err != nil {
			return 0, err
		}
		a, b := poly[i], poly[(i+1)%n]
		leftDen.Mul(a.U.Denom(), b.V.Denom())
		rightDen.Mul(b.U.Denom(), a.V.Denom())
		termDen.Mul(&leftDen, &rightDen)
		left.Mul(a.U.Num(), b.V.Num())
		left.Mul(&left, &rightDen)
		right.Mul(b.U.Num(), a.V.Num())
		right.Mul(&right, &leftDen)
		left.Sub(&left, &right)
		next.Mul(&num, &termDen)
		next.Add(&next, part.Mul(&den, &left))
		den.Mul(&den, &termDen)
		num.Set(&next)
		gcd.GCD(nil, nil, &num, &den)
		if gcd.BitLen() > 1 {
			num.Quo(&num, &gcd)
			den.Quo(&den, &gcd)
		}
	}
	return num.Sign(), nil
}

// EarClipX triangulates a weakly-simple counter-clockwise polygon (given as
// indices into pts) by exact ear clipping. Unlike the float cap triangulator
// it NEVER drops a collinear vertex — every input vertex appears in the
// output, which is what keeps a conforming subdivision conforming — so only
// strictly convex, unblocked ears are clipped. A stall means the polygon is
// not weakly simple, which is an internal error, never a wrong mesh.
func EarClipX(budget *proofbound.WorkBudget, pts []Xp2, poly []int) ([][3]int, error) {
	idx := make([]int, len(poly))
	for i, vi := range poly {
		if err := budget.Step(); err != nil {
			return nil, err
		}
		idx[i] = vi
	}
	tris := make([][3]int, 0, len(idx))
	for len(idx) > 3 {
		if err := budget.Step(); err != nil {
			return nil, err
		}
		n := len(idx)
		clipped := false
		for i := range n {
			if err := budget.Step(); err != nil {
				return nil, err
			}
			ia, ib, ic := idx[(i-1+n)%n], idx[i], idx[(i+1)%n]
			if Cross2xSign(pts[ia], pts[ib], pts[ic]) <= 0 {
				continue
			}
			blocked, err := EarBlockedX(budget, pts, idx, i)
			if err != nil {
				return nil, err
			}
			if blocked {
				continue
			}
			tris = append(tris, [3]int{ia, ib, ic})
			for j := i; j+1 < len(idx); j++ {
				if err := budget.Step(); err != nil {
					return nil, err
				}
				idx[j] = idx[j+1]
			}
			idx = idx[:len(idx)-1]
			clipped = true
			break
		}
		if !clipped {
			return nil, fmt.Errorf(`%w: exact ear clipping stalled on a boolean subdivision polygon`, decaderr.ErrBooleanFailed)
		}
	}
	if Cross2xSign(pts[idx[0]], pts[idx[1]], pts[idx[2]]) > 0 {
		tris = append(tris, [3]int{idx[0], idx[1], idx[2]})
	} else if Cross2xSign(pts[idx[0]], pts[idx[1]], pts[idx[2]]) < 0 {
		return nil, fmt.Errorf(`%w: a boolean subdivision polygon closed clockwise`, decaderr.ErrBooleanFailed)
	}
	return tris, nil
}

// EarBlockedX reports whether another polygon vertex lies inside the closed
// candidate ear — the exact analog of earBlocked, except that every OTHER
// vertex can block (collinear duplicates included), which is the conservative
// direction.
func EarBlockedX(budget *proofbound.WorkBudget, pts []Xp2, idx []int, i int) (bool, error) {
	n := len(idx)
	ip, in := (i-1+n)%n, (i+1)%n
	a, b, c := pts[idx[ip]], pts[idx[i]], pts[idx[in]]
	minU := math.Min(a.Fu, math.Min(b.Fu, c.Fu))
	maxU := math.Max(a.Fu, math.Max(b.Fu, c.Fu))
	minV := math.Min(a.Fv, math.Min(b.Fv, c.Fv))
	maxV := math.Max(a.Fv, math.Max(b.Fv, c.Fv))
	finiteBox := a.FloatFinite && b.FloatFinite && c.FloatFinite
	for j := range n {
		if err := budget.Step(); err != nil {
			return false, err
		}
		if j == ip || j == i || j == in {
			continue
		}
		p := pts[idx[j]]
		// Float conversion is monotone. A strict float separation therefore
		// proves exact separation, while equal rounded coordinates fall
		// through to the exact predicate.
		if finiteBox && p.FloatFinite &&
			(p.Fu < minU || p.Fu > maxU || p.Fv < minV || p.Fv > maxV) {
			continue
		}
		if p.U.Cmp(a.U) == 0 && p.V.Cmp(a.V) == 0 {
			continue
		}
		if p.U.Cmp(c.U) == 0 && p.V.Cmp(c.V) == 0 {
			continue
		}
		if PointInTriX(p, a, b, c) {
			return true, nil
		}
	}
	return false, nil
}

// AxisRays is the deterministic retry list for the parity test: the six
// axis-aligned directions. axis names the swept coordinate, dir its sense,
// and (u, v) the two projection coordinates.
var AxisRays = [6]struct {
	Axis, U, V int
	Dir        int
}{
	{0, 1, 2, 1}, {0, 1, 2, -1},
	{1, 2, 0, 1}, {1, 2, 0, -1},
	{2, 0, 1, 1}, {2, 0, 1, -1},
}

// CoordOf reads one coordinate of a float vertex by axis index.
func CoordOf(v r3.Vec, axis int) float64 {
	switch axis {
	case 0:
		return v.X
	case 1:
		return v.Y
	default:
		return v.Z
	}
}

// RatCoordOf materialises one exact coordinate of p as a big.Rat, by axis
// index — the projection into Xp2's own (untouched) rational domain. This is
// the one place a homogeneous coordinate pays a normalisation to become a
// value; a sign-only reader wants XIntCoordOf instead.
func RatCoordOf(p proofbound.Xpt, axis int) *big.Rat {
	switch axis {
	case 0:
		return new(big.Rat).SetFrac(p.X, p.W)
	case 1:
		return new(big.Rat).SetFrac(p.Y, p.W)
	default:
		return new(big.Rat).SetFrac(p.Z, p.W)
	}
}

// XIntCoordOf reads one coordinate's raw homogeneous numerator by axis index,
// with no normalisation at all: since the denominator (p.w) is always
// positive, this integer's sign already IS the coordinate's sign, which is
// what every sign-only reader in this file actually wants.
func XIntCoordOf(p proofbound.Xpt, axis int) *big.Int {
	switch axis {
	case 0:
		return p.X
	case 1:
		return p.Y
	default:
		return p.Z
	}
}

// ParityMesh is one mesh's vertex-projection cache for the parity kernel: the
// mesh's own vertex and facet buffers, held by reference, plus one lazily
// filled projection slice per swept axis.
//
// A cache belongs to ONE prepared operand within ONE operation. Every path that
// holds one — KeepSide, ClassifyRegion, and the near-miss depth witness scan —
// runs serially on its caller's goroutine, so an entry is never read while
// another goroutine writes it. There is deliberately no global, no pool and no
// Document field: a cache outliving its operation would outlive the buffers it
// projects.
//
// An entry is IMMUTABLE once initialized. Cross2xSign and Cross2x only read an
// Xp2's rationals — neither ever uses one as an arithmetic destination — so a
// projection shared across every query of this mesh classifies exactly as the
// fresh per-facet copy it replaces.
type ParityMesh struct {
	Verts []r3.Vec
	Tris  [][3]int
	// projections[axis][vi] is vertex vi projected onto the plane a ray
	// sweeping axis leaves. Both rays of an axis carry the same (u, v) pair
	// (AxisRays), so one slot per axis serves both senses. A slot stays nil
	// until that axis is first swept, and within a slot a nil u marks an entry
	// still unfilled: a materialized projection's coordinates are always
	// non-nil rationals, an exact zero included, so the two states never alias.
	Projections [3][]Xp2
	// boxes[axis][ti] is facet ti's projected box on that same plane, filled
	// on the facet's first visit for that axis and immutable from there. A
	// slot stays nil until the axis is first swept, and ParityFacetBox.built
	// marks an individual entry filled.
	Boxes [3][]ParityFacetBox
	// unfiltered turns the projected-box rejection off, leaving every query to
	// the exact sign tests alone. It exists so a differential test can run the
	// same query both ways and require the same answer; nothing in production
	// sets it.
	Unfiltered bool
}

// ParityAreaErrCoef and ParityAreaFloor bound projectedFacetBox's own float
// evaluation of the projected area. The true forward error of a 2x2
// determinant over float64 inputs is a few units in the last place of its
// permanent, so the relative coefficient carries about three decades of
// margin — enough to cover the rounding of the permanent and the threshold
// themselves. The floor is one absolute term for the gradual-underflow crumbs
// a relative bound cannot speak for: each is at most 2⁻¹⁰⁷⁵ and there are a
// handful, so 2⁻¹⁰⁰⁰ dominates them together.
const (
	ParityAreaErrCoef = 1e-12
	ParityAreaFloor   = 0x1p-1000
)

// ParityFacetBox is one facet's projection onto the plane a ray sweeping some
// axis leaves, reduced to a coordinate box plus the one fact that makes the box
// usable as a rejection filter.
//
// The bounds are EXACT, not an outward enclosure. A parity mesh's vertices are
// float64 coordinates and a projection just selects two of them, so each bound
// IS one of the triangle's own coordinates with no rounding to widen. The query
// side is what rounds: Xp2 caches its rational coordinate's nearest float
// (NewXP2), and round-to-nearest is monotone — x ≤ y implies rn(x) ≤ rn(y) — so
// rn(q) strictly past a bound proves q strictly past it exactly. The converse
// never holds, which is why equality decides nothing and falls through to the
// exact predicate.
//
// nondegenerate records that the projected triangle's area is provably nonzero,
// and without it the box says nothing about the answer.
// MeshParityPreparedContext turns a strict separation into a `continue` only
// because the three edge signs of a nondegenerate triangle cannot all be
// non-positive — they sum to twice its signed area — so a point outside it
// always shows the loop both a negative and a positive sign. A projection that
// collapses to a segment sums to zero instead, and every point on that
// segment's LINE, however far outside this box, makes all three signs vanish
// and the ray AMBIGUOUS. Skipping such a facet would drop an ambiguity the
// reference path reports, so a facet whose projected area cannot be proven
// nonzero is never filtered.
type ParityFacetBox struct {
	MinU, MaxU, MinV, MaxV float64
	Nondegenerate          bool
	Built                  bool
}

// rejects reports whether a query projected to (fu, fv) provably lies strictly
// outside this facet's projection. The caller owes it a finite (fu, fv) —
// MeshParityPreparedContext checks the query's own floatFinite once per scan
// rather than once per facet.
func (b *ParityFacetBox) Rejects(fu, fv float64) bool {
	return b.Nondegenerate &&
		(fu < b.MinU || fu > b.MaxU || fv < b.MinV || fv > b.MaxV)
}

// buildFacetBox builds facet ti's box on the (u, v) plane a ray sweeping axis
// leaves. A non-finite coordinate leaves nondegenerate false: the box would not
// bound anything, and an unusable box costs only the rejections it does not
// make.
//
// The area is decided adaptively, the same discipline as OrientSign: a float
// determinant whose magnitude clears its own error bound proves the sign, and
// anything inside that bound falls back to the exact 2D cross over the cached
// projections. The exact leg is what a coarse tessellation needs — a thin
// facet's float determinant can sit inside the bound while its true area is
// plainly nonzero — and it is paid once per facet and axis, against the many
// queries that then read the answer.
func (pm *ParityMesh) BuildFacetBox(axis, u, v, ti int) ParityFacetBox {
	tri := pm.Tris[ti]
	a, b, c := pm.Verts[tri[0]], pm.Verts[tri[1]], pm.Verts[tri[2]]
	au, av := CoordOf(a, u), CoordOf(a, v)
	bu, bv := CoordOf(b, u), CoordOf(b, v)
	cu, cv := CoordOf(c, u), CoordOf(c, v)
	box := ParityFacetBox{
		MinU:  math.Min(au, math.Min(bu, cu)),
		MaxU:  math.Max(au, math.Max(bu, cu)),
		MinV:  math.Min(av, math.Min(bv, cv)),
		MaxV:  math.Max(av, math.Max(bv, cv)),
		Built: true,
	}
	if proofbound.IsNonFinite(box.MinU) || proofbound.IsNonFinite(box.MaxU) ||
		proofbound.IsNonFinite(box.MinV) || proofbound.IsNonFinite(box.MaxV) {
		return box
	}
	left, right := (bu-au)*(cv-av), (bv-av)*(cu-au)
	bound := ParityAreaErrCoef*(math.Abs(left)+math.Abs(right)) + ParityAreaFloor
	if det := left - right; det > bound || det < -bound {
		box.Nondegenerate = true
		return box
	}
	qa := pm.VertexProjection(axis, u, v, tri[0])
	qb := pm.VertexProjection(axis, u, v, tri[1])
	qc := pm.VertexProjection(axis, u, v, tri[2])
	box.Nondegenerate = Cross2xSign(qa, qb, qc) != 0
	return box
}

// facetBoxes returns the per-facet projected-box slice for one swept axis,
// allocating it on that axis's first sweep. Both rays of an axis project onto
// the same plane (AxisRays), so one slice serves both senses.
func (pm *ParityMesh) FacetBoxes(axis int) []ParityFacetBox {
	if pm.Boxes[axis] == nil {
		pm.Boxes[axis] = make([]ParityFacetBox, len(pm.Tris))
	}
	return pm.Boxes[axis]
}

// NewParityMesh prepares verts and tris for repeated parity queries. It stores
// the caller's buffers by reference and materializes no projection at all: an
// axis slice is allocated on that axis's first sweep, and a vertex's projection
// on its own first use.
func NewParityMesh(verts []r3.Vec, tris [][3]int) *ParityMesh {
	return &ParityMesh{Verts: verts, Tris: tris}
}

// vertexProjection returns vertex vi projected onto the (u, v) plane a ray
// sweeping axis leaves, constructing it on first use and caching it unchanged.
// The returned Xp2 is read-only: its rationals are the cache's own, so a caller
// must never make one the destination of an arithmetic operation.
func (pm *ParityMesh) VertexProjection(axis, u, v, vi int) Xp2 {
	slot := pm.Projections[axis]
	if slot == nil {
		slot = make([]Xp2, len(pm.Verts))
		pm.Projections[axis] = slot
	}
	if slot[vi].U == nil {
		vert := pm.Verts[vi]
		slot[vi] = NewXP2(freeform.MustRatOf(CoordOf(vert, u)), freeform.MustRatOf(CoordOf(vert, v)))
	}
	return slot[vi]
}

// meshParity reports, exactly, whether p lies inside the closed float-vertex
// mesh restricted to the given facet subset: the crossing parity of an
// axis-aligned ray. A ray the point's projection meets at a facet's projected
// boundary is ambiguous and the next axis is tried; a p exactly ON a facet is
// onBoundary. All six axes ambiguous is a genuine failure — never a guess.
//
// This is the raw-buffer entry point: it prepares a single-use projection cache
// and answers one query through it. A caller holding an operand across many
// queries wants MeshParityPreparedContext with that operand's own cache.
func MeshParityContext(ctx context.Context, p proofbound.Xpt, verts []r3.Vec, tris [][3]int, subset []int) (bool, bool, error) {
	return MeshParityPreparedContext(ctx, p, NewParityMesh(verts, tris), subset)
}

// MeshParityPreparedContext is MeshParityContext over a prepared mesh whose
// vertex projections persist between queries. The classification is identical:
// only where each projection's rationals come from changes, and the cache hands
// back the same value the per-facet construction built.
func MeshParityPreparedContext(ctx context.Context, p proofbound.Xpt, prepared *ParityMesh, subset []int) (bool, bool, error) {
	for _, ray := range AxisRays {
		crossings := 0
		ambiguous := false
		onBoundary := false
		// The query's projection depends only on the ray, so it is built once
		// per nonempty scan rather than once per facet, and read-only from
		// there — Cross2xSign and Cross2x never mutate an Xp2, so one shared
		// value classifies exactly as a per-facet copy did. Constructing it
		// inside the loop, after the periodic cancellation check, is what
		// keeps an empty subset paying nothing and a canceled context
		// returning before the first conversion.
		var pa Xp2
		// The projected-box rejection needs the query's own float coordinates,
		// so it can only be armed once pa exists, and its per-facet boxes are
		// allocated in the same place — an empty subset still pays nothing.
		var boxes []ParityFacetBox
		boxFilter := false
		for i, ti := range subset {
			if i%256 == 0 {
				if err := ctx.Err(); err != nil {
					return false, false, err
				}
			}
			if i == 0 {
				pa = NewXP2(RatCoordOf(p, ray.U), RatCoordOf(p, ray.V))
				boxFilter = !prepared.Unfiltered && pa.FloatFinite
				if boxFilter {
					boxes = prepared.FacetBoxes(ray.Axis)
				}
			}
			if boxFilter {
				// A facet whose projection provably has area and provably does
				// not reach the query's projected coordinate classifies
				// STRICTLY OUTSIDE below, whichever way its three signs fall
				// (ParityFacetBox). Skipping it therefore removes three exact
				// sign tests and changes no answer, and because it is exactly
				// the facets that would `continue` that go, the subset's order
				// and the facet an ambiguity is first seen at are untouched.
				box := &boxes[ti]
				if !box.Built {
					*box = prepared.BuildFacetBox(ray.Axis, ray.U, ray.V, ti)
				}
				if box.Rejects(pa.Fu, pa.Fv) {
					continue
				}
			}
			tri := prepared.Tris[ti]
			qa := prepared.VertexProjection(ray.Axis, ray.U, ray.V, tri[0])
			qb := prepared.VertexProjection(ray.Axis, ray.U, ray.V, tri[1])
			qc := prepared.VertexProjection(ray.Axis, ray.U, ray.V, tri[2])
			s1 := Cross2xSign(qa, qb, pa)
			s2 := Cross2xSign(qb, qc, pa)
			s3 := Cross2xSign(qc, qa, pa)
			neg := s1 < 0 || s2 < 0 || s3 < 0
			pos := s1 > 0 || s2 > 0 || s3 > 0
			if neg && pos {
				continue // strictly outside the projection
			}
			if s1 == 0 || s2 == 0 || s3 == 0 {
				// On the projected boundary: the ray may graze an edge or a
				// vertex, and the count would be unreliable — try another axis.
				ambiguous = true
				break
			}
			// Strictly inside the projection: the projected area is nonzero,
			// so the plane normal's swept component cannot vanish.
			a, b, c := prepared.Verts[tri[0]], prepared.Verts[tri[1]], prepared.Verts[tri[2]]
			xa, xb, xc := proofbound.XptOf(a), proofbound.XptOf(b), proofbound.XptOf(c)
			n := Xcross(proofbound.Xsub(xb, xa), proofbound.Xsub(xc, xa))
			nAxis := XIntCoordOf(n, ray.Axis)
			if nAxis.Sign() == 0 {
				ambiguous = true
				break
			}
			// t = tNum/nAxis decides the crossing; nAxis is already proven
			// nonzero above, so its sign alone tells the division's sign
			// without ever forming the quotient — only t's sign is read, and
			// proofbound.XdotNum's raw numerator carries that sign with no normalisation
			// anywhere in the chain (docs/evaluator-design.md §9's
			// reject-only discipline extends to never paying for a value
			// nothing but Sign() consumes).
			tNum := proofbound.XdotNum(proofbound.Xsub(xa, p), n)
			switch s := tNum.Sign() * nAxis.Sign() * ray.Dir; {
			case s > 0:
				crossings++
			case tNum.Sign() == 0:
				onBoundary = true
			}
			if onBoundary {
				break
			}
		}
		if onBoundary {
			return false, true, nil
		}
		if ambiguous {
			continue
		}
		return crossings%2 == 1, false, nil
	}
	return false, false, fmt.Errorf(`%w: every parity ray was ambiguous`, decaderr.ErrBooleanFailed)
}
