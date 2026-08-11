package decad

import (
	"context"
	"fmt"
	"math"
	"math/big"

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
// to its canonical form only at vertex emission (xpt.key), because welding is
// by exact identity (boolean_mesh.go's stitchFacetsContext) and a homogeneous
// point has many spellings. A sign decided exactly is a topology decision
// that cannot flip (core §2.1), which is what makes the stitched output
// watertight by construction on the tessellated geometry.

// xpt is an exact 3D point carried in homogeneous integer form: the same
// representation xhp documents below, and structurally identical to it — the
// two convert for free — so xpt keeps its own name and every call site
// survives, rather than xhp replacing it outright. The zero value is
// unusable; construct through xptOf or the arithmetic helpers, every one of
// which strips the common power of two before returning (the growth control;
// see xhpStripTwos). Two spellings of one point exist (any positive common
// factor of x, y, z and w denotes the same coordinate), so exact identity for
// welding goes through the canonical form (key), never the raw fields.
type xpt struct{ x, y, z, w *big.Int }

// xptOf lifts a finite float vertex into exact homogeneous coordinates. A
// float64 is an exact rational, so no information is lost.
func xptOf(v r3.Vec) xpt { return xpt(xhpStripTwos(xhpOf(v))) }

// vec rounds the exact point to the nearest float64 coordinates.
func (p xpt) vec() r3.Vec { return xhpVec(xhp(p)) }

// key is the exact identity of the point: two points weld exactly when their
// CANONICAL homogeneous coordinates are identical — stitching by shared exact
// vertices, never by distance (docs/evaluator-design.md §9). A homogeneous
// point has many spellings, so the raw fields are never compared directly;
// xhpCanon collapses every spelling of one coordinate to one four-tuple
// before the key is built.
func (p xpt) key() string {
	c := xhpCanon(xhp(p))
	return c.x.String() + "|" + c.y.String() + "|" + c.z.String() + "|" + c.w.String()
}

// xsub is a − b, exact, with the common power of two stripped on return (the
// growth control every construction pays — see xhpStripTwos).
func xsub(a, b xpt) xpt { return xpt(xhpStripTwos(xhpSub(xhp(a), xhp(b)))) }

// xcross is a × b, exact, stripped the same way as xsub.
func xcross(a, b xpt) xpt { return xpt(xhpStripTwos(xhpCross(xhp(a), xhp(b)))) }

// xdotNum is the raw numerator of a·b over the positive denominator a.w·b.w —
// the value a sign-only consumer reads directly, and the value a
// cross-multiplied comparison reads without ever dividing.
func xdotNum(a, b xpt) *big.Int { return xhpDotNum(xhp(a), xhp(b)) }

// xdotSign is the sign of a·b, decided as a plain integer sign: the shared
// denominator a.w·b.w is always positive, so the numerator's sign IS the
// dot product's sign.
func xdotSign(a, b xpt) int { return xdotNum(a, b).Sign() }

// xdotRat materialises a·b as a big.Rat — the one place this dot product pays
// a normalisation, and only when a caller genuinely needs the rational VALUE
// rather than a sign.
func xdotRat(a, b xpt) *big.Rat {
	den := new(big.Int).Mul(a.w, b.w)
	return new(big.Rat).SetFrac(xdotNum(a, b), den)
}

// xlerp is a + t·(b − a) for t = tn/td, exact, with the common power of two
// stripped on return — the growth control that keeps a chain of lerps from
// growing its denominator multiplicatively at every link (measured: 14113
// bits unreduced at lerp depth 6, 462 bits stripped after every step).
func xlerp(a, b xpt, tn, td *big.Int) xpt { return xpt(xhpStripTwos(xhpLerp(xhp(a), xhp(b), tn, td))) }

// orientNum is the exact value of det[b−a, c−a, d−a] as an integer numerator
// over a positive denominator, formed without ever materialising a big.Rat:
// positive when d lies on the side the counter-clockwise normal of (a, b, c)
// points to.
func orientNum(a, b, c, d xpt) (num, den *big.Int) {
	ha, hb, hc, hd := xhp(a), xhp(b), xhp(c), xhp(d)
	ba, ca, da := xhpSub(hb, ha), xhpSub(hc, ha), xhpSub(hd, ha)
	cr := xhpCross(ba, ca)
	return xhpDotNum(cr, da), new(big.Int).Mul(cr.w, da.w)
}

// orientSignExact is the exact sign of det[b−a, c−a, d−a], decided as a plain
// integer sign with no big.Rat and no normalisation anywhere in the chain —
// xhpOrientSign's own guarantee, carried through xpt.
func orientSignExact(a, b, c, d xpt) int {
	return xhpOrientSign(xhp(a), xhp(b), xhp(c), xhp(d))
}

// orientRat materialises det[b−a, c−a, d−a] as a big.Rat — the one place this
// value pays a normalisation, for the rare caller that needs the value rather
// than the sign.
func orientRat(a, b, c, d xpt) *big.Rat {
	num, den := orientNum(a, b, c, d)
	return new(big.Rat).SetFrac(num, den)
}

// orientSign is the adaptive-precision sign of det[b−a, c−a, d−a] for float
// inputs: a float evaluation whose forward error provably cannot cross zero
// decides the generic case; anything inside the error bound falls back to the
// exact value — the §9 discipline, so a sign is never wrong.
func orientSign(a, b, c, d r3.Vec) int {
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
			return 1
		}
		return -1
	}
	return orientSignExact(xptOf(a), xptOf(b), xptOf(c), xptOf(d))
}

// orientSignMixed is the exact plane-side sign of a homogeneous probe against
// a float triangle: positive on the triangle's counter-clockwise-normal side.
func orientSignMixed(a, b, c r3.Vec, d xpt) int {
	return orientSignExact(xptOf(a), xptOf(b), xptOf(c), d)
}

// xhp is an exact 3D point in homogeneous integer form: (x, y, z) is an
// integer numerator triple over one shared positive denominator w — the point
// it denotes is (x/w, y/w, z/w). big.Int carries no normalisation step of its
// own, unlike big.Rat, which runs a full Lehmer GCD on every construction
// whose denominator is not exactly 1 (rat.go's norm) — dyadic denominators
// included, since norm skips the GCD only when the denominator is 1. Every
// predicate built from these four integers is therefore a plain integer sign
// test.
//
// The invariant w > 0 is load-bearing: every helper below reduces a
// division's sign to the sign of a product of denominators, which only holds
// when every denominator involved is positive. xhpSub and xhpCross each
// combine two operands whose own w is positive into a result whose w is their
// PRODUCT — again positive by construction, with no branch needed — so the
// invariant propagates through every point/vector this file builds except
// one: xhpLerp's lerp-parameter denominator can arrive negative, and it
// renormalises explicitly before folding it in.
//
// A homogeneous point has many spellings — (x, y, z, w) and (2x, 2y, 2z, 2w)
// denote the same coordinate — so exact identity (welding) never compares raw
// fields; it goes through the canonical form (xhpCanon).
type xhp struct{ x, y, z, w *big.Int }

// xhpOf lifts a finite float vertex into homogeneous integer coordinates. A
// float64 is an exact dyadic rational, so each coordinate's own numerator and
// denominator (mustRatOf, clearance_poly.go) combine over one shared
// denominator — the other two coordinates' own denominators — with no
// rounding.
func xhpOf(v r3.Vec) xhp {
	rx, ry, rz := mustRatOf(v.X), mustRatOf(v.Y), mustRatOf(v.Z)
	dx, dy, dz := rx.Denom(), ry.Denom(), rz.Denom()
	return xhp{
		x: new(big.Int).Mul(rx.Num(), new(big.Int).Mul(dy, dz)),
		y: new(big.Int).Mul(ry.Num(), new(big.Int).Mul(dx, dz)),
		z: new(big.Int).Mul(rz.Num(), new(big.Int).Mul(dx, dy)),
		w: new(big.Int).Mul(dx, new(big.Int).Mul(dy, dz)),
	}
}

// xhpSub is p − q, exact: a homogeneous vector over the positive denominator
// p.w·q.w.
func xhpSub(p, q xhp) xhp {
	return xhp{
		x: new(big.Int).Sub(new(big.Int).Mul(p.x, q.w), new(big.Int).Mul(q.x, p.w)),
		y: new(big.Int).Sub(new(big.Int).Mul(p.y, q.w), new(big.Int).Mul(q.y, p.w)),
		z: new(big.Int).Sub(new(big.Int).Mul(p.z, q.w), new(big.Int).Mul(q.z, p.w)),
		w: new(big.Int).Mul(p.w, q.w),
	}
}

// xhpCross is a × b, exact: its numerators over the positive denominator
// a.w·b.w.
func xhpCross(a, b xhp) xhp {
	return xhp{
		x: new(big.Int).Sub(new(big.Int).Mul(a.y, b.z), new(big.Int).Mul(a.z, b.y)),
		y: new(big.Int).Sub(new(big.Int).Mul(a.z, b.x), new(big.Int).Mul(a.x, b.z)),
		z: new(big.Int).Sub(new(big.Int).Mul(a.x, b.y), new(big.Int).Mul(a.y, b.x)),
		w: new(big.Int).Mul(a.w, b.w),
	}
}

// xhpDotNum is the NUMERATOR of a·b over the positive denominator a.w·b.w.
// The denominator is never formed: every consumer either reads only this
// numerator's sign, or divides it back out at the one point a value is
// actually published (xhpRat).
func xhpDotNum(a, b xhp) *big.Int {
	s := new(big.Int).Mul(a.x, b.x)
	s.Add(s, new(big.Int).Mul(a.y, b.y))
	s.Add(s, new(big.Int).Mul(a.z, b.z))
	return s
}

// xhpLerp is a + t·(b − a) for t = tn/td, exact. td may arrive negative; a.w
// and b.w are already positive by invariant, so the result's own w — their
// product with td — is renormalised by flipping td's (and tn's) sign first,
// which leaves the value t = tn/td unchanged.
func xhpLerp(a, b xhp, tn, td *big.Int) xhp {
	n, d := tn, td
	if d.Sign() < 0 {
		n = new(big.Int).Neg(n)
		d = new(big.Int).Neg(d)
	}
	// a + t·(b−a) = a·(d−n)/d + b·n/d = [a·(d−n)·b.w + b·n·a.w] / (a.w·b.w·d).
	diff := new(big.Int).Sub(d, n)
	axis := func(av, bv *big.Int) *big.Int {
		term := new(big.Int).Mul(av, diff)
		term.Mul(term, b.w)
		other := new(big.Int).Mul(bv, n)
		other.Mul(other, a.w)
		return term.Add(term, other)
	}
	w := new(big.Int).Mul(a.w, b.w)
	w.Mul(w, d)
	return xhp{x: axis(a.x, b.x), y: axis(a.y, b.y), z: axis(a.z, b.z), w: w}
}

// xhpOrientSign is the exact sign of det[b−a, c−a, d−a], decided as a plain
// integer sign with no big.Rat and no normalisation anywhere in the chain:
// every intermediate xhp carries a positive denominator by construction, so
// the final numerator's sign IS the determinant's sign.
func xhpOrientSign(a, b, c, d xhp) int {
	ba, ca, da := xhpSub(b, a), xhpSub(c, a), xhpSub(d, a)
	return xhpDotNum(xhpCross(ba, ca), da).Sign()
}

// xhpRat materialises p's three coordinates as big.Rat — the one place a
// homogeneous point pays a normalisation, and only when a caller genuinely
// needs a rational VALUE rather than a sign.
func xhpRat(p xhp) (x, y, z *big.Rat) {
	return new(big.Rat).SetFrac(p.x, p.w), new(big.Rat).SetFrac(p.y, p.w), new(big.Rat).SetFrac(p.z, p.w)
}

// xhpVec rounds p to the nearest float64 coordinates.
func xhpVec(p xhp) r3.Vec {
	x, y, z := xhpRat(p)
	fx, _ := x.Float64()
	fy, _ := y.Float64()
	fz, _ := z.Float64()
	return r3.Vec{X: fx, Y: fy, Z: fz}
}

// xhpStripTwos divides all four integers by their largest common power of
// two — the growth control. It costs no GCD (TrailingZeroBits and Rsh only),
// which is what makes it cheap enough to run on every construction; the full
// GCD xhpCanon runs is not (its own doc comment).
func xhpStripTwos(p xhp) xhp {
	tz := p.w.TrailingZeroBits()
	for _, v := range [3]*big.Int{p.x, p.y, p.z} {
		if v.Sign() == 0 {
			continue
		}
		if t := v.TrailingZeroBits(); t < tz {
			tz = t
		}
	}
	if tz == 0 {
		return p
	}
	return xhp{
		x: new(big.Int).Rsh(p.x, tz),
		y: new(big.Int).Rsh(p.y, tz),
		z: new(big.Int).Rsh(p.z, tz),
		w: new(big.Int).Rsh(p.w, tz),
	}
}

// xhpCanon reduces p to its unique canonical spelling: every one of the four
// integers divided by gcd(|x|, |y|, |z|, w). This is the FULL GCD the
// representation otherwise exists to avoid, so it is reserved for the one
// place a homogeneous point's many spellings must collapse to one —
// vertex-emission identity (key, welding) — and must NEVER be called per
// arithmetic operation: measured, a depth-3 xhpLerp chain drops from
// big.Rat's 31.4us to 9.6us with xhpStripTwos alone, but only to 23.4us with
// a full xhpCanon on every construction — nearly the whole win, given back.
func xhpCanon(p xhp) xhp {
	g := new(big.Int).Set(p.w)
	for _, v := range [3]*big.Int{p.x, p.y, p.z} {
		if v.Sign() == 0 {
			continue
		}
		g.GCD(nil, nil, g, v)
	}
	return xhp{
		x: new(big.Int).Quo(p.x, g),
		y: new(big.Int).Quo(p.y, g),
		z: new(big.Int).Quo(p.z, g),
		w: new(big.Int).Quo(p.w, g),
	}
}

const (
	// segFilterErrCoef covers segFilter.tooFar's own float evaluation: 64·u,
	// against the magnitude the branch taken carries. tooFar derives it.
	segFilterErrCoef = 64 * unitRoundoff
	// segFilterFloor is one absolute term covering every gradual-underflow
	// crumb the same evaluation can commit. Each is at most 2⁻¹⁰⁷⁵ and there
	// are a few dozen of them, so 2⁻¹⁰⁰⁰ dominates them all together. It is an
	// absolute term at the FINAL threshold, so it speaks only for a crumb
	// nothing amplifies afterwards; segFilterMinNormal is what keeps the
	// interior branch inside that promise.
	segFilterFloor = 0x1p-1000
	// segFilterMinNormal is the smallest positive NORMAL float64. tooFar's
	// forward-error bound is a RELATIVE one and holds only over normalized
	// arithmetic; a subnormal intermediate carries absolute error instead, and
	// a later division by a small quantity amplifies that crumb back up to the
	// scale of the answer, which segFilterFloor does not cover.
	segFilterMinNormal = 0x1p-1022
	// segFilterMinDot is the smallest c₁ whose SQUARE is still normal. The
	// interior branch works at the (D·L)² scale, which reaches the subnormal
	// range at the SQUARE ROOT of the coordinate scale that would underflow ww
	// or vv — so the positivity guards on those two say nothing about it.
	segFilterMinDot = 0x1p-511
)

// segAdmissionRadius2 is τ², the squared distance from the FLOAT segment beyond
// which a candidate provably cannot be an interior point of the EXACT one. It
// is read once per conforming pass: maxAbs is the largest |coordinate| over
// every vertex of the stitched mesh, and slack is that pass's own grid slack.
//
// DERIVATION. Write a, b, p for the exact rational endpoints and candidate and
// A, B, P for their float64 roundings (xpt.vec, round to nearest). Suppose the
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
// grid box that precedes it (conformOnce), which is that pass's own statement
// of how far a float approximation may sit from its exact point.
//
// τ² underflows to zero only for τ below 2⁻⁵³⁷, and at that scale
// segFilterFloor already demands a distance above 2⁻⁵⁰⁰ before tooFar will
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
//     newConformScan cannot produce one (its slack is built from a vector
//     length and a positive cell width), so this is a precondition made
//     total rather than a live defect.
//   - AN OVERFLOWED τ². τ passes 2⁻⁵³⁷'s mirror at about 1.34e154, above which
//     τ·τ saturates. Nothing is proven about a saturated threshold, and +Inf is
//     the reading that costs only the rejections the filter would have made on
//     a mesh whose coordinates run past 1e204.
func segAdmissionRadius2(slack, maxAbs float64) float64 {
	if !(slack >= 0 && slack <= math.MaxFloat64) || !(maxAbs >= 0 && maxAbs <= math.MaxFloat64) {
		return math.Inf(1)
	}
	tau := slack + (maxAbs*0x1p-50 + segFilterFloor)
	if tau2 := upRound(tau * tau); !isNonFinite(tau2) {
		return tau2
	}
	return math.Inf(1)
}

// segFilter is the reject-only float pre-filter in front of the exact
// onSegmentInterior3 predicate (docs/evaluator-design.md §9). The conforming
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
type segFilter struct {
	// a and b are the float roundings of the segment's exact endpoints, v is
	// the float b − a, and vv is the float v·v.
	a, b, v r3.Vec
	vv      float64
	// tau2 is τ² from segAdmissionRadius2.
	tau2 float64
}

func newSegFilter(a, b r3.Vec, tau2 float64) segFilter {
	v := b.Sub(a)
	return segFilter{a: a, b: b, v: v, vv: v.Dot(v), tau2: tau2}
}

// tooFar reports whether p — the float rounding of a candidate's exact
// coordinates — is PROVABLY farther than τ from the float segment [a, b]. A
// false answer means "not proven", never "close enough".
//
// The value is the textbook clamped projection and segFilterErrCoef covers its
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
// which the budget swallows whole. So segFilterErrCoef = 64·u is 2.6× the worst
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
// the segment computes d² = W and is REJECTED. segFilterFloor cannot cover it,
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
//     line above, and segFilterErrCoef·mag cannot overflow a finite mag, so the
//     left-hand side is ALWAYS finite. A non-finite τ² therefore never meets a
//     non-finite left-hand side: segAdmissionRadius2 returns +Inf for every
//     threshold it cannot state, and a finite value is never above +Inf.
func (f segFilter) tooFar(p r3.Vec) bool {
	if !(f.vv > 0) || isNonFinite(f.vv) {
		// The float segment has nothing to project onto — both endpoints
		// rounded to one float, or v underflowed — or it is long enough that a
		// component of v, or vv itself, saturated. Abstaining is always sound.
		return false
	}
	w := p.Sub(f.a)
	ww := w.Dot(w)
	c1 := w.Dot(f.v)
	if isNonFinite(ww) || isNonFinite(c1) {
		// Both feed the branch decision below, so a saturated one does not
		// widen the answer, it picks the wrong formula for it.
		return false
	}
	d2, mag := ww, ww
	if c1 > 0 {
		if c1 < f.vv {
			// c₁²/V, divided before it is squared. The guard is written so a
			// NaN fails it: every intermediate this branch could form — the
			// two below and the c₁² the unscaled arrangement would form — has
			// to be normal, or the branch has no proven bound and abstains.
			t := c1 / f.vv
			proj := t * c1
			if !(c1 >= segFilterMinDot && t >= segFilterMinNormal && proj >= segFilterMinNormal) {
				return false
			}
			d2 = ww - proj
		} else {
			e := p.Sub(f.b)
			d2 = e.Dot(e)
			if isNonFinite(d2) {
				return false
			}
			mag = d2
		}
	}
	return d2-(segFilterErrCoef*mag+segFilterFloor) > f.tau2
}

// floatInterval is a proven float64 enclosure [lo, hi] of an exact rational
// value: the reject-only pre-filter in front of triTriClassify's non-coplanar
// arm (triTriMissesFilter, below) is built entirely on this type, the same
// "outward-rounded float arithmetic, exact fallback for anything it cannot
// prove" shape segFilter uses for the conforming pass's own segment test. It
// is float64's own enclosure, deliberately apart from ratInterval
// (moments.go) and capblend_contour.go's ivPoint/ivCarrier, which enclose in
// big.Rat and serve a different proof (a rational bound, not a float filter).
//
// Every method below returns EITHER a proper enclosure — lo and hi both
// finite, lo ≤ hi, and the true value proven to lie in [lo, hi] — OR the
// sentinel fivAbstain ({-Inf, +Inf}), which encloses every value trivially
// and is this type's uniform "cannot prove anything here" answer. No method
// ever returns a half-finite interval, so a caller need only test one
// sentinel to know whether an answer was reached.
//
// ROUNDING. Each arithmetic method computes its result with plain float64
// operations (round-to-nearest) and then widens the low end down and the high
// end up by exactly one ulp with math.Nextafter. Round-to-nearest error is at
// most half an ulp of the computed value, so a full ulp of outward widening
// covers it with room to spare — the same margin segAdmissionRadius2's
// derivation leans on. A binary operation with two interval operands has up
// to four sign combinations at its corners (mul, div); the result takes the
// min/max of all of them before the same one-ulp widening, which is the
// textbook outward-rounded interval evaluation.
//
// NON-FINITE INPUTS AND OUTPUTS ABSTAIN, NEVER PROPAGATE. Every method checks
// every intermediate it forms for finiteness before trusting it, and returns
// fivAbstain the moment one fails — mirroring segFilter.tooFar's own
// "saturation is tested, not argued" discipline. A saturated intermediate
// does not merely widen an answer, it can flip which branch a later decision
// takes (Inf compares false against everything finite), so nothing downstream
// may read a value this type has not first proven finite.
type floatInterval struct{ lo, hi float64 }

// fivAbstain is the everywhere-abstaining interval.
var fivAbstain = floatInterval{lo: math.Inf(-1), hi: math.Inf(1)}

// abstains reports whether iv is the abstain sentinel.
func (a floatInterval) abstains() bool { return a == fivAbstain }

// contains reports whether x lies within the closed interval.
func (a floatInterval) contains(x float64) bool { return a.lo <= x && x <= a.hi }

// disjoint reports whether a and b, as closed intervals, share no point.
func (a floatInterval) disjoint(b floatInterval) bool { return a.hi < b.lo || b.hi < a.lo }

// fivPoint lifts a float64 that is itself an EXACT value (never itself the
// product of a rounding) into a degenerate interval: lo = hi = x needs no
// widening, because there is no rounding error to cover. Every triangle
// vertex coordinate triTriMissesFilter reads is such a value — a float64 IS
// an exact rational (boolean_exact.go's own opening comment) — so this is the
// leaf constructor for every vertex coordinate the filter touches.
func fivPoint(x float64) floatInterval {
	if isNonFinite(x) {
		return fivAbstain
	}
	return floatInterval{lo: x, hi: x}
}

// fivRounded lifts a float64 that is itself a CORRECTLY-ROUNDED conversion of
// some exact value — at most half an ulp off truth, the guarantee
// big.Rat.Float64 makes — into an interval with one full ulp of margin on
// each side, which covers that half-ulp with the same spare-half-ulp margin
// every other widening in this type carries. triTriMissesFilter's na/nb
// arguments are xpt.vec() results, so they are exactly this case.
func fivRounded(x float64) floatInterval {
	if isNonFinite(x) {
		return fivAbstain
	}
	return floatInterval{lo: math.Nextafter(x, math.Inf(-1)), hi: math.Nextafter(x, math.Inf(1))}
}

func (a floatInterval) add(b floatInterval) floatInterval {
	lo, hi := a.lo+b.lo, a.hi+b.hi
	if isNonFinite(lo) || isNonFinite(hi) {
		return fivAbstain
	}
	return floatInterval{lo: math.Nextafter(lo, math.Inf(-1)), hi: math.Nextafter(hi, math.Inf(1))}
}

func (a floatInterval) sub(b floatInterval) floatInterval {
	lo, hi := a.lo-b.hi, a.hi-b.lo
	if isNonFinite(lo) || isNonFinite(hi) {
		return fivAbstain
	}
	return floatInterval{lo: math.Nextafter(lo, math.Inf(-1)), hi: math.Nextafter(hi, math.Inf(1))}
}

func (a floatInterval) mul(b floatInterval) floatInterval {
	p0, p1, p2, p3 := a.lo*b.lo, a.lo*b.hi, a.hi*b.lo, a.hi*b.hi
	if isNonFinite(p0) || isNonFinite(p1) || isNonFinite(p2) || isNonFinite(p3) {
		return fivAbstain
	}
	lo := math.Min(math.Min(p0, p1), math.Min(p2, p3))
	hi := math.Max(math.Max(p0, p1), math.Max(p2, p3))
	return floatInterval{lo: math.Nextafter(lo, math.Inf(-1)), hi: math.Nextafter(hi, math.Inf(1))}
}

// div guards the one case the other three operations do not have: a
// denominator interval containing zero makes the quotient unbounded (or
// undefined, at zero itself), so it returns fivAbstain outright rather than
// forming a division that could saturate or, worse, land on a finite value
// that means nothing.
func (a floatInterval) div(b floatInterval) floatInterval {
	if isNonFinite(a.lo) || isNonFinite(a.hi) || isNonFinite(b.lo) || isNonFinite(b.hi) {
		return fivAbstain
	}
	if b.lo <= 0 && b.hi >= 0 {
		return fivAbstain
	}
	q0, q1, q2, q3 := a.lo/b.lo, a.lo/b.hi, a.hi/b.lo, a.hi/b.hi
	if isNonFinite(q0) || isNonFinite(q1) || isNonFinite(q2) || isNonFinite(q3) {
		return fivAbstain
	}
	lo := math.Min(math.Min(q0, q1), math.Min(q2, q3))
	hi := math.Max(math.Max(q0, q1), math.Max(q2, q3))
	return floatInterval{lo: math.Nextafter(lo, math.Inf(-1)), hi: math.Nextafter(hi, math.Inf(1))}
}

// fivVec is a 3-vector of floatIntervals: the interval mirror of xpt, over
// float64 rather than big.Rat.
type fivVec struct{ x, y, z floatInterval }

func fivVecOf(v r3.Vec) fivVec { return fivVec{fivPoint(v.X), fivPoint(v.Y), fivPoint(v.Z)} }

func fivSub(a, b fivVec) fivVec { return fivVec{a.x.sub(b.x), a.y.sub(b.y), a.z.sub(b.z)} }

func fivDot(a, b fivVec) floatInterval { return a.x.mul(b.x).add(a.y.mul(b.y)).add(a.z.mul(b.z)) }

func fivCross(a, b fivVec) fivVec {
	return fivVec{
		a.y.mul(b.z).sub(a.z.mul(b.y)),
		a.z.mul(b.x).sub(a.x.mul(b.z)),
		a.x.mul(b.y).sub(a.y.mul(b.x)),
	}
}

// fivOrient is orientVal's interval mirror: the proven enclosure of
// det[b−a, c−a, d−a], the same expression planeCrossings' val(i) evaluates
// exactly for the crossing parameter t = vi/(vi − vj).
func fivOrient(a, b, c, d fivVec) floatInterval {
	return fivDot(fivCross(fivSub(b, a), fivSub(c, a)), fivSub(d, a))
}

// triSpanOnLine is triTriMissesFilter's per-triangle half: a proven [lo, hi]
// enclosure of the range triangle t occupies when its plane crossings against
// the OTHER triangle o are projected onto dir — the float mirror of
// orderOnLine over planeCrossings' own two cases (a zero-sign vertex sits
// exactly on the crossing; a sign-changing edge crosses it at t). signs is
// the caller's own already-decided sign triple (orientSign, possibly via its
// exact fallback) for t's vertices against o's plane — this filter never
// re-derives a sign, only asks which of planeCrossings' branches it selects,
// exactly as the exact code does.
//
// The bound is deliberately a SUPERSET of the true occupied range: it takes
// every candidate planeCrossings would have considered — including one a
// wide interval later drops from contention exactly the way dedupePoints or
// the ≤2-points invariant would — and folds in its own projected interval, so
// the returned span can only be wider than the truth, never narrower. A
// wider span makes the filter LESS likely to prove disjointness, never more:
// the reject-only direction is unaffected by the extra slack. ok is false —
// the uniform abstain signal — the moment any step could not be bounded, or
// no candidate was found at all (which allOneSide's own gate, run before this
// filter is ever called, has already ruled out for a real non-coplanar pair).
func triSpanOnLine(t, o [3]fivVec, signs [3]int, dir fivVec) (floatInterval, bool) {
	lo, hi := math.Inf(1), math.Inf(-1)
	found := false
	widen := func(p fivVec) bool {
		proj := fivDot(p, dir)
		if proj.abstains() {
			return false
		}
		lo = math.Min(lo, proj.lo)
		hi = math.Max(hi, proj.hi)
		found = true
		return true
	}
	for i := range 3 {
		j := (i + 1) % 3
		if signs[i] == 0 && !widen(t[i]) {
			return floatInterval{}, false
		}
		if signs[i]*signs[j] >= 0 {
			continue
		}
		vi := fivOrient(o[0], o[1], o[2], t[i])
		vj := fivOrient(o[0], o[1], o[2], t[j])
		frac := vi.div(vi.sub(vj))
		if frac.abstains() {
			return floatInterval{}, false
		}
		p := fivVec{
			x: t[i].x.add(frac.mul(t[j].x.sub(t[i].x))),
			y: t[i].y.add(frac.mul(t[j].y.sub(t[i].y))),
			z: t[i].z.add(frac.mul(t[j].z.sub(t[i].z))),
		}
		if !widen(p) {
			return floatInterval{}, false
		}
	}
	if !found {
		return floatInterval{}, false
	}
	return floatInterval{lo: lo, hi: hi}, true
}

// triTriMissesFilter is the reject-only float pre-filter in front of
// triTriClassify's non-coplanar rational arm (docs/evaluator-design.md §9),
// dispatched ahead of planeCrossings for exactly the reason segFilter sits
// ahead of onSegmentInterior3: the circular fixture that motivated it enumerates
// 66,008 facet pairs into that arm, and only 1,860 of them (fu158) have any
// real contact, so almost every call pays a full math/big.Rat cross product to
// establish what a dozen float operations already establish — the pair's
// planes cross too far outside both triangles to meet at all.
//
// It reproduces triTriClassify's own non-coplanar computation — planeCrossings
// per triangle, then the projection onto dir = na × nb that orderOnLine
// compares — entirely in interval arithmetic (triSpanOnLine, above), and
// returns true — PROVEN no contact — only when the two triangles' projected
// spans are proven disjoint. Every other outcome, abstention included,
// returns false: the pair still goes to the exact predicate, which decides it
// with no help from this filter. That asymmetry is the whole soundness
// argument, in segFilter's own words: a filter that could ADMIT would be an
// admission gate on a residual, which this package forbids outright, and a
// filter that could reject a true contact would hand back a non-conforming
// subdivision — a WRONG boolean, not a slow one.
//
// ta, tb are the operands' own float corners, read as exact point intervals
// (fivPoint — a float64 vertex coordinate is exact, never itself a
// rounding). na, nb are xpt.vec() — the correctly-rounded float64 conversion
// of the pair's exact rational normals — read with fivRounded's extra ulp of
// margin for that rounding. sa, sb are the vertex-against-the-other-plane
// sign triples orientSign already decided; see triSpanOnLine for how they
// steer the reconstruction.
//
// THE SHALLOW-ANGLE CASE is where this filter is expected to abstain most
// often, and correctly so: when na and nb are nearly parallel, dir = na × nb
// is near zero, so every fivDot projection carries an interval wide relative
// to its own magnitude, and frac's denominator interval is far more likely to
// straddle zero. Both drive triSpanOnLine to its abstain return, sending the
// pair to the exact predicate — costing one rational classification, never a
// wrong answer.
func triTriMissesFilter(ta, tb [3]r3.Vec, na, nb r3.Vec, sa, sb [3]int) bool {
	a := [3]fivVec{fivVecOf(ta[0]), fivVecOf(ta[1]), fivVecOf(ta[2])}
	b := [3]fivVec{fivVecOf(tb[0]), fivVecOf(tb[1]), fivVecOf(tb[2])}
	dir := fivCross(
		fivVec{fivRounded(na.X), fivRounded(na.Y), fivRounded(na.Z)},
		fivVec{fivRounded(nb.X), fivRounded(nb.Y), fivRounded(nb.Z)},
	)

	spanA, ok := triSpanOnLine(a, b, sa, dir)
	if !ok {
		return false
	}
	spanB, ok := triSpanOnLine(b, a, sb, dir)
	if !ok {
		return false
	}
	return spanA.disjoint(spanB)
}

// xp2 is an exact 2D point (a plane projection of an xpt).
type xp2 struct{ u, v *big.Rat }

// key2 is the exact 2D identity.
func (p xp2) key2() string { return p.u.RatString() + "|" + p.v.RatString() }

// cross2x is the exact value of (b − a) × (c − a): positive when a, b, c turn
// counter-clockwise.
//
// The exact result remains available for callers that need more than its sign.
func cross2x(a, b, c xp2) *big.Rat {
	bu := new(big.Rat).Sub(b.u, a.u)
	bv := new(big.Rat).Sub(b.v, a.v)
	cu := new(big.Rat).Sub(c.u, a.u)
	cv := new(big.Rat).Sub(c.v, a.v)
	return new(big.Rat).Sub(new(big.Rat).Mul(bu, cv), new(big.Rat).Mul(bv, cu))
}

// cross2xSign uses a conservative float filter for the common case and keeps
// the exact rational predicate for values close enough to zero that rounding
// could change the answer. The scale uses the original coordinates, not only
// their float differences, so cancellation during subtraction is covered too.
func cross2xSign(a, b, c xp2) int {
	au, _ := a.u.Float64()
	av, _ := a.v.Float64()
	bu, _ := b.u.Float64()
	bv, _ := b.v.Float64()
	cu, _ := c.u.Float64()
	cv, _ := c.v.Float64()
	if !math.IsNaN(au) && !math.IsInf(au, 0) &&
		!math.IsNaN(av) && !math.IsInf(av, 0) &&
		!math.IsNaN(bu) && !math.IsInf(bu, 0) &&
		!math.IsNaN(bv) && !math.IsInf(bv, 0) &&
		!math.IsNaN(cu) && !math.IsInf(cu, 0) &&
		!math.IsNaN(cv) && !math.IsInf(cv, 0) {
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
	return cross2x(a, b, c).Sign()
}

// pointInTriX reports whether p lies inside or on the closed triangle a, b,
// c, whichever way it is wound — the exact analog of pointInTri.
func pointInTriX(p, a, b, c xp2) bool {
	d1 := cross2xSign(a, b, p)
	d2 := cross2xSign(b, c, p)
	d3 := cross2xSign(c, a, p)
	hasNeg := d1 < 0 || d2 < 0 || d3 < 0
	hasPos := d1 > 0 || d2 > 0 || d3 > 0
	return !hasNeg || !hasPos
}

// onSegment2 reports whether p lies on the closed segment (a, b) — collinear
// and within the endpoints. interior additionally excludes the endpoints.
func onSegment2(a, b, p xp2) (bool, bool) {
	if cross2xSign(a, b, p) != 0 {
		return false, false
	}
	// Collinear: order along the dominant axis of the segment.
	du := new(big.Rat).Sub(b.u, a.u)
	dv := new(big.Rat).Sub(b.v, a.v)
	var lo, hi, x *big.Rat
	if du.Sign() != 0 {
		lo, hi, x = a.u, b.u, p.u
	} else if dv.Sign() != 0 {
		lo, hi, x = a.v, b.v, p.v
	} else {
		// A zero-length segment holds only its own point.
		eq := p.u.Cmp(a.u) == 0 && p.v.Cmp(a.v) == 0
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

// pointInPoly2 reports whether p lies strictly inside the simple polygon —
// exact parity along a +u ray with the half-open rule, so a crossing at a
// shared vertex counts exactly once. A p on the boundary reports onBoundary.
func pointInPoly2(budget *workBudget, poly []xp2, p xp2) (bool, bool, error) {
	n := len(poly)
	for i := range n {
		if err := budget.step(); err != nil {
			return false, false, err
		}
		on, _ := onSegment2(poly[i], poly[(i+1)%n], p)
		if on {
			return false, true, nil
		}
	}
	inside := false
	for i := range n {
		if err := budget.step(); err != nil {
			return false, false, err
		}
		a, b := poly[i], poly[(i+1)%n]
		if (a.v.Cmp(p.v) <= 0) == (b.v.Cmp(p.v) <= 0) {
			continue
		}
		// u of the crossing at height p.v: a.u + (p.v−a.v)·(b.u−a.u)/(b.v−a.v).
		t := new(big.Rat).Quo(new(big.Rat).Sub(p.v, a.v), new(big.Rat).Sub(b.v, a.v))
		u := new(big.Rat).Add(a.u, new(big.Rat).Mul(t, new(big.Rat).Sub(b.u, a.u)))
		if u.Cmp(p.u) > 0 {
			inside = !inside
		}
	}
	return inside, false, nil
}

// polyArea2 is twice the exact signed area of the polygon.
func polyArea2(budget *workBudget, poly []xp2) (*big.Rat, error) {
	total := new(big.Rat)
	n := len(poly)
	for i := range n {
		if err := budget.step(); err != nil {
			return nil, err
		}
		a, b := poly[i], poly[(i+1)%n]
		total.Add(total, new(big.Rat).Sub(new(big.Rat).Mul(a.u, b.v), new(big.Rat).Mul(b.u, a.v)))
	}
	return total, nil
}

// earClipX triangulates a weakly-simple counter-clockwise polygon (given as
// indices into pts) by exact ear clipping. Unlike the float cap triangulator
// it NEVER drops a collinear vertex — every input vertex appears in the
// output, which is what keeps a conforming subdivision conforming — so only
// strictly convex, unblocked ears are clipped. A stall means the polygon is
// not weakly simple, which is an internal error, never a wrong mesh.
func earClipX(budget *workBudget, pts []xp2, poly []int) ([][3]int, error) {
	idx := make([]int, len(poly))
	for i, vi := range poly {
		if err := budget.step(); err != nil {
			return nil, err
		}
		idx[i] = vi
	}
	tris := make([][3]int, 0, len(idx))
	for len(idx) > 3 {
		if err := budget.step(); err != nil {
			return nil, err
		}
		n := len(idx)
		clipped := false
		for i := range n {
			if err := budget.step(); err != nil {
				return nil, err
			}
			ia, ib, ic := idx[(i-1+n)%n], idx[i], idx[(i+1)%n]
			if cross2xSign(pts[ia], pts[ib], pts[ic]) <= 0 {
				continue
			}
			blocked, err := earBlockedX(budget, pts, idx, i)
			if err != nil {
				return nil, err
			}
			if blocked {
				continue
			}
			tris = append(tris, [3]int{ia, ib, ic})
			for j := i; j+1 < len(idx); j++ {
				if err := budget.step(); err != nil {
					return nil, err
				}
				idx[j] = idx[j+1]
			}
			idx = idx[:len(idx)-1]
			clipped = true
			break
		}
		if !clipped {
			return nil, fmt.Errorf(`%w: exact ear clipping stalled on a boolean subdivision polygon`, ErrBooleanFailed)
		}
	}
	if cross2xSign(pts[idx[0]], pts[idx[1]], pts[idx[2]]) > 0 {
		tris = append(tris, [3]int{idx[0], idx[1], idx[2]})
	} else if cross2xSign(pts[idx[0]], pts[idx[1]], pts[idx[2]]) < 0 {
		return nil, fmt.Errorf(`%w: a boolean subdivision polygon closed clockwise`, ErrBooleanFailed)
	}
	return tris, nil
}

// earBlockedX reports whether another polygon vertex lies inside the closed
// candidate ear — the exact analog of earBlocked, except that every OTHER
// vertex can block (collinear duplicates included), which is the conservative
// direction.
func earBlockedX(budget *workBudget, pts []xp2, idx []int, i int) (bool, error) {
	n := len(idx)
	ip, in := (i-1+n)%n, (i+1)%n
	a, b, c := pts[idx[ip]], pts[idx[i]], pts[idx[in]]
	for j := range n {
		if err := budget.step(); err != nil {
			return false, err
		}
		if j == ip || j == i || j == in {
			continue
		}
		p := pts[idx[j]]
		if p.u.Cmp(a.u) == 0 && p.v.Cmp(a.v) == 0 {
			continue
		}
		if p.u.Cmp(c.u) == 0 && p.v.Cmp(c.v) == 0 {
			continue
		}
		if pointInTriX(p, a, b, c) {
			return true, nil
		}
	}
	return false, nil
}

// axisRays is the deterministic retry list for the parity test: the six
// axis-aligned directions. axis names the swept coordinate, dir its sense,
// and (u, v) the two projection coordinates.
var axisRays = [6]struct {
	axis, u, v int
	dir        int
}{
	{0, 1, 2, 1}, {0, 1, 2, -1},
	{1, 2, 0, 1}, {1, 2, 0, -1},
	{2, 0, 1, 1}, {2, 0, 1, -1},
}

// coordOf reads one coordinate of a float vertex by axis index.
func coordOf(v r3.Vec, axis int) float64 {
	switch axis {
	case 0:
		return v.X
	case 1:
		return v.Y
	default:
		return v.Z
	}
}

// ratCoordOf materialises one exact coordinate of p as a big.Rat, by axis
// index — the projection into xp2's own (untouched) rational domain. This is
// the one place a homogeneous coordinate pays a normalisation to become a
// value; a sign-only reader wants xIntCoordOf instead.
func ratCoordOf(p xpt, axis int) *big.Rat {
	switch axis {
	case 0:
		return new(big.Rat).SetFrac(p.x, p.w)
	case 1:
		return new(big.Rat).SetFrac(p.y, p.w)
	default:
		return new(big.Rat).SetFrac(p.z, p.w)
	}
}

// xIntCoordOf reads one coordinate's raw homogeneous numerator by axis index,
// with no normalisation at all: since the denominator (p.w) is always
// positive, this integer's sign already IS the coordinate's sign, which is
// what every sign-only reader in this file actually wants.
func xIntCoordOf(p xpt, axis int) *big.Int {
	switch axis {
	case 0:
		return p.x
	case 1:
		return p.y
	default:
		return p.z
	}
}

// meshParity reports, exactly, whether p lies inside the closed float-vertex
// mesh restricted to the given facet subset: the crossing parity of an
// axis-aligned ray. A ray the point's projection meets at a facet's projected
// boundary is ambiguous and the next axis is tried; a p exactly ON a facet is
// onBoundary. All six axes ambiguous is a genuine failure — never a guess.
func meshParityContext(ctx context.Context, p xpt, verts []r3.Vec, tris [][3]int, subset []int) (bool, bool, error) {
	for _, ray := range axisRays {
		crossings := 0
		ambiguous := false
		onBoundary := false
		for i, ti := range subset {
			if i%256 == 0 {
				if err := ctx.Err(); err != nil {
					return false, false, err
				}
			}
			tri := tris[ti]
			a, b, c := verts[tri[0]], verts[tri[1]], verts[tri[2]]
			pa := xp2{ratCoordOf(p, ray.u), ratCoordOf(p, ray.v)}
			qa := xp2{mustRatOf(coordOf(a, ray.u)), mustRatOf(coordOf(a, ray.v))}
			qb := xp2{mustRatOf(coordOf(b, ray.u)), mustRatOf(coordOf(b, ray.v))}
			qc := xp2{mustRatOf(coordOf(c, ray.u)), mustRatOf(coordOf(c, ray.v))}
			s1 := cross2xSign(qa, qb, pa)
			s2 := cross2xSign(qb, qc, pa)
			s3 := cross2xSign(qc, qa, pa)
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
			xa, xb, xc := xptOf(a), xptOf(b), xptOf(c)
			n := xcross(xsub(xb, xa), xsub(xc, xa))
			nAxis := xIntCoordOf(n, ray.axis)
			if nAxis.Sign() == 0 {
				ambiguous = true
				break
			}
			// t = tNum/nAxis decides the crossing; nAxis is already proven
			// nonzero above, so its sign alone tells the division's sign
			// without ever forming the quotient — only t's sign is read, and
			// xdotNum's raw numerator carries that sign with no normalisation
			// anywhere in the chain (docs/evaluator-design.md §9's
			// reject-only discipline extends to never paying for a value
			// nothing but Sign() consumes).
			tNum := xdotNum(xsub(xa, p), n)
			switch s := tNum.Sign() * nAxis.Sign() * ray.dir; {
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
	return false, false, fmt.Errorf(`%w: every parity ray was ambiguous`, ErrBooleanFailed)
}
