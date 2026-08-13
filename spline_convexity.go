package decad

import (
	"context"
	"fmt"
	"math/big"
)

// This file is docs/spline-design.md §6.5: a proof of one wall edge's
// curvature sign from the Bernstein coefficients of the curvature numerator
// K = u'v" − v'u", over the exact polynomial Bézier chain spline_bezier.go's
// §5.1 conversion produces, or a refusal (Table R row R19) where no single
// sign covers the whole chain.
//
// The subject is TOTAL over what that conversion can hand this section: a
// span, a joint between two spans, a run of collapsed spans, a chain that
// closes on itself, and the reversal freeformBezierSpans reports beside its
// spans. Every one of those shapes lands in a stated verdict or a stated
// refusal below (§6.5's Table K), and nothing here samples the curve or
// reads its control polygon's own turns — a sign is proven, never measured
// or estimated.
//
// It reduces to clearance_poly.go's certified Sturm-chain root engine for
// the speed precondition, exactly as §6.2's directional extreme does, and to
// spline_extreme.go's own bernsteinSplit for the fixed-depth subdivision —
// reused rather than forked, per §6.5's own instruction.
//
// What is deliberately NOT built here: the wall edge's `convex` bool. §6.5's
// own orientation convention still has to fold this certificate's sign
// against the loop's own walk role (outer counter-clockwise, hole
// clockwise), which is a topology-level decision belonging to the evaluator
// that reads Table R row R6's build refusal — the one this file does not
// lift.

// freeformConvexitySign is one wall edge's fold verdict: whether the chain's
// own curvature keeps one strict sign, or the chain is a straight walk. It is
// the certificate's whole output — never the `convex` bool itself, which a
// caller reads by applying §6.5's orientation convention to this sign.
type freeformConvexitySign int

const (
	// freeformConvexityStraight is fold verdict 0: K is identically zero on
	// every live span and no joint turns off the line they lie on. §6.5
	// routes it to evaluator §3's loop-role rule rather than to a sign, and
	// it is the fold's own identity element.
	freeformConvexityStraight freeformConvexitySign = iota
	// freeformConvexityPositive is a chain whose curvature this certificate
	// proves strictly positive throughout, in the chain's own unreversed
	// parameter sense — freeformWallConvexityContext applies the reversal
	// negation once, at the very end.
	freeformConvexityPositive
	// freeformConvexityNegative is freeformConvexityPositive's mirror.
	freeformConvexityNegative
)

// freeformWallConvexityContext is this file's entry point: docs/spline-design.md
// §6.5's whole certificate over one converted wall-edge chain.
//
// closed names whether spans came from a chain that closes on itself
// (ClosedSplineSeg) — one wall edge with no free ends, whose closing joint
// between the last live span and the first is INTERIOR to that edge and
// folds like every other. reversed is freeformBezierSpans's own reversal
// report: spans is always the UNREVERSED chain in the UNREVERSED order, so
// every span verdict and every joint below is computed on it before the one
// negation at the end.
//
// work is the RECORD's free-form work counter (docs/spline-design.md §5.2),
// threaded to spanConvexitySignContext, which charges it — never minted here,
// for the same reason walkOf never mints one: the R7 ceiling bounds one
// record's total free-form work, so a counter minted per certificate would
// hand this pass a fresh full ceiling instead of spending down what the
// record's other free-form passes already charged.
func freeformWallConvexityContext(ctx context.Context, spans []bezierSpan, closed, reversed bool, work *freeformWork) (freeformConvexitySign, error) {
	verdict := freeformConvexityStraight
	firstLive, prevLive := -1, -1
	for i, span := range spans {
		if err := ctx.Err(); err != nil {
			return 0, err
		}
		if spanCollapsed(span) {
			// Table K: a collapsed span has no verdict and no joint of its
			// own — skip it, and the run it belongs to is bridged by the
			// next live span's joint against prevLive below.
			//
			// This scan runs BEFORE the per-span charge below, which a review
			// read as unbounded work outside the budget. It is not: spans
			// reach here only from freeformBezierSpans, whose every arm
			// charges this same record counter per span as it converts —
			// closedSplineBezierSpans a rationalLift plus 4n, splineBezierSpans
			// its quadratic clampedConversionCost, a fit spline 64 per point —
			// so a chain long enough to make this scan cost anything has
			// already paid more for its own conversion. Measured on the real
			// path: a ClosedSplineSeg of 174000 identical controls is the
			// longest chain freeformWorkLimit admits at all, its conversion
			// charges 1044000 units and takes 1.80s, and this scan over its
			// spans adds 194ms; 175000 controls refuses at conversion (R7)
			// before reaching here. The loop also polls ctx.Err() per span, so
			// a cancelled context leaves it immediately.
			continue
		}
		sign, err := spanConvexitySignContext(ctx, span, work)
		if err != nil {
			return 0, err
		}
		if firstLive < 0 {
			firstLive = i
		} else {
			joint, err := jointConvexitySign(spans[prevLive], span)
			if err != nil {
				return 0, err
			}
			if verdict, err = foldConvexitySign(verdict, joint); err != nil {
				return 0, err
			}
		}
		if verdict, err = foldConvexitySign(verdict, sign); err != nil {
			return 0, err
		}
		prevLive = i
	}
	if firstLive < 0 {
		// Table K: a chain whose every span collapses never reaches this
		// section — it is a zero-length walk, refused R14 by §5.1's own rule
		// before any wall edge is decided. Reaching here anyway is a caller
		// defect, not a curvature question, so it reads the same sentinel
		// that upstream refusal would have.
		return 0, errFreeformConvexityNoLiveSpan
	}
	if closed {
		// The closing joint between the last live span and the first is
		// interior to this one wall edge, even when firstLive == prevLive:
		// a run of collapsed spans wrapping all the way around a chain with
		// exactly one live span still pairs that span's own last control
		// edge against its own first, which is the "same run pairs around
		// the closing joint instead" clause of §6.5's collapsed-run rule.
		joint, err := jointConvexitySign(spans[prevLive], spans[firstLive])
		if err != nil {
			return 0, err
		}
		if verdict, err = foldConvexitySign(verdict, joint); err != nil {
			return 0, err
		}
	}
	if reversed {
		verdict = negateConvexitySign(verdict)
	}
	return verdict, nil
}

// spanConvexitySignContext is one live (non-collapsed) span's own verdict:
// §6.5's speed precondition, then its Bernstein-coefficient certificate at
// the span's STATED curvature degree, subdivided on a mixed sign down to the
// fixed depth cap.
//
// The whole certificate's cost is charged FIRST, before requireSpanSpeedRegularContext
// builds its Sturm chain — §5.2's charge-early rule — so a record whose
// certificate cannot fit the remaining budget refuses before any of that
// chain, or the subdivision below it, allocates.
func spanConvexitySignContext(ctx context.Context, span bezierSpan, work *freeformWork) (freeformConvexitySign, error) {
	if err := work.step(freeformConvexityCost(len(span))); err != nil {
		return 0, err
	}
	if err := requireSpanSpeedRegularContext(ctx, span); err != nil {
		return 0, err
	}
	k := rpTrim(curvatureNumerator(span))
	if len(k) == 0 {
		// K identically zero: C' and C" are parallel across the whole span —
		// the precondition just proved the speed nonzero there — so the span
		// is confined to one straight line. Verdict 0 (Table K).
		return freeformConvexityStraight, nil
	}
	coeffs := rpToBernstein(k, statedCurvatureDegree(span))
	return bernsteinCurvatureSignContext(ctx, coeffs, freeformLengthDepth)
}

// curvatureNumerator is §6.2's K = u'v" − v'u" for one polynomial span: the
// quantity whose sign IS the curve's signed-curvature sign. It reads the
// shipped exact Bernstein-to-monomial restatement spline_moments.go already
// integrates through (spanCoordinatePolys, rpFromBernstein), so nothing here
// rounds and nothing forks a second basis conversion.
func curvatureNumerator(span bezierSpan) ratPoly {
	u, v := spanCoordinatePolys(span)
	du, dv := rpDeriv(u), rpDeriv(v)
	ddu, ddv := rpDeriv(du), rpDeriv(dv)
	return rpSub(rpMul(du, ddv), rpMul(dv, ddu))
}

// statedCurvatureDegree is §6.5's STATED Bernstein degree for a span of
// degree p (len(span)-1 control-point legs): 2p−3 for p ≥ 2, and the
// degree-0 all-zero form for p = 1, whose formula 2p−3 = −1 names no array
// at all. A degree-1 span's K is always the zero polynomial (its curvatureNumerator
// returns the empty ratPoly directly, since a 2-point net's second difference
// does not exist), so spanConvexitySignContext never actually calls
// rpToBernstein with this degree-0 result — it is stated here only so the
// function is total over every degree Table K names, exactly as the section
// itself is.
func statedCurvatureDegree(span bezierSpan) int {
	p := len(span) - 1
	if d := 2*p - 3; d > 0 {
		return d
	}
	return 0
}

// freeformConvexityCost is the conservative preflight of one span's §6.5
// certificate, charged before requireSpanSpeedRegularContext's Sturm chain or
// bernsteinCurvatureSignContext's subdivision allocates anything.
//
// The certificate is two passes over the span, and the charge sums their own
// conservative shapes rather than inventing a third: the speed precondition
// isolates roots of a degree-O(p) polynomial by the SAME Sturm-chain engine
// §6.2's directional extreme reduces to, so it is charged at that bracket's
// own preflight, freeformExtremeCost; and the Bernstein sign check, when
// mixed, subdivides by exact midpoint de Casteljau down to freeformLengthDepth
// levels over the curvature numerator K's OWN coefficient count — the stated
// degree plus one, never the span's control count — in the exact shape
// freeformBracketCost's own arc-length subdivision already charges.
func freeformConvexityCost(controls int) uint64 {
	if controls < 2 {
		return 0
	}
	speed := freeformExtremeCost(controls)
	// statedCurvatureDegree's own formula, off the control count directly:
	// 2p-3 for p >= 2 (p = controls-1), and the degree-0 all-zero form below it.
	degree := 0
	if d := 2*(controls-1) - 3; d > 0 {
		degree = d
	}
	kControls := uint64(degree + 1)
	if kControls < 2 {
		return speed
	}
	leaves := uint64(1) << freeformLengthDepth
	perSplit := costMul(kControls, kControls-1)
	perLeaf := kControls
	subdivide := costAdd(costMul(leaves-1, perSplit), costMul(leaves, perLeaf))
	return costAdd(speed, subdivide)
}

// rpToBernstein restates a monomial ratPoly of degree at most `degree` as the
// Bernstein form of exactly that degree — spline_moments.go's rpFromBernstein
// run in reverse. It is the standard degree-preserving (or degree-elevating,
// when p's true degree sits below `degree`) change of basis:
//
//	b_i = Σ_{k=0}^{i} [C(i,k)/C(degree,k)] · a_k
//
// closed under exact rational arithmetic, so nothing rounds. binomialRat is
// spline_moments.go's own binomial coefficient, reused rather than
// reimplemented.
func rpToBernstein(p ratPoly, degree int) []*big.Rat {
	out := make([]*big.Rat, degree+1)
	for i := range out {
		sum := new(big.Rat)
		for k := 0; k <= i && k < len(p); k++ {
			term := new(big.Rat).Quo(binomialRat(i, k), binomialRat(degree, k))
			term.Mul(term, p[k])
			sum.Add(sum, term)
		}
		out[i] = sum
	}
	return out
}

// bernsteinCurvatureSignContext reads a Bernstein coefficient set's signs
// (§6.5's own rule) and, when they are mixed, subdivides by exact midpoint de
// Casteljau — spline_extreme.go's own bernsteinSplit, reused rather than
// forked — down to the fixed depth cap freeformLengthDepth
// (spline_length.go), the same constant §6.1's own arc-length bracket
// subdivides to. A child still mixed at that depth refuses, Table R row R19.
//
// The joint a split CREATES is known 0 and is never folded explicitly here:
// §6.5 proves the two meeting control edges are the identical vector, so
// foldConvexitySign(left, right) already reads the correct chain verdict
// without a separate joint term.
func bernsteinCurvatureSignContext(ctx context.Context, coeffs []*big.Rat, depth int) (freeformConvexitySign, error) {
	if err := ctx.Err(); err != nil {
		return 0, err
	}
	sawPositive, sawNegative := false, false
	for _, c := range coeffs {
		switch c.Sign() {
		case 1:
			sawPositive = true
		case -1:
			sawNegative = true
		}
	}
	switch {
	case sawPositive && !sawNegative:
		// The convex-hull property: every coefficient >= 0 with at least one
		// strictly > 0 bounds K >= 0 across the WHOLE span, and K is not the
		// zero polynomial (a genuinely nonzero trimmed K's Bernstein form,
		// at any embedding degree, cannot be all zero). The span never turns
		// the other way.
		return freeformConvexityPositive, nil
	case sawNegative && !sawPositive:
		return freeformConvexityNegative, nil
	}
	if depth <= 0 {
		return 0, errFreeformConvexityDepthCap
	}
	left, right := bernsteinSplit(coeffs, half)
	leftSign, err := bernsteinCurvatureSignContext(ctx, left, depth-1)
	if err != nil {
		return 0, err
	}
	rightSign, err := bernsteinCurvatureSignContext(ctx, right, depth-1)
	if err != nil {
		return 0, err
	}
	return foldConvexitySign(leftSign, rightSign)
}

// half is the fixed midpoint parameter every subdivision below splits at —
// §6.5 states the subdivision is always at the midpoint, never a measured
// target.
var half = big.NewRat(1, 2)

// requireSpanSpeedRegularContext is §6.5's own precondition, run ONCE per
// span before a single curvature coefficient is read: S = u'² + v'² must
// have no root on the CLOSED span [0, 1]. sign(K) is the signed curvature's
// own sign only where the speed is nonzero, and K stays perfectly well
// behaved at a cusp, so this has to close before a coefficient is read at
// all.
//
// It reduces to clearance_poly.go's certified root engine exactly as §6.2's
// directional extreme does, rather than forking a second one: a half-open
// (0, 1] Sturm root count (sturmCount) paired with the endpoint value S(0)
// covers the closed span, because a half-open count alone misses a root
// sitting exactly at the span's own start (a net whose first two control
// points coincide).
func requireSpanSpeedRegularContext(ctx context.Context, span bezierSpan) error {
	u, v := spanCoordinatePolys(span)
	du, dv := rpDeriv(u), rpDeriv(v)
	s := rpAdd(rpMul(du, du), rpMul(dv, dv))
	if rpEval(s, new(big.Rat)).Sign() == 0 {
		return errFreeformConvexitySpeedAtStart
	}
	chain, err := sturmChainIntContext(ctx, rpSquareFree(rpTrim(s)))
	if err != nil {
		return err
	}
	if sturmCount(chain, new(big.Rat), big.NewRat(1, 1)) != 0 {
		return errFreeformConvexitySpeedInterior
	}
	return nil
}

// spanCollapsed is §5.1's collapsed span: every control point of the span the
// same point, so the span has no nonzero control edge, no direction, and (per
// Table K) no verdict or joint of its own.
func spanCollapsed(span bezierSpan) bool {
	for i := 1; i < len(span); i++ {
		if span[i].u.Cmp(span[0].u) != 0 || span[i].v.Cmp(span[0].v) != 0 {
			return false
		}
	}
	return true
}

// jointConvexitySign is §6.5's own joint verdict between two consecutive
// live spans: the cross product of the incoming span's LAST nonzero control
// edge with the outgoing span's FIRST. Both are already proven nonzero by
// requireSpanSpeedRegularContext (S(t_lo) and S(t_hi) are exactly those
// edges' squared lengths, up to the degree factor), so "nonzero" here never
// searches inside an admitted span — it only carries the collapsed-span skip
// the caller already applies.
//
// A positive or negative cross is the joint's own turn, never a tolerance
// call: consecutive spans share the joint point exactly (§5.1) and every
// control coordinate is exact rational. A zero cross with the two edges
// pointing the same way is verdict 0 (parallel tangents turn off no line); a
// zero cross with them pointing OPPOSITE ways is a reversal the walk doubles
// back at, which no curvature sign covers — refuse, R19.
func jointConvexitySign(incoming, outgoing bezierSpan) (freeformConvexitySign, error) {
	inU, inV := controlEdgeVector(incoming[len(incoming)-2], incoming[len(incoming)-1])
	outU, outV := controlEdgeVector(outgoing[0], outgoing[1])
	cross := new(big.Rat).Sub(new(big.Rat).Mul(inU, outV), new(big.Rat).Mul(inV, outU))
	switch cross.Sign() {
	case 1:
		return freeformConvexityPositive, nil
	case -1:
		return freeformConvexityNegative, nil
	}
	dot := new(big.Rat).Add(new(big.Rat).Mul(inU, outU), new(big.Rat).Mul(inV, outV))
	if dot.Sign() < 0 {
		return 0, errFreeformConvexityReversedJoint
	}
	return freeformConvexityStraight, nil
}

// controlEdgeVector is the exact rational vector from one control point to
// the next — the quantity every joint verdict crosses.
func controlEdgeVector(from, to ratPoint) (u, v *big.Rat) {
	return new(big.Rat).Sub(to.u, from.u), new(big.Rat).Sub(to.v, from.v)
}

// foldConvexitySign is §6.5's own fold: 0 is the identity, a sign folds with
// itself unchanged, and a positive verdict meeting a negative is a curvature
// sign change the chain genuinely has — refuse, R19. It serves every fold
// this file runs: span with span, span with joint, and a subdivided span's
// two children (whose created joint is known 0 and so needs no separate
// term).
func foldConvexitySign(a, b freeformConvexitySign) (freeformConvexitySign, error) {
	switch {
	case a == freeformConvexityStraight:
		return b, nil
	case b == freeformConvexityStraight:
		return a, nil
	case a == b:
		return a, nil
	default:
		return 0, errFreeformConvexityConflict
	}
}

// negateConvexitySign applies §6.5's one reversal negation, over the
// UNREVERSED chain's own fold: reversing a span's parameter negates C' and
// leaves C" unchanged, so it negates K. Negating 0 is 0, so a straight walk
// recorded in either sense still takes the loop-role rule, and a refusal is
// never negated — freeformWallConvexityContext applies this only to a
// completed, non-error verdict.
func negateConvexitySign(s freeformConvexitySign) freeformConvexitySign {
	switch s {
	case freeformConvexityPositive:
		return freeformConvexityNegative
	case freeformConvexityNegative:
		return freeformConvexityPositive
	default:
		return freeformConvexityStraight
	}
}

// errFreeformConvexitySpeedAtStart and errFreeformConvexitySpeedInterior are
// Table R row R19's two speed-precondition causes: S(t_lo) = 0 (a net whose
// first two control points coincide, escaping a half-open root count) and a
// nonzero half-open root count on (0, 1] (an interior or end-of-span cusp).
// Both are ErrUnsupported, never ErrDegenerate — the curve exists, and only
// its curvature sign at a point with no defined tangent is unproven.
var (
	errFreeformConvexitySpeedAtStart = fmt.Errorf(
		`%w: a free-form span's speed vanishes at its own start, so no curvature sign covers it there`,
		ErrUnsupported,
	)
	errFreeformConvexitySpeedInterior = fmt.Errorf(
		`%w: a free-form span's speed is not proven nonzero across its own closed parameter range`,
		ErrUnsupported,
	)
)

// errFreeformConvexityDepthCap is Table R row R19's subdivision cause: a
// span's Bernstein curvature-sign coefficients are still mixed after
// freeformLengthDepth levels of exact midpoint subdivision. The mixture is
// not always a hull over-estimate — a curve whose curvature genuinely changes
// sign inside the span never resolves to one sign at any depth — so the cap
// is what stops the recursion rather than a target it is expected to reach.
var errFreeformConvexityDepthCap = fmt.Errorf(
	`%w: a free-form span's curvature-sign certificate is still mixed at the fixed subdivision depth cap of %d levels`,
	ErrUnsupported, freeformLengthDepth,
)

// errFreeformConvexityReversedJoint is Table R row R19's joint cause: two
// consecutive live spans meet with their tangents pointing opposite ways —
// the walk doubles back on itself, and no curvature sign covers a reversal.
var errFreeformConvexityReversedJoint = fmt.Errorf(
	`%w: a free-form wall edge's joint reverses — the walk doubles back on itself there`,
	ErrUnsupported,
)

// errFreeformConvexityConflict is Table R row R19's fold cause: two proven
// signs disagree, whether across two spans, a span and a joint, or two
// dyadic children — a curvature sign change the chain genuinely has.
var errFreeformConvexityConflict = fmt.Errorf(
	`%w: a free-form wall edge's chain has curvature signs that conflict across its spans or joints`,
	ErrUnsupported,
)

// errFreeformConvexityNoLiveSpan guards a caller defect rather than a
// reachable input: Table K states that a chain whose every span collapses
// never reaches this section at all, refused R14 by §5.1's own zero-length
// rule first. ErrDegenerate matches that upstream reading.
var errFreeformConvexityNoLiveSpan = fmt.Errorf(
	`%w: a free-form wall edge's converted chain holds no live span to certify`,
	ErrDegenerate,
)
