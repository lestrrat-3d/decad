package freeform

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"

	"github.com/lestrrat-3d/decad/internal/survey2d"
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
// spline_extreme.go's own BernsteinSplit for the fixed-depth subdivision —
// reused rather than forked, per §6.5's own instruction.
//
// What this file does NOT build: the wall edge's `convex` bool itself. §6.5's
// own orientation convention folds this certificate's sign against the loop's
// own walk role (outer counter-clockwise, hole clockwise) — a topology-level
// decision — which is wired in extrude.go's buildLoopSidesAs (§10 P4b, Table R
// row R6 retired), the one evaluator that owns Table R row R6's build path.

// FreeformConvexitySign is one wall edge's fold verdict: whether the chain's
// own curvature keeps one strict sign, or the chain is a straight walk. It is
// the certificate's whole output — never the `convex` bool itself, which a
// caller reads by applying §6.5's orientation convention to this sign.
type FreeformConvexitySign int

const (
	// FreeformConvexityStraight is fold verdict 0: K is identically zero on
	// every live span and no joint turns off the line they lie on. §6.5
	// routes it to evaluator §3's loop-role rule rather than to a sign, and
	// it is the fold's own identity element.
	FreeformConvexityStraight FreeformConvexitySign = iota
	// FreeformConvexityPositive is a chain whose curvature this certificate
	// proves strictly positive throughout, in the chain's own unreversed
	// parameter sense — FreeformWallConvexityContext applies the reversal
	// negation once, at the very end.
	FreeformConvexityPositive
	// FreeformConvexityNegative is FreeformConvexityPositive's mirror.
	FreeformConvexityNegative
)

// FreeformWallConvexityContext is this file's entry point: docs/spline-design.md
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
// fitInterpolated names whether spans came from FitSplineSeg's §5.1.2
// conversion (extrude.go's survey2d.SegmentWalk.fitInterpolated, itself set from
// spline_fit.go's isFitSplineSeg read on the segment walkOf resolved — the
// normalized value form, since walkOf normalizes before freeformWalk ever
// sees it). Every joint INTERIOR to that conversion's chain is verdict 0 BY
// WHERE IT COMES FROM — the same clause that already covers the joint a
// SUBDIVISION creates
// (BernsteinCurvatureSignContext's own comment) — never by this file's cross
// product: §5.1.2's natural-cubic interpolant is C¹ at every active fit point
// by its own definition, so the joint POINT is shared exactly but the cross
// that would fold its sign carries decad's exact rational lift of sketch's own
// ROUNDED float solve for SecondDerivs, not a turn of the curve the record
// names (docs/spline-design.md §6.5, §5.1.2). Reading that rounding noise as
// geometry is exactly what a real fixture measures: an involute gear flank's
// 13 interior joints alternate sign under the cross product though every one
// of its 14 spans independently proves the SAME curvature sign. So when
// fitInterpolated is set, the loop below never calls JointConvexitySign for an
// interior joint — it folds each live span's own verdict directly against the
// running one, with no joint term between them.
//
// The rule is stated over FitSplineSeg's INTERIOR joints alone (§6.5's own
// words: "The rule reaches no further ... it does not generalise to every
// conversion joint"), and the closing joint below is NOT one of those. closed
// and fitInterpolated CAN both hold: freeformWalk's closed is start == end on
// the CONVERTED chain's own endpoints, decided purely by coordinate identity
// and blind to which kind produced the chain, and record.go's own
// FitSplineSeg validation never forbids Fit[0] == Fit[last] — nothing stops a
// caller-built record from closing a fit-spline chain on itself. Where it
// does, that closing joint meets sketch's interpolant at its own two NATURAL
// ends (SecondDerivs is zero at Points[0] and Points[k-1] by the natural
// boundary condition §5.1.2 takes as given), which are solved independently
// of one another and carry no C¹ relationship at all — unlike an interior
// joint, where the SAME tridiagonal solve ties the two adjoining spans'
// tangents together and only its OWN rounding shows up in the cross product.
// So the closing joint is a genuine corner exactly as any other chain's is,
// and folds by the cross product below regardless of fitInterpolated.
//
// work is the RECORD's free-form work counter (docs/spline-design.md §5.2),
// threaded to SpanConvexitySignContext, which charges it — never minted here,
// for the same reason walkOf never mints one: the R7 ceiling bounds one
// record's total free-form work, so a counter minted per certificate would
// hand this pass a fresh full ceiling instead of spending down what the
// record's other free-form passes already charged.
func FreeformWallConvexityContext(ctx context.Context, spans []survey2d.BezierSpan, closed, reversed, fitInterpolated bool, work *FreeformWork) (FreeformConvexitySign, error) {
	verdict := FreeformConvexityStraight
	firstLive, prevLive := -1, -1
	for i, span := range spans {
		if err := ctx.Err(); err != nil {
			return 0, err
		}
		if SpanCollapsed(span) {
			// Table K: a collapsed span has no verdict and no joint of its
			// own — skip it, and the run it belongs to is bridged by the
			// next live span's joint against prevLive below.
			//
			// This scan runs BEFORE the per-span charge below, which a review
			// read as unbounded work outside the budget. It is not: spans
			// reach here only from freeformBezierSpans, whose every arm
			// charges this same record counter per span as it converts —
			// closedSplineBezierSpans a rationalLift plus 4n, splineBezierSpans
			// its quadratic ClampedConversionCost, a fit spline 64 per point —
			// so a chain long enough to make this scan cost anything has
			// already paid more for its own conversion. Measured on the real
			// path: a ClosedSplineSeg of 174000 identical controls is the
			// longest chain FreeformWorkLimit admits at all, its conversion
			// charges 1044000 units and takes 1.80s, and this scan over its
			// spans adds 194ms; 175000 controls refuses at conversion (R7)
			// before reaching here. The loop also polls ctx.Err() per span, so
			// a cancelled context leaves it immediately.
			continue
		}
		sign, err := SpanConvexitySignContext(ctx, span, work)
		if err != nil {
			return 0, err
		}
		if firstLive < 0 {
			firstLive = i
		} else if !fitInterpolated {
			// A genuine C0 joint (or one whose provenance this caller has not
			// marked otherwise): fold its own cross-product verdict in before
			// the span's. fitInterpolated skips this term entirely — the
			// joint above is verdict 0 by construction, not by measurement.
			joint, err := JointConvexitySign(spans[prevLive], span)
			if err != nil {
				return 0, err
			}
			if verdict, err = FoldConvexitySign(verdict, joint); err != nil {
				return 0, err
			}
		}
		if verdict, err = FoldConvexitySign(verdict, sign); err != nil {
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
		return 0, ErrFreeformConvexityNoLiveSpan
	}
	if closed {
		// The closing joint between the last live span and the first is
		// interior to this one wall edge, even when firstLive == prevLive:
		// a run of collapsed spans wrapping all the way around a chain with
		// exactly one live span still pairs that span's own last control
		// edge against its own first, which is the "same run pairs around
		// the closing joint instead" clause of §6.5's collapsed-run rule.
		joint, err := JointConvexitySign(spans[prevLive], spans[firstLive])
		if err != nil {
			return 0, err
		}
		if verdict, err = FoldConvexitySign(verdict, joint); err != nil {
			return 0, err
		}
	}
	if reversed {
		verdict = NegateConvexitySign(verdict)
	}
	return verdict, nil
}

// SpanConvexitySignContext is one live (non-collapsed) span's own verdict:
// §6.5's speed precondition, then its Bernstein-coefficient certificate at
// the span's STATED curvature degree, subdivided on a mixed sign down to the
// fixed depth cap.
//
// The whole certificate's cost is charged FIRST, before RequireSpanSpeedRegularContext
// builds its Sturm chain — §5.2's charge-early rule — so a record whose
// certificate cannot fit the remaining budget refuses before any of that
// chain, or the subdivision below it, allocates.
func SpanConvexitySignContext(ctx context.Context, span survey2d.BezierSpan, work *FreeformWork) (FreeformConvexitySign, error) {
	if err := work.Step(FreeformConvexityCost(len(span))); err != nil {
		return 0, err
	}
	if err := RequireSpanSpeedRegularContext(ctx, span); err != nil {
		return 0, err
	}
	k := RpTrim(CurvatureNumerator(span))
	if len(k) == 0 {
		// K identically zero: C' and C" are parallel across the whole span —
		// the precondition just proved the speed nonzero there — so the span
		// is confined to one straight line. Verdict 0 (Table K).
		return FreeformConvexityStraight, nil
	}
	coeffs := RpToBernstein(k, StatedCurvatureDegree(span))
	return BernsteinCurvatureSignContext(ctx, coeffs, FreeformLengthDepth)
}

// CurvatureNumerator is §6.2's K = u'v" − v'u" for one polynomial span: the
// quantity whose sign IS the curve's signed-curvature sign. It reads the
// shipped exact Bernstein-to-monomial restatement spline_moments.go already
// integrates through (SpanCoordinatePolys, RpFromBernstein), so nothing here
// rounds and nothing forks a second basis conversion.
func CurvatureNumerator(span survey2d.BezierSpan) RatPoly {
	u, v := SpanCoordinatePolys(span)
	du, dv := RpDeriv(u), RpDeriv(v)
	ddu, ddv := RpDeriv(du), RpDeriv(dv)
	return RpSub(RpMul(du, ddv), RpMul(dv, ddu))
}

// StatedCurvatureDegree is §6.5's STATED Bernstein degree for a span of
// degree p (len(span)-1 control-point legs): 2p−3 for p ≥ 2, and the
// degree-0 all-zero form for p = 1, whose formula 2p−3 = −1 names no array
// at all. A degree-1 span's K is always the zero polynomial (its CurvatureNumerator
// returns the empty RatPoly directly, since a 2-point net's second difference
// does not exist), so SpanConvexitySignContext never actually calls
// RpToBernstein with this degree-0 result — it is stated here only so the
// function is total over every degree Table K names, exactly as the section
// itself is.
func StatedCurvatureDegree(span survey2d.BezierSpan) int {
	p := len(span) - 1
	if d := 2*p - 3; d > 0 {
		return d
	}
	return 0
}

// FreeformConvexityCost is the conservative preflight of one span's §6.5
// certificate, charged before RequireSpanSpeedRegularContext's Sturm chain or
// BernsteinCurvatureSignContext's subdivision allocates anything.
//
// The certificate is two passes over the span, and the charge sums their own
// conservative shapes rather than inventing a third: the speed precondition
// isolates roots of a degree-O(p) polynomial by the SAME Sturm-chain engine
// §6.2's directional extreme reduces to, so it is charged at that bracket's
// own preflight, FreeformExtremeCost; and the Bernstein sign check, when
// mixed, subdivides by exact midpoint de Casteljau down to FreeformLengthDepth
// levels over the curvature numerator K's OWN coefficient count — the stated
// degree plus one, never the span's control count — in the exact shape
// FreeformBracketCost's own arc-length subdivision already charges.
func FreeformConvexityCost(controls int) uint64 {
	if controls < 2 {
		return 0
	}
	speed := FreeformExtremeCost(controls)
	// StatedCurvatureDegree's own formula, off the control count directly:
	// 2p-3 for p >= 2 (p = controls-1), and the degree-0 all-zero form below it.
	degree := 0
	if d := 2*(controls-1) - 3; d > 0 {
		degree = d
	}
	kControls := uint64(degree + 1)
	if kControls < 2 {
		return speed
	}
	leaves := uint64(1) << FreeformLengthDepth
	perSplit := CostMul(kControls, kControls-1)
	perLeaf := kControls
	subdivide := CostAdd(CostMul(leaves-1, perSplit), CostMul(leaves, perLeaf))
	return CostAdd(speed, subdivide)
}

// RpToBernstein restates a monomial RatPoly of degree at most `degree` as the
// Bernstein form of exactly that degree — spline_moments.go's RpFromBernstein
// run in reverse. It is the standard degree-preserving (or degree-elevating,
// when p's true degree sits below `degree`) change of basis:
//
//	b_i = Σ_{k=0}^{i} [C(i,k)/C(degree,k)] · a_k
//
// closed under exact rational arithmetic, so nothing rounds. BinomialRat is
// spline_moments.go's own binomial coefficient, reused rather than
// reimplemented.
func RpToBernstein(p RatPoly, degree int) []*big.Rat {
	out := make([]*big.Rat, degree+1)
	for i := range out {
		sum := new(big.Rat)
		for k := 0; k <= i && k < len(p); k++ {
			term := new(big.Rat).Quo(BinomialRat(i, k), BinomialRat(degree, k))
			term.Mul(term, p[k])
			sum.Add(sum, term)
		}
		out[i] = sum
	}
	return out
}

// BernsteinCurvatureSignContext reads a Bernstein coefficient set's signs
// (§6.5's own rule) and, when they are mixed, subdivides by exact midpoint de
// Casteljau — spline_extreme.go's own BernsteinSplit, reused rather than
// forked — down to the fixed depth cap FreeformLengthDepth
// (spline_length.go), the same constant §6.1's own arc-length bracket
// subdivides to. A child still mixed at that depth refuses, Table R row R19.
//
// The joint a split CREATES is known 0 and is never folded explicitly here:
// §6.5 proves the two meeting control edges are the identical vector, so
// FoldConvexitySign(left, right) already reads the correct chain verdict
// without a separate joint term.
func BernsteinCurvatureSignContext(ctx context.Context, coeffs []*big.Rat, depth int) (FreeformConvexitySign, error) {
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
		return FreeformConvexityPositive, nil
	case sawNegative && !sawPositive:
		return FreeformConvexityNegative, nil
	}
	if depth <= 0 {
		return 0, ErrFreeformConvexityDepthCap
	}
	left, right := BernsteinSplit(coeffs, Half)
	leftSign, err := BernsteinCurvatureSignContext(ctx, left, depth-1)
	if err != nil {
		return 0, err
	}
	rightSign, err := BernsteinCurvatureSignContext(ctx, right, depth-1)
	if err != nil {
		return 0, err
	}
	return FoldConvexitySign(leftSign, rightSign)
}

// Half is the fixed midpoint parameter every subdivision below splits at —
// §6.5 states the subdivision is always at the midpoint, never a measured
// target.
var Half = big.NewRat(1, 2)

// RequireSpanSpeedRegularContext is §6.5's own precondition, run ONCE per
// span before a single curvature coefficient is read: S = u'² + v'² must
// have no root on the CLOSED span [0, 1]. sign(K) is the signed curvature's
// own sign only where the speed is nonzero, and K stays perfectly well
// behaved at a cusp, so this has to close before a coefficient is read at
// all.
//
// It reduces to clearance_poly.go's certified root engine exactly as §6.2's
// directional extreme does, rather than forking a second one: a half-open
// (0, 1] Sturm root count (SturmCount) paired with the endpoint value S(0)
// covers the closed span, because a half-open count alone misses a root
// sitting exactly at the span's own start (a net whose first two control
// points coincide).
func RequireSpanSpeedRegularContext(ctx context.Context, span survey2d.BezierSpan) error {
	u, v := SpanCoordinatePolys(span)
	du, dv := RpDeriv(u), RpDeriv(v)
	s := RpAdd(RpMul(du, du), RpMul(dv, dv))
	if RpEval(s, new(big.Rat)).Sign() == 0 {
		return ErrFreeformConvexitySpeedAtStart
	}
	chain, err := SturmChainIntContext(ctx, RpSquareFree(RpTrim(s)))
	if err != nil {
		return err
	}
	if SturmCount(chain, new(big.Rat), big.NewRat(1, 1)) != 0 {
		return ErrFreeformConvexitySpeedInterior
	}
	return nil
}

// SpanCollapsed is §5.1's collapsed span: every control point of the span the
// same point, so the span has no nonzero control edge, no direction, and (per
// Table K) no verdict or joint of its own.
func SpanCollapsed(span survey2d.BezierSpan) bool {
	for i := 1; i < len(span); i++ {
		if span[i].U.Cmp(span[0].U) != 0 || span[i].V.Cmp(span[0].V) != 0 {
			return false
		}
	}
	return true
}

// JointConvexitySign is §6.5's own joint verdict between two consecutive
// live spans: the cross product of the incoming span's LAST nonzero control
// edge with the outgoing span's FIRST. Both are already proven nonzero by
// RequireSpanSpeedRegularContext (S(t_lo) and S(t_hi) are exactly those
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
func JointConvexitySign(incoming, outgoing survey2d.BezierSpan) (FreeformConvexitySign, error) {
	inU, inV := ControlEdgeVector(incoming[len(incoming)-2], incoming[len(incoming)-1])
	outU, outV := ControlEdgeVector(outgoing[0], outgoing[1])
	cross := new(big.Rat).Sub(new(big.Rat).Mul(inU, outV), new(big.Rat).Mul(inV, outU))
	switch cross.Sign() {
	case 1:
		return FreeformConvexityPositive, nil
	case -1:
		return FreeformConvexityNegative, nil
	}
	dot := new(big.Rat).Add(new(big.Rat).Mul(inU, outU), new(big.Rat).Mul(inV, outV))
	if dot.Sign() < 0 {
		return 0, ErrFreeformConvexityReversedJoint
	}
	return FreeformConvexityStraight, nil
}

// ControlEdgeVector is the exact rational vector from one control point to
// the next — the quantity every joint verdict crosses.
func ControlEdgeVector(from, to survey2d.RatPoint) (u, v *big.Rat) {
	return new(big.Rat).Sub(to.U, from.U), new(big.Rat).Sub(to.V, from.V)
}

// FoldConvexitySign is §6.5's own fold: 0 is the identity, a sign folds with
// itself unchanged, and a positive verdict meeting a negative is a curvature
// sign change the chain genuinely has — refuse, R19. It serves every fold
// this file runs: span with span, span with joint, and a subdivided span's
// two children (whose created joint is known 0 and so needs no separate
// term).
func FoldConvexitySign(a, b FreeformConvexitySign) (FreeformConvexitySign, error) {
	switch {
	case a == FreeformConvexityStraight:
		return b, nil
	case b == FreeformConvexityStraight:
		return a, nil
	case a == b:
		return a, nil
	default:
		return 0, ErrFreeformConvexityConflict
	}
}

// NegateConvexitySign applies §6.5's one reversal negation, over the
// UNREVERSED chain's own fold: reversing a span's parameter negates C' and
// leaves C" unchanged, so it negates K. Negating 0 is 0, so a straight walk
// recorded in either sense still takes the loop-role rule, and a refusal is
// never negated — FreeformWallConvexityContext applies this only to a
// completed, non-error verdict.
func NegateConvexitySign(s FreeformConvexitySign) FreeformConvexitySign {
	switch s {
	case FreeformConvexityPositive:
		return FreeformConvexityNegative
	case FreeformConvexityNegative:
		return FreeformConvexityPositive
	default:
		return FreeformConvexityStraight
	}
}

// ErrFreeformConvexitySpeedAtStart and ErrFreeformConvexitySpeedInterior are
// Table R row R19's two speed-precondition causes: S(t_lo) = 0 (a net whose
// first two control points coincide, escaping a half-open root count) and a
// nonzero half-open root count on (0, 1] (an interior or end-of-span cusp).
// Both are ErrUnsupported, never ErrDegenerate — the curve exists, and only
// its curvature sign at a point with no defined tangent is unproven.
var (
	ErrFreeformConvexitySpeedAtStart = fmt.Errorf(
		`%w: a free-form span's speed vanishes at its own start, so no curvature sign covers it there`,
		decaderr.ErrUnsupported,
	)
	ErrFreeformConvexitySpeedInterior = fmt.Errorf(
		`%w: a free-form span's speed is not proven nonzero across its own closed parameter range`,
		decaderr.ErrUnsupported,
	)
)

// ErrFreeformConvexityDepthCap is Table R row R19's subdivision cause: a
// span's Bernstein curvature-sign coefficients are still mixed after
// FreeformLengthDepth levels of exact midpoint subdivision. The mixture is
// not always a hull over-estimate — a curve whose curvature genuinely changes
// sign inside the span never resolves to one sign at any depth — so the cap
// is what stops the recursion rather than a target it is expected to reach.
var ErrFreeformConvexityDepthCap = fmt.Errorf(
	`%w: a free-form span's curvature-sign certificate is still mixed at the fixed subdivision depth cap of %d levels`,
	decaderr.ErrUnsupported, FreeformLengthDepth,
)

// ErrFreeformConvexityReversedJoint is Table R row R19's joint cause: two
// consecutive live spans meet with their tangents pointing opposite ways —
// the walk doubles back on itself, and no curvature sign covers a reversal.
var ErrFreeformConvexityReversedJoint = fmt.Errorf(
	`%w: a free-form wall edge's joint reverses — the walk doubles back on itself there`,
	decaderr.ErrUnsupported,
)

// ErrFreeformConvexityConflict is Table R row R19's fold cause: two proven
// signs disagree, whether across two spans, a span and a joint, or two
// dyadic children — a curvature sign change the chain genuinely has.
var ErrFreeformConvexityConflict = fmt.Errorf(
	`%w: a free-form wall edge's chain has curvature signs that conflict across its spans or joints`,
	decaderr.ErrUnsupported,
)

// ErrFreeformConvexityNoLiveSpan guards a caller defect rather than a
// reachable input: Table K states that a chain whose every span collapses
// never reaches this section at all, refused R14 by §5.1's own zero-length
// rule first. ErrDegenerate matches that upstream reading.
var ErrFreeformConvexityNoLiveSpan = fmt.Errorf(
	`%w: a free-form wall edge's converted chain holds no live span to certify`,
	decaderr.ErrDegenerate,
)
