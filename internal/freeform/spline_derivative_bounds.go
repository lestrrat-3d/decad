package freeform

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// This file derives parameter-matched deviation and speed bounds from a Tier A
// Bézier span's hodograph. Its metered primitives share spline_sagitta.go's
// work model.

// ChordVectorCost is SpanChordVector's own per-call operation count: 2 Sub.
const ChordVectorCost = 2

// ChordSquaredCost is SpanChordSquared's own per-call operation count, over and
// above the chord vector it asks SpanChordVector for: 2 Mul + 1 Add.
const ChordSquaredCost = 3

// HodographGapCost is SpanHodographGapUpper's own per-index operation count: 3
// for hu (Sub, Mul, Sub), 3 for hv, 2 Mul + 1 Add for the squared norm, and 1
// Cmp against its running maximum. It is charged per POINT rather than per
// index, which over-covers the loop's own n−1 indices and absorbs the degree
// rational the loop builds once inside that slack.
const HodographGapCost = 10

// RatQuarterCost is RatQuarterOf's own per-call operation count: 1 big.NewRat
// for the exact one-quarter factor, then a NORMALISING big.Rat.Mul — a GCD plus
// a division of numerator and of denominator, the same 3 ratPointAt charges its
// own normalising SetFrac. 1 + 3 = 4.
const RatQuarterCost = 4

// This section is internal/proofbound/bounds.go's proofbound.CellChordCurveAreaUpper's matchedDeltaUpper
// obligation (its own doc comment, F1's rule): a PARAMETER-MATCHED bound on
// |curve(s) − chord(s)| at the SAME s, which is a STRONGER, DIFFERENT claim
// than SpanSagittaUpper's SET-distance sagitta. No caller may pass the sagitta where
// this is owed. Every function below is Tier A / polynomial-Bézier only, for
// the identical reason SpanSagittaUpper is (spline_sagitta.go's header): a
// rational span never reaches here (Table R row R10 refuses it first). Each is
// metered on its own call, like every other primitive in this file.

// SpanChordVector returns a Tier A span's own chord vector Δ = P_p − P_0, the
// shared quantity SpanHodographGapUpper and SpanSpeedUpper each build on.
//
// It charges ChordVectorCost at its own operand width, first.
func SpanChordVector(w *FreeformWork, span BezierSpan) (*big.Rat, *big.Rat, error) {
	a, b := span[0], span[len(span)-1]
	if err := w.Step(CostMul(ChordVectorCost, WidthUnits(RatBitWidth(a.U, a.V, b.U, b.V)))); err != nil {
		return nil, nil, err
	}
	return new(big.Rat).Sub(b.U, a.U), new(big.Rat).Sub(b.V, a.V), nil
}

// SpanChordSquared is the exact squared length of SpanChordVector's own Δ.
//
// It charges ChordSquaredCost for its own two multiplications and one addition,
// first; the vector it squares is SpanChordVector's own charge, spent there.
func SpanChordSquared(w *FreeformWork, span BezierSpan) (*big.Rat, error) {
	dxU, dxV, err := SpanChordVector(w, span)
	if err != nil {
		return nil, err
	}
	if err := w.Step(CostMul(ChordSquaredCost, WidthUnits(RatBitWidth(dxU, dxV)))); err != nil {
		return nil, err
	}
	return new(big.Rat).Add(new(big.Rat).Mul(dxU, dxU), new(big.Rat).Mul(dxV, dxV)), nil
}

// SpanHodographGapSquared is the exact-rational core both hodograph readings
// share: it returns d² = max_i ‖ p·(P_{i+1} − P_i) − Δ ‖², the SQUARED velocity
// gap of a Tier A span of degree p with chord Δ = P_p − P_0, and never rounds.
//
// The velocity C'(t) is itself a Bézier — the HODOGRAPH, degree p−1, with
// Bernstein control points p·(P_{i+1} − P_i) (docs/spline-design.md §6.2's
// direction-cone row already reuses this same hull for a different question) —
// so C'(t) − Δ is the Bézier with control points p·(P_{i+1} − P_i) − Δ, and the
// convex-hull property bounds its norm at every t by the largest control
// point's own norm.
//
// Returning the SQUARE rather than its root is what lets each reading commit
// its own single outward rounding on the quantity it actually publishes:
// SpanHodographGapUpper roots this value, SpanMatchedDeltaUpper roots a quarter
// of it. Neither scales a float another reading already rounded.
//
// A span with fewer than 2 control points has no chord and no hodograph
// (degree < 1), so it reports an exact 0 without charging — the same shape
// DyadicSpanSagittaUpper's own n==0 guard takes, for the same reason. Both
// callers screen that case out first, so the guard is the defensive floor and
// never the path a reading takes. A COLLAPSED span (every control point
// coincident, §5.1) needs no separate case either: Δ is then the zero vector
// and every hodograph coefficient reduces to p·0 − 0 = 0, so d² reads exactly
// 0 — that span's true (zero) velocity gap — from the general formula, never
// bolted on.
//
// It charges HodographGapCost per control point of its OWN span, at its own
// operand width, before the hull scan runs — its control count is the operand's
// shape, never a caller's loop bound — and the chord vector charges itself on
// its own call.
func SpanHodographGapSquared(w *FreeformWork, span BezierSpan) (*big.Rat, error) {
	n := len(span)
	if n < 2 {
		return new(big.Rat), nil
	}
	dxU, dxV, err := SpanChordVector(w, span)
	if err != nil {
		return nil, err
	}
	if err := w.Step(CostMul(CostMul(HodographGapCost, uint64(n)), WidthUnits(SpanBitWidth(span)))); err != nil {
		return nil, err
	}
	p := big.NewRat(int64(n-1), 1)

	var maxSq *big.Rat
	for i := 0; i+1 < n; i++ {
		hu := new(big.Rat).Sub(span[i+1].U, span[i].U)
		hu.Mul(hu, p)
		hu.Sub(hu, dxU)
		hv := new(big.Rat).Sub(span[i+1].V, span[i].V)
		hv.Mul(hv, p)
		hv.Sub(hv, dxV)
		sq := new(big.Rat).Add(new(big.Rat).Mul(hu, hu), new(big.Rat).Mul(hv, hv))
		if maxSq == nil || sq.Cmp(maxSq) > 0 {
			maxSq = sq
		}
	}
	return maxSq, nil
}

// RatQuarterOf returns the exact rational q/4, the radicand
// SpanMatchedDeltaUpper roots so that its own halving happens over the
// rationals rather than on a published float. big.Rat carries no exponent
// range, so the quotient is exact for every q, however small.
//
// It charges RatQuarterCost at q's own width, first.
func RatQuarterOf(w *FreeformWork, q *big.Rat) (*big.Rat, error) {
	if err := w.Step(CostMul(RatQuarterCost, WidthUnits(RatBitWidth(q)))); err != nil {
		return nil, err
	}
	return new(big.Rat).Mul(q, big.NewRat(1, 4)), nil
}

// SpanHodographGapUpper bounds d = max_t ‖C'(t) − Δ‖ for a Tier A span: the
// outward square root of SpanHodographGapSquared's own exact hull maximum,
//
//	d = max_i ‖ p·(P_{i+1} − P_i) − Δ ‖
//
// The ONLY rounding is that one outward ChargedRatSqrtUp — the same
// single-rounding shape DyadicSpanSagittaUpper already commits, for a different
// quantity. A span with fewer than 2 control points has no hodograph at all, so
// it reports 0 without charging, the same reading and the same shape
// SpanSpeedUpper's own guard takes.
//
// Beyond that guard it holds no charge of its own; every unit it spends is
// spent by the exact scan and the outward rounding it calls.
func SpanHodographGapUpper(w *FreeformWork, span BezierSpan) (float64, error) {
	if len(span) < 2 {
		return 0, nil
	}
	maxSq, err := SpanHodographGapSquared(w, span)
	if err != nil {
		return 0, err
	}
	return ChargedRatSqrtUp(w, maxSq)
}

// SpanBitWidth is the widest bit length a converted span's own control
// coordinates carry, the operand width every per-span charge over a BezierSpan
// scales by.
func SpanBitWidth(span BezierSpan) int {
	widest := 0
	for _, p := range span {
		widest = max(widest, RatBitWidth(p.U, p.V))
	}
	return widest
}

// SpanMatchedDeltaUpper is internal/proofbound/bounds.go's proofbound.CellChordCurveAreaUpper
// matchedDeltaUpper obligation for a Tier A span, under the span's own
// NATIVE parameter — the span-uniform fraction t in [0, 1] — and NEVER a
// constant-arc-length one. A caller pairing on constant arc length
// (proofbound.CellChordCurveAreaUpper's own derivation) must convert, or must not use
// this value directly; it bounds |C(t) − (P_0 + t·Δ)| at the SAME t, not the
// arc-length-matched deviation.
//
// Derivation: g(t) = C(t) − (P_0 + t·Δ) has g(0) = g(1) = 0 (a Bézier
// interpolates its own endpoints exactly) and g'(t) = C'(t) − Δ, so
// ‖g'(t)‖ ≤ d (SpanHodographGapUpper). Integrating from either end,
// ‖g(t)‖ ≤ min(t, 1−t)·d ≤ d/2 — the bound reported here.
//
// The halving is performed over the RATIONALS and not on any published float:
// d/2 is the outward square root of d²/4, so the reading commits exactly one
// outward rounding, on the quantity it publishes. Halving an already-rounded
// float d would be unsound at the bottom of the range — every positive d² at or
// below 2⁻²¹⁴⁸ roots to the smallest subnormal, whose float half underflows to
// +0, and a published 0 states that the deviation is exactly zero. internal/proofbound/bounds.go's
// proofbound.CellChordCurveAreaUpper gates its whole chord-to-curve leg on
// matchedDelta > 0, so that 0 would drop a real leg out of a proven allowance
// rather than merely narrow it. big.Rat has no underflow, so d²/4 stays exactly
// positive and the root reports the smallest subnormal, which does bound it.
//
// This is a STRONGER and DIFFERENT quantity than SpanSagittaUpper's own
// SET-distance sagitta (every curve point sits within the sagitta of SOME
// chord point). Never substitute one for the other: passing the sagitta
// where this parameter-matched bound is owed silently upgrades a
// SET-distance claim into one it was never proven to carry — internal/proofbound/bounds.go's
// own matchedDeltaUpper doc comment states the rule (F1), and
// TestSpanMatchedDeltaUpperEnclosesWhatTheSagittaMisses pins the
// counterexample: a span whose every control point sits exactly ON its own
// chord segment (sagitta exactly 0) can still carry a large parameter-matched
// deviation, which only this function — never the sagitta — bounds.
//
// A span with fewer than 2 control points has no hodograph and no deviation to
// bound, so it reports 0 without charging. Beyond that guard it holds no charge
// of its own; every unit it spends is spent by the exact scan, the exact
// quartering and the outward rounding it calls.
func SpanMatchedDeltaUpper(w *FreeformWork, span BezierSpan) (float64, error) {
	if len(span) < 2 {
		return 0, nil
	}
	maxSq, err := SpanHodographGapSquared(w, span)
	if err != nil {
		return 0, err
	}
	quarter, err := RatQuarterOf(w, maxSq)
	if err != nil {
		return 0, err
	}
	return ChargedRatSqrtUp(w, quarter)
}

// SpanSpeedUpper bounds a Tier A span's own tangent speed ‖C'(t)‖ at every t:
// ‖C'(t)‖ = ‖Δ + (C'(t) − Δ)‖ ≤ ‖Δ‖ + d (SpanHodographGapUpper), rounded
// outward. It is always at least the span's own chord length ‖Δ‖, since d is
// never negative — which is what proofbound.CellChordCurveAreaUpper's own tangent-
// magnitude argument (its doc comment's eA bullet: "a chord never exceeds
// the arc it subtends") requires of a caller's arc-length-speed claim: a
// speed bound that could fall below the chord length would understate the
// very quantity that argument depends on staying above it.
//
// A span with fewer than 2 control points has no chord and no hodograph, so
// it reports 0, matching SpanHodographGapUpper's own guard. It holds no charge
// of its own; every unit it spends is spent by the three primitives it calls.
func SpanSpeedUpper(w *FreeformWork, span BezierSpan) (float64, error) {
	if len(span) < 2 {
		return 0, nil
	}
	chordSq, err := SpanChordSquared(w, span)
	if err != nil {
		return 0, err
	}
	chord, err := ChargedRatSqrtUp(w, chordSq)
	if err != nil {
		return 0, err
	}
	gap, err := SpanHodographGapUpper(w, span)
	if err != nil {
		return 0, err
	}
	return proofbound.AbsSumUpper(chord, gap), nil
}

// TangentDeviationCoefficientCost is SpanTangentDeviationCoefficients' own
// per-point operation count: 3 for the u coefficient (Sub, Mul, Sub), 3 for v,
// 2 for the running binomial C(q,i+1) = C(q,i)·(q−i)/(i+1) (one big.Int Mul
// and one Quo), and 2 NORMALISING big.Rat.Mul by that binomial at 3 each.
// 3 + 3 + 2 + 6 = 14. It is charged per POINT rather than per coefficient,
// which over-covers the loop's own n−1 coefficients.
const TangentDeviationCoefficientCost = 14

// TangentEnergyPairCost is BernsteinSquaredNormIntegral's own per-PAIR
// operation count: one dot product of two coefficients folded into its
// product coefficient, 2 normalising Mul and 2 normalising Add at 3 each. 12.
const TangentEnergyPairCost = 12

// TangentEnergyDegreeCost is BernsteinSquaredNormIntegral's own per-product-
// coefficient operation count: the running binomial C(2q,k+1) =
// C(2q,k)·(2q−k)/(k+1) (one big.Int Mul and one Quo), then a normalising Quo
// by C(2q,k) and a normalising Add into the sum, at 3 each. 2 + 3 + 3 = 8.
const TangentEnergyDegreeCost = 8

// TangentEnergyCloseCost is BernsteinSquaredNormIntegral's closing normalising
// Quo by 2q+1. 3.
const TangentEnergyCloseCost = 3

// RatFloatUpCost is ChargedRatFloatUp's own per-call operation count: the
// big.Rat.Float64 conversion, the exact lift of that float back into a
// rational, one Cmp, and at most one Nextafter. 4.
const RatFloatUpCost = 4

// ChargedRatFloatUp is the metered entry point for proofbound.RatFloatUp, the
// one outward rounding a reading commits when the quantity it publishes is the
// exact rational itself rather than its square root.
//
// It charges RatFloatUpCost at the rational's own width, first.
func ChargedRatFloatUp(w *FreeformWork, q *big.Rat) (float64, error) {
	if err := w.Step(CostMul(RatFloatUpCost, WidthUnits(RatBitWidth(q)))); err != nil {
		return 0, err
	}
	return proofbound.RatFloatUp(q), nil
}

// SpanTangentDeviationCoefficients returns the SCALED Bernstein coefficients of
// a Tier A span's tangent deviation e(t) = C'(t) − Δ, Δ = P_p − P_0 the span's
// own chord vector:
//
//	g_i = C(q,i)·(p·(P_{i+1} − P_i) − Δ),   i = 0..q,  q = p − 1,
//
// so that e(t) = Σ g_i·t^i·(1−t)^(q−i). The unscaled coefficients are the ones
// SpanHodographGapSquared bounds: C'(t) is the hodograph Σ p·(P_{i+1} − P_i)·
// B_i^q(t), and Δ = Σ Δ·B_i^q(t) because the Bernstein basis sums to 1. The
// binomial is folded in here so BernsteinSquaredNormIntegral multiplies plain
// monomials t^i·(1−t)^(q−i).
//
// A span with fewer than 2 control points has no chord and no hodograph, so it
// returns no coefficients without charging, SpanHodographGapSquared's own
// guard. Otherwise the chord vector charges itself, then this function charges
// TangentDeviationCoefficientCost per control point at the span's own width
// widened by the control count, which covers the binomial's q+1 bits and the
// degree factor's bits, first.
func SpanTangentDeviationCoefficients(w *FreeformWork, span BezierSpan) ([]RatPoint, error) {
	n := len(span)
	if n < 2 {
		return nil, nil
	}
	dxU, dxV, err := SpanChordVector(w, span)
	if err != nil {
		return nil, err
	}
	if err := w.Step(CostMul(CostMul(TangentDeviationCoefficientCost, uint64(n)), WidthUnits(SpanBitWidth(span)+n))); err != nil {
		return nil, err
	}
	q := n - 2
	p := big.NewRat(int64(n-1), 1)
	binom := big.NewInt(1)
	out := make([]RatPoint, q+1)
	for i := range q + 1 {
		scale := new(big.Rat).SetInt(binom)
		gu := new(big.Rat).Sub(span[i+1].U, span[i].U)
		gu.Mul(gu, p)
		gu.Sub(gu, dxU)
		gu.Mul(gu, scale)
		gv := new(big.Rat).Sub(span[i+1].V, span[i].V)
		gv.Mul(gv, p)
		gv.Sub(gv, dxV)
		gv.Mul(gv, scale)
		out[i] = RatPoint{U: gu, V: gv}
		binom.Mul(binom, big.NewInt(int64(q-i)))
		binom.Quo(binom, big.NewInt(int64(i+1)))
	}
	return out, nil
}

// BernsteinSquaredNormIntegral is the EXACT integral over t in [0, 1] of
// |e(t)|², e(t) = Σ g_i·t^i·(1−t)^(q−i) with the scaled coefficients
// SpanTangentDeviationCoefficients returns. Expanding the square,
//
//	|e(t)|² = Σ_k c_k·t^k·(1−t)^(2q−k),   c_k = Σ_{i+j=k} g_i·g_j,
//
// and the Beta integral ∫ t^k·(1−t)^(2q−k) dt = k!·(2q−k)!/(2q+1)! =
// 1/((2q+1)·C(2q,k)) gives
//
//	∫ |e|² dt = Σ_k c_k / ((2q+1)·C(2q,k)),
//
// a finite sum of products and quotients of exact rationals, so the result is
// the integral itself and never an enclosure of it.
//
// No coefficients means no deviation, an exact 0 without charging. Otherwise it
// charges its whole count first — TangentEnergyPairCost per (i, j) pair,
// TangentEnergyDegreeCost per k, TangentEnergyCloseCost once — at a width of
// twice the widest coefficient (a product's operands) plus 3(q+1) bits. Those
// bits cover the denominator the sum accumulates from the binomials: it
// divides lcm over k of C(2q,k), which is lcm(1..2q+1)/(2q+1), and lcm(1..m)
// is below e^(1.03883·m) (Rosser and Schoenfeld's bound on Chebyshev's ψ), so
// fewer than 1.5·(2q+1) ≤ 3(q+1) bits.
func BernsteinSquaredNormIntegral(w *FreeformWork, g []RatPoint) (*big.Rat, error) {
	if len(g) == 0 {
		return new(big.Rat), nil
	}
	q := len(g) - 1
	width := 0
	for _, c := range g {
		width = max(width, RatBitWidth(c.U, c.V))
	}
	cost := CostAdd(
		CostAdd(
			CostMul(TangentEnergyPairCost, CostMul(uint64(q+1), uint64(q+1))),
			CostMul(TangentEnergyDegreeCost, uint64(2*q+1)),
		),
		TangentEnergyCloseCost,
	)
	if err := w.Step(CostMul(cost, WidthUnits(2*width+3*(q+1)))); err != nil {
		return nil, err
	}
	sum := new(big.Rat)
	binom := big.NewInt(1)
	for k := range 2*q + 1 {
		ck := new(big.Rat)
		for i := max(0, k-q); i <= min(q, k); i++ {
			ck.Add(ck, new(big.Rat).Mul(g[i].U, g[k-i].U))
			ck.Add(ck, new(big.Rat).Mul(g[i].V, g[k-i].V))
		}
		ck.Quo(ck, new(big.Rat).SetInt(binom))
		sum.Add(sum, ck)
		binom.Mul(binom, big.NewInt(int64(2*q-k)))
		binom.Quo(binom, big.NewInt(int64(k+1)))
	}
	return sum.Quo(sum, big.NewRat(int64(2*q+1), 1)), nil
}

// SpanTangentEnergyUpper is docs/loft-design.md §5.2's free-form
// tangentEnergy_k: a proven upper bound on
//
//	J = ∫₀¹ |C'(t) − Δ|² dt
//
// for a Tier A span under its own NATIVE parameter t — the parameter
// SpanSpeedUpper and SpanMatchedDeltaUpper are stated under, and the one a
// same-kind free-form loft cell shares between its two sides (§5.1). It is
// internal/proofbound/bounds.go's proofbound.CellChordCurveAreaAllow
// tangentEnergyUpper obligation, discharged without the constant-speed premise
// proofbound.UniformSpeedTangentEnergyUpper needs: e(t) = C'(t) − Δ is a
// polynomial with exact rational Bernstein coefficients
// (SpanTangentDeviationCoefficients), so J is an exact rational
// (BernsteinSquaredNormIntegral), and its one outward rounding
// (ChargedRatFloatUp) is the only step between it and the published float.
//
// The consumer's sharp arm also needs ∫e dt = 0. That holds under any
// parametrization: ∫C'(t) dt = C(1) − C(0) = Δ, because a Bézier interpolates
// its own end control points.
//
// A span with fewer than 2 control points has no chord, so it reports 0
// without charging. A degree-1 span reports an exact 0 from the general
// formula, since every coefficient p·(P_1 − P_0) − Δ is the zero vector. A J
// past the float64 range rounds to +Inf, which the consumer reads as no energy
// proof and answers with its premise-free arm. The only error is the counter's
// own Table R row R7 refusal.
func SpanTangentEnergyUpper(w *FreeformWork, span BezierSpan) (float64, error) {
	if len(span) < 2 {
		return 0, nil
	}
	g, err := SpanTangentDeviationCoefficients(w, span)
	if err != nil {
		return 0, err
	}
	energy, err := BernsteinSquaredNormIntegral(w, g)
	if err != nil {
		return 0, err
	}
	return ChargedRatFloatUp(w, energy)
}
