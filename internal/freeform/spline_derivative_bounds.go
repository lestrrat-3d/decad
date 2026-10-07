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
