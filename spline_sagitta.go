package decad

import (
	"fmt"
	"math"
	"math/big"
)

// This file is docs/spline-design.md §6.2.1: the bound a chord commits against
// a Tier A free-form span, and §6.2.1's shared consumer — a dyadic station
// chain built by MEASURING that bound and bisecting whatever cell misses its
// target (§6.1's "NEVER size a depth from a rate" rule, restated here for a
// target-driven loop rather than §6.1's own fixed-depth one).
//
// Both pieces work in EXACT rational arithmetic end to end; the one rounding
// either of them commits is the final ratSqrtUp (spline_length.go) that turns
// a proven rational squared distance into a published float64 upper bound.
// Neither piece takes a rational (non-unit-weight) span: Tier A is unit-weight
// only (docs/spline-design.md Table F), so a rational span never reaches this
// file — it refuses earlier, at Table R row R10, inside freeformBezierSpans
// (spline_bezier.go). a10-plan.md's risk register item R5 covers this
// narrowing explicitly, and docs/spline-design.md §11 excludes the sagitta
// row from needing a rational fixture for the same reason §6.2.1 gives: the
// control-point-to-chord-segment argument reads no weight or parameterisation
// at all, so it holds unchanged on a rational span even though this file
// never has to prove that itself.

// chordSegmentSquaredDistance is the exact squared distance from point p to
// the closed segment a→(a+bax, a+bay), a CLAMPED rational projection: with
// n = (p−a)·(bax, bay) and d = (bax, bay)·(bax, bay), the closest point on the
// segment sits at parameter t = n/d clamped to [0, 1], and the returned value
// is |p − (a + t·(bax, bay))|².
//
// bax, bay and d are the chord's own vector and its squared length, computed
// ONCE by the caller and shared across every control point of the span the
// chord belongs to — this function never recomputes them, which is what keeps
// one span's whole sagitta reading linear in its control count rather than
// quadratic.
//
// d == 0 means a and the chord's other end coincide, so the segment is a
// single point: there is nothing to clamp, and the general formula's own
// numerator/denominator both vanish together, so the distance is read
// directly as |p−a|². That is not a different quantity from the clamped
// projection — it is what the projection degenerates to algebraically once
// the chord has zero length — so it is stated here only to avoid a division
// by zero, never as a special case of the bound itself.
func chordSegmentSquaredDistance(p, a ratPoint, bax, bay, d *big.Rat) *big.Rat {
	pax := new(big.Rat).Sub(p.u, a.u)
	pay := new(big.Rat).Sub(p.v, a.v)
	if d.Sign() == 0 {
		return new(big.Rat).Add(new(big.Rat).Mul(pax, pax), new(big.Rat).Mul(pay, pay))
	}
	t := new(big.Rat).Quo(new(big.Rat).Add(new(big.Rat).Mul(pax, bax), new(big.Rat).Mul(pay, bay)), d)
	switch {
	case t.Sign() < 0:
		t.SetInt64(0)
	case t.Cmp(big.NewRat(1, 1)) > 0:
		t.SetInt64(1)
	}
	cu := new(big.Rat).Add(a.u, new(big.Rat).Mul(t, bax))
	cv := new(big.Rat).Add(a.v, new(big.Rat).Mul(t, bay))
	du := new(big.Rat).Sub(p.u, cu)
	dv := new(big.Rat).Sub(p.v, cv)
	return new(big.Rat).Add(new(big.Rat).Mul(du, du), new(big.Rat).Mul(dv, dv))
}

// dyadicSpanSagittaUpper is docs/spline-design.md §6.2.1's bound, over the
// dyadic form pairStations' own bisection already holds: the maximum,
// over every one of the span's OWN control points, of that point's exact
// distance to the chord SEGMENT joining the span's first and last control
// point — never to the chord's carrier LINE, and never the parametric
// deviation |C(t) − L(t)|.
//
// §6.2.1 derives why the control-point maximum dominates the curve's own
// departure from its chord: distance to a convex set is a convex function, so
// its maximum over the control hull is attained at a control point (never in
// the hull's interior), and every point the curve passes through is a convex
// combination of those same control points — positive weights included, so
// the argument holds unchanged on a rational span even though this evaluator
// never reaches one (Tier A is unit-weight only; see this file's own header).
// A COLLAPSED span — every control point coincident, so the chord itself is a
// single point — is not a separate case: distance to a one-point convex set
// is convex like any other, so the same maximum-at-a-control-point argument
// applies, and because every control point then equals the chord's own single
// point, the maximum it reports is exactly 0 — that span's true deviation,
// derived from the general formula rather than bolted onto it as a special
// case.
//
// The single rounding is the final ratSqrtUp: the exact rational maximum
// squared distance is rounded OUTWARD once, so the published float64 is an
// over-statement of the true bound, never an understatement. Where that exact
// maximum's root itself runs past the representable float64 range,
// ratSqrtUp's own contract returns +Inf — a valid, if useless, upper bound;
// this function has no error return of its own to refuse with, and a caller
// that needs a decision on that (pairStations, via its own station cap) makes
// it by continuing to bisect until its cap fires rather than by trusting a
// bound this wide.
func dyadicSpanSagittaUpper(s dyadicSpan) float64 {
	n := len(s.points)
	if n == 0 {
		return 0
	}
	a := s.ratPointAt(0)
	b := s.ratPointAt(n - 1)
	bax := new(big.Rat).Sub(b.u, a.u)
	bay := new(big.Rat).Sub(b.v, a.v)
	d := new(big.Rat).Add(new(big.Rat).Mul(bax, bax), new(big.Rat).Mul(bay, bay))

	var maxSq *big.Rat
	for i := range n {
		p := s.ratPointAt(i)
		sq := chordSegmentSquaredDistance(p, a, bax, bay, d)
		if maxSq == nil || sq.Cmp(maxSq) > 0 {
			maxSq = sq
		}
	}
	return ratSqrtUp(maxSq)
}

// spanSagittaUpper is dyadicSpanSagittaUpper's entry point for a caller
// holding a converted bezierSpan rather than an already-split dyadicSpan —
// freeformBezierSpans' own output, before any bisection has run. It converts
// once through dyadicSpanOf (spline_length.go) and reuses the identical
// arithmetic dyadicSpanSagittaUpper runs on every dyadic cell pairStations
// bisects, so the sagitta bound exists in exactly one place regardless of
// which form a caller starts from.
func spanSagittaUpper(span bezierSpan) float64 {
	return dyadicSpanSagittaUpper(dyadicSpanOf(span))
}

// ratPointAt reconstructs split value i's exact rational coordinate:
// numerator / (den · 2^exp), the inverse of the factored form dyadicSpanOf
// and split (spline_length.go) build it in. Every value that form holds is
// exact — a den·2^exp scaling and an integer numerator, never a normalised
// rational — so this reconstruction loses nothing: the ratPoint it returns is
// the value the split produced, to the last bit, and big.Rat.SetFrac copies
// rather than aliases its arguments, so the result shares no storage with the
// dyadicSpan it was read from.
func (s dyadicSpan) ratPointAt(i int) ratPoint {
	p := s.points[i]
	scale := new(big.Int).Lsh(s.den, p.exp)
	return ratPoint{
		u: new(big.Rat).SetFrac(p.u, scale),
		v: new(big.Rat).SetFrac(p.v, scale),
	}
}

// sagittaMeasureCostPerPoint is the constant per-control-point charge behind
// sagittaMeasureCost: chordSegmentSquaredDistance's own clamped projection
// runs a handful of big.Rat multiplications and additions per point (two to
// form p−a, two more for the dot product, one division, two for the clamped
// point, two for the final squared distance) once the chord's own vector and
// squared length are shared across the whole span — the same "small constant
// times the size term" shape freeformBracketCost (spline_length.go) already
// charges for its own per-leaf work.
const sagittaMeasureCostPerPoint = 8

// sagittaMeasureCost is one dyadic cell's own sagitta-measurement charge,
// linear in its control count.
func sagittaMeasureCost(n int) uint64 {
	return costMul(uint64(n), sagittaMeasureCostPerPoint)
}

// sagittaSplitCost is one dyadic cell's own de Casteljau bisection charge:
// dyadicSpan.split (spline_length.go) runs n(n−1)/2 exact dyadicMidpoint
// blends for a span of n control points — the identical shape
// freeformBracketCost already charges as "perSplit" for its own fixed-depth
// bisection in spline_length.go, restated here because a target-driven
// bisection charges it per cell rather than per fixed level.
func sagittaSplitCost(n int) uint64 {
	return costMul(uint64(n), uint64(n-1))
}

// pairStations generates docs/spline-design.md §6.2.1's shared dyadic chord
// station chain for two paired Tier A free-form span chains — the primitive a
// same-kind free-form loft pairing (a10-plan.md Part 3 PR 9) composes per
// paired segment, and the one place the sagitta bound above is turned into an
// actual chord chain rather than a single span's own reading.
//
// Span-count match is the CALLER's gate, not this function's — a10-plan.md
// Part 3 PR 9's own Table S row S17 owns the caller-facing refusal — but it is
// asserted here defensively too, because a mismatched pair has no shared
// parameter domain for the cell scheme below to walk: ErrUnsupported, never a
// panic on an out-of-range index.
//
// THE STATION SET IS SHARED BY CONSTRUCTION, stated here as the reason the two
// returned lists always carry the same length rather than as an assumption
// this function relies on. The parameter domain is span-uniform: span i of an
// m-span chain covers [i/m, (i+1)/m], and a CELL is a dyadic sub-interval of
// one span, represented as one dyadicSpan per side. Every cell is bisected on
// BOTH sides together, at t = 1/2 of that cell, through dyadicSpan.split
// (spline_length.go) — never one side alone — so after any number of
// bisections the two sides still hold the identical set of dyadic cell
// boundaries, and the two station lists this function returns are the same
// length by that construction, not by a count taken afterward.
//
// MEASURE THEN BISECT, NEVER SIZE A DEPTH FROM A RATE — docs/spline-design.md
// §6.1's rule, restated for a loop that (unlike §6.1's own fixed-depth
// bracket) DOES have a target to stop on. Starting at one cell per span, each
// cell's sagitta is measured on BOTH sides (dyadicSpanSagittaUpper above); a
// cell whose max(sagitta0, sagitta1) exceeds target is bisected and both
// halves are measured again, recursively, until every surviving cell meets
// the target or the station cap below refuses.
//
// sagittaUpper is the MEASURED post-subdivision MAXIMUM over every surviving
// cell and both sides — never a sum: a boundary point lies in exactly one
// cell, so only the widest cell's own departure bounds the whole chain.
//
// Every returned station is an EXACT rational point ON the curve, never
// rounded here: dyadicSpan.split is exact midpoint de Casteljau, and a Bézier
// interpolates its own first and last control point exactly, so every dyadic
// cell boundary — including the two ends of the whole chain, taken directly
// from the recorded chains' own first and last control points — is an exact
// point the curve itself passes through. The two returned lists are ordered
// start-to-end in the shared parameter domain, one entry per cell boundary
// (a chain bisected into k cells returns k+1 stations per side, with no
// duplicate at a span join: consecutive spans already share their boundary
// control point exactly, per bezierSpan's own doc comment).
//
// The HARD CAP bounds the total number of dyadic cells this call may examine,
// counting every cell measured whether it is ultimately accepted or split
// further. It reuses maxChordsPerWalk and errTooManyChords (tessellate.go;
// docs/spline-design.md Table R row R8) rather than minting a new ceiling,
// and it is a STRICTER form of that existing cap's own stated meaning ("more
// than N chords on one curve"): every accepted leaf was also examined, so
// bounding the examined count at maxChordsPerWalk can only refuse sooner than
// a cap counting accepted leaves alone would, never later, and the two sides
// share one cell count by construction (above) so there is one ceiling for
// the pair, not one per side. Reaching the cap guarantees termination even
// for a target this evaluator cannot reach at all — a pathological or
// unrepresentable target never spins forever; it refuses.
//
// work0/work1 are the two records' OWN free-form work counters
// (docs/spline-design.md §5.2) — this function mints no counter of its own.
// Side 0's cost charges work0 and side 1's charges work1, independently,
// because the two sides' spans can carry different control counts even on a
// same-kind pairing. Each cell's own measurement cost (sagittaMeasureCost)
// and, where it is bisected, split cost (sagittaSplitCost) are charged before
// the work runs, through the same saturating costMul/costAdd arithmetic every
// other free-form charge in this package uses; an exhausted budget returns
// freeformWork.step's own Table R row R7 refusal unchanged, and a nil counter
// is tolerated exactly as freeformWork.step already tolerates one.
//
// DETERMINISM: the recursion below is a fixed left-to-right, depth-first walk
// over an ordered slice of spans, with no map anywhere on the path from the
// two input chains to the two output lists, so the same two span chains and
// target produce a bit-identical station list — same rationals, same length —
// on every call.
func pairStations(spans0, spans1 []bezierSpan, target float64, work0, work1 *freeformWork) ([]ratPoint, []ratPoint, float64, error) {
	if len(spans0) != len(spans1) {
		return nil, nil, 0, fmt.Errorf(
			`%w: two paired free-form span chains of different length (%d vs %d) share no common dyadic parameter domain`,
			ErrUnsupported, len(spans0), len(spans1),
		)
	}
	if len(spans0) == 0 {
		return nil, nil, 0, fmt.Errorf(`%w: a paired free-form station chain needs at least one span on each side`, ErrDegenerate)
	}

	gen := &sagittaStationWalk{target: target, work0: work0, work1: work1}
	for i := range spans0 {
		if err := gen.walkCell(dyadicSpanOf(spans0[i]), dyadicSpanOf(spans1[i])); err != nil {
			return nil, nil, 0, err
		}
	}
	// The whole chain's own final station is the last span's own last control
	// point on each side, read directly off the ORIGINAL (unsplit) chain
	// rather than the recursion's own bookkeeping: a Bézier interpolates its
	// last control point exactly, and dyadicSpan.split's own "right" half
	// always carries that same point unchanged at every depth (spline_length.go's
	// split leaves right[n-1] = the original last point, untouched by any
	// blend), so reading it here is the identical value the deepest possible
	// recursion would have produced, without walking there.
	last0 := spans0[len(spans0)-1]
	last1 := spans1[len(spans1)-1]
	gen.stations0 = append(gen.stations0, last0[len(last0)-1])
	gen.stations1 = append(gen.stations1, last1[len(last1)-1])
	return gen.stations0, gen.stations1, gen.sagittaUpper, nil
}

// sagittaStationWalk accumulates one pairStations call's own state across its
// recursive cell walk: the two growing station lists, the running sagitta
// maximum, and the shared examined-cell count the hard cap reads.
type sagittaStationWalk struct {
	target               float64
	work0, work1         *freeformWork
	stations0, stations1 []ratPoint
	sagittaUpper         float64
	cells                int
}

// walkCell measures one dyadic cell pair and either accepts it — appending
// its own start station on each side and folding its measured sagitta into
// the running maximum — or bisects it on BOTH sides together and recurses
// left then right, in that order, which is what makes the whole walk
// deterministic and left-to-right (pairStations' own doc comment).
func (g *sagittaStationWalk) walkCell(c0, c1 dyadicSpan) error {
	g.cells++
	if g.cells > maxChordsPerWalk {
		return errTooManyChords
	}
	n0, n1 := len(c0.points), len(c1.points)
	if err := g.work0.step(sagittaMeasureCost(n0)); err != nil {
		return err
	}
	if err := g.work1.step(sagittaMeasureCost(n1)); err != nil {
		return err
	}
	sag0 := dyadicSpanSagittaUpper(c0)
	sag1 := dyadicSpanSagittaUpper(c1)
	if math.Max(sag0, sag1) <= g.target {
		g.stations0 = append(g.stations0, c0.ratPointAt(0))
		g.stations1 = append(g.stations1, c1.ratPointAt(0))
		g.sagittaUpper = math.Max(g.sagittaUpper, math.Max(sag0, sag1))
		return nil
	}

	if err := g.work0.step(sagittaSplitCost(n0)); err != nil {
		return err
	}
	if err := g.work1.step(sagittaSplitCost(n1)); err != nil {
		return err
	}
	left0, right0 := c0.split()
	left1, right1 := c1.split()
	if err := g.walkCell(left0, left1); err != nil {
		return err
	}
	return g.walkCell(right0, right1)
}
