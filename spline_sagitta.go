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

// bezierSpan reconstructs a dyadicSpan's own control points as a bezierSpan —
// the form spanMatchedDeltaUpper/spanHodographGapUpper read — by calling
// ratPointAt over every index. It is ratPointAt's own doc comment's
// "reconstruction" extended to a whole span rather than one point: every
// value is exact and the result shares no storage with s.
func (s dyadicSpan) bezierSpan() bezierSpan {
	span := make(bezierSpan, len(s.points))
	for i := range s.points {
		span[i] = s.ratPointAt(i)
	}
	return span
}

// This block is the file's cost model. Every charge below is built from ONE
// named per-operation term, and each term is derived by COUNTING the exact
// operations the code it names actually performs — never estimated as "a
// handful". A term is deliberately a slight OVER-count where an operation's
// own cost is not uniform (a normalising big.Rat.SetFrac is charged for its
// GCD and both divisions, not as one unit), because a charge is spent BEFORE
// the work it pays for and only an over-count keeps freeformWork a real upper
// bound.
//
// The two measurements this file charges are separate readings over separate
// code, so they carry separate per-point terms rather than one shared
// constant: the sagitta reading runs chordSegmentSquaredDistance, the
// matched-delta reading runs spanHodographGapUpper, and the two counts differ.

// ratPointReconstructCost is dyadicSpan.ratPointAt's own per-point charge: one
// big.Int.Lsh for the shared scale, then two big.Rat.SetFrac. SetFrac
// NORMALISES — a GCD plus a division of each of numerator and denominator —
// so it is charged 3, making it the heaviest single operation on the per-point
// path, and it is charged here rather than left free as the reconstruction it
// is. 1 + 3 + 3 = 7.
const ratPointReconstructCost = 7

// chordProjectionCost is chordSegmentSquaredDistance's own per-point charge
// plus the caller's running-maximum comparison: 2 Sub for p−a, 2 Mul + 1 Add
// for the dot product, 1 Quo for t, 1 Cmp + 1 SetInt64 for the clamp, 2 Mul +
// 2 Add for the clamped point, 2 Sub for the difference, 2 Mul + 1 Add for the
// squared distance = 17, then 1 Cmp against dyadicSpanSagittaUpper's own
// running maximum. The d == 0 branch runs strictly fewer operations (2 Sub +
// 2 Mul + 1 Add), so this same term bounds a collapsed chord too.
const chordProjectionCost = 18

// hodographGapCost is spanHodographGapUpper's own per-index charge: 3 for hu
// (Sub, Mul, Sub), 3 for hv, 2 Mul + 1 Add for the squared norm, and 1 Cmp
// against its running maximum. It is charged per POINT rather than per index,
// which over-covers the loop's own n−1 indices and absorbs spanChordVector's
// once-per-call 2 Sub inside that slack.
const hodographGapCost = 10

// ratSqrtUpCost is the per-CALL charge for the single outward rounding each
// measurement commits (ratSqrtUp, spline_length.go). It is bounded, not
// open-ended: ratSqrtSeed costs at most 4 (a big.Float SetRat, MantExp and
// Float64), and the directed walk runs at most sqrtAdjustLimit iterations of
// two ratSquare probes (1 floatRat + 1 Mul + 1 Cmp each) plus one Nextafter.
// 4 + 8·7 = 60, charged as 64. It is a per-call term because one ratSqrtUp
// rounds the whole span's selected maximum, never one per point.
const ratSqrtUpCost = 64

// dyadicConversionCostPerPoint is dyadicSpanOf's own per-point charge
// (spline_length.go): two ratLCM folds over the running denominator (a GCD, a
// Quo and a Mul each = 3) and two scaledNumerator scalings (a Quo and a Mul
// each = 2). 6 + 4 = 10. pairStations charges it before converting each input
// span, so the conversion that opens the walk is paid for like every step
// inside it.
const dyadicConversionCostPerPoint = 10

// sagittaMeasureCostPerPoint is one control point's whole sagitta-measurement
// charge: dyadicSpanSagittaUpper reconstructs the point (ratPointReconstructCost)
// and then projects it onto the shared chord (chordProjectionCost). The chord's
// own vector and squared length are computed once per span and shared across
// every point, which is what keeps this charge linear rather than quadratic.
const sagittaMeasureCostPerPoint = ratPointReconstructCost + chordProjectionCost

// matchedDeltaMeasureCostPerPoint is one control point's whole matched-delta
// charge, derived on its OWN code path rather than borrowed from the sagitta's:
// dyadicSpan.bezierSpan reconstructs the point (ratPointReconstructCost) and
// spanHodographGapUpper then reads it (hodographGapCost). spanMatchedDeltaUpper's
// own halving is one exact division by a power of two, absorbed in the same
// slack hodographGapCost's per-point over-count already carries.
const matchedDeltaMeasureCostPerPoint = ratPointReconstructCost + hodographGapCost

// sagittaMeasureCost is one dyadic cell's own sagitta-measurement charge:
// linear in its control count, plus the one outward rounding the reading ends
// with.
func sagittaMeasureCost(n int) uint64 {
	return costAdd(costMul(uint64(n), sagittaMeasureCostPerPoint), ratSqrtUpCost)
}

// matchedDeltaMeasureCost is one dyadic cell's own matched-delta charge, the
// same shape sagittaMeasureCost carries over its own per-point term. The two
// are separate functions because they pay for separate work; neither may stand
// in for the other.
func matchedDeltaMeasureCost(n int) uint64 {
	return costAdd(costMul(uint64(n), matchedDeltaMeasureCostPerPoint), ratSqrtUpCost)
}

// dyadicConversionCost is one input span's own dyadicSpanOf charge, linear in
// its control count.
func dyadicConversionCost(n int) uint64 {
	return costMul(uint64(n), dyadicConversionCostPerPoint)
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
// matchedDelta is bounds.go's cellChordCurveAreaUpper's own matchedDeltaUpper
// obligation (its own doc comment, F1's rule), ONE ENTRY PER SURVIVING CELL,
// in the same left-to-right order the two station lists carry: cell k's own
// entry is max(spanMatchedDeltaUpper(side0's own dyadic sub-span),
// spanMatchedDeltaUpper(side1's own)) — this file's own PARAMETER-MATCHED
// bound under the span-uniform native fraction, measured on the SAME dyadic
// sub-span pairStations already split BOTH sides to when the cell was
// accepted, never sagittaUpper's SET-distance sagitta reused as a stand-in.
// len(matchedDelta) is always len(stations0)-1: one entry per CELL, where
// the two station lists carry one entry per cell BOUNDARY (this function's
// own doc comment, below).
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
// The HARD CAP bounds the number of CHORD CELLS the finished chain carries —
// the accepted leaves this call returns stations for — and never the number of
// cells its recursion happens to visit on the way to them. It reuses
// maxChordsPerWalk and errTooManyChords (tessellate.go; docs/spline-design.md
// Table R row R8) rather than minting a new ceiling, and it binds at exactly
// the count that cap's own message states ("more than N chords on one curve"):
// a walk refuses when, and only when, the chain it is building would carry
// more than maxChordsPerWalk chords. The two sides share one chord count by
// construction (above), so there is one ceiling for the pair, not one per
// side.
//
// The cap is charged AT EACH SPLIT, before the split runs, because a split is
// the only thing that raises the chord count: it replaces one cell of the
// walk's frontier — the cells created but not yet accepted — with two, so
// accepted-plus-frontier, a proven LOWER bound on the finished chain's own
// chord count, grows by exactly one. Refusing when that sum would pass the
// ceiling therefore refuses on the chord count itself, never on a visit tally
// that overstates it. The chain starts with one frontier cell per span, so a
// chain of more spans than the ceiling admits refuses up front, before any
// cell is measured.
//
// TERMINATION rides on that same charge rather than on a separate node or
// depth ceiling. walkCell recurses only after a split, every split raises
// accepted-plus-frontier by one, and that sum starts at the span count and may
// never pass maxChordsPerWalk — so the walk runs at most (maxChordsPerWalk −
// span count) splits, and visits at most twice that many cells, before it
// either finishes or refuses. A target this evaluator
// cannot reach at all — pathological or unrepresentable — never spins forever;
// it refuses.
//
// work0/work1 are the two records' OWN free-form work counters
// (docs/spline-design.md §5.2) — this function mints no counter of its own.
// Side 0's cost charges work0 and side 1's charges work1, independently,
// because the two sides' spans can carry different control counts even on a
// same-kind pairing. Four charges are spent, each BEFORE the work it pays for
// and each derived on its own code path by this file's own cost model: the
// input conversion (dyadicConversionCost), every cell's sagitta measurement
// (sagittaMeasureCost), an accepted cell's matched-delta measurement
// (matchedDeltaMeasureCost, at ACCEPT time only), and a bisected cell's split
// (sagittaSplitCost). They use the same saturating costMul/costAdd arithmetic
// every other free-form charge in this package uses; an exhausted budget
// returns freeformWork.step's own Table R row R7 refusal unchanged, and a nil
// counter is tolerated exactly as freeformWork.step already tolerates one.
//
// DETERMINISM: the recursion below is a fixed left-to-right, depth-first walk
// over an ordered slice of spans, with no map anywhere on the path from the
// two input chains to the two output lists, so the same two span chains and
// target produce a bit-identical station list — same rationals, same length —
// on every call.
func pairStations(spans0, spans1 []bezierSpan, target float64, work0, work1 *freeformWork) ([]ratPoint, []ratPoint, []float64, float64, error) { //nolint:unparam // matchedDelta has no consumer yet in this commit; a10-plan.md Part 3 PR 9's own free-form loft station arm (loft_build.go's loftFreeformCellStations, the very next change in this series) is its first caller.
	if len(spans0) != len(spans1) {
		return nil, nil, nil, 0, fmt.Errorf(
			`%w: two paired free-form span chains of different length (%d vs %d) share no common dyadic parameter domain`,
			ErrUnsupported, len(spans0), len(spans1),
		)
	}
	if len(spans0) == 0 {
		return nil, nil, nil, 0, fmt.Errorf(`%w: a paired free-form station chain needs at least one span on each side`, ErrDegenerate)
	}
	// Every span carries at least one chord even when it needs no bisection at
	// all, so a chain of more spans than the ceiling admits already exceeds the
	// chord count errTooManyChords names.
	if len(spans0) > maxChordsPerWalk {
		return nil, nil, nil, 0, errTooManyChords
	}
	// A span with no control points at all is not a Bézier of any degree — it
	// has no chord and no curve, unlike a COLLAPSED span (every control point
	// coincident, §5.1), which still has a degree and a (single-point) chord.
	// dyadicSpan.split preserves point count at every depth (spline_length.go),
	// so refusing it HERE, before the first dyadicSpanOf conversion, is enough
	// to keep every cell walkCell ever sees at n >= 1: dyadicSpanSagittaUpper's
	// own n==0 guard exists for a caller that reaches it directly (spanSagittaUpper,
	// with no error return of its own to refuse with), not for this walk, whose
	// accept branch unconditionally reads ratPointAt(0) — reachable only because
	// that guard's 0 answer let the cell through, an index-out-of-range panic
	// otherwise, never a wrong Measurement.
	for i := range spans0 {
		if len(spans0[i]) == 0 || len(spans1[i]) == 0 {
			return nil, nil, nil, 0, fmt.Errorf(
				`%w: a free-form span with no control points at index %d has no chord to bisect`,
				ErrDegenerate, i,
			)
		}
	}

	gen := &sagittaStationWalk{target: target, work0: work0, work1: work1, frontier: len(spans0)}
	for i := range spans0 {
		// The dyadicSpanOf conversion that opens the walk is charged like
		// every step inside it: it runs its own exact big.Int arithmetic per
		// control point (dyadicConversionCost), and leaving it free would let
		// a chain of very wide spans do unbounded work before the first cell
		// is ever measured.
		if err := work0.step(dyadicConversionCost(len(spans0[i]))); err != nil {
			return nil, nil, nil, 0, err
		}
		if err := work1.step(dyadicConversionCost(len(spans1[i]))); err != nil {
			return nil, nil, nil, 0, err
		}
		if err := gen.walkCell(dyadicSpanOf(spans0[i]), dyadicSpanOf(spans1[i])); err != nil {
			return nil, nil, nil, 0, err
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
	//
	// COPIED, not aliased: every other station in the two returned lists comes
	// from ratPointAt, whose own doc comment guarantees the *big.Rat it returns
	// shares no storage with the dyadicSpan it was read from. Appending the
	// caller's own last0[len(last0)-1]/last1[len(last1)-1] ratPoint directly
	// would break that guarantee for this ONE station — it would alias the
	// input span's own *big.Rat fields, so a caller mutating a returned station
	// in place would silently corrupt the span it originally passed in.
	last0 := spans0[len(spans0)-1][len(spans0[len(spans0)-1])-1]
	last1 := spans1[len(spans1)-1][len(spans1[len(spans1)-1])-1]
	gen.stations0 = append(gen.stations0, ratPoint{u: new(big.Rat).Set(last0.u), v: new(big.Rat).Set(last0.v)})
	gen.stations1 = append(gen.stations1, ratPoint{u: new(big.Rat).Set(last1.u), v: new(big.Rat).Set(last1.v)})
	return gen.stations0, gen.stations1, gen.matchedDelta, gen.sagittaUpper, nil
}

// sagittaStationWalk accumulates one pairStations call's own state across its
// recursive cell walk: the two growing station lists, the parallel per-cell
// matchedDelta list, the running sagitta maximum, and the two shared counts
// the hard cap reads — chords, the cells already accepted as chords of the
// finished chain, and frontier, the cells created but not yet accepted. Their
// sum is a proven lower bound on the chord count the finished chain carries:
// it starts at one cell per span, holds steady when a cell is accepted, and
// rises by one per split (pairStations' own doc comment).
type sagittaStationWalk struct {
	target               float64
	work0, work1         *freeformWork
	stations0, stations1 []ratPoint
	matchedDelta         []float64
	sagittaUpper         float64
	chords               int
	frontier             int
}

// walkCell measures one dyadic cell pair and either accepts it — appending
// its own start station on each side, folding its measured sagitta into the
// running maximum, and appending its own matchedDeltaUpper obligation
// (bounds.go's cellChordCurveAreaUpper, F1's rule) to matchedDelta — or
// bisects it on BOTH sides together and recurses left then right, in that
// order, which is what makes the whole walk deterministic and left-to-right
// (pairStations' own doc comment).
//
// The matched-delta reading is spanMatchedDeltaUpper of each side's OWN
// accepted dyadic sub-span — a PARAMETER-MATCHED bound under the span-uniform
// native fraction, never sag0/sag1's SET-distance sagitta: the two coincide
// only for a line or a circular arc under its own uniform-angle
// parametrization (bounds.go's own doc comment), neither of which this file
// ever reaches (spanSagittaUpper/spanMatchedDeltaUpper are Tier A free-form
// only). c0.bezierSpan()/c1.bezierSpan() reconstruct the accepted cell's own
// control points exactly (dyadicSpan.ratPointAt, this file's own reading), so
// the matched-delta measurement reads the SAME dyadic sub-span the sagitta
// measurement above already split to, never a re-derived one.
//
// Accepting moves this cell off the frontier and into the chord count, leaving
// their sum unchanged; splitting raises it by one, which is why the cap is
// read here and only here, before the split runs and before its cost is
// charged. Since the recursion below is reachable only through that charged
// split, the same read is what bounds the walk's own depth and breadth
// (pairStations' own doc comment states the termination argument in full).
func (g *sagittaStationWalk) walkCell(c0, c1 dyadicSpan) error {
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
		// The matched-delta charge is its OWN derived cost, never the
		// sagitta's reused: c0.bezierSpan() reconstructs every control point
		// again and spanHodographGapUpper reads a different set of exact
		// operations from chordSegmentSquaredDistance's projection, so the two
		// per-point terms differ and each is counted on its own code path
		// (this file's own cost model).
		if err := g.work0.step(matchedDeltaMeasureCost(n0)); err != nil {
			return err
		}
		if err := g.work1.step(matchedDeltaMeasureCost(n1)); err != nil {
			return err
		}
		md0 := spanMatchedDeltaUpper(c0.bezierSpan())
		md1 := spanMatchedDeltaUpper(c1.bezierSpan())
		g.frontier--
		g.chords++
		g.stations0 = append(g.stations0, c0.ratPointAt(0))
		g.stations1 = append(g.stations1, c1.ratPointAt(0))
		g.matchedDelta = append(g.matchedDelta, math.Max(md0, md1))
		g.sagittaUpper = math.Max(g.sagittaUpper, math.Max(sag0, sag1))
		return nil
	}

	if g.chords+g.frontier+1 > maxChordsPerWalk {
		return errTooManyChords
	}
	if err := g.work0.step(sagittaSplitCost(n0)); err != nil {
		return err
	}
	if err := g.work1.step(sagittaSplitCost(n1)); err != nil {
		return err
	}
	g.frontier++
	left0, right0 := c0.split()
	left1, right1 := c1.split()
	if err := g.walkCell(left0, left1); err != nil {
		return err
	}
	return g.walkCell(right0, right1)
}

// This section is bounds.go's cellChordCurveAreaUpper's matchedDeltaUpper
// obligation (its own doc comment, F1's rule): a PARAMETER-MATCHED bound on
// |curve(s) − chord(s)| at the SAME s, which is a STRONGER, DIFFERENT claim
// than the SET-distance sagitta above. No caller may pass the sagitta where
// this is owed. Every function below is Tier A / polynomial-Bézier only, for
// the identical reason spanSagittaUpper is (this file's own header): a
// rational span never reaches here (Table R row R10 refuses it first).

// spanChordVector returns a Tier A span's own chord vector Δ = P_p − P_0, the
// shared quantity spanHodographGapUpper and spanSpeedUpper each build on.
func spanChordVector(span bezierSpan) (dxU, dxV *big.Rat) {
	a, b := span[0], span[len(span)-1]
	return new(big.Rat).Sub(b.u, a.u), new(big.Rat).Sub(b.v, a.v)
}

// spanChordSquared is the exact squared length of spanChordVector's own Δ.
func spanChordSquared(span bezierSpan) *big.Rat {
	dxU, dxV := spanChordVector(span)
	return new(big.Rat).Add(new(big.Rat).Mul(dxU, dxU), new(big.Rat).Mul(dxV, dxV))
}

// spanHodographGapUpper bounds d = max_t ‖C'(t) − Δ‖ for a Tier A span of
// degree p with chord Δ = P_p − P_0: the velocity C'(t) is itself a Bézier —
// the HODOGRAPH, degree p−1, with Bernstein control points p·(P_{i+1} − P_i)
// (docs/spline-design.md §6.2's direction-cone row already reuses this same
// hull for a different question) — so C'(t) − Δ is the Bézier with control
// points p·(P_{i+1} − P_i) − Δ, and the convex-hull property bounds its norm
// at every t by the largest control point's own norm:
//
//	d = max_i ‖ p·(P_{i+1} − P_i) − Δ ‖
//
// Exact rational throughout; the ONLY rounding is one outward ratSqrtUp of
// the exact squared norm the maximum selects — the same single-rounding
// shape dyadicSpanSagittaUpper already commits, for a different quantity.
//
// A span with fewer than 2 control points has no chord and no hodograph
// (degree < 1), so it reports 0 rather than reading an empty slice — the
// same shape dyadicSpanSagittaUpper's own n==0 guard takes, for the same
// reason. A COLLAPSED span (every control point coincident, §5.1) needs no
// separate case either: Δ is then the zero vector and every hodograph
// coefficient reduces to p·0 − 0 = 0, so d reads exactly 0 — that span's
// true (zero) velocity gap — from the general formula, never bolted on.
func spanHodographGapUpper(span bezierSpan) float64 {
	n := len(span)
	if n < 2 {
		return 0
	}
	dxU, dxV := spanChordVector(span)
	p := big.NewRat(int64(n-1), 1)

	var maxSq *big.Rat
	for i := 0; i+1 < n; i++ {
		hu := new(big.Rat).Sub(span[i+1].u, span[i].u)
		hu.Mul(hu, p)
		hu.Sub(hu, dxU)
		hv := new(big.Rat).Sub(span[i+1].v, span[i].v)
		hv.Mul(hv, p)
		hv.Sub(hv, dxV)
		sq := new(big.Rat).Add(new(big.Rat).Mul(hu, hu), new(big.Rat).Mul(hv, hv))
		if maxSq == nil || sq.Cmp(maxSq) > 0 {
			maxSq = sq
		}
	}
	return ratSqrtUp(maxSq)
}

// spanMatchedDeltaUpper is bounds.go's cellChordCurveAreaUpper
// matchedDeltaUpper obligation for a Tier A span, under the span's own
// NATIVE parameter — the span-uniform fraction t in [0, 1] — and NEVER a
// constant-arc-length one. A caller pairing on constant arc length
// (cellChordCurveAreaUpper's own derivation) must convert, or must not use
// this value directly; it bounds |C(t) − (P_0 + t·Δ)| at the SAME t, not the
// arc-length-matched deviation.
//
// Derivation: g(t) = C(t) − (P_0 + t·Δ) has g(0) = g(1) = 0 (a Bézier
// interpolates its own endpoints exactly) and g'(t) = C'(t) − Δ, so
// ‖g'(t)‖ ≤ d (spanHodographGapUpper). Integrating from either end,
// ‖g(t)‖ ≤ min(t, 1−t)·d ≤ d/2 — the bound reported here, rounded outward.
// Dividing the already-outward-rounded d by the exact power of two 2
// introduces no further rounding of its own; the upRound wrap is defensive,
// matching this package's other bound helpers.
//
// This is a STRONGER and DIFFERENT quantity than spanSagittaUpper's own
// SET-distance sagitta (every curve point sits within the sagitta of SOME
// chord point). Never substitute one for the other: passing the sagitta
// where this parameter-matched bound is owed silently upgrades a
// SET-distance claim into one it was never proven to carry — bounds.go's
// own matchedDeltaUpper doc comment states the rule (F1), and
// TestSpanMatchedDeltaUpperEnclosesWhatTheSagittaMisses pins the
// counterexample: a span whose every control point sits exactly ON its own
// chord segment (sagitta exactly 0) can still carry a large parameter-matched
// deviation, which only this function — never the sagitta — bounds.
func spanMatchedDeltaUpper(span bezierSpan) float64 {
	return upRound(spanHodographGapUpper(span) / 2)
}

// spanSpeedUpper bounds a Tier A span's own tangent speed ‖C'(t)‖ at every t:
// ‖C'(t)‖ = ‖Δ + (C'(t) − Δ)‖ ≤ ‖Δ‖ + d (spanHodographGapUpper), rounded
// outward. It is always at least the span's own chord length ‖Δ‖, since d is
// never negative — which is what cellChordCurveAreaUpper's own tangent-
// magnitude argument (its doc comment's eA bullet: "a chord never exceeds
// the arc it subtends") requires of a caller's arc-length-speed claim: a
// speed bound that could fall below the chord length would understate the
// very quantity that argument depends on staying above it.
//
// A span with fewer than 2 control points has no chord and no hodograph, so
// it reports 0, matching spanHodographGapUpper's own guard.
func spanSpeedUpper(span bezierSpan) float64 {
	if len(span) < 2 {
		return 0
	}
	return absSumUpper(ratSqrtUp(spanChordSquared(span)), spanHodographGapUpper(span))
}
