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
//
// EVERY EXACT-ARITHMETIC PRIMITIVE HERE METERS ITSELF. A primitive takes the
// *freeformWork counter that pays for it, charges its own documented cost as
// its first statement, and returns freeformWork.step's own Table R row R7
// refusal unchanged — having done no work at all — when the counter cannot
// cover it. Nothing above a primitive restates how many times it runs, so the
// multiplicity of a charge IS the number of calls the code makes, and a caller
// that invokes a primitive k times pays k times by construction. The cost
// model below owns the per-operation terms; each closure reads only its own
// operand's shape (its control count, its numerator and denominator bit
// lengths, its dyadic exponent) and never a caller's loop bound.
// spline_sagitta_metering_internal_test.go is the durable guard on that rule:
// it fails if this file runs exact arithmetic anywhere but inside a metered
// primitive's own body.

// --- the cost model ---

// This block is the file's cost model. Every term below is derived by COUNTING
// the exact operations the code it names actually performs — never estimated as
// "a handful". A term is deliberately a slight OVER-count where an operation's
// own cost is not uniform (a normalising big.Rat.SetFrac is charged for its GCD
// and both divisions, not as one unit), because a charge is spent BEFORE the
// work it pays for and only an over-count keeps freeformWork a real upper
// bound.
//
// The terms are OPERATION COUNTS, and an operation count alone is not a bound
// on work: a big.Int call on a value thousands of bits wide costs orders of
// magnitude more machine time than the same call on a machine word. So every
// charge multiplies its operation count by widthUnits of the operand width its
// own value carries, and one charged unit stands for one exact operation on at
// most one 64-bit word rather than for one call of unbounded size.

// widthUnits converts an operand's own bit width into the number of charged
// units ONE exact operation on it costs: one unit per 64-bit word the value
// occupies, and never fewer than one, so a machine-word operand still pays its
// operation count unchanged.
//
// The scaling is LINEAR in the word count, which is exactly the growth of the
// shifts, additions, subtractions, comparisons and copies that dominate this
// file's arithmetic. It TRACKS rather than dominates the super-linear ones (a
// multiplication, and the GCD inside a normalising SetFrac), so a unit is a
// proportional cost signal there rather than a proof of constant cost — which
// is what the counter needs to stop a deep walk from spending a bounded number
// of unbounded operations, the failure a count-only model admits.
func widthUnits(bits int) uint64 {
	if bits <= 0 {
		return 1
	}
	return uint64(1 + bits/64)
}

// ratBitWidth is the widest bit length the given exact rationals carry, across
// every numerator and denominator. A nil operand contributes nothing: it is the
// absent running maximum a fold starts from, never a value with a width.
func ratBitWidth(rs ...*big.Rat) int {
	widest := 0
	for _, r := range rs {
		if r == nil {
			continue
		}
		widest = max(widest, r.Num().BitLen(), r.Denom().BitLen())
	}
	return widest
}

// ratPointReconstructCost is dyadicSpan.ratPointAt's own per-CALL operation
// count: one big.Int.Lsh for the shared scale, then two big.Rat.SetFrac.
// SetFrac NORMALISES — a GCD plus a division of each of numerator and
// denominator — so it is charged 3, making it the heaviest single operation on
// the per-point path, and it is charged rather than left free as the
// reconstruction it is. 1 + 3 + 3 = 7.
const ratPointReconstructCost = 7

// ratPointCopyCost is ratPointCopy's own per-call operation count: two
// big.Rat.Set, one per coordinate.
const ratPointCopyCost = 2

// chordFrameCost is ratChordFrame's own per-call operation count: 2 Sub for the
// chord vector, then 2 Mul + 1 Add for its squared length. 5.
const chordFrameCost = 5

// chordProjectionCost is chordSegmentSquaredDistance's own per-call operation
// count: 2 Sub for p−a, 2 Mul + 1 Add for the dot product, 1 Quo for t, 1 Cmp +
// 1 SetInt64 for the clamp, 2 Mul + 2 Add for the clamped point, 2 Sub for the
// difference, 2 Mul + 1 Add for the squared distance = 17. The d == 0 branch
// runs strictly fewer operations (2 Sub + 2 Mul + 1 Add), so this same term
// bounds a collapsed chord too. The running-maximum comparison is NOT folded in
// here: it is ratRunningMax's own charge, spent by ratRunningMax on its own call.
const chordProjectionCost = 17

// ratCompareCost is ratRunningMax's own per-call operation count: one big.Rat.Cmp.
const ratCompareCost = 1

// chordVectorCost is spanChordVector's own per-call operation count: 2 Sub.
const chordVectorCost = 2

// chordSquaredCost is spanChordSquared's own per-call operation count, over and
// above the chord vector it asks spanChordVector for: 2 Mul + 1 Add.
const chordSquaredCost = 3

// hodographGapCost is spanHodographGapUpper's own per-index operation count: 3
// for hu (Sub, Mul, Sub), 3 for hv, 2 Mul + 1 Add for the squared norm, and 1
// Cmp against its running maximum. It is charged per POINT rather than per
// index, which over-covers the loop's own n−1 indices and absorbs the degree
// rational the loop builds once inside that slack.
const hodographGapCost = 10

// ratQuarterCost is ratQuarterOf's own per-call operation count: 1 big.NewRat
// for the exact one-quarter factor, then a NORMALISING big.Rat.Mul — a GCD plus
// a division of numerator and of denominator, the same 3 ratPointAt charges its
// own normalising SetFrac. 1 + 3 = 4.
const ratQuarterCost = 4

// ratSqrtUpCost is chargedRatSqrtUp's own per-call operation count for the
// single outward rounding each measurement commits (ratSqrtUp,
// spline_length.go). It is bounded, not open-ended: ratSqrtSeed costs at most 4
// (a big.Float SetRat, MantExp and Float64), and the directed walk runs at most
// sqrtAdjustLimit iterations of two ratSquare probes (1 floatRat + 1 Mul + 1
// Cmp each) plus one Nextafter. 4 + 8·7 = 60, charged as 64. It is a per-call
// term because one ratSqrtUp rounds a whole span's selected maximum, never one
// per point.
const ratSqrtUpCost = 64

// dyadicConversionCostPerPoint is dyadicSpanOf's own per-point operation count
// (spline_length.go): two ratLCM folds over the running denominator (a GCD, a
// Quo and a Mul each = 3) and two scaledNumerator scalings (a Quo and a Mul
// each = 2). 6 + 4 = 10.
const dyadicConversionCostPerPoint = 10

// dyadicMidpointOps is one exact dyadicMidpoint blend's own operation count
// (spline_length.go): two alignedSum, each a big.Int.Lsh plus an Add, plus the
// second operand's own Lsh where its shift is nonzero. 2·3 = 6, the branch that
// shifts BOTH numerators, since only an over-count bounds the other.
const dyadicMidpointOps = 6

// dyadicBlendOpsPerPair is dyadicMidpointOps with dyadicSplit's own halving
// folded in: a split of n control points blends n(n−1)/2 de Casteljau pairs, so
// the blend total is n(n−1) times this, and the saturating multiply never has a
// ceiling to divide afterwards.
const dyadicBlendOpsPerPair = dyadicMidpointOps / 2

// dyadicSplitBookkeepingOps is dyadicSpan.split's own per-point operation count
// outside the blends: three slice allocations and one copy of the parent's own
// points, then one append into each half per level. 4.
const dyadicSplitBookkeepingOps = 4

// dyadicSplitOps is the exact big.Int operation count ONE de Casteljau
// bisection of an n-control span performs: n(n−1)/2 midpoint blends at
// dyadicMidpointOps each, plus the split's own allocations, copy and appends.
// dyadicSpan.split (spline_length.go) spends it scaled by its own operand
// width, and it is the only description of that count the metered surface has.
//
// freeformBracketCost (spline_length.go) charges the same bisections in a
// DIFFERENT unit — one per coordinate blend, roughly a third of this — and
// deliberately does not read this closed form. Its own doc comment owns why:
// that ceiling is whole-record and already 91% spent by a shipping record, so
// converting it to this unit would refuse a capability rather than account for
// one.
func dyadicSplitOps(n uint64) uint64 {
	if n < 2 {
		return costMul(dyadicSplitBookkeepingOps, n)
	}
	return costAdd(
		costMul(costMul(n, n-1), dyadicBlendOpsPerPair),
		costMul(dyadicSplitBookkeepingOps, n),
	)
}

// dyadicSpanOfCharge is dyadicSpanOf's own charge (spline_length.go), read off
// the span it is handed and nothing else. The width is an upper bound on the
// operands the conversion actually builds: the running least common multiple's
// bit length is at most the SUM of the denominators folded into it, and
// scaledNumerator then multiplies a numerator by a quotient of that multiple.
func dyadicSpanOfCharge(span bezierSpan) uint64 {
	denBits, numBits := 0, 0
	for _, p := range span {
		denBits += p.u.Denom().BitLen() + p.v.Denom().BitLen()
		numBits = max(numBits, p.u.Num().BitLen(), p.v.Num().BitLen())
	}
	return costMul(
		costMul(dyadicConversionCostPerPoint, uint64(len(span))),
		widthUnits(denBits+numBits),
	)
}

// --- the metered primitives ---

// chordSegmentSquaredDistance is the exact squared distance from point p to
// the closed segment a→(a+bax, a+bay), a CLAMPED rational projection: with
// n = (p−a)·(bax, bay) and d = (bax, bay)·(bax, bay), the closest point on the
// segment sits at parameter t = n/d clamped to [0, 1], and the returned value
// is |p − (a + t·(bax, bay))|².
//
// bax, bay and d are the chord's own vector and its squared length, computed
// ONCE per span by ratChordFrame and shared across every control point of the
// span the chord belongs to — this function never recomputes them, which is what
// keeps one span's whole sagitta reading linear in its control count rather than
// quadratic.
//
// d == 0 means a and the chord's other end coincide, so the segment is a
// single point: there is nothing to clamp, and the general formula's own
// numerator/denominator both vanish together, so the distance is read
// directly as |p−a|². That is not a different quantity from the clamped
// projection — it is what the projection degenerates to algebraically once
// the chord has zero length — so it is stated here only to avoid a division
// by zero, never as a special case of the bound itself.
//
// It charges chordProjectionCost at its own operand width, first, and returns
// having done nothing when the counter cannot cover it.
func chordSegmentSquaredDistance(w *freeformWork, p, a ratPoint, bax, bay, d *big.Rat) (*big.Rat, error) {
	if err := w.step(costMul(chordProjectionCost, widthUnits(ratBitWidth(p.u, p.v, a.u, a.v, bax, bay, d)))); err != nil {
		return nil, err
	}
	pax := new(big.Rat).Sub(p.u, a.u)
	pay := new(big.Rat).Sub(p.v, a.v)
	if d.Sign() == 0 {
		return new(big.Rat).Add(new(big.Rat).Mul(pax, pax), new(big.Rat).Mul(pay, pay)), nil
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
	return new(big.Rat).Add(new(big.Rat).Mul(du, du), new(big.Rat).Mul(dv, dv)), nil
}

// ratChordFrame is the shared chord frame every sagitta reading projects
// against: the vector from a to b and that vector's own exact squared length,
// built ONCE per span so chordSegmentSquaredDistance never rebuilds it per
// point.
//
// It charges chordFrameCost at its own operand width, first.
func ratChordFrame(w *freeformWork, a, b ratPoint) (*big.Rat, *big.Rat, *big.Rat, error) {
	if err := w.step(costMul(chordFrameCost, widthUnits(ratBitWidth(a.u, a.v, b.u, b.v)))); err != nil {
		return nil, nil, nil, err
	}
	bax := new(big.Rat).Sub(b.u, a.u)
	bay := new(big.Rat).Sub(b.v, a.v)
	d := new(big.Rat).Add(new(big.Rat).Mul(bax, bax), new(big.Rat).Mul(bay, bay))
	return bax, bay, d, nil
}

// ratRunningMax folds one candidate into a running exact maximum. A nil running
// maximum is the fold's own start — the candidate wins with no comparison at
// all — and the charge is spent unconditionally anyway, because a charge that
// skipped a branch would make the count depend on the data rather than on the
// call.
//
// It charges ratCompareCost at its own operand width, first.
func ratRunningMax(w *freeformWork, best, candidate *big.Rat) (*big.Rat, error) {
	if err := w.step(costMul(ratCompareCost, widthUnits(ratBitWidth(best, candidate)))); err != nil {
		return nil, err
	}
	if best == nil || candidate.Cmp(best) > 0 {
		return candidate, nil
	}
	return best, nil
}

// ratPointCopy duplicates an exact rational point so the copy shares no storage
// with its source. It is a metered primitive rather than a free convenience
// because a big.Rat.Set of a wide value copies every word of it.
//
// It charges ratPointCopyCost at its own operand width, first.
func ratPointCopy(w *freeformWork, p ratPoint) (ratPoint, error) {
	if err := w.step(costMul(ratPointCopyCost, widthUnits(ratBitWidth(p.u, p.v)))); err != nil {
		return ratPoint{}, err
	}
	return ratPoint{u: new(big.Rat).Set(p.u), v: new(big.Rat).Set(p.v)}, nil
}

// chargedRatSqrtUp is this file's metered entry point for ratSqrtUp
// (spline_length.go), the one outward rounding a free-form bound commits. Every
// reading in this file rounds through it and none calls ratSqrtUp directly, so
// the number of roundings charged is the number performed.
//
// ratSqrtUp itself keeps its unmetered signature for the ANALYTIC readers that
// share it — a prism's arc radius, a revolve's amplitude, a cap band's contour
// (extrude.go, revolve.go, capblend_contour.go, moments.go, loft_moments.go,
// bounds.go) — none of which walks a free-form record and none of which holds a
// freeformWork counter to charge.
//
// It charges ratSqrtUpCost at the radicand's own width, first.
func chargedRatSqrtUp(w *freeformWork, q *big.Rat) (float64, error) {
	if err := w.step(costMul(ratSqrtUpCost, widthUnits(ratBitWidth(q)))); err != nil {
		return 0, err
	}
	return ratSqrtUp(q), nil
}

// ratPointAt reconstructs split value i's exact rational coordinate:
// numerator / (den · 2^exp), the inverse of the factored form dyadicSpanOf
// and split (spline_length.go) build it in. Every value that form holds is
// exact — a den·2^exp scaling and an integer numerator, never a normalised
// rational — so this reconstruction loses nothing: the ratPoint it returns is
// the value the split produced, to the last bit, and big.Rat.SetFrac copies
// rather than aliases its arguments, so the result shares no storage with the
// dyadicSpan it was read from.
//
// It charges ratPointReconstructCost at value i's OWN width, first — the
// denominator den·2^exp it normalises against, or either numerator, whichever
// is widest — so a reconstruction at depth pays for the wider integers depth
// gave it.
func (s dyadicSpan) ratPointAt(w *freeformWork, i int) (ratPoint, error) {
	p := s.points[i]
	if err := w.step(costMul(ratPointReconstructCost, widthUnits(s.valueWidth(p)))); err != nil {
		return ratPoint{}, err
	}
	scale := new(big.Int).Lsh(s.den, p.exp)
	return ratPoint{
		u: new(big.Rat).SetFrac(p.u, scale),
		v: new(big.Rat).SetFrac(p.v, scale),
	}, nil
}

// bezierSpan reconstructs a dyadicSpan's own control points as a bezierSpan —
// the form spanMatchedDeltaUpper/spanHodographGapUpper read — by calling
// ratPointAt over every index. It is ratPointAt's own doc comment's
// "reconstruction" extended to a whole span rather than one point: every
// value is exact and the result shares no storage with s.
//
// It holds no charge of its own; every unit it spends is ratPointAt's, spent
// once per index it actually reconstructs.
func (s dyadicSpan) bezierSpan(w *freeformWork) (bezierSpan, error) {
	span := make(bezierSpan, len(s.points))
	for i := range s.points {
		p, err := s.ratPointAt(w, i)
		if err != nil {
			return nil, err
		}
		span[i] = p
	}
	return span, nil
}

// --- the sagitta reading ---

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
// The single rounding is the final chargedRatSqrtUp: the exact rational maximum
// squared distance is rounded OUTWARD once, so the published float64 is an
// over-statement of the true bound, never an understatement. Where that exact
// maximum's root itself runs past the representable float64 range, ratSqrtUp's
// own contract returns +Inf — a valid, if useless, upper bound; this function's
// only error is the counter's own refusal, and a caller that needs a decision on
// a bound that wide (pairStations, via its own station cap) makes it by
// continuing to bisect until its cap fires rather than by trusting it.
//
// It holds no aggregate charge of its own. Its cost is exactly the charges its
// own calls spend — the two chord-end reconstructions, the chord frame, then one
// reconstruction, one projection and one comparison per control point, and the
// one outward rounding — so the two chord-end reads that a per-point aggregate
// silently omitted are paid for here by the simple fact that the code makes
// them.
func dyadicSpanSagittaUpper(w *freeformWork, s dyadicSpan) (float64, error) {
	n := len(s.points)
	if n == 0 {
		return 0, nil
	}
	a, err := s.ratPointAt(w, 0)
	if err != nil {
		return 0, err
	}
	b, err := s.ratPointAt(w, n-1)
	if err != nil {
		return 0, err
	}
	bax, bay, d, err := ratChordFrame(w, a, b)
	if err != nil {
		return 0, err
	}

	var maxSq *big.Rat
	for i := range n {
		p, err := s.ratPointAt(w, i)
		if err != nil {
			return 0, err
		}
		sq, err := chordSegmentSquaredDistance(w, p, a, bax, bay, d)
		if err != nil {
			return 0, err
		}
		maxSq, err = ratRunningMax(w, maxSq, sq)
		if err != nil {
			return 0, err
		}
	}
	return chargedRatSqrtUp(w, maxSq)
}

// spanSagittaUpper is dyadicSpanSagittaUpper's entry point for a caller
// holding a converted bezierSpan rather than an already-split dyadicSpan —
// freeformBezierSpans' own output, before any bisection has run. It converts
// once through dyadicSpanOf (spline_length.go) and reuses the identical
// arithmetic dyadicSpanSagittaUpper runs on every dyadic cell pairStations
// bisects, so the sagitta bound exists in exactly one place regardless of
// which form a caller starts from. Both phases charge the counter it is
// handed, each on its own call.
func spanSagittaUpper(w *freeformWork, span bezierSpan) (float64, error) {
	s, err := dyadicSpanOf(w, span)
	if err != nil {
		return 0, err
	}
	return dyadicSpanSagittaUpper(w, s)
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
// Side 0's work charges work0 and side 1's charges work1, independently,
// because the two sides' spans can carry different control counts even on a
// same-kind pairing. NOTHING HERE STATES A COST: every unit spent is spent
// inside the primitive that does the work — the conversion, each
// reconstruction, each projection, each comparison, each outward rounding, each
// split, each final-station copy — so the multiplicity of every charge is the
// number of calls this walk makes and never a number restated at a call site.
// An exhausted budget returns freeformWork.step's own Table R row R7 refusal
// unchanged, and a nil counter is tolerated exactly as freeformWork.step
// already tolerates one.
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
	// whose only error is the counter's), not for this walk, whose accept branch
	// unconditionally reads ratPointAt(0) — reachable only because that guard's 0
	// answer let the cell through, an index-out-of-range panic otherwise, never a
	// wrong Measurement.
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
		// The dyadicSpanOf conversion that opens the walk charges itself, like
		// every step inside it: it runs its own exact big.Int arithmetic per
		// control point, and leaving it free would let a chain of very wide
		// spans do unbounded work before the first cell is ever measured.
		cell0, err := dyadicSpanOf(work0, spans0[i])
		if err != nil {
			return nil, nil, nil, 0, err
		}
		cell1, err := dyadicSpanOf(work1, spans1[i])
		if err != nil {
			return nil, nil, nil, 0, err
		}
		if err := gen.walkCell(cell0, cell1); err != nil {
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
	// in place would silently corrupt the span it originally passed in. The copy
	// runs through ratPointCopy, which charges for it, because copying a wide
	// rational is real work and every other reconstruction in this walk pays.
	last0 := spans0[len(spans0)-1][len(spans0[len(spans0)-1])-1]
	last1 := spans1[len(spans1)-1][len(spans1[len(spans1)-1])-1]
	end0, err := ratPointCopy(work0, last0)
	if err != nil {
		return nil, nil, nil, 0, err
	}
	end1, err := ratPointCopy(work1, last1)
	if err != nil {
		return nil, nil, nil, 0, err
	}
	gen.stations0 = append(gen.stations0, end0)
	gen.stations1 = append(gen.stations1, end1)
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
// read here and only here, before the split runs. Since the recursion below is
// reachable only through that split, the same read is what bounds the walk's own
// depth and breadth (pairStations' own doc comment states the termination
// argument in full).
//
// NO CHARGE IS SPENT HERE. Every measurement, reconstruction and bisection below
// charges its own counter from inside the primitive that performs it, so this
// function never restates what any of them costs or how often it runs.
func (g *sagittaStationWalk) walkCell(c0, c1 dyadicSpan) error {
	sag0, err := dyadicSpanSagittaUpper(g.work0, c0)
	if err != nil {
		return err
	}
	sag1, err := dyadicSpanSagittaUpper(g.work1, c1)
	if err != nil {
		return err
	}
	if math.Max(sag0, sag1) <= g.target {
		span0, err := c0.bezierSpan(g.work0)
		if err != nil {
			return err
		}
		span1, err := c1.bezierSpan(g.work1)
		if err != nil {
			return err
		}
		md0, err := spanMatchedDeltaUpper(g.work0, span0)
		if err != nil {
			return err
		}
		md1, err := spanMatchedDeltaUpper(g.work1, span1)
		if err != nil {
			return err
		}
		start0, err := c0.ratPointAt(g.work0, 0)
		if err != nil {
			return err
		}
		start1, err := c1.ratPointAt(g.work1, 0)
		if err != nil {
			return err
		}
		g.frontier--
		g.chords++
		g.stations0 = append(g.stations0, start0)
		g.stations1 = append(g.stations1, start1)
		g.matchedDelta = append(g.matchedDelta, math.Max(md0, md1))
		g.sagittaUpper = math.Max(g.sagittaUpper, math.Max(sag0, sag1))
		return nil
	}

	if g.chords+g.frontier+1 > maxChordsPerWalk {
		return errTooManyChords
	}
	left0, right0, err := c0.split(g.work0)
	if err != nil {
		return err
	}
	left1, right1, err := c1.split(g.work1)
	if err != nil {
		return err
	}
	g.frontier++
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
// rational span never reaches here (Table R row R10 refuses it first). Each is
// metered on its own call, like every other primitive in this file.

// spanChordVector returns a Tier A span's own chord vector Δ = P_p − P_0, the
// shared quantity spanHodographGapUpper and spanSpeedUpper each build on.
//
// It charges chordVectorCost at its own operand width, first.
func spanChordVector(w *freeformWork, span bezierSpan) (*big.Rat, *big.Rat, error) {
	a, b := span[0], span[len(span)-1]
	if err := w.step(costMul(chordVectorCost, widthUnits(ratBitWidth(a.u, a.v, b.u, b.v)))); err != nil {
		return nil, nil, err
	}
	return new(big.Rat).Sub(b.u, a.u), new(big.Rat).Sub(b.v, a.v), nil
}

// spanChordSquared is the exact squared length of spanChordVector's own Δ.
//
// It charges chordSquaredCost for its own two multiplications and one addition,
// first; the vector it squares is spanChordVector's own charge, spent there.
func spanChordSquared(w *freeformWork, span bezierSpan) (*big.Rat, error) {
	dxU, dxV, err := spanChordVector(w, span)
	if err != nil {
		return nil, err
	}
	if err := w.step(costMul(chordSquaredCost, widthUnits(ratBitWidth(dxU, dxV)))); err != nil {
		return nil, err
	}
	return new(big.Rat).Add(new(big.Rat).Mul(dxU, dxU), new(big.Rat).Mul(dxV, dxV)), nil
}

// spanHodographGapSquared is the exact-rational core both hodograph readings
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
// spanHodographGapUpper roots this value, spanMatchedDeltaUpper roots a quarter
// of it. Neither scales a float another reading already rounded.
//
// A span with fewer than 2 control points has no chord and no hodograph
// (degree < 1), so it reports an exact 0 without charging — the same shape
// dyadicSpanSagittaUpper's own n==0 guard takes, for the same reason. Both
// callers screen that case out first, so the guard is the defensive floor and
// never the path a reading takes. A COLLAPSED span (every control point
// coincident, §5.1) needs no separate case either: Δ is then the zero vector
// and every hodograph coefficient reduces to p·0 − 0 = 0, so d² reads exactly
// 0 — that span's true (zero) velocity gap — from the general formula, never
// bolted on.
//
// It charges hodographGapCost per control point of its OWN span, at its own
// operand width, before the hull scan runs — its control count is the operand's
// shape, never a caller's loop bound — and the chord vector charges itself on
// its own call.
func spanHodographGapSquared(w *freeformWork, span bezierSpan) (*big.Rat, error) {
	n := len(span)
	if n < 2 {
		return new(big.Rat), nil
	}
	dxU, dxV, err := spanChordVector(w, span)
	if err != nil {
		return nil, err
	}
	if err := w.step(costMul(costMul(hodographGapCost, uint64(n)), widthUnits(spanBitWidth(span)))); err != nil {
		return nil, err
	}
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
	return maxSq, nil
}

// ratQuarterOf returns the exact rational q/4, the radicand
// spanMatchedDeltaUpper roots so that its own halving happens over the
// rationals rather than on a published float. big.Rat carries no exponent
// range, so the quotient is exact for every q, however small.
//
// It charges ratQuarterCost at q's own width, first.
func ratQuarterOf(w *freeformWork, q *big.Rat) (*big.Rat, error) {
	if err := w.step(costMul(ratQuarterCost, widthUnits(ratBitWidth(q)))); err != nil {
		return nil, err
	}
	return new(big.Rat).Mul(q, big.NewRat(1, 4)), nil
}

// spanHodographGapUpper bounds d = max_t ‖C'(t) − Δ‖ for a Tier A span: the
// outward square root of spanHodographGapSquared's own exact hull maximum,
//
//	d = max_i ‖ p·(P_{i+1} − P_i) − Δ ‖
//
// The ONLY rounding is that one outward chargedRatSqrtUp — the same
// single-rounding shape dyadicSpanSagittaUpper already commits, for a different
// quantity. A span with fewer than 2 control points has no hodograph at all, so
// it reports 0 without charging, the same reading and the same shape
// spanSpeedUpper's own guard takes.
//
// Beyond that guard it holds no charge of its own; every unit it spends is
// spent by the exact scan and the outward rounding it calls.
func spanHodographGapUpper(w *freeformWork, span bezierSpan) (float64, error) {
	if len(span) < 2 {
		return 0, nil
	}
	maxSq, err := spanHodographGapSquared(w, span)
	if err != nil {
		return 0, err
	}
	return chargedRatSqrtUp(w, maxSq)
}

// spanBitWidth is the widest bit length a converted span's own control
// coordinates carry, the operand width every per-span charge over a bezierSpan
// scales by.
func spanBitWidth(span bezierSpan) int {
	widest := 0
	for _, p := range span {
		widest = max(widest, ratBitWidth(p.u, p.v))
	}
	return widest
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
// ‖g(t)‖ ≤ min(t, 1−t)·d ≤ d/2 — the bound reported here.
//
// The halving is performed over the RATIONALS and not on any published float:
// d/2 is the outward square root of d²/4, so the reading commits exactly one
// outward rounding, on the quantity it publishes. Halving an already-rounded
// float d would be unsound at the bottom of the range — every positive d² at or
// below 2⁻²¹⁴⁸ roots to the smallest subnormal, whose float half underflows to
// +0, and a published 0 states that the deviation is exactly zero. bounds.go's
// cellChordCurveAreaUpper gates its whole chord-to-curve leg on
// matchedDelta > 0, so that 0 would drop a real leg out of a proven allowance
// rather than merely narrow it. big.Rat has no underflow, so d²/4 stays exactly
// positive and the root reports the smallest subnormal, which does bound it.
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
//
// A span with fewer than 2 control points has no hodograph and no deviation to
// bound, so it reports 0 without charging. Beyond that guard it holds no charge
// of its own; every unit it spends is spent by the exact scan, the exact
// quartering and the outward rounding it calls.
func spanMatchedDeltaUpper(w *freeformWork, span bezierSpan) (float64, error) {
	if len(span) < 2 {
		return 0, nil
	}
	maxSq, err := spanHodographGapSquared(w, span)
	if err != nil {
		return 0, err
	}
	quarter, err := ratQuarterOf(w, maxSq)
	if err != nil {
		return 0, err
	}
	return chargedRatSqrtUp(w, quarter)
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
// it reports 0, matching spanHodographGapUpper's own guard. It holds no charge
// of its own; every unit it spends is spent by the three primitives it calls.
func spanSpeedUpper(w *freeformWork, span bezierSpan) (float64, error) {
	if len(span) < 2 {
		return 0, nil
	}
	chordSq, err := spanChordSquared(w, span)
	if err != nil {
		return 0, err
	}
	chord, err := chargedRatSqrtUp(w, chordSq)
	if err != nil {
		return 0, err
	}
	gap, err := spanHodographGapUpper(w, span)
	if err != nil {
		return 0, err
	}
	return absSumUpper(chord, gap), nil
}
