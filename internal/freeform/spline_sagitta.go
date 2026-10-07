package freeform

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// This file implements docs/spline-design.md §6.2.1's chord sagitta bound for
// a Tier A free-form span. Its dyadic station consumer is in spline_stations.go.
//
// The bound works in EXACT rational arithmetic; its one rounding is the final
// proofbound.RatSqrtUp (spline_length.go) that turns
// a proven rational squared distance into a published float64 upper bound.
// It does not take a rational (non-unit-weight) span: Tier A is unit-weight
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
// *FreeformWork counter that pays for it, charges its own documented cost as
// its first statement, and returns FreeformWork.step's own Table R row R7
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
// work it pays for and only an over-count keeps FreeformWork a real upper
// bound.
//
// The terms are OPERATION COUNTS, and an operation count alone is not a bound
// on work: a big.Int call on a value thousands of bits wide costs orders of
// magnitude more machine time than the same call on a machine word. So every
// charge multiplies its operation count by WidthUnits of the operand width its
// own value carries, and one charged unit stands for one exact operation on at
// most one 64-bit word rather than for one call of unbounded size.

// WidthUnits converts an operand's own bit width into the number of charged
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
func WidthUnits(bits int) uint64 {
	if bits <= 0 {
		return 1
	}
	return uint64(1 + bits/64)
}

// RatBitWidth is the widest bit length the given exact rationals carry, across
// every numerator and denominator. A nil operand contributes nothing: it is the
// absent running maximum a fold starts from, never a value with a width.
func RatBitWidth(rs ...*big.Rat) int {
	widest := 0
	for _, r := range rs {
		if r == nil {
			continue
		}
		widest = max(widest, r.Num().BitLen(), r.Denom().BitLen())
	}
	return widest
}

// RatPointReconstructCost is DyadicSpan.ratPointAt's own per-CALL operation
// count: one big.Int.Lsh for the shared scale, then two big.Rat.SetFrac.
// SetFrac NORMALISES — a GCD plus a division of each of numerator and
// denominator — so it is charged 3, making it the heaviest single operation on
// the per-point path, and it is charged rather than left free as the
// reconstruction it is. 1 + 3 + 3 = 7.
const RatPointReconstructCost = 7

// RatPointCopyCost is RatPointCopy's own per-call operation count: two
// big.Rat.Set, one per coordinate.
const RatPointCopyCost = 2

// ChordFrameCost is RatChordFrame's own per-call operation count: 2 Sub for the
// chord vector, then 2 Mul + 1 Add for its squared length. 5.
const ChordFrameCost = 5

// ChordProjectionCost is ChordSegmentSquaredDistance's own maximum per-call
// operation count: 2 Sub for p−a, 2 Mul + 1 Add for the dot product, 1 Sign for
// the zero-length check, 1 Sign + 1 Cmp for the interval checks, 2 Sub for p−b,
// and 2 Mul + 1 Add for the squared distance = 13. The collapsed and interior
// branches run fewer operations, so this same term bounds every path. The
// running-maximum comparison is NOT folded in here: it is RatRunningMax's own
// charge, spent by RatRunningMax on its own call.
const ChordProjectionCost = 13

// RatCompareCost is RatRunningMax's own per-call operation count: one big.Rat.Cmp.
const RatCompareCost = 1

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

// RatSqrtUpCost is ChargedRatSqrtUp's own per-call operation count for the
// single outward rounding each measurement commits (proofbound.RatSqrtUp,
// spline_length.go). It is bounded, not open-ended: proofbound.RatSqrtSeed costs at most 4
// (a big.Float SetRat, MantExp and Float64), and the directed walk runs at most
// proofbound.SqrtAdjustLimit iterations of two ratSquare probes (1 floatRat + 1 Mul + 1
// Cmp each) plus one Nextafter. 4 + 8·7 = 60, charged as 64. It is a per-call
// term because one proofbound.RatSqrtUp rounds a whole span's selected maximum, never one
// per point.
const RatSqrtUpCost = 64

// DyadicConversionCostPerPoint is DyadicSpanOf's own per-point operation count
// (spline_length.go): two RatLCM folds over the running denominator (a GCD, a
// Quo and a Mul each = 3) and two ScaledNumerator scalings (a Quo and a Mul
// each = 2). 6 + 4 = 10.
const DyadicConversionCostPerPoint = 10

// DyadicMidpointOps is one exact DyadicMidpoint blend's own operation count
// (spline_length.go): two AlignedSum, each a big.Int.Lsh plus an Add, plus the
// second operand's own Lsh where its shift is nonzero. 2·3 = 6, the branch that
// shifts BOTH numerators, since only an over-count bounds the other.
const DyadicMidpointOps = 6

// DyadicBlendOpsPerPair is DyadicMidpointOps with dyadicSplit's own halving
// folded in: a split of n control points blends n(n−1)/2 de Casteljau pairs, so
// the blend total is n(n−1) times this, and the saturating multiply never has a
// ceiling to divide afterwards.
const DyadicBlendOpsPerPair = DyadicMidpointOps / 2

// DyadicSplitBookkeepingOps is DyadicSpan.split's own per-point operation count
// outside the blends: three slice allocations and one copy of the parent's own
// points, then one append into each half per level. 4.
const DyadicSplitBookkeepingOps = 4

// DyadicSplitOps is the exact big.Int operation count ONE de Casteljau
// bisection of an n-control span performs: n(n−1)/2 midpoint blends at
// DyadicMidpointOps each, plus the split's own allocations, copy and appends.
// DyadicSpan.split (spline_length.go) spends it scaled by its own operand
// width, and it is the only description of that count the metered surface has.
//
// FreeformBracketCost (spline_length.go) charges the same bisections in a
// DIFFERENT unit — one per coordinate blend, roughly a third of this — and
// deliberately does not read this closed form. Its own doc comment owns why:
// that ceiling is whole-record and already 91% spent by a shipping record, so
// converting it to this unit would refuse a capability rather than account for
// one.
func DyadicSplitOps(n uint64) uint64 {
	if n < 2 {
		return CostMul(DyadicSplitBookkeepingOps, n)
	}
	return CostAdd(
		CostMul(CostMul(n, n-1), DyadicBlendOpsPerPair),
		CostMul(DyadicSplitBookkeepingOps, n),
	)
}

// DyadicSpanOfCharge is DyadicSpanOf's own charge (spline_length.go), read off
// the span it is handed and nothing else. The width is an upper bound on the
// operands the conversion actually builds: the running least common multiple's
// bit length is at most the SUM of the denominators folded into it, and
// ScaledNumerator then multiplies a numerator by a quotient of that multiple.
func DyadicSpanOfCharge(span BezierSpan) uint64 {
	denBits, numBits := 0, 0
	for _, p := range span {
		denBits += p.U.Denom().BitLen() + p.V.Denom().BitLen()
		numBits = max(numBits, p.U.Num().BitLen(), p.V.Num().BitLen())
	}
	return CostMul(
		CostMul(DyadicConversionCostPerPoint, uint64(len(span))),
		WidthUnits(denBits+numBits),
	)
}

// --- the metered primitives ---

// ChordSegmentSquaredDistance is the exact squared distance from point p to
// the closed segment a→(a+bax, a+bay). It uses n = (p−a)·(bax, bay) and
// d = (bax, bay)·(bax, bay) to select the nearest endpoint when n is outside
// [0, d]. When n is strictly inside that interval, the returned value is the
// equivalent exact cross-product form cross(p−a, chord)²/d, avoiding a
// rational projection point and its subsequent subtraction.
//
// bax, bay and d are the chord's own vector and its squared length, computed
// ONCE per span by RatChordFrame and shared across every control point of the
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
// It charges the maximum exact-operation path, ChordProjectionCost, at its own
// operand width, first, and returns having done nothing when the counter cannot
// cover it. The charge is fixed per call so the work bound does not depend on
// which exact branch the input takes.
func ChordSegmentSquaredDistance(w *FreeformWork, p, a RatPoint, bax, bay, d *big.Rat) (*big.Rat, error) {
	if err := w.Step(CostMul(ChordProjectionCost, WidthUnits(RatBitWidth(p.U, p.V, a.U, a.V, bax, bay, d)))); err != nil {
		return nil, err
	}
	pax := new(big.Rat).Sub(p.U, a.U)
	pay := new(big.Rat).Sub(p.V, a.V)
	if d.Sign() == 0 {
		return new(big.Rat).Add(new(big.Rat).Mul(pax, pax), new(big.Rat).Mul(pay, pay)), nil
	}
	n := new(big.Rat).Add(new(big.Rat).Mul(pax, bax), new(big.Rat).Mul(pay, bay))
	if n.Sign() < 0 {
		return new(big.Rat).Add(new(big.Rat).Mul(pax, pax), new(big.Rat).Mul(pay, pay)), nil
	}
	if n.Cmp(d) >= 0 {
		pbx := new(big.Rat).Sub(pax, bax)
		pby := new(big.Rat).Sub(pay, bay)
		return new(big.Rat).Add(new(big.Rat).Mul(pbx, pbx), new(big.Rat).Mul(pby, pby)), nil
	}
	cross := new(big.Rat).Sub(new(big.Rat).Mul(pax, bay), new(big.Rat).Mul(pay, bax))
	return new(big.Rat).Quo(new(big.Rat).Mul(cross, cross), d), nil
}

// ChordEndpointSquaredDistance pays the same projection charge at the same
// operand width as ChordSegmentSquaredDistance. A chord endpoint has exact
// squared distance zero, so its projection arithmetic can be skipped after
// the charge succeeds without changing a budget refusal.
func ChordEndpointSquaredDistance(w *FreeformWork, p, a RatPoint, bax, bay, d *big.Rat) (*big.Rat, error) {
	if err := w.Step(CostMul(ChordProjectionCost, WidthUnits(RatBitWidth(p.U, p.V, a.U, a.V, bax, bay, d)))); err != nil {
		return nil, err
	}
	return new(big.Rat), nil
}

// RatChordFrame is the shared chord frame every sagitta reading projects
// against: the vector from a to b and that vector's own exact squared length,
// built ONCE per span so ChordSegmentSquaredDistance never rebuilds it per
// point.
//
// It charges ChordFrameCost at its own operand width, first.
func RatChordFrame(w *FreeformWork, a, b RatPoint) (*big.Rat, *big.Rat, *big.Rat, error) {
	if err := w.Step(CostMul(ChordFrameCost, WidthUnits(RatBitWidth(a.U, a.V, b.U, b.V)))); err != nil {
		return nil, nil, nil, err
	}
	bax := new(big.Rat).Sub(b.U, a.U)
	bay := new(big.Rat).Sub(b.V, a.V)
	d := new(big.Rat).Add(new(big.Rat).Mul(bax, bax), new(big.Rat).Mul(bay, bay))
	return bax, bay, d, nil
}

// RatRunningMax folds one candidate into a running exact maximum. A nil running
// maximum is the fold's own start — the candidate wins with no comparison at
// all — and the charge is spent unconditionally anyway, because a charge that
// skipped a branch would make the count depend on the data rather than on the
// call.
//
// It charges RatCompareCost at its own operand width, first.
func RatRunningMax(w *FreeformWork, best, candidate *big.Rat) (*big.Rat, error) {
	if err := w.Step(CostMul(RatCompareCost, WidthUnits(RatBitWidth(best, candidate)))); err != nil {
		return nil, err
	}
	if best == nil || candidate.Cmp(best) > 0 {
		return candidate, nil
	}
	return best, nil
}

// RatPointCopy duplicates an exact rational point so the copy shares no storage
// with its source. It is a metered primitive rather than a free convenience
// because a big.Rat.Set of a wide value copies every word of it.
//
// It charges RatPointCopyCost at its own operand width, first.
func RatPointCopy(w *FreeformWork, p RatPoint) (RatPoint, error) {
	if err := w.Step(CostMul(RatPointCopyCost, WidthUnits(RatBitWidth(p.U, p.V)))); err != nil {
		return RatPoint{}, err
	}
	return RatPoint{U: new(big.Rat).Set(p.U), V: new(big.Rat).Set(p.V)}, nil
}

// ChargedRatSqrtUp is this file's metered entry point for proofbound.RatSqrtUp
// (spline_length.go), the one outward rounding a free-form bound commits. Every
// reading in this file rounds through it and none calls proofbound.RatSqrtUp directly, so
// the number of roundings charged is the number performed.
//
// proofbound.RatSqrtUp itself keeps its unmetered signature for the ANALYTIC readers that
// share it — a prism's arc radius, a revolve's amplitude, a cap band's contour
// (extrude.go, revolve.go, capblend_contour.go, moments.go, loft_moments.go,
// internal/proofbound/bounds.go) — none of which walks a free-form record and none of which holds a
// FreeformWork counter to charge.
//
// It charges RatSqrtUpCost at the radicand's own width, first.
func ChargedRatSqrtUp(w *FreeformWork, q *big.Rat) (float64, error) {
	if err := w.Step(CostMul(RatSqrtUpCost, WidthUnits(RatBitWidth(q)))); err != nil {
		return 0, err
	}
	return proofbound.RatSqrtUp(q), nil
}

// ChargedRatSqrtDown is ChargedRatSqrtUp's inward twin: the metered entry point
// for proofbound.RatSqrtDown (spline_length.go), for the one reading in this file that
// owes a proven LOWER bound rather than an upper one — a chorded cell's own
// chord length, which an arc-versus-chord deficit subtracts and so must never
// read above the chord it stands for.
//
// The two roundings walk the same directed search over the same exact
// comparison, so they cost the same and charge the same RatSqrtUpCost at the
// radicand's own width, first.
func ChargedRatSqrtDown(w *FreeformWork, q *big.Rat) (float64, error) {
	if err := w.Step(CostMul(RatSqrtUpCost, WidthUnits(RatBitWidth(q)))); err != nil {
		return 0, err
	}
	return proofbound.RatSqrtDown(q), nil
}

// ratPointAt reconstructs split value i's exact rational coordinate:
// numerator / (den · 2^exp), the inverse of the factored form DyadicSpanOf
// and split (spline_length.go) build it in. Every value that form holds is
// exact — a den·2^exp scaling and an integer numerator, never a normalised
// rational — so this reconstruction loses nothing: the RatPoint it returns is
// the value the split produced, to the last bit, and big.Rat.SetFrac copies
// rather than aliases its arguments, so the result shares no storage with the
// DyadicSpan it was read from.
//
// It charges RatPointReconstructCost at value i's OWN width, first — the
// denominator den·2^exp it normalises against, or either numerator, whichever
// is widest — so a reconstruction at depth pays for the wider integers depth
// gave it.
func (s DyadicSpan) RatPointAt(w *FreeformWork, i int) (RatPoint, error) {
	p := s.Points[i]
	if err := w.Step(CostMul(RatPointReconstructCost, WidthUnits(s.ValueWidth(p)))); err != nil {
		return RatPoint{}, err
	}
	scale := new(big.Int).Lsh(s.Den, p.Exp)
	return RatPoint{
		U: new(big.Rat).SetFrac(p.U, scale),
		V: new(big.Rat).SetFrac(p.V, scale),
	}, nil
}

// BezierSpan reconstructs a DyadicSpan's own control points as a BezierSpan —
// the form SpanMatchedDeltaUpper/SpanHodographGapUpper read — by calling
// ratPointAt over every index. It is ratPointAt's own doc comment's
// "reconstruction" extended to a whole span rather than one point: every
// value is exact and the result shares no storage with s.
//
// It holds no charge of its own; every unit it spends is ratPointAt's, spent
// once per index it actually reconstructs.
func (s DyadicSpan) BezierSpan(w *FreeformWork) (BezierSpan, error) {
	span := make(BezierSpan, len(s.Points))
	for i := range s.Points {
		p, err := s.RatPointAt(w, i)
		if err != nil {
			return nil, err
		}
		span[i] = p
	}
	return span, nil
}

// --- the sagitta reading ---

// DyadicSpanSagittaUpper is docs/spline-design.md §6.2.1's bound, over the
// dyadic form PairStations' own bisection already holds: the maximum,
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
// The single rounding is the final ChargedRatSqrtUp: the exact rational maximum
// squared distance is rounded OUTWARD once, so the published float64 is an
// over-statement of the true bound, never an understatement. Where that exact
// maximum's root itself runs past the representable float64 range, proofbound.RatSqrtUp's
// own contract returns +Inf — a valid, if useless, upper bound; this function's
// only error is the counter's own refusal, and a caller that needs a decision on
// a bound that wide (PairStations, via its own station cap) makes it by
// continuing to bisect until its cap fires rather than by trusting it.
//
// It holds no aggregate charge of its own. Its cost is exactly the charges its
// own calls spend — the two chord-end reconstructions, the chord frame, then one
// reconstruction, one projection and one comparison per control point, and the
// one outward rounding — so the two chord-end reads that a per-point aggregate
// silently omitted are paid for here by the simple fact that the code makes
// them.
func DyadicSpanSagittaUpper(w *FreeformWork, s DyadicSpan) (float64, error) {
	n := len(s.Points)
	if n == 0 {
		return 0, nil
	}
	a, err := s.RatPointAt(w, 0)
	if err != nil {
		return 0, err
	}
	b, err := s.RatPointAt(w, n-1)
	if err != nil {
		return 0, err
	}
	bax, bay, d, err := RatChordFrame(w, a, b)
	if err != nil {
		return 0, err
	}

	var maxSq *big.Rat
	for i := range n {
		p, err := s.RatPointAt(w, i)
		if err != nil {
			return 0, err
		}
		var sq *big.Rat
		if i == 0 || i == n-1 {
			sq, err = ChordEndpointSquaredDistance(w, p, a, bax, bay, d)
		} else {
			sq, err = ChordSegmentSquaredDistance(w, p, a, bax, bay, d)
		}
		if err != nil {
			return 0, err
		}
		maxSq, err = RatRunningMax(w, maxSq, sq)
		if err != nil {
			return 0, err
		}
	}
	return ChargedRatSqrtUp(w, maxSq)
}

// DyadicSpanSagittaUpperWithSpan is the PairStations walk's fused reading. It
// reconstructs the exact control points once, uses them for the sagitta, and
// returns the same BezierSpan to the accepted cell's matched-delta reading.
// Rejected cells discard the reconstructed span after this call, but accepted
// cells avoid a second full ratPointAt pass before SpanMatchedDeltaUpper.
//
// This helper holds no charge of its own. The reconstruction, projection,
// comparison and rounding each charge the primitive that performs them, just
// as they do in DyadicSpanSagittaUpper.
func DyadicSpanSagittaUpperWithSpan(w *FreeformWork, s DyadicSpan) (float64, BezierSpan, error) {
	span, err := s.BezierSpan(w)
	if err != nil {
		return 0, nil, err
	}
	if len(span) == 0 {
		return 0, span, nil
	}
	a, b := span[0], span[len(span)-1]
	bax, bay, d, err := RatChordFrame(w, a, b)
	if err != nil {
		return 0, nil, err
	}

	var maxSq *big.Rat
	for i, p := range span {
		var sq *big.Rat
		if i == 0 || i == len(span)-1 {
			sq, err = ChordEndpointSquaredDistance(w, p, a, bax, bay, d)
		} else {
			sq, err = ChordSegmentSquaredDistance(w, p, a, bax, bay, d)
		}
		if err != nil {
			return 0, nil, err
		}
		maxSq, err = RatRunningMax(w, maxSq, sq)
		if err != nil {
			return 0, nil, err
		}
	}
	bound, err := ChargedRatSqrtUp(w, maxSq)
	if err != nil {
		return 0, nil, err
	}
	return bound, span, nil
}

// SpanSagittaUpper is DyadicSpanSagittaUpper's entry point for a caller
// holding a converted BezierSpan rather than an already-split DyadicSpan —
// freeformBezierSpans' own output, before any bisection has run. It converts
// once through DyadicSpanOf (spline_length.go) and reuses the identical
// arithmetic DyadicSpanSagittaUpper runs on every dyadic cell PairStations
// bisects, so the sagitta bound exists in exactly one place regardless of
// which form a caller starts from. Both phases charge the counter it is
// handed, each on its own call.
func SpanSagittaUpper(w *FreeformWork, span BezierSpan) (float64, error) {
	s, err := DyadicSpanOf(w, span)
	if err != nil {
		return 0, err
	}
	return DyadicSpanSagittaUpper(w, s)
}

// This section is internal/proofbound/bounds.go's proofbound.CellChordCurveAreaUpper's matchedDeltaUpper
// obligation (its own doc comment, F1's rule): a PARAMETER-MATCHED bound on
// |curve(s) − chord(s)| at the SAME s, which is a STRONGER, DIFFERENT claim
// than the SET-distance sagitta above. No caller may pass the sagitta where
// this is owed. Every function below is Tier A / polynomial-Bézier only, for
// the identical reason SpanSagittaUpper is (this file's own header): a
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
