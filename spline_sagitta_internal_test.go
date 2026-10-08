package decad

import (
	"errors"
	"fmt"
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/freeform"

	"github.com/lestrrat-3d/decad/internal/proofbound"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/stretchr/testify/require"
)

// This file asserts docs/spline-design.md §6.2.1's chord-sagitta bound
// (freeform.SpanSagittaUpper) and the shared dyadic station generator built on top of
// it (freeform.PairStations), both in internal/freeform/spline_sagitta.go.
//
// FALSIFICATION LEDGER — a10-plan.md Part 3 PR 8's own mandatory protocol.
// Every leg below was broken IN internal/freeform/spline_sagitta.go, `go test` was run against
// the fixture that exists to catch it, the fixture was watched go RED, and
// the file was then restored (`git diff` confirmed clean before this file was
// committed). A leg with no red run has its redundancy argued instead, never
// silently skipped.
//
//   - Remove the [0,1] clamp in freeform.ChordSegmentSquaredDistance (project onto the
//     chord's carrier LINE instead of the SEGMENT): TestSpanSagittaUpper
//     EnclosesOvershootingChordSegment went red — the reported bound fell to
//     ~0.01 (both interior control points' perpendicular distance to the
//     line), under the dense-sample deviation of ~0.76 the test requires it
//     to enclose.
//   - Take the max over interior control points only, excluding span[0] and
//     span[len-1]: NOT separately falsified, argued redundant instead. The
//     two excluded points ARE the chord segment's own endpoints, so their
//     clamped distance to it is always exactly 0 (t clamps to themselves);
//     excluding two points whose contribution is always the additive
//     identity of a max cannot change any max this file computes, on any
//     span, ever. No fixture can tell the two forms apart.
//   - Swap proofbound.RatSqrtUp for proofbound.RatSqrtDown in freeform.DyadicSpanSagittaUpper's final
//     rounding: TestSpanSagittaUpperRoundsOutward went red — the returned
//     bound, converted to a big.Float, fell strictly BELOW the exact rational
//     maximum squared distance's own proven square root, violating the
//     outward-rounding contract the dense-sample tests are too coarse (a few
//     ulps) to catch on their own.
//   - Return the last examined cell's own sagitta instead of the running
//     maximum in walkCell: TestPairStationsSagittaUpperIsAMaximumNeverTheLast
//     Cell went red — a chain whose FIRST span bulges far off its chord and
//     whose SECOND is nearly flat reported the second span's tiny sagitta
//     instead of the first span's large one.
//   - Charge only work0, never work1, in walkCell: TestPairStationsChargesBoth
//     CountersSeparately went red — work1.spent stayed exactly 0 after a build
//     that visibly subdivided.
//   - Charge work1's cost using n0 (side 0's control count) instead of n1:
//     NOT separately falsified as its own red run, argued redundant instead.
//     TestPairStationsChargesBothCountersSeparately already pins work1.spent
//     STRICTLY GREATER than work0.spent from the two sides' differing control
//     counts (8 vs 4); charging n0's cost to work1 would make work1.spent
//     exactly equal work0.spent (both driven by the same n=4 charge under the
//     same cell count), which fails that inequality outright. No separate
//     fixture is needed to catch the same swap the length-asymmetry test
//     already depends on to pass at all.
//   - Drop the len(spans0) != len(spans1) gate (index the shorter chain past
//     its own end instead of refusing): NOT run by deliberately corrupting
//     the gate, because the failure mode is a panic (index out of range) on
//     the very fixture TestPairStationsSpanCountMismatchRefuses builds, not a
//     quietly wrong Measurement — a panic is its own unambiguous, maximally
//     visible falsification signal, and removing the length check and running
//     that single test was confirmed to panic before the gate was restored.
//   - Charge the hard cap per cell VISITED instead of per chord ACCEPTED
//     (count every walkCell entry against freeform.MaxChordsPerWalk):
//     TestPairStationsAcceptsTheStatedChordCap went red — a walk needing
//     exactly freeform.MaxChordsPerWalk chords was refused with freeform.ErrTooManyChords
//     barely halfway through, because a binary refinement visits 2L−m cells
//     for L chords over m spans and so trips a visit-charged ceiling at
//     roughly half the chord count that ceiling's own message names.
//   - Drop freeform.PairStations' span-count entry guard:
//     TestPairStationsSpanCountPastTheCapRefusesUpFront went red — a chain of
//     freeform.MaxChordsPerWalk+1 straight spans returned that many chords and no
//     error, one past the count freeform.ErrTooManyChords names.
//   - Move the cap charge from the SPLIT to the accept (count only chords
//     already accepted, with nothing charged before a bisection): NOT
//     separately falsified, argued instead. The two forms return the
//     identical verdict on every walk that terminates, since the accepted
//     count reaches the ceiling either way; they differ only on a walk that
//     never accepts a cell at all, where the accept-time form charges nothing
//     and leaves the recursion with no bound whatsoever. That difference is a
//     HANG rather than a wrong answer, and the fixture that expresses it — an
//     unreachable target such as NaN, which no comparison ever satisfies —
//     costs ~20 s and hundreds of megabytes even in the shipped form that
//     bounds it correctly, so it is argued here rather than paid for on every
//     run.
//
// SECOND PASS — an independent adversarial audit of this file, closing gaps
// its own falsification did not reach the first time:
//
//   - A1: freeform.ChordSegmentSquaredDistance's d==0 branch (a degenerate CHORD, e.g.
//     a closed free-form loop whose first and last control point coincide —
//     not a collapsed SPAN) returning 0 instead of |p-a|^2:
//     TestSpanSagittaUpperClosedLoopChordIsAPointNotZero went red — the
//     reported bound fell from ~5.099 to 0 on a net whose farthest control
//     point sits exactly sqrt(26) from the shared chord point. This gap
//     existed because the test file's own independent oracle,
//     independentMaxChordSquaredDistance, asserted abLenSq.Sign() nonzero and
//     so could not exercise this branch at all; it is now fixed to answer
//     |p-a|^2 directly for a degenerate chord, the same derivation
//     freeform.ChordSegmentSquaredDistance's own doc comment states.
//   - A2: the accept test math.Max(sag0, sag1) <= target narrowed to
//     sag0 <= target alone: TestPairStationsAcceptTestRequiresBothSidesUnderTarget
//     went red — a cell whose side 0 already met a target=0.01 but whose side
//     1 sat at 5.0 was accepted whole, and the returned sagittaUpper (5.0)
//     blew past the target it was asked to honor.
//   - A3: the running maximum g.sagittaUpper = math.Max(g.sagittaUpper,
//     math.Max(sag0, sag1)) narrowed to fold only sag0:
//     TestPairStationsSagittaUpperReflectsTheLargerSide went red — a cell
//     accepted whole with side 1's reading (5.0) far above side 0's (~1e-4)
//     reported sagittaUpper ~1e-4 instead of ~5.0, a four-orders-of-magnitude
//     under-report of the published bound.
//   - A4: walkCell's own left-then-right recursion order swapped to
//     right-then-left: TestPairStationsStationsAdvanceMonotonicallyAlongTheChain
//     went red — a quarter circle's own returned stations no longer advanced
//     monotonically in angle, regressing partway through the chain exactly as
//     a scrambled walk order predicts.
//   - A5: the accept branch's own c0.ratPointAt(0)/c1.ratPointAt(0) swapped
//     for c0.ratPointAt(len-1)/c1.ratPointAt(len-1) (each cell's LAST control
//     point instead of its FIRST):
//     TestPairStationsFirstAndLastStationAreTheChainEndpointsExactly went red
//     — stations[0] no longer equaled the chain's own start control point,
//     exactly.
//   - A6: the cost model charged one flat 8 units per control point for both
//     readings, left ratPointAt's reconstruction, freeform.DyadicSpanOf's conversion
//     and the per-call proofbound.RatSqrtUp entirely free, and so was not the upper
//     bound its own doc comment claimed. Each charge is now counted on its own
//     code path. Five mutations were each run red against the new fixtures:
//     deleting the conversion charge (TestPairStationsChargesEveryPhaseOfA
//     WalkThatNeverSplits and TestPairStationsBudgetBindsAtTheWholeCharged
//     Total), paying the matched delta at the sagitta's rate (the same phase
//     fixture), zeroing freeform.RatPointReconstructCost, dropping the per-call
//     freeform.RatSqrtUpCost, and narrowing freeform.ChordProjectionCost back to 8
//     (TestSagittaCostTermsMatchTheOperationsTheyName and
//     TestSagittaAndMatchedDeltaCostsAreDerivedSeparately). Note what stays
//     GREEN under all five: TestPairStationsChargesBothCountersSeparately,
//     which pins only the RATIO between two differently-sized sides' charges
//     — a ratio-only fixture cannot see a rescaled or missing cost at all.
//   - B1: freeform.DyadicSpanSagittaUpper's own n==0 guard was already dead-ended —
//     nothing in freeform.PairStations prevented a zero-control-point span from
//     reaching walkCell, whose accept branch then panicked on
//     c0.ratPointAt(0)'s empty slice. Removing the new entry-level guard and
//     running TestPairStationsRefusesAZeroControlSpanInsteadOfPanicking
//     reproduced exactly that panic (index out of range on
//     freeform.DyadicSpan.ratPointAt); freeform.PairStations now refuses a zero-control span on
//     either side with ErrDegenerate before any cell is walked, so the
//     dead-ended guard's own 0 answer is no longer reachable from this walk.
//   - B2: the final station's own append reverted from a fresh
//     freeform.RatPoint{u: new(big.Rat).Set(...), v: new(big.Rat).Set(...)} copy back
//     to appending the caller's own last control point freeform.RatPoint directly:
//     TestPairStationsFinalStationDoesNotAliasTheInputSpan went red —
//     mutating the returned station's own *big.Rat in place changed the
//     caller's input span, violating ratPointAt's own non-aliasing contract
//     every OTHER station in the two returned lists already carries.
//
// THIRD PASS — the cost model moved from caller-side aggregates into the
// callees, so a charge's multiplicity is the call count rather than a number a
// caller restates. Every leg below was broken in internal/freeform/spline_sagitta.go or
// internal/freeform/spline_length.go, watched go RED against the fixture that exists to catch it,
// and restored:
//
//   - C1: the two chord-end reconstructions in freeform.DyadicSpanSagittaUpper run
//     unmetered (the exact blind spot a per-point aggregate had, since it
//     multiplied one rate by n and the chord ends are n+1 and n+2):
//     TestSagittaAndMatchedDeltaChargeTheirOwnCodePaths went red at 144 units
//     against 158, and both walk-total fixtures with it.
//   - C2: freeform.RatChordFrame's shared chord vector and squared length run unmetered
//     — five exact operations per span that the old model charged nothing for:
//     the same two fixtures went red, at 153 and 616 units.
//   - C3: the accept branch's own station reads run unmetered:
//     TestPairStationsChargesEveryPhaseOfAWalkThatNeverSplits went red at 612
//     against 626, and TestPairStationsBudgetBindsAtTheWholeChargedTotal with it.
//   - C4: the two final-station copies run unmetered:
//     TestPairStationsChargesEveryPhaseOfAWalkThatNeverSplits went red at 624.
//   - C5: freeform.RatRunningMax's own comparison runs unmetered: red at 155 and 620.
//   - C6: freeform.DyadicMidpointOps narrowed from 6 back to the 2 per blend the old
//     split charge paid: TestSagittaCostTermsMatchTheOperationsTheyName went
//     red against its own independent derivation of what an aligned sum runs.
//   - C7: freeform.DyadicSplitBookkeepingOps zeroed (a split's three allocations, its
//     copy and its 2n appends left free): the same fixture went red. This leg
//     had NO red run on its first attempt — both split fixtures derived from the
//     constant they were meant to check — and the independent derivation was
//     added because of it.
//   - C8: freeform.DyadicSpan.split's own charge deleted:
//     TestDyadicSplitChargesItselfAndRefusesBeforeSplitting went red at 0 units
//     against 52.
//   - C9: freeform.WidthUnits flattened to 1, and separately the three primitives that
//     scale by it left on a count-only charge:
//     TestChargesScaleWithOperandWidth went red both ways — decisively on the
//     second, where the identical hull scan over 4096-bit coordinates charged
//     the SAME 96 units as over machine words.
//   - C10: freeform.RatChordFrame renamed without renaming it in the guard's own metered
//     set: TestSplineSagittaRunsNoExactArithmeticOutsideAMeteredPrimitive went
//     red, so a primitive cannot be silently dropped out of the metered surface.
//   - C11: an unmetered new(big.Rat).Add added to walkCell: the same guard went
//     red on all four exact-arithmetic sites, naming the function and the
//     operation. This is the leg that keeps the class closed rather than the
//     numbers repaired.
//
// Part C's three primitives (freeform.SpanHodographGapUpper, freeform.SpanMatchedDeltaUpper,
// freeform.SpanSpeedUpper) were new proofs rather than repairs of an existing leg when
// they landed, so only the fourth pass below records a mutation against one of
// them. Their own soundness rests on
// TestSpanMatchedDeltaUpperEnclosesWhatTheSagittaMisses (the decisive
// zigzag-hugging fixture proving the sagitta is NOT a substitute) and
// TestHodographBoundsAreExactlyZeroOnCollapsedAndStraightUniformSpans, which a
// quick sanity check confirmed IS sensitive to a broken derivation: dropping
// the hodograph's own "- Delta" term (silencing it via a x0 multiply so the
// build still compiles) left the dense-sample enclosure tests green — a wider
// bound still encloses — but turned the straight-uniform-span's own EXACT
// zero reading into 2, which the zero-reading fixture caught immediately.

// FOURTH PASS — the matched delta's own halving moved out of float arithmetic
// and into the exact rational radicand. The leg below was broken in
// internal/freeform/spline_sagitta.go, watched go RED against the fixture that exists to catch
// it, and restored:
//
//   - D1: freeform.SpanMatchedDeltaUpper's exact quartering replaced by the float
//     halving proofbound.UpRound(gap / 2) of the already-rounded hodograph gap:
//     TestSpanMatchedDeltaUpperNeverUnderflowsASubnormalGapToZero went red on
//     every row of its window — the published bound read exactly 0 where the
//     true parameter-matched deviation is positive, which is an under-cover
//     and not a rounding, since internal/proofbound/bounds.go's proofbound.CellChordCurveAreaUpper gates its
//     whole chord-to-curve leg on matchedDelta > 0.
//
// ratSpan is spline_extreme_internal_test.go's own helper (same package): it
// builds a freeform.BezierSpan directly from plane-local float coordinates, for tests
// that exercise this file's machinery without going through a recorded
// segment.

// quarterCircleFitSpans converts a 5-point Tier A FitSplineSeg through the
// same radius-5 quarter circle loft_chord_calibration_internal_test.go's own
// wedgeFitSpline builds via sketch (k*pi/8 for k = 0..4) — built directly on
// the record here, with no sketch dependency, since fitSplineBezierSpans
// (spline_fit.go) takes a FitSplineSeg's Fit points on their own.
func quarterCircleFitSpans(t *testing.T) []freeform.BezierSpan {
	t.Helper()
	const radius = 5.0
	fit := make([]Point2, 5)
	for k := range fit {
		theta := float64(k) * math.Pi / 8
		fit[k] = Point2{U: radius * math.Cos(theta), V: radius * math.Sin(theta)}
	}
	spans, err := fitSplineBezierSpans(FitSplineSeg{Fit: fit, TStart: 0, TEnd: 1}, freeform.NewFreeformWork())
	require.NoError(t, err)
	require.NotEmpty(t, spans)
	return spans
}

// scaleSpans returns a new chain with every control point's coordinate
// multiplied by the exact integer factor — used to build a second "side" of
// genuinely different absolute sagitta from the same shape, over exact
// rationals so the relationship freeform.PairStations must preserve (station1 = factor
// * station0 at every shared dyadic parameter) is checkable bit-for-bit.
func scaleSpans(spans []freeform.BezierSpan, factor int64) []freeform.BezierSpan {
	f := big.NewRat(factor, 1)
	out := make([]freeform.BezierSpan, len(spans))
	for i, span := range spans {
		s := make(freeform.BezierSpan, len(span))
		for j, p := range span {
			s[j] = freeform.RatPoint{U: new(big.Rat).Mul(p.U, f), V: new(big.Rat).Mul(p.V, f)}
		}
		out[i] = s
	}
	return out
}

// maxSagittaAtDepth measures a single span's own §6.2.1 sagitta at a FIXED
// uniform dyadic depth, over the same production primitives freeform.PairStations
// itself bisects with (freeform.DyadicSpanOf, freeform.DyadicSpan.split, freeform.DyadicSpanSagittaUpper)
// — a different (uniform, non-adaptive) traversal of the real machinery,
// never a parallel reimplementation of the bound it measures.
func maxSagittaAtDepth(t *testing.T, s freeform.DyadicSpan, depth int) float64 {
	t.Helper()
	if depth == 0 {
		bound, err := freeform.DyadicSpanSagittaUpper(nil, s)
		require.NoError(t, err)
		return bound
	}
	left, right, err := s.Split(nil)
	require.NoError(t, err)
	return math.Max(maxSagittaAtDepth(t, left, depth-1), maxSagittaAtDepth(t, right, depth-1))
}

func levelSagitta(t *testing.T, span freeform.BezierSpan, depth int) float64 {
	t.Helper()
	s, err := freeform.DyadicSpanOf(nil, span)
	require.NoError(t, err)
	return maxSagittaAtDepth(t, s, depth)
}

// Every bound below is read through one of these four unmetered readers. Each
// primitive now takes the counter that pays for it and returns that counter's
// own refusal (internal/freeform/spline_sagitta.go's header); a nil counter never refuses, so a
// fixture measuring a BOUND rather than a CHARGE reads it here and asserts the
// error away once instead of at every call.
func sagittaOf(t *testing.T, span freeform.BezierSpan) float64 {
	t.Helper()
	bound, err := freeform.SpanSagittaUpper(nil, span)
	require.NoError(t, err)
	return bound
}

func hodographGapOf(t *testing.T, span freeform.BezierSpan) float64 {
	t.Helper()
	bound, err := freeform.SpanHodographGapUpper(nil, span)
	require.NoError(t, err)
	return bound
}

func matchedDeltaOf(t *testing.T, span freeform.BezierSpan) float64 {
	t.Helper()
	bound, err := freeform.SpanMatchedDeltaUpper(nil, span)
	require.NoError(t, err)
	return bound
}

func speedOf(t *testing.T, span freeform.BezierSpan) float64 {
	t.Helper()
	bound, err := freeform.SpanSpeedUpper(nil, span)
	require.NoError(t, err)
	return bound
}

// denseChordSegmentDeviation samples a single-span chain densely and returns
// the maximum true distance from a sampled curve point to the chord SEGMENT
// joining the chain's own first and last control point — the falsifier a
// bound built from the chord's carrier LINE, or from the parametric deviation
// |C(t) - L(t)|, cannot survive. The cached float de Casteljau oracle is the
// independent evaluator used for this large sample count; the exact-rational
// oracle remains the conversion check in spline_bezier_internal_test.go.
func denseChordSegmentDeviation(t *testing.T, span freeform.BezierSpan, samples int) float64 {
	t.Helper()
	floatSpan := floatBezierSpanOf(span)
	ax, ay := evalFloatBezierSpan(floatSpan, 0)
	bx, by := evalFloatBezierSpan(floatSpan, 1)
	dx, dy := bx-ax, by-ay
	d := dx*dx + dy*dy
	maxDev := 0.0
	for i := 0; i <= samples; i++ {
		at := float64(i) / float64(samples)
		cx, cy := evalFloatBezierSpan(floatSpan, at)
		var px, py float64
		if d == 0 {
			px, py = ax, ay
		} else {
			s := ((cx-ax)*dx + (cy-ay)*dy) / d
			s = math.Max(0, math.Min(1, s))
			px, py = ax+s*dx, ay+s*dy
		}
		dev := math.Hypot(cx-px, cy-py)
		maxDev = math.Max(maxDev, dev)
	}
	return maxDev
}

// carrierLineDistanceUpper is the BROKEN mechanism §6.2.1 itself warns
// against: the maximum distance from each control point to the chord's
// infinite CARRIER LINE, with no [0,1] clamp at all. It exists only so
// TestSpanSagittaUpperEnclosesOvershootingChordSegment can show it fails to
// enclose the true deviation — never used as a bound anywhere in production.
func carrierLineDistanceUpper(t *testing.T, span freeform.BezierSpan) float64 {
	t.Helper()
	ax, ay := floatOfRatPoint(t, span[0])
	bx, by := floatOfRatPoint(t, span[len(span)-1])
	dx, dy := bx-ax, by-ay
	norm := math.Hypot(dx, dy)
	require.Positive(t, norm, "the fixture's chord must have positive length for a carrier line to exist")
	maxDist := 0.0
	for _, p := range span {
		px, py := floatOfRatPoint(t, p)
		cross := (px-ax)*dy - (py-ay)*dx
		maxDist = math.Max(maxDist, math.Abs(cross)/norm)
	}
	return maxDist
}

func floatOfRatPoint(t *testing.T, p freeform.RatPoint) (float64, float64) {
	t.Helper()
	pt, ok := point2Of(p)
	require.True(t, ok, "a test fixture's control point must be representable")
	return pt.U, pt.V
}

// --- 1. the overshooting net (docs/spline-design.md §6.2.1, §11) ---

func TestSpanSagittaUpperEnclosesOvershootingChordSegment(t *testing.T) {
	t.Parallel()
	span := ratSpan([][2]float64{{0, 0}, {-3, 0.01}, {4, 0.01}, {1, 0}})

	dense := denseChordSegmentDeviation(t, span, 200_000)
	require.InDelta(t, 0.76, dense, 0.01, "the dense-sample deviation must match §6.2.1's own stated ~0.76")

	bound := sagittaOf(t, span)
	require.GreaterOrEqual(t, bound, dense, "the reported bound must ENCLOSE the dense-sample deviation")

	// The broken carrier-LINE mechanism does not enclose it: every control
	// point sits within 0.01 of the line through the chord's own ends, exactly
	// §6.2.1's own worked example, so the line-only reading UNDERSTATES the
	// true departure by roughly two orders of magnitude.
	broken := carrierLineDistanceUpper(t, span)
	require.Less(t, broken, dense, "a carrier-LINE bound must fail to enclose the dense-sample deviation — this is the mechanism §6.2.1 rejects")
	require.InDelta(t, 0.01, broken, 0.005, "the broken line-only reading must match §6.2.1's own stated ~0.01")
}

// --- 2. collinear (polynomial) controls distinguish sagitta from |C-L| ---

// TestSpanSagittaUpperDistinguishesFromParametricDeviation is §6.2.1's second
// bullet, restated in Tier A. §6.2.1's own worked example there — collinear
// controls at weights 1,1,100 — is a RATIONAL span, which Tier A does not
// admit (a10-plan.md risk R5; a non-unit-weight span refuses earlier, at
// Table R row R10, inside freeformBezierSpans). The polynomial analogue makes
// the identical point without any weight: four control points collinear on
// v=0, each with u strictly inside [0, 1] — (0,0), (0.1,0), (0.1,0), (1,0).
// Every control point already lies ON the segment [(0,0),(1,0)] (clamped
// distance 0 for each), so the reported sagitta is exactly 0 — that segment
// bounds the whole curve, since a Bézier is a convex combination of collinear
// points confined to [0,1] on the line. The curve is NOT, however, the
// uniform-rate linear interpolant L(t) = (1-t)*(0,0) + t*(1,0): Bernstein
// blending over non-uniformly-spaced collinear controls moves the curve along
// the line at an UNEVEN rate, so C(0.5) sits well short of L(0.5) = (0.5, 0)
// even though both are on the same line and the sagitta is exactly 0.
func TestSpanSagittaUpperDistinguishesFromParametricDeviation(t *testing.T) {
	t.Parallel()
	span := ratSpan([][2]float64{{0, 0}, {0.1, 0}, {0.1, 0}, {1, 0}})

	bound := sagittaOf(t, span)
	require.Zero(t, bound, "every control point already sits on the chord segment, so the sagitta is exactly 0")

	cx, cy := evalSpans(t, []freeform.BezierSpan{span}, 0.5)
	require.Zero(t, cy, "the curve never leaves the line v=0")
	lx := 0.5 // L(0.5) on the naive uniform-rate interpolant between (0,0) and (1,0)
	require.Greater(t, math.Abs(cx-lx), 0.2,
		"the parametric deviation |C(0.5)-L(0.5)| is well over 0.2 even though the sagitta the curve actually commits is exactly 0")
}

// --- 3. a genuinely collapsed span reports sagitta exactly 0 ---

func TestSpanSagittaUpperCollapsedSpanIsExactZero(t *testing.T) {
	t.Parallel()
	span := ratSpan([][2]float64{{2, 3}, {2, 3}, {2, 3}, {2, 3}})
	require.Equal(t, 0.0, sagittaOf(t, span), "a span whose control points all coincide reports sagitta exactly 0, by exact float equality")
}

// --- 4. station determinism ---

func TestPairStationsStationDeterminism(t *testing.T) {
	t.Parallel()
	spans := quarterCircleFitSpans(t)
	const target = 1e-4

	s0a, s1a, _, sagA, err := freeform.PairStations(spans, spans, target, nil, nil)
	require.NoError(t, err)
	s0b, s1b, _, sagB, err := freeform.PairStations(spans, spans, target, nil, nil)
	require.NoError(t, err)

	require.Equal(t, sagA, sagB, "the achieved sagittaUpper must be bit-identical across calls")
	require.Len(t, s0b, len(s0a))
	require.Len(t, s1b, len(s1a))
	for i := range s0a {
		require.Zero(t, s0a[i].U.Cmp(s0b[i].U), "station %d side0 U must be bit-identical", i)
		require.Zero(t, s0a[i].V.Cmp(s0b[i].V), "station %d side0 V must be bit-identical", i)
		require.Zero(t, s1a[i].U.Cmp(s1b[i].U), "station %d side1 U must be bit-identical", i)
		require.Zero(t, s1a[i].V.Cmp(s1b[i].V), "station %d side1 V must be bit-identical", i)
	}
}

// --- 5. sagitta vs level: strictly decreasing, and settling on the smallest ---

func TestPairStationsSagittaStrictlyDecreasesWithLevel(t *testing.T) {
	t.Parallel()
	spans := quarterCircleFitSpans(t)
	span := spans[0]

	previous := math.Inf(1)
	for depth := range 7 {
		s := levelSagitta(t, span, depth)
		require.Less(t, s, previous, "depth %d must strictly narrow the sagitta bound", depth)
		previous = s
	}
}

// TestPairStationsSettlesOnSmallestLevelForTarget pins that the generator
// never subdivides past what a target needs, and that it subdivides LESS for
// a laxer target — both read off the target, never off a hard-coded leaf or
// level count. A per-cell ADAPTIVE walk is not obliged to match a UNIFORM
// depth-d tree exactly (a smoothly-varying span can converge faster in some
// regions than others), so "smallest" is pinned two ways instead of by exact
// leaf-count equality: (1) the leaf count at depth d's own target never
// exceeds the uniform depth-d ceiling 2^d — the generator cannot need MORE
// cells than uniform refinement to depth d already guarantees suffices — and
// (2) switching to the strictly coarser target derived from level d-1 yields
// STRICTLY FEWER leaves, proving the walk actually adapts to how fine the
// target is rather than subdividing to some fixed amount regardless of it.
func TestPairStationsSettlesOnSmallestLevelForTarget(t *testing.T) {
	t.Parallel()
	spans := quarterCircleFitSpans(t)
	span := spans[0]

	// Find the smallest depth d whose UNIFORM bisection already meets a
	// target set just above level d's own measured value — d itself is
	// discovered from the level sequence, never a hard-coded number.
	d := 0
	for levelSagitta(t, span, d) > levelSagitta(t, span, 4) {
		d++
	}
	require.LessOrEqual(t, d, 4)
	require.Positive(t, d, "the fixture needs at least one level of refinement for this test to exercise anything")
	targetFine := levelSagitta(t, span, d) * (1 + 1e-9)
	targetCoarse := levelSagitta(t, span, d-1) * (1 + 1e-9)
	require.Greater(t, targetCoarse, targetFine, "level d-1's own target must be strictly laxer than level d's")

	single := []freeform.BezierSpan{span}

	s0Fine, _, _, sagFine, err := freeform.PairStations(single, single, targetFine, nil, nil)
	require.NoError(t, err)
	require.LessOrEqual(t, sagFine, targetFine, "the achieved sagitta must honor the fine target")
	leavesFine := len(s0Fine) - 1
	require.LessOrEqual(t, leavesFine, 1<<d,
		"the generator must never need more cells than uniform refinement to depth %d already guarantees suffices", d)

	s0Coarse, _, _, sagCoarse, err := freeform.PairStations(single, single, targetCoarse, nil, nil)
	require.NoError(t, err)
	require.LessOrEqual(t, sagCoarse, targetCoarse, "the achieved sagitta must honor the coarse target")
	leavesCoarse := len(s0Coarse) - 1
	require.LessOrEqual(t, leavesCoarse, 1<<(d-1),
		"the generator must never need more cells than uniform refinement to depth %d already guarantees suffices", d-1)

	require.Less(t, leavesCoarse, leavesFine,
		"a strictly laxer target must settle on strictly fewer cells, proving the walk tracks the target rather than a fixed depth")
}

// --- 6. over-cap refuses freeform.ErrTooManyChords ---

func TestPairStationsOverCapRefuses(t *testing.T) {
	t.Parallel()
	spans := []freeform.BezierSpan{parabolaSpan()}
	_, _, _, _, err := freeform.PairStations(spans, spans, 1e-20, nil, nil) //nolint:dogsled // stations/sagitta discarded; only the refusal is under test
	require.Error(t, err)
	require.ErrorIs(t, err, freeform.ErrTooManyChords)
	require.ErrorIs(t, err, ErrUnsupported)
}

// parabolaSpan is a quadratic Bézier over an exact integer control net whose
// §6.2.1 sagitta is SELF-SIMILAR under bisection: a quadratic's middle control
// point lands at exactly a quarter of its parent's own offset from the chord
// in each half, so every cell at a given dyadic depth carries the identical
// bound, and a target read off depth d settles the walk on exactly 2^d chords.
// That is what lets the two cap fixtures below name an exact chord count
// instead of approaching one, and the small integer net keeps every rational
// the walk builds cheap enough for a 2^14-chord walk to stay fast.
func parabolaSpan() freeform.BezierSpan {
	return ratSpan([][2]float64{{0, 0}, {1, 1}, {2, 0}})
}

// straightSpanFrom is a collinear, evenly spaced quadratic span: every control
// point lies ON its own chord segment, so its sagitta is exactly 0, and it is
// accepted whole as ONE chord at depth 0 against any target. It contributes a
// chord to a chain without contributing a bisection.
func straightSpanFrom(u float64) freeform.BezierSpan {
	return ratSpan([][2]float64{{u, 0}, {u + 1, 0}, {u + 2, 0}})
}

// capDepth is the uniform bisection depth whose leaves number exactly
// freeform.MaxChordsPerWalk, read off the cap itself rather than written down as a
// tuned number.
func capDepth(t *testing.T) int {
	t.Helper()
	d := 0
	for 1<<d < freeform.MaxChordsPerWalk {
		d++
	}
	require.Equal(t, freeform.MaxChordsPerWalk, 1<<d,
		"these fixtures read their refinement depth off the cap; a cap that is not a power of two needs a different construction")
	return d
}

// TestPairStationsAcceptsTheStatedChordCap puts the cap where its own message
// puts it. freeform.ErrTooManyChords reads "more than freeform.MaxChordsPerWalk chords on one
// curve", so a walk that settles on EXACTLY freeform.MaxChordsPerWalk chords is inside
// the ceiling and must be built, not refused.
//
// This is the fixture that goes red if the cap ever binds below the count it
// names. A binary refinement visits nearly two cells for every chord it
// accepts (2L−m cells for L chords over m spans), so a ceiling charged per
// cell VISITED rather than per chord ACCEPTED refuses this walk barely halfway
// through it, at a chord count under half the one the message states.
func TestPairStationsAcceptsTheStatedChordCap(t *testing.T) {
	t.Parallel()
	span := parabolaSpan()
	chain := []freeform.BezierSpan{span}
	target := levelSagitta(t, span, capDepth(t))

	s0, s1, _, sag, err := freeform.PairStations(chain, chain, target, nil, nil)
	require.NoError(t, err, "a walk needing exactly the chord count the cap names must be built")
	require.Len(t, s0, freeform.MaxChordsPerWalk+1,
		"a chain of exactly maxChordsPerWalk chords carries one more station than that")
	require.Len(t, s1, len(s0), "both sides share one station set by construction")
	require.LessOrEqual(t, sag, target, "the achieved sagitta must honor the target it settled on")
}

// TestPairStationsRefusesOneChordPastTheStatedCap is the other half of the
// same boundary. The identical parabola walk gains ONE straight span, which is
// accepted whole at depth 0, so the chain needs freeform.MaxChordsPerWalk+1 chords —
// the first count the message's "more than" covers — and the walk refuses.
func TestPairStationsRefusesOneChordPastTheStatedCap(t *testing.T) {
	t.Parallel()
	span := parabolaSpan()
	chain := []freeform.BezierSpan{span, straightSpanFrom(2)}
	target := levelSagitta(t, span, capDepth(t))

	_, _, _, _, err := freeform.PairStations(chain, chain, target, nil, nil) //nolint:dogsled // stations/sagitta discarded; only the refusal is under test
	require.ErrorIs(t, err, freeform.ErrTooManyChords)
	require.ErrorIs(t, err, ErrUnsupported)
}

// TestPairStationsSpanCountPastTheCapRefusesUpFront pins the entry guard: each
// span carries at least one chord even when it needs no bisection at all, so a
// chain of more spans than the cap admits already exceeds the chord count the
// message names, and refuses before a single cell is measured. The target here
// is deliberately lax enough that every span would otherwise be accepted whole.
func TestPairStationsSpanCountPastTheCapRefusesUpFront(t *testing.T) {
	t.Parallel()
	chain := make([]freeform.BezierSpan, freeform.MaxChordsPerWalk+1)
	for i := range chain {
		chain[i] = straightSpanFrom(float64(2 * i))
	}

	_, _, _, _, err := freeform.PairStations(chain, chain, 1, nil, nil) //nolint:dogsled // stations/sagitta discarded; only the refusal is under test
	require.ErrorIs(t, err, freeform.ErrTooManyChords)
	require.ErrorIs(t, err, ErrUnsupported)
}

// --- 7. over-budget refuses Table R row R7 ---

func TestPairStationsOverBudgetRefusesR7(t *testing.T) {
	t.Parallel()
	spans := quarterCircleFitSpans(t)
	exhausted := &freeform.FreeformWork{Spent: freeform.FreeformWorkLimit}
	_, _, _, _, err := freeform.PairStations(spans, spans, 1e-9, exhausted, freeform.NewFreeformWork()) //nolint:dogsled // stations/sagitta discarded; only the refusal is under test
	require.Error(t, err)
	require.ErrorIs(t, err, ErrUnsupported)
}

// --- 8. shared station set: same length AND same dyadic parameter fractions ---

// TestPairStationsSharedStationSetAcrossDifferentScale pairs a span with an
// exact integer scaling of itself (factor 5): the two curves have
// GENUINELY different absolute sagitta at every cell (scaling multiplies
// every squared distance by 25 and every sagitta by 5 exactly), yet
// freeform.PairStations always bisects both sides from the SAME cell together
// (internal/freeform/spline_sagitta.go's own doc comment), so the two station lists must land
// on the identical dyadic parameter fractions. That is checked directly, not
// merely by length: because de Casteljau blending is linear, station1[k]
// must equal EXACTLY 5*station0[k] for every k, which only holds if index k
// on both sides names the same parameter — a mismatched tree would break the
// exact ratio at whatever index the trees first diverged.
func TestPairStationsSharedStationSetAcrossDifferentScale(t *testing.T) {
	t.Parallel()
	base := quarterCircleFitSpans(t)
	scaled := scaleSpans(base, 5)

	target := levelSagitta(t, base[0], 2) // fine enough to force several splits
	s0, s1, _, _, err := freeform.PairStations(base, scaled, target, nil, nil)
	require.NoError(t, err)
	require.Equal(t, len(s0), len(s1))
	require.Greater(t, len(s0), len(base)+1, "the target must force genuine subdivision for this test to exercise the shared-cell claim")

	five := big.NewRat(5, 1)
	for k := range s0 {
		wantU := new(big.Rat).Mul(s0[k].U, five)
		wantV := new(big.Rat).Mul(s0[k].V, five)
		require.Zero(t, wantU.Cmp(s1[k].U), "station %d: side1 must be the exact 5x scaling of side0 at the same dyadic parameter fraction", k)
		require.Zero(t, wantV.Cmp(s1[k].V), "station %d: side1 must be the exact 5x scaling of side0 at the same dyadic parameter fraction", k)
	}
}

// --- 9. both counters are charged, on their own side ---

// TestPairStationsChargesBothCountersSeparately pairs a low-degree span (n=4)
// on side 0 with a hand-built high-degree span (n=8) covering the same
// parameter domain on side 1. Both sides are bisected the SAME number of
// times (they always split together), so a per-side cost that actually reads
// each side's own control count must charge side 1 strictly more than side 0
// — proving the two counters are independent AND correctly attributed, not
// merely both nonzero.
func TestPairStationsChargesBothCountersSeparately(t *testing.T) {
	t.Parallel()
	small := ratSpan([][2]float64{{0, 0}, {1, 3}, {2, -3}, {3, 0}})
	big8 := ratSpan([][2]float64{
		{0, 0}, {0.4, 3}, {0.9, -2}, {1.3, 3},
		{1.7, -3}, {2.1, 2}, {2.6, -3}, {3, 0},
	})

	target := math.Min(levelSagitta(t, small, 2), levelSagitta(t, big8, 2))
	work0, work1 := freeform.NewFreeformWork(), freeform.NewFreeformWork()
	_, _, _, _, err := freeform.PairStations([]freeform.BezierSpan{small}, []freeform.BezierSpan{big8}, target, work0, work1) //nolint:dogsled // stations/sagitta discarded; only the counter split is under test
	require.NoError(t, err)

	require.Positive(t, work0.Spent, "side 0's own counter must be charged")
	require.Positive(t, work1.Spent, "side 1's own counter must be charged")
	require.Greater(t, work1.Spent, work0.Spent,
		"side 1's higher control count must cost strictly more under the same split/measure counts, proving each side's charge reads its OWN control count")
}

// --- extra: sagittaUpper is a maximum over every cell, never the last one ---

// TestPairStationsSagittaUpperIsAMaximumNeverTheLastCell pairs a strongly
// bulging span (chained first) with a nearly-flat one (chained second) — both
// accepted without any splitting at a target set just above the bulging
// span's own whole-span sagitta — so the reported sagittaUpper must reflect
// the FIRST (bulging) cell's own large reading, not the LAST (flat) cell's
// tiny one.
func TestPairStationsSagittaUpperIsAMaximumNeverTheLastCell(t *testing.T) {
	t.Parallel()
	bulge := ratSpan([][2]float64{{0, 0}, {0, 5}, {1, 5}, {1, 0}})
	flat := ratSpan([][2]float64{{0, 0}, {0.33, 0.0001}, {0.66, 0.0001}, {1, 0}})

	bulgeSag := sagittaOf(t, bulge)
	flatSag := sagittaOf(t, flat)
	require.Greater(t, bulgeSag, flatSag*100, "the fixture needs a large gap between the two spans' own readings")

	target := bulgeSag * (1 + 1e-9)
	spans0 := []freeform.BezierSpan{bulge, flat}
	spans1 := []freeform.BezierSpan{bulge, flat}
	_, _, _, sagUp, err := freeform.PairStations(spans0, spans1, target, nil, nil) //nolint:dogsled // stations/matchedDelta discarded; only sagittaUpper and err matter here.
	require.NoError(t, err)
	require.InEpsilon(t, bulgeSag, sagUp, 1e-9, "sagittaUpper must be the running MAXIMUM (the first, bulging cell), never the last cell's own tiny reading")
}

// --- extra: freeform.PairStations refuses a span-count mismatch defensively ---

func TestPairStationsSpanCountMismatchRefuses(t *testing.T) {
	t.Parallel()
	spans := quarterCircleFitSpans(t)
	_, _, _, _, err := freeform.PairStations(spans, spans[:len(spans)-1], 1e-6, nil, nil) //nolint:dogsled // stations/sagitta discarded; only the refusal is under test
	require.Error(t, err)
	require.ErrorIs(t, err, ErrUnsupported)
	require.False(t, errors.Is(err, ErrDegenerate))
}

// --- extra: the final rounding is genuinely outward, not merely close ---

// TestSpanSagittaUpperRoundsOutward proves the outward-rounding contract
// directly over exact rationals, at a precision no dense-sample test could
// resolve: the exact maximum squared distance is computed independently (a
// different formula, not freeform.ChordSegmentSquaredDistance's own code path — the
// single interior control point's own clamped-projection distance, worked out
// by hand for this fixture) and its square root is bracketed to 200 bits with
// math/big.Float.Sqrt. freeform.SpanSagittaUpper's returned float64 must sit AT OR
// ABOVE that bracket, never below it — the property that distinguishes
// proofbound.RatSqrtUp from proofbound.RatSqrtDown, and one dense sampling at float64 precision is
// too coarse (both round within a handful of ulps of the true root) to catch
// on its own.
func TestSpanSagittaUpperRoundsOutward(t *testing.T) {
	t.Parallel()
	// A degree-3 span whose chord (0,0)-(5,3) is NOT axis-aligned, so the
	// clamped-projection distance genuinely mixes both coordinates rather
	// than reducing to a single coordinate difference (which would square to
	// a perfect square of that same float, and round-trip through sqrt
	// exactly regardless of rounding direction). By hand: for the interior
	// control point (1,2), n = 1*5+2*3 = 11, d = 5^2+3^2 = 34, t = 11/34
	// (inside [0,1]); the closest point is (55/34, 33/34), and the exact
	// squared distance works out to 49/34 -- irrational once rooted, since
	// 34 has no square factor, so no float64 lands on its root exactly.
	span := ratSpan([][2]float64{{0, 0}, {1, 2}, {3, -1}, {5, 3}})

	exactMaxSq := independentMaxChordSquaredDistance(t, span)
	require.Positive(t, exactMaxSq.Sign())

	ref := new(big.Float).SetPrec(200).SetRat(exactMaxSq)
	ref.Sqrt(ref)

	bound := sagittaOf(t, span)
	require.False(t, math.IsNaN(bound))
	boundFloat := new(big.Float).SetPrec(200).SetFloat64(bound)
	require.GreaterOrEqual(t, boundFloat.Cmp(ref), 0,
		"the reported bound must round OUTWARD of the true root, never below it")
}

// independentMaxChordSquaredDistance is TestSpanSagittaUpperRoundsOutward's
// own independent oracle: it re-derives the same maximum-squared-distance-to-
// segment quantity freeform.ChordSegmentSquaredDistance computes, but via a separately
// written clamped-projection formula (Cramer-style, no shared helper with
// internal/freeform/spline_sagitta.go), so agreement between the two is the §1 falsifier
// working rather than one call site echoing the other's arithmetic.
func independentMaxChordSquaredDistance(t *testing.T, span freeform.BezierSpan) *big.Rat {
	t.Helper()
	a, b := span[0], span[len(span)-1]
	abx := new(big.Rat).Sub(b.U, a.U)
	aby := new(big.Rat).Sub(b.V, a.V)
	abLenSq := new(big.Rat).Add(new(big.Rat).Mul(abx, abx), new(big.Rat).Mul(aby, aby))

	var maxSq *big.Rat
	for _, p := range span {
		apx := new(big.Rat).Sub(p.U, a.U)
		apy := new(big.Rat).Sub(p.V, a.V)
		var sq *big.Rat
		if abLenSq.Sign() == 0 {
			// The chord's own two ends coincide (a == b), so the segment
			// degenerates to the single point a — §6.2.1's own explicit
			// "distance to a one-point set" case (internal/freeform/spline_sagitta.go's own
			// freeform.ChordSegmentSquaredDistance doc comment). There is nothing to
			// clamp: the distance is |p-a|^2 directly, worked out here by an
			// independent formula rather than deferring to that function's
			// own d==0 branch, which is exactly the branch A1's own
			// falsification ledger entry targets.
			sq = new(big.Rat).Add(new(big.Rat).Mul(apx, apx), new(big.Rat).Mul(apy, apy))
		} else {
			dot := new(big.Rat).Add(new(big.Rat).Mul(apx, abx), new(big.Rat).Mul(apy, aby))
			s := new(big.Rat).Quo(dot, abLenSq)
			if s.Sign() < 0 {
				s = big.NewRat(0, 1)
			} else if s.Cmp(big.NewRat(1, 1)) > 0 {
				s = big.NewRat(1, 1)
			}
			qx := new(big.Rat).Add(a.U, new(big.Rat).Mul(s, abx))
			qy := new(big.Rat).Add(a.V, new(big.Rat).Mul(s, aby))
			ex := new(big.Rat).Sub(p.U, qx)
			ey := new(big.Rat).Sub(p.V, qy)
			sq = new(big.Rat).Add(new(big.Rat).Mul(ex, ex), new(big.Rat).Mul(ey, ey))
		}
		if maxSq == nil || sq.Cmp(maxSq) > 0 {
			maxSq = sq
		}
	}
	return maxSq
}

// --- A1: the degenerate-chord branch must report |p-a|^2, never a silent 0 ---

// TestSpanSagittaUpperClosedLoopChordIsAPointNotZero pins the audit's own
// most serious finding: a closed free-form loop — first and last control
// point coincident, so the chord collapses to a single point — is exactly a
// shape a loft cap can contribute, and freeform.ChordSegmentSquaredDistance's own
// d==0 branch must still report the true |p-a|^2 for it, never a bolted-on
// 0. The net (0,0) (1,5) (-1,5) (0,0) is a non-collapsed control polygon (the
// FIRST and LAST points coincide; the interior two do not) whose farthest
// control point sits exactly sqrt(26) from the shared chord point.
func TestSpanSagittaUpperClosedLoopChordIsAPointNotZero(t *testing.T) {
	t.Parallel()
	span := ratSpan([][2]float64{{0, 0}, {1, 5}, {-1, 5}, {0, 0}})

	exactMaxSq := independentMaxChordSquaredDistance(t, span)
	require.Zero(t, new(big.Rat).Sub(exactMaxSq, big.NewRat(26, 1)).Sign(),
		"the chord collapses to the origin, so the farthest control point (+-1,5) sits exactly sqrt(26) away")

	// The CURVE's own dense-sampled deviation from the (single-point) chord is
	// only a LOWER bound on the control-point maximum above — the curve is a
	// convex BLEND of the control points, not the control points themselves,
	// so its own departure can and does sit below the hull's own extreme. The
	// bound must still enclose it (that is the contract), never equal it.
	dense := denseChordSegmentDeviation(t, span, 200_000)

	bound := sagittaOf(t, span)
	require.GreaterOrEqual(t, bound, dense, "the reported bound must ENCLOSE the dense-sample deviation")
	require.InDelta(t, math.Sqrt(26), bound, 0.01,
		"a collapsed CHORD (not a collapsed span) must report the true sqrt(26) |p-a| distance, never a silent 0")
}

// --- A2: the accept test must honor BOTH sides, never side 0 alone ---

// TestPairStationsAcceptTestRequiresBothSidesUnderTarget pairs a side whose
// own sagitta already sits under the target with a side whose own sagitta
// sits far over it, so a correct implementation MUST keep subdividing (both
// sides bisect together) until side 1 also meets the target, while an accept
// test reading sag0 alone would accept the very first (unsplit) cell and
// publish a returned sagittaUpper the target never actually bounds.
func TestPairStationsAcceptTestRequiresBothSidesUnderTarget(t *testing.T) {
	t.Parallel()
	small := ratSpan([][2]float64{{0, 0}, {0.33, 0.0001}, {0.66, 0.0001}, {1, 0}})
	large := ratSpan([][2]float64{{0, 0}, {0, 5}, {1, 5}, {1, 0}})

	const target = 0.01
	require.LessOrEqual(t, sagittaOf(t, small), target,
		"side 0 alone must already sit inside the target, or the accept-test bug this fixture targets has nothing to catch")
	require.Greater(t, sagittaOf(t, large), target,
		"side 1 alone must sit outside the target, forcing real subdivision under a correct accept test")

	spans0 := []freeform.BezierSpan{small}
	spans1 := []freeform.BezierSpan{large}
	_, _, _, sagUp, err := freeform.PairStations(spans0, spans1, target, nil, nil) //nolint:dogsled // stations/matchedDelta discarded; only sagittaUpper and err matter here.
	require.NoError(t, err)
	require.LessOrEqual(t, sagUp, target,
		"the achieved sagittaUpper must honor the target on BOTH sides, never side 0 alone")
}

// --- A3: the returned sagittaUpper must reflect the LARGER side, never side 0 alone ---

// TestPairStationsSagittaUpperReflectsTheLargerSide pairs two sides whose
// single (unsplit) cell already meets the target on both — so the walk
// accepts the whole span with NO bisection at all — and pins that the
// RETURNED sagittaUpper reflects the larger of the two readings. A running
// maximum that folds only side 0 into sagittaUpper would report the smaller
// side's own tiny value here instead, understating the published bound by
// the same mechanism the audit measured as a 1000x under-report on a scaled
// pairing.
func TestPairStationsSagittaUpperReflectsTheLargerSide(t *testing.T) {
	t.Parallel()
	small := ratSpan([][2]float64{{0, 0}, {0.33, 0.0001}, {0.66, 0.0001}, {1, 0}})
	large := ratSpan([][2]float64{{0, 0}, {0, 5}, {1, 5}, {1, 0}})

	smallSag := sagittaOf(t, small)
	largeSag := sagittaOf(t, large)
	require.Greater(t, largeSag, smallSag*100, "the fixture needs a large gap between the two sides' own readings")

	// Just above the LARGER side's own reading, so the single cell accepts
	// whole with no bisection at all — the returned value is then a direct
	// readout of whatever the fold computed, not an artifact of subdivision.
	target := largeSag * (1 + 1e-9)
	spans0 := []freeform.BezierSpan{small}
	spans1 := []freeform.BezierSpan{large}
	_, _, _, sagUp, err := freeform.PairStations(spans0, spans1, target, nil, nil) //nolint:dogsled // stations/matchedDelta discarded; only sagittaUpper and err matter here.
	require.NoError(t, err)
	require.InEpsilon(t, largeSag, sagUp, 1e-9,
		"sagittaUpper must reflect the LARGER side's own reading, never the smaller side's")
}

// --- A4: station ORDER — consecutive stations must advance along the chain ---

// TestPairStationsStationsAdvanceMonotonicallyAlongTheChain walks a quarter
// circle (a curve whose angle atan2(v,u) increases strictly and monotonically
// from 0 to pi/2 along its own true parameter) refined finely enough to force
// subdivision across multiple cells and spans, then asserts the returned
// stations' own angles never regress and that consecutive stations sit close
// together. A walk that recursed right-then-left instead of left-then-right
// would scramble the chain's own start into the middle of the list, which
// this monotonicity check cannot survive.
func TestPairStationsStationsAdvanceMonotonicallyAlongTheChain(t *testing.T) {
	t.Parallel()
	spans := quarterCircleFitSpans(t)
	target := levelSagitta(t, spans[0], 3)

	s0, _, _, _, err := freeform.PairStations(spans, spans, target, nil, nil) //nolint:dogsled // stations1/matchedDelta/sagittaUpper discarded; only s0 and err matter here.
	require.NoError(t, err)
	require.Greater(t, len(s0), len(spans)+1,
		"the target must force genuine subdivision across the chain for this test to exercise cross-cell order")

	prevAngle := math.Inf(-1)
	var maxGap float64
	var prevU, prevV float64
	for i, p := range s0 {
		u, v := floatOfRatPoint(t, p)
		angle := math.Atan2(v, u)
		require.GreaterOrEqual(t, angle, prevAngle,
			"station %d must advance the parameter monotonically along the quarter-circle chain, never regress", i)
		if i > 0 {
			maxGap = math.Max(maxGap, math.Hypot(u-prevU, v-prevV))
		}
		prevAngle, prevU, prevV = angle, u, v
	}
	require.Less(t, maxGap, 1.0,
		"consecutive stations must sit close together along a chain refined to a fine target, never scattered by a scrambled walk order")
}

// --- A5: station IDENTITY — the two ends are the chain's own endpoints, exactly ---

// TestPairStationsFirstAndLastStationAreTheChainEndpointsExactly asserts
// stations[0] and the final station are the chain's own start and end
// control points, as exact rational equalities. Emitting a cell's LAST
// control point instead of its FIRST would drop the chain's true start (the
// leftmost leaf's own end is an interior boundary, never the chain start
// once genuinely subdivided) and duplicate its end.
func TestPairStationsFirstAndLastStationAreTheChainEndpointsExactly(t *testing.T) {
	t.Parallel()
	spans := quarterCircleFitSpans(t)
	target := levelSagitta(t, spans[0], 3)

	s0, s1, _, _, err := freeform.PairStations(spans, spans, target, nil, nil)
	require.NoError(t, err)
	require.Greater(t, len(s0), len(spans)+1, "the target must force genuine subdivision for this test to exercise anything")

	requireExactRatPoint := func(t *testing.T, want, got freeform.RatPoint, msg string) {
		t.Helper()
		require.Zero(t, want.U.Cmp(got.U), "%s: U", msg)
		require.Zero(t, want.V.Cmp(got.V), "%s: V", msg)
	}

	requireExactRatPoint(t, spans[0][0], s0[0], "first station side0 must be the chain's own start control point")
	requireExactRatPoint(t, spans[0][0], s1[0], "first station side1 must be the chain's own start control point")
	last := spans[len(spans)-1]
	requireExactRatPoint(t, last[len(last)-1], s0[len(s0)-1], "final station side0 must be the chain's own end control point")
	requireExactRatPoint(t, last[len(last)-1], s1[len(s1)-1], "final station side1 must be the chain's own end control point")
}

// --- A6: charged work MAGNITUDE, not merely its cross-side ratio ---

// TestSagittaCostTermsMatchTheOperationsTheyName re-derives every named term
// of this file's cost model from the operation count its own doc comment
// enumerates, written here as an explicit sum rather than borrowed from the
// constant. A term narrowed below the work it pays for turns freeform.FreeformWork from
// an upper bound into an under-report, so each term is pinned against its own
// derivation and not against itself.
func TestSagittaCostTermsMatchTheOperationsTheyName(t *testing.T) {
	t.Parallel()
	// ratPointAt: 1 big.Int.Lsh, then 2 big.Rat.SetFrac, each NORMALISING (a
	// GCD plus a division of numerator and of denominator).
	const ratPointAtOps = 1 + 3 + 3
	require.Equal(t, ratPointAtOps, freeform.RatPointReconstructCost,
		"ratPointAt's reconstruction must be charged, and charged for SetFrac's own normalisation")

	// freeform.ChordSegmentSquaredDistance's non-degenerate branch. The running-maximum
	// comparison is NOT folded in here: freeform.RatRunningMax charges it on its own
	// call, so one comparison is charged per fold rather than per projection.
	// The most expensive path computes p-a, the dot product, the zero-length
	// check and two interval comparisons, then p-b and its squared length.
	const chordProjectionOps = 2 + (2 + 1) + 1 + 1 + 1 + 2 + (2 + 1)
	require.Equal(t, chordProjectionOps, freeform.ChordProjectionCost,
		"every exact operation the clamped projection performs must be charged")
	require.Equal(t, 1, freeform.RatCompareCost, "the running-maximum comparison is one big.Rat.Cmp")

	// freeform.RatChordFrame: 2 Sub for the chord vector, 2 Mul + 1 Add for its squared
	// length. This is the per-call work the old per-point aggregate charged
	// nothing at all for.
	const chordFrameOps = 2 + (2 + 1)
	require.Equal(t, chordFrameOps, freeform.ChordFrameCost,
		"the shared chord frame every projection reads must be charged")
	require.Equal(t, 2, freeform.ChordVectorCost, "spanChordVector is two big.Rat.Sub")
	require.Equal(t, 2+1, freeform.ChordSquaredCost, "spanChordSquared squares and sums that vector")
	require.Equal(t, 2, freeform.RatPointCopyCost, "a station copy is two big.Rat.Set")

	// freeform.SpanHodographGapSquared per index: hu, hv, the squared norm, the compare.
	const hodographOps = 3 + 3 + (2 + 1) + 1
	require.Equal(t, hodographOps, freeform.HodographGapCost,
		"every exact operation the hodograph hull scan performs must be charged")

	// freeform.RatQuarterOf: one big.NewRat for the factor, then a NORMALISING Mul (a GCD
	// plus a division of numerator and of denominator).
	const ratQuarterOps = 1 + 3
	require.Equal(t, ratQuarterOps, freeform.RatQuarterCost,
		"the exact quartering the matched delta roots must be charged, and charged for the Mul's own normalisation")

	// proofbound.RatSqrtUp: the seed, then at most proofbound.SqrtAdjustLimit walks of two ratSquare
	// probes (floatRat, Mul, Cmp) plus one Nextafter.
	const ratSqrtUpOps = 4 + proofbound.SqrtAdjustLimit*(2*3+1)
	require.GreaterOrEqual(t, freeform.RatSqrtUpCost, ratSqrtUpOps,
		"the per-call outward rounding must be charged, and charged for its whole bounded walk")

	// freeform.DyadicSpanOf per point: two freeform.RatLCM folds (GCD, Quo, Mul) and two
	// freeform.ScaledNumerator scalings (Quo, Mul).
	const dyadicConversionOps = 2*3 + 2*2
	require.Equal(t, dyadicConversionOps, freeform.DyadicConversionCostPerPoint,
		"dyadicSpanOf's own conversion arithmetic must be charged")

	// freeform.DyadicMidpoint: two freeform.AlignedSum, each a big.Int.Lsh plus an Add, plus the
	// second operand's own Lsh where its shift is nonzero — the branch that
	// shifts BOTH, since only an over-count bounds the other.
	const midpointOps = 2 * (1 + 1 + 1)
	require.Equal(t, midpointOps, freeform.DyadicMidpointOps,
		"one exact midpoint blend runs two aligned sums, each of which may shift both operands")

	// split's own per-control-point bookkeeping, outside the blends: three
	// slice allocations and one copy of the parent's points, then one append
	// into each half per level. It is O(n) work that a blend count alone leaves
	// entirely free.
	const splitBookkeepingOps = 3 + 1
	require.Equal(t, splitBookkeepingOps, freeform.DyadicSplitBookkeepingOps,
		"a split's own allocations, copy and appends must be charged, not left free beside its blends")
}

// TestDyadicSplitOpsCountsEveryOperationOneBisectionRuns is the split's own
// under-charge fixture. A bisection of n control points runs n(n-1)/2 midpoint
// blends and each blend is SIX big.Int operations, not the two a charge read
// off the blend count alone spends; it also allocates three slices, copies the
// parent's points and appends into both halves at every level.
//
// freeform.FreeformBracketCost stays on its own coordinate-blend unit and is pinned here
// as STRICTLY CHEAPER than this count, so the gap between the two is a measured
// fact of the tree rather than something a reader has to notice. Closing it
// would refuse the involute record that spends 91% of the shared ceiling today,
// which makes it a §6.1 decision about freeform.FreeformWorkLimit and not an accounting
// repair (freeform.FreeformBracketCost's own doc comment).
func TestDyadicSplitOpsCountsEveryOperationOneBisectionRuns(t *testing.T) {
	t.Parallel()
	for _, n := range []uint64{2, 3, 4, 8, 32} {
		blends := n * (n - 1) / 2
		require.Equal(t, blends*freeform.DyadicMidpointOps+freeform.DyadicSplitBookkeepingOps*n, freeform.DyadicSplitOps(n), "n=%d", n)
		require.Greater(t, freeform.DyadicSplitOps(n), 2*blends,
			"n=%d: two units per blend is the under-charge the metered unit replaces", n)
	}

	leaves := uint64(1) << freeform.FreeformLengthDepth
	require.Less(t,
		freeform.FreeformBracketCost(4),
		freeform.CostAdd(freeform.CostMul(leaves-1, freeform.DyadicSplitOps(4)), freeform.CostMul(leaves, 4)),
		"the arc-length preflight charges its own coordinate-blend unit, knowingly below the metered operation count")
}

// TestDyadicSplitChargesItselfAndRefusesBeforeSplitting pins the callee-side
// contract on the one primitive two files share: split spends its own charge as
// its first statement, at its own operand width, and an exhausted counter gets
// Table R row R7 back with no bisection performed.
func TestDyadicSplitChargesItselfAndRefusesBeforeSplitting(t *testing.T) {
	t.Parallel()
	cell, err := freeform.DyadicSpanOf(nil, ratSpan([][2]float64{{0, 0}, {1, 2}, {2, 2}, {3, 0}}))
	require.NoError(t, err)

	work := freeform.NewFreeformWork()
	_, _, err = cell.Split(work)
	require.NoError(t, err)
	require.Equal(t, freeform.CostMul(freeform.DyadicSplitOps(4), freeform.WidthUnits(cell.SpanWidth())), work.Spent,
		"a split charges its own cost, read off its own control count and operand width")

	exhausted := &freeform.FreeformWork{Spent: freeform.FreeformWorkLimit}
	_, _, err = cell.Split(exhausted)
	require.ErrorIs(t, err, ErrUnsupported, "an exhausted counter refuses the split rather than running it")
}

// TestChargesScaleWithOperandWidth is axis F's own fixture. A charged unit is
// one exact operation on at most one 64-bit word, so the SAME primitive over a
// value thousands of bits wide costs strictly — and proportionally — more than
// over a machine word. A count-only model charges the two the same, which is
// how one full spend of the ceiling could take a quarter of a second on one
// walk and fifteen seconds on another.
func TestChargesScaleWithOperandWidth(t *testing.T) {
	t.Parallel()
	require.Equal(t, uint64(1), freeform.WidthUnits(0), "a value with no bits still pays its operation count once")
	require.Equal(t, uint64(1), freeform.WidthUnits(63))
	require.Equal(t, uint64(2), freeform.WidthUnits(64), "one more unit per 64-bit word")
	require.Equal(t, uint64(1+4096/64), freeform.WidthUnits(4096))

	narrow := ratSpan([][2]float64{{0, 0}, {1, 1}, {2, 0}})
	huge := new(big.Rat).SetFrac(new(big.Int).Lsh(big.NewInt(1), 4096), big.NewInt(3))
	wide := make(freeform.BezierSpan, len(narrow))
	for i, p := range narrow {
		wide[i] = freeform.RatPoint{U: new(big.Rat).Mul(p.U, huge), V: new(big.Rat).Mul(p.V, huge)}
	}

	narrowWork, wideWork := freeform.NewFreeformWork(), freeform.NewFreeformWork()
	_, err := freeform.SpanHodographGapUpper(narrowWork, narrow)
	require.NoError(t, err)
	_, err = freeform.SpanHodographGapUpper(wideWork, wide)
	require.NoError(t, err)

	require.Positive(t, narrowWork.Spent)
	require.Greater(t, wideWork.Spent, 30*narrowWork.Spent,
		"the same hull scan over 4096-bit coordinates must cost orders of magnitude more than over machine words")
}

// TestSagittaAndMatchedDeltaChargeTheirOwnCodePaths pins each reading's total
// against the primitive calls its own code actually makes, and decisively that
// the two are NOT the same number: the matched-delta reading runs
// freeform.SpanHodographGapSquared and one exact quartering where the sagitta runs
// freeform.ChordSegmentSquaredDistance.
//
// It is also the multiplicity fixture. The sagitta reading reconstructs n+2
// points — the two chord ends in ADDITION to its own loop's n — and a charge
// stated at a caller as one rate times n cannot see the extra two at all.
func TestSagittaAndMatchedDeltaChargeTheirOwnCodePaths(t *testing.T) {
	t.Parallel()
	const n = 3
	span := ratSpan([][2]float64{{0, 0}, {1, 1}, {2, 0}})
	cell, err := freeform.DyadicSpanOf(nil, span)
	require.NoError(t, err)
	require.Len(t, cell.Points, n)

	sagWork := freeform.NewFreeformWork()
	_, err = freeform.DyadicSpanSagittaUpper(sagWork, cell)
	require.NoError(t, err)

	mdWork := freeform.NewFreeformWork()
	reconstructed, err := cell.BezierSpan(mdWork)
	require.NoError(t, err)
	_, err = freeform.SpanMatchedDeltaUpper(mdWork, reconstructed)
	require.NoError(t, err)

	// Written from the literals, never from the constants under test. Every
	// coordinate here is a small integer, so every operand is one machine word
	// and each operation count is spent once.
	require.Equal(t, uint64((n+2)*7+5+n*(13+1)+64), sagWork.Spent,
		"the sagitta pays for n+2 reconstructions, one chord frame, n projections, n comparisons and one rounding")
	require.Equal(t, uint64(n*7+2+10*n+4+64), mdWork.Spent,
		"the matched delta pays for n reconstructions, one chord vector, its own per-point hull scan, one exact quartering and one rounding")
	require.NotEqual(t, sagWork.Spent, mdWork.Spent,
		"the two readings must carry their own separately derived charge, never one reused for the other")

	// Replay the original fused reading with its full endpoint projections.
	// Each case must return the same bound and spend the same amount, including
	// when the chord collapses or the control values need multiple words.
	for name, points := range map[string][][2]float64{
		"one point":       {{0, 0}},
		"two points":      {{0, 0}, {2, 1}},
		"overshoot":       {{0, 0}, {-3, 0.01}, {4, 0.01}, {1, 0}},
		"interior":        {{0, 0}, {1, 2}, {3, -1}, {5, 3}},
		"collinear":       {{0, 0}, {1, 0}, {2, 0}},
		"collapsed chord": {{0, 0}, {1, 5}, {-1, 5}, {0, 0}},
		"wide":            {{0, 0}, {1e100, 2e100}, {3e100, 0}},
	} {
		t.Run(name, func(t *testing.T) {
			cell, err := freeform.DyadicSpanOf(nil, ratSpan(points))
			require.NoError(t, err)

			reference := func(w *freeform.FreeformWork) (float64, error) {
				span, err := cell.BezierSpan(w)
				if err != nil {
					return 0, err
				}
				a, b := span[0], span[len(span)-1]
				bax, bay, d, err := freeform.RatChordFrame(w, a, b)
				if err != nil {
					return 0, err
				}
				var maxSq *big.Rat
				for _, p := range span {
					sq, err := freeform.ChordSegmentSquaredDistance(w, p, a, bax, bay, d)
					if err != nil {
						return 0, err
					}
					maxSq, err = freeform.RatRunningMax(w, maxSq, sq)
					if err != nil {
						return 0, err
					}
				}
				return freeform.ChargedRatSqrtUp(w, maxSq)
			}

			originalWork := freeform.NewFreeformWork()
			original, err := reference(originalWork)
			require.NoError(t, err)
			optimizedWork := freeform.NewFreeformWork()
			optimized, _, err := freeform.DyadicSpanSagittaUpperWithSpan(optimizedWork, cell)
			require.NoError(t, err)
			require.Equal(t, original, optimized)
			require.Equal(t, originalWork.Spent, optimizedWork.Spent)

			exact := &freeform.FreeformWork{Spent: freeform.FreeformWorkLimit - originalWork.Spent}
			_, _, err = freeform.DyadicSpanSagittaUpperWithSpan(exact, cell)
			require.NoError(t, err)
			require.Equal(t, freeform.FreeformWorkLimit, exact.Spent)
			short := &freeform.FreeformWork{Spent: freeform.FreeformWorkLimit - originalWork.Spent + 1}
			_, _, err = freeform.DyadicSpanSagittaUpperWithSpan(short, cell)
			require.ErrorIs(t, err, ErrUnsupported)
		})
	}
}

// TestPairStationsChargesEveryPhaseOfAWalkThatNeverSplits pins the whole
// charged total against an independently written closed form, on the one walk
// shape whose charges are fully determined: a target no cell can miss, so every
// span is converted once, measured once, and accepted once, with no split
// anywhere. Dropping any phase — the conversion, either chord-end read, the
// chord frame, the accepted cell's own station read, or the final station's
// copy — changes this number.
func TestPairStationsChargesEveryPhaseOfAWalkThatNeverSplits(t *testing.T) {
	t.Parallel()
	// Two sides of deliberately different control counts, so a charge reading
	// the wrong side's count is visible as well.
	side0 := []freeform.BezierSpan{
		ratSpan([][2]float64{{0, 0}, {1, 1}, {2, 0}}),
		ratSpan([][2]float64{{2, 0}, {3, -1}, {4, 0}}),
	}
	side1 := []freeform.BezierSpan{
		ratSpan([][2]float64{{0, 0}, {1, 2}, {2, 2}, {3, 0}}),
		ratSpan([][2]float64{{3, 0}, {4, -2}, {5, -2}, {6, 0}}),
	}

	work0, work1 := freeform.NewFreeformWork(), freeform.NewFreeformWork()
	_, _, _, _, err := freeform.PairStations(side0, side1, math.Inf(1), work0, work1) //nolint:dogsled // only the charged totals are under test
	require.NoError(t, err)

	require.Equal(t, 2*neverSplittingSpanCharge(3)+2, work0.Spent,
		"side 0's total must be its own per-span charge twice, plus the one final-station copy")
	require.Equal(t, 2*neverSplittingSpanCharge(4)+2, work1.Spent,
		"side 1's total must be its own per-span charge twice, plus the one final-station copy")
}

// neverSplittingSpanCharge is what ONE accepted, never-bisected span of n small
// integer control points costs its own side's counter, written from the
// literals and never from the constants under test: 10n to convert, then the
// fused sagitta reading (n reconstructions at 7, one chord frame at 5, n
// projections at 13, n comparisons at 1, one rounding at 64), then the
// matched-delta reading (one chord vector at 2, n hull indices at 10, one
// exact quartering at 4, one rounding at 64), then the accepted cell's own
// start station at 7. The sagitta's reconstructed span is reused by matched
// delta. Every coordinate is one machine word, so no width multiplier applies.
func neverSplittingSpanCharge(n uint64) uint64 {
	convert := 10 * n
	sagitta := n*7 + 5 + n*(13+1) + 64
	matched := 2 + 10*n + 4 + 64
	station := uint64(7)
	return convert + sagitta + matched + station
}

// TestPairStationsBudgetBindsAtTheWholeChargedTotal pins that the budget
// actually BINDS at the total the charges sum to, on the same never-splitting
// walk: a counter holding exactly that total finishes, and one unit short
// refuses. A charge that is dropped or narrowed lowers the binding point, so a
// walk this fixture expects to refuse would instead succeed.
func TestPairStationsBudgetBindsAtTheWholeChargedTotal(t *testing.T) {
	t.Parallel()
	span := ratSpan([][2]float64{{0, 0}, {1, 1}, {2, 0}})
	total := neverSplittingSpanCharge(3) + 2 // the one span, plus the final-station copy

	exact := &freeform.FreeformWork{Spent: freeform.FreeformWorkLimit - total}
	_, _, _, _, err := freeform.PairStations([]freeform.BezierSpan{span}, []freeform.BezierSpan{span}, math.Inf(1), exact, freeform.NewFreeformWork()) //nolint:dogsled // only the budget boundary is under test
	require.NoError(t, err, "a counter holding exactly the charged total must finish")
	require.Equal(t, freeform.FreeformWorkLimit, exact.Spent)

	short := &freeform.FreeformWork{Spent: freeform.FreeformWorkLimit - total + 1}
	_, _, _, _, err = freeform.PairStations([]freeform.BezierSpan{span}, []freeform.BezierSpan{span}, math.Inf(1), short, freeform.NewFreeformWork()) //nolint:dogsled // only the budget boundary is under test
	require.Error(t, err, "one unit short of the charged total must refuse")
	require.ErrorIs(t, err, ErrUnsupported)
}

// --- B1: a zero-control-point span must refuse, never panic ---

// TestPairStationsRefusesAZeroControlSpanInsteadOfPanicking pins the fix for
// freeform.DyadicSpanSagittaUpper's own dead-ended n==0 guard: with no entry-level
// gate, a zero-control span reaches walkCell, both sides' sagitta reads 0
// (freeform.DyadicSpanSagittaUpper's own guard), the accept test passes trivially,
// and the accept branch's own c0.ratPointAt(0) panics with an index out of
// range on the empty points slice. freeform.PairStations must refuse this input
// cleanly, on either side, before any cell is ever walked.
func TestPairStationsRefusesAZeroControlSpanInsteadOfPanicking(t *testing.T) {
	t.Parallel()
	empty := freeform.BezierSpan{}
	line := ratSpan([][2]float64{{0, 0}, {1, 0}})

	t.Run("side0", func(t *testing.T) {
		_, _, _, _, err := freeform.PairStations([]freeform.BezierSpan{empty}, []freeform.BezierSpan{line}, 1, nil, nil)
		require.Error(t, err)
		require.ErrorIs(t, err, ErrDegenerate)
	})
	t.Run("side1", func(t *testing.T) {
		_, _, _, _, err := freeform.PairStations([]freeform.BezierSpan{line}, []freeform.BezierSpan{empty}, 1, nil, nil)
		require.Error(t, err)
		require.ErrorIs(t, err, ErrDegenerate)
	})
}

// --- B2: the final station must not alias the caller's own input span ---

// TestPairStationsFinalStationDoesNotAliasTheInputSpan mutates a RETURNED
// station's own *big.Rat in place and checks the caller's input span is
// unaffected. ratPointAt's own doc comment guarantees non-aliasing for every
// OTHER station; the final station (appended after the recursive walk,
// straight off the original chain's own last control point) must carry the
// same guarantee.
func TestPairStationsFinalStationDoesNotAliasTheInputSpan(t *testing.T) {
	t.Parallel()
	span := ratSpan([][2]float64{{0, 0}, {1, 0}})
	spans := []freeform.BezierSpan{span}

	// target=1 is far above this straight span's own (zero) sagitta, so the
	// single cell accepts whole with no bisection — the final station is
	// exactly the code path under test.
	s0, _, _, _, err := freeform.PairStations(spans, spans, 1, nil, nil) //nolint:dogsled // stations1/matchedDelta/sagittaUpper discarded; only s0 and err matter here.
	require.NoError(t, err)

	wantU := new(big.Rat).Set(span[len(span)-1].U) // the input's own value, before any mutation
	last := s0[len(s0)-1]
	last.U.Add(last.U, big.NewRat(1, 1)) // mutate the RETURNED station in place

	require.Zero(t, wantU.Cmp(span[len(span)-1].U),
		"mutating a returned station must never change the caller's own input span")
}

// --- C: matched-delta primitives (internal/proofbound/bounds.go's proofbound.CellChordCurveAreaUpper
// matchedDeltaUpper obligation) — freeform.SpanHodographGapUpper, freeform.SpanMatchedDeltaUpper,
// freeform.SpanSpeedUpper ---

// denseMatchedDeviation samples a single-span chain densely and returns the
// maximum true |C(t) - (P_0 + t*Delta)| over the span's own NATIVE parameter
// t — the parameter-matched deviation freeform.SpanMatchedDeltaUpper bounds, sampled
// through the same independent de Casteljau oracle (evalSpans) the sagitta
// tests already use, never through any of internal/freeform/spline_sagitta.go's own machinery.
func denseMatchedDeviation(t *testing.T, span freeform.BezierSpan, samples int) float64 {
	t.Helper()
	floatSpan := floatBezierSpanOf(span)
	ax, ay := evalFloatBezierSpan(floatSpan, 0)
	bx, by := evalFloatBezierSpan(floatSpan, 1)
	dx, dy := bx-ax, by-ay
	maxDev := 0.0
	for i := 0; i <= samples; i++ {
		at := float64(i) / float64(samples)
		cx, cy := evalFloatBezierSpan(floatSpan, at)
		lx, ly := ax+at*dx, ay+at*dy
		maxDev = math.Max(maxDev, math.Hypot(cx-lx, cy-ly))
	}
	return maxDev
}

// denseSpeedSample samples ||C'(t)|| densely via a central finite difference
// over evalSpans — an INDEPENDENT numerical estimate, never a reuse of
// freeform.SpanHodographGapUpper's own exact-rational hodograph.
func denseSpeedSample(t *testing.T, span freeform.BezierSpan, samples int) float64 {
	t.Helper()
	const h = 1e-5
	floatSpan := floatBezierSpanOf(span)
	maxSpeed := 0.0
	for i := 0; i <= samples; i++ {
		at := float64(i) / float64(samples)
		at0, at1 := math.Max(0, at-h), math.Min(1, at+h)
		if at1 <= at0 {
			continue
		}
		x0, y0 := evalFloatBezierSpan(floatSpan, at0)
		x1, y1 := evalFloatBezierSpan(floatSpan, at1)
		speed := math.Hypot(x1-x0, y1-y0) / (at1 - at0)
		maxSpeed = math.Max(maxSpeed, speed)
	}
	return maxSpeed
}

// zigzagHuggingSpan is the free-form analogue of bounds_chord_internal_test.go's
// own TestCellChordCurveAreaUpperRefusesTheSagittaZigzag counterexample: every
// control point sits exactly ON the chord segment [(0,0),(1,0)] (collinear,
// v=0 throughout), so the sagitta is EXACTLY 0 — the strongest possible zero,
// by exact float equality, never merely a small one — while three of the four
// control points cluster at u=0.001. Bernstein blending over that
// non-uniformly-spaced collinear net packs almost all of the curve's own
// motion along the parameter into short stretches near t=0 and t=1, leaving
// the curve's own position at t=0.5 far short of the chord's own midpoint —
// the SAME mechanism TestSpanSagittaUpperDistinguishesFromParametricDeviation
// already demonstrates for a milder clustering, pushed here into a decisive
// numeric gap between the sagitta and the true parameter-matched deviation.
func zigzagHuggingSpan() freeform.BezierSpan {
	return ratSpan([][2]float64{{0, 0}, {0.001, 0}, {0.001, 0}, {1, 0}})
}

// TestSpanMatchedDeltaUpperEnclosesWhatTheSagittaMisses is the decisive C4
// fixture: it densely proves the sagitta of 0 FAILS to bound the true
// parameter-matched deviation on zigzagHuggingSpan, and that
// freeform.SpanMatchedDeltaUpper (d/2) DOES bound it. This is F1's rule made
// concrete: proofbound.CellChordCurveAreaUpper's matchedDeltaUpper obligation is a
// strictly stronger claim than the sagitta, and confusing the two is exactly
// the unsoundness this function exists to prevent a downstream caller from
// committing.
func TestSpanMatchedDeltaUpperEnclosesWhatTheSagittaMisses(t *testing.T) {
	t.Parallel()
	span := zigzagHuggingSpan()

	sagitta := sagittaOf(t, span)
	require.Zero(t, sagitta, "every control point sits exactly on the chord segment, so the sagitta is exactly 0")

	dense := denseMatchedDeviation(t, span, 200_000)
	require.Greater(t, dense, 0.3, "the fixture needs a substantial true parameter-matched deviation for this test to mean anything")

	require.Less(t, sagitta, dense,
		"pinning F1's own violation: a sagitta of 0 must FAIL to bound the true parameter-matched deviation of %.6g", dense)

	matched := matchedDeltaOf(t, span)
	require.GreaterOrEqual(t, matched, dense,
		"spanMatchedDeltaUpper must ENCLOSE the true parameter-matched deviation, where the sagitta above does not")
}

// TestSpanMatchedDeltaUpperEnclosesDenseSampleOnOrdinarySpans checks the
// bound holds — not merely on the decisive counterexample above — on an
// ordinary curved chain.
func TestSpanMatchedDeltaUpperEnclosesDenseSampleOnOrdinarySpans(t *testing.T) {
	t.Parallel()
	spans := quarterCircleFitSpans(t)
	for i, span := range spans {
		dense := denseMatchedDeviation(t, span, 20_000)
		matched := matchedDeltaOf(t, span)
		require.GreaterOrEqual(t, matched, dense,
			"span %d: matchedDeltaUpper must enclose the dense-sampled parameter-matched deviation", i)
	}
}

// negativePowerOfTwo builds the exact rational 2^-k, a value big.Rat holds
// with no exponent range of its own and therefore no underflow, however large
// k grows.
func negativePowerOfTwo(k uint) *big.Rat {
	return new(big.Rat).SetFrac(big.NewInt(1), new(big.Int).Lsh(big.NewInt(1), k))
}

// nearMidpointQuadraticSpan builds the quadratic span
// P_0 = (0,0), P_1 = (1/2 + eps, 0), P_2 = (1,0), whose every quantity is a
// closed form in eps: the hodograph gap is d = 2*eps (both control points of
// C'(t) - Delta read +-2*eps), and the TRUE parameter-matched deviation at
// t = 1/2 is |C(1/2) - (P_0 + Delta/2)| = eps/2, since C(1/2) is the Bezier
// midpoint (P_0 + 2*P_1 + P_2)/4 = 1/2 + eps/2 while the chord's own midpoint
// is 1/2. Every control point is collinear, so the span's sagitta is exactly 0
// and only the parameter-matched reading sees the deviation at all.
func nearMidpointQuadraticSpan(eps *big.Rat) freeform.BezierSpan {
	half := big.NewRat(1, 2)
	return freeform.BezierSpan{
		{U: new(big.Rat), V: new(big.Rat)},
		{U: new(big.Rat).Add(half, eps), V: new(big.Rat)},
		{U: big.NewRat(1, 1), V: new(big.Rat)},
	}
}

// TestSpanMatchedDeltaUpperNeverUnderflowsASubnormalGapToZero is the D1
// fixture. It pins the whole window in which a float halving of the published
// hodograph gap loses the bound outright, rather than the single span that
// first exposed it.
//
// The window is not a knife edge. proofbound.RatSqrtUp reports the smallest subnormal for
// EVERY positive exact radicand at or below (2^-1074)^2, because big.Rat has no
// underflow to lose the radicand in, so every span whose exact gap d sits at or
// below 2^-1074 publishes the same subnormal d. Halving that float in float
// arithmetic gives exactly 0 (2^-1075 is the tie between 0 and 2^-1074, and
// round-to-nearest-even takes the zero), so the whole window would publish a
// matched delta of 0 against a positive true deviation. Every row asserts the
// float halving DOES underflow, so the row is proven to sit inside the window
// and not merely to pass.
//
// A published 0 is not a narrow bound here, it is an absent one:
// proofbound.CellChordCurveAreaUpper (internal/proofbound/bounds.go) gates its chord-to-curve leg on
// matchedDelta > 0 and drops the leg entirely at 0. The enclosure is therefore
// asserted over the exact rationals, against eps/2 itself — a dense float
// sample cannot even represent the deviations in this window.
func TestSpanMatchedDeltaUpperNeverUnderflowsASubnormalGapToZero(t *testing.T) {
	t.Parallel()
	// eps must sit at or below 2^-1075 for the gap d = 2*eps to land in the
	// window; 2^-1101 is the span the audit reproduced with.
	for _, exp := range []uint{1075, 1101, 2000, 9000} {
		t.Run(fmt.Sprintf("eps=2^-%d", exp), func(t *testing.T) {
			eps := negativePowerOfTwo(exp)
			span := nearMidpointQuadraticSpan(eps)

			gap := hodographGapOf(t, span)
			require.Equal(t, math.SmallestNonzeroFloat64, gap,
				"the exact gap 2*eps rounds outward to the smallest subnormal, which is what puts this row in the window")
			require.Zero(t, gap/2,
				"halving the published float gap underflows to zero, which is the loss this fixture exists to catch")

			matched := matchedDeltaOf(t, span)
			require.Positive(t, matched,
				"the true parameter-matched deviation is eps/2 > 0, so the published bound must be positive too")

			trueDeviation := new(big.Rat).Quo(eps, big.NewRat(2, 1))
			published := proofarith.FloatRat(matched)
			require.NotNil(t, published)
			require.GreaterOrEqual(t, published.Cmp(trueDeviation), 0,
				"the published bound must enclose the exact deviation eps/2")

			require.Zero(t, sagittaOf(t, span),
				"the controls are collinear, so only the parameter-matched reading sees this deviation")
		})
	}
}

// TestSpanMatchedDeltaUpperHalvesAnOrdinaryGapExactly checks the same exact
// path away from the bottom of the range: where no rounding is forced, the
// matched delta reads exactly half the hodograph gap, so moving the halving
// into the radicand costs an ordinary span nothing.
func TestSpanMatchedDeltaUpperHalvesAnOrdinaryGapExactly(t *testing.T) {
	t.Parallel()
	// d = 2*eps = 1 exactly for eps = 1/2, and 1/2 is representable.
	span := nearMidpointQuadraticSpan(big.NewRat(1, 2))
	require.Equal(t, 1.0, hodographGapOf(t, span))
	require.Equal(t, 0.5, matchedDeltaOf(t, span))

	for i, span := range quarterCircleFitSpans(t) {
		gap := hodographGapOf(t, span)
		matched := matchedDeltaOf(t, span)
		require.LessOrEqual(t, matched, proofbound.UpRound(gap/2),
			"span %d: rooting d^2/4 must never read above the float halving of d", i)
		require.Positive(t, matched, "span %d: a curved span carries a positive parameter-matched deviation", i)
	}
}

// TestSpanSpeedUpperEnclosesDenseSampleAndNeverFallsBelowChordLength checks
// both of freeform.SpanSpeedUpper's own obligations: it encloses a dense finite-
// difference sample of ||C'(t)||, and it never reads below the span's own
// chord length — the floor proofbound.CellChordCurveAreaUpper's own tangent-magnitude
// argument depends on.
func TestSpanSpeedUpperEnclosesDenseSampleAndNeverFallsBelowChordLength(t *testing.T) {
	t.Parallel()
	spans := quarterCircleFitSpans(t)
	for i, span := range spans {
		dense := denseSpeedSample(t, span, 2000)
		speed := speedOf(t, span)
		require.GreaterOrEqual(t, speed, dense*(1-1e-6),
			"span %d: speed bound must enclose the dense-sampled ||C'(t)||", i)

		ax, ay := floatOfRatPoint(t, span[0])
		bx, by := floatOfRatPoint(t, span[len(span)-1])
		chordLen := math.Hypot(bx-ax, by-ay)
		require.GreaterOrEqual(t, speed, chordLen, "span %d: speed bound must never fall below the chord length", i)
	}
}

// TestHodographBoundsAreExactlyZeroOnCollapsedAndStraightUniformSpans checks
// the zero readings §6.2.1's own philosophy predicts from the general
// formulas, with no special case: on a fully COLLAPSED span (every control
// point coincident, chord length 0 too), all three quantities read exactly
// 0. On a STRAIGHT, uniformly-spaced span (collinear controls at equal
// parameter spacing — genuine constant-speed motion along a chord of
// POSITIVE length), the two GAP terms (freeform.SpanHodographGapUpper,
// freeform.SpanMatchedDeltaUpper) still read exactly 0, because a uniformly-spaced
// collinear net's hodograph is the CONSTANT vector Delta itself at every
// control point; the speed bound is NOT zero there — it reads exactly the
// span's own chord length, since d=0 leaves nothing to widen it by.
func TestHodographBoundsAreExactlyZeroOnCollapsedAndStraightUniformSpans(t *testing.T) {
	t.Parallel()
	collapsed := ratSpan([][2]float64{{2, 3}, {2, 3}, {2, 3}, {2, 3}})
	require.Zero(t, hodographGapOf(t, collapsed))
	require.Zero(t, matchedDeltaOf(t, collapsed))
	require.Zero(t, speedOf(t, collapsed), "a fully collapsed span (chord length 0 too) has zero speed as well as zero deviation")

	straight := ratSpan([][2]float64{{0, 0}, {1, 0}, {2, 0}}) // uniformly-spaced collinear controls: constant-speed line
	require.Zero(t, hodographGapOf(t, straight))
	require.Zero(t, matchedDeltaOf(t, straight))
	chordLen := math.Hypot(2, 0)
	require.InEpsilon(t, chordLen, speedOf(t, straight), 1e-12,
		"a straight uniformly-spaced span's speed bound must equal its own chord length exactly (d=0), never merely enclose it")
}

// --- freeform.ChainStations: the one-sided twin (docs/tessellation-reach-design.md §5) ---

// TestChainStationsIsOneSideOfThePairWalk pins the refactor's own contract: a
// chain walked on its own must settle on exactly the cells the pair walk
// settles on when both of its sides are that same chain, and must report the
// same measured sagitta. The station lists differ in one documented way only —
// freeform.ChainStations EXCLUDES the chain's final boundary from stations and carries
// it in end instead — so the pair's own list is reproduced by appending end.
func TestChainStationsIsOneSideOfThePairWalk(t *testing.T) {
	t.Parallel()
	spans := quarterCircleFitSpans(t)
	const target = 1e-4

	paired, _, _, pairSag, err := freeform.PairStations(spans, spans, target, nil, nil)
	require.NoError(t, err)

	chain, err := freeform.ChainStations(spans, target, nil)
	require.NoError(t, err)

	require.Equal(t, pairSag, chain.Sagitta, "the one-sided walk must measure the identical sagitta, bit for bit")
	require.Len(t, chain.Stations, len(paired)-1,
		"the chain carries one station per CELL; the pair carries one per cell BOUNDARY, which is one more")
	for i := range chain.Stations {
		require.Zero(t, paired[i].U.Cmp(chain.Stations[i].U), "station %d U must be the pair walk's own", i)
		require.Zero(t, paired[i].V.Cmp(chain.Stations[i].V), "station %d V must be the pair walk's own", i)
	}
	last := paired[len(paired)-1]
	require.Zero(t, last.U.Cmp(chain.End.U), "the chain's end must be the pair walk's own final station")
	require.Zero(t, last.V.Cmp(chain.End.V))
}

// TestChainStationsHonorsItsTargetAndLoosensWithIt asserts the measured sagitta
// stays at or under the target it was asked for, and that a strictly laxer
// target settles on strictly fewer cells — the walk tracks its target rather
// than a fixed depth.
func TestChainStationsHonorsItsTargetAndLoosensWithIt(t *testing.T) {
	t.Parallel()
	spans := quarterCircleFitSpans(t)

	fine, err := freeform.ChainStations(spans, 1e-4, nil)
	require.NoError(t, err)
	require.LessOrEqual(t, fine.Sagitta, 1e-4, "the measured sagitta must honor the target it was asked for")

	coarse, err := freeform.ChainStations(spans, 1e-2, nil)
	require.NoError(t, err)
	require.LessOrEqual(t, coarse.Sagitta, 1e-2)
	require.Less(t, len(coarse.Stations), len(fine.Stations),
		"a laxer target must settle on strictly fewer cells")
}

// TestChainStationsBracketsEveryCellArcByItsChord pins the per-cell readings
// the chorded wall's area slack subtracts: one arc-length upper bound and one
// chord-length lower bound per accepted cell, parallel to the stations, with
// the arc never below the chord. On a straight, uniformly-spaced span the two
// converge on that span's own exact chord length from opposite sides, which is
// what proves each is rounded in its own direction rather than both in one.
func TestChainStationsBracketsEveryCellArcByItsChord(t *testing.T) {
	t.Parallel()
	curved, err := freeform.ChainStations(quarterCircleFitSpans(t), 1e-3, nil)
	require.NoError(t, err)
	require.Len(t, curved.CellArcUpper, len(curved.Stations), "one arc reading per accepted cell")
	require.Len(t, curved.CellChordLower, len(curved.Stations), "one chord reading per accepted cell")
	summedChord, summedArc := 0.0, 0.0
	for k := range curved.CellArcUpper {
		require.Positive(t, curved.CellChordLower[k], "cell %d spans a positive chord", k)
		require.GreaterOrEqual(t, curved.CellArcUpper[k], curved.CellChordLower[k],
			"cell %d: a chord never exceeds the arc it subtends", k)
		summedChord += curved.CellChordLower[k]
		summedArc += curved.CellArcUpper[k]
	}
	// The fixture interpolates a quarter circle of radius 5, whose arc length
	// is 5*pi/2. The inscribed chords of a chain this fine sit just under that
	// figure and the speed bounds sit above it, so the true length is bracketed.
	quarter := 5 * math.Pi / 2
	require.Less(t, summedChord, quarter, "the inscribed chords never reach the arc they subtend")
	require.Greater(t, summedChord, quarter*0.99, "a chain this fine loses well under a percent to chording")
	require.Greater(t, summedArc, quarter, "the summed speed bounds must enclose the arc length")

	// A collinear, uniformly spaced diagonal span: sagitta exactly 0, so it is
	// accepted whole, and its exact chord length is sqrt(2), which no float64
	// represents. That is what makes the two roundings distinguishable — each
	// must land on its OWN side of the exact value, one ulp apart.
	diagonal := ratSpan([][2]float64{{0, 0}, {0.5, 0.5}, {1, 1}})
	straight, err := freeform.ChainStations([]freeform.BezierSpan{diagonal}, 1, nil)
	require.NoError(t, err)
	require.Len(t, straight.Stations, 1, "a collinear span carries sagitta 0 and is accepted whole")
	require.Zero(t, straight.Sagitta)
	require.GreaterOrEqual(t, straight.CellArcUpper[0]*straight.CellArcUpper[0], 2.0,
		"the arc reading must square to at least the exact squared chord length 2")
	require.LessOrEqual(t, straight.CellChordLower[0]*straight.CellChordLower[0], 2.0,
		"the chord reading must square to at most the exact squared chord length 2")
	require.Greater(t, straight.CellArcUpper[0], straight.CellChordLower[0],
		"the two readings must straddle the exact value, never both land on one side of it")
	require.InDelta(t, math.Sqrt2, straight.CellChordLower[0], 1e-15)
}

// TestChainStationsRefusals pins every entry gate the one-sided walk inherits
// from the pair walk: a chain with no span at all, a span carrying no control
// point, a chain of more spans than the chord cap admits, and a target no
// bisection this cap allows can reach.
func TestChainStationsRefusals(t *testing.T) {
	t.Parallel()
	t.Run("no span", func(t *testing.T) {
		_, err := freeform.ChainStations(nil, 1, nil)
		require.ErrorIs(t, err, ErrDegenerate)
	})
	t.Run("a span with no control point", func(t *testing.T) {
		_, err := freeform.ChainStations([]freeform.BezierSpan{{}}, 1, nil)
		require.ErrorIs(t, err, ErrDegenerate)
	})
	t.Run("more spans than the cap admits", func(t *testing.T) {
		spans := make([]freeform.BezierSpan, freeform.MaxChordsPerWalk+1)
		for i := range spans {
			spans[i] = straightSpanFrom(float64(2 * i))
		}
		_, err := freeform.ChainStations(spans, 1, nil)
		require.ErrorIs(t, err, freeform.ErrTooManyChords)
		require.ErrorIs(t, err, ErrUnsupported)
	})
	t.Run("a target past the cap's reach", func(t *testing.T) {
		_, err := freeform.ChainStations([]freeform.BezierSpan{parabolaSpan()}, 1e-20, nil)
		require.ErrorIs(t, err, freeform.ErrTooManyChords)
		require.ErrorIs(t, err, ErrUnsupported)
	})
}

// TestChainStationsChargesTheCounterItIsHanded pins that the one-sided walk
// meters itself on the caller's own record counter: an exhausted counter
// refuses with Table R row R7's sentinel and the walk does no work at all.
func TestChainStationsChargesTheCounterItIsHanded(t *testing.T) {
	t.Parallel()
	spans := quarterCircleFitSpans(t)

	work := freeform.NewFreeformWork()
	_, err := freeform.ChainStations(spans, 1e-3, work)
	require.NoError(t, err)
	require.Positive(t, work.Spent, "the walk must charge the counter it was handed")

	exhausted := &freeform.FreeformWork{Spent: freeform.FreeformWorkLimit}
	_, err = freeform.ChainStations(spans, 1e-3, exhausted)
	require.ErrorIs(t, err, ErrUnsupported)
}

// --- D: the free-form tangent energy (docs/loft-design.md §5.2's
// tangentEnergy_k row) — freeform.SpanTangentEnergyUpper ---

// denseTangentEnergy integrates |C'(t) − Δ|² over a span's native [0, 1] in
// float64 by 4-point Gauss-Legendre on 256 equal pieces. C'(t) is the float
// hodograph p·(P_{i+1} − P_i) evaluated by evalFloatBezierSpan's independent de
// Casteljau, never by freeform's exact coefficient path.
func denseTangentEnergy(span freeform.BezierSpan) float64 {
	fs := floatBezierSpanOf(span)
	p := float64(len(fs) - 1)
	hod := make(floatBezierSpan, len(fs)-1)
	for i := range hod {
		hod[i] = [2]float64{p * (fs[i+1][0] - fs[i][0]), p * (fs[i+1][1] - fs[i][1])}
	}
	du, dv := fs[len(fs)-1][0]-fs[0][0], fs[len(fs)-1][1]-fs[0][1]
	nodes := [4][2]float64{
		{-0.3399810435848563, 0.6521451548625461},
		{0.3399810435848563, 0.6521451548625461},
		{-0.8611363115940526, 0.3478548451374538},
		{0.8611363115940526, 0.3478548451374538},
	}
	const pieces = 256
	total := 0.0
	for k := range pieces {
		mid := (float64(k) + 0.5) / pieces
		for _, nd := range nodes {
			at := mid + nd[0]/(2*pieces)
			u, v := evalFloatBezierSpan(hod, at)
			total += nd[1] / (2 * pieces) * ((u-du)*(u-du) + (v-dv)*(v-dv))
		}
	}
	return total
}

func tangentEnergyOf(t *testing.T, span freeform.BezierSpan) float64 {
	t.Helper()
	energy, err := freeform.SpanTangentEnergyUpper(nil, span)
	require.NoError(t, err)
	return energy
}

// TestSpanTangentEnergyUpperIsTheExactIntegral pins the energy as the
// integral itself, rounded outward once, never an enclosure wider than that
// rounding.
//
// The quadratic P_0 = (0,0), P_1 = (1/2 + ε, 0), P_2 = (1,0) has the closed
// form C'(t) − Δ = 2ε·(1 − 2t), so J = 4ε²/3, and at ε = 1/8 the published
// value must be exactly 1/48 rounded up. A degree-1 span has C' = Δ, so J is
// exactly 0. On the A10b fit spline's spans, a cubic and a quartic, J agrees
// with an independent float quadrature to 1e-12 relative.
//
// Shown to fail first: with BernsteinSquaredNormIntegral's 1/C(2q,k) weight
// dropped (every k weighted 1), the closed-form 1/48 case fails. With the
// binomial scaling in SpanTangentDeviationCoefficients dropped, the quartic's
// quadrature comparison fails. The cubics cannot see that mutation: for q = 2
// it changes J by h_1·(h_0 + h_1 + h_2)/10, and that sum is 3·∫e dt = 0.
func TestSpanTangentEnergyUpperIsTheExactIntegral(t *testing.T) {
	t.Parallel()
	require.Equal(t, proofbound.RatFloatUp(big.NewRat(1, 48)), tangentEnergyOf(t, nearMidpointQuadraticSpan(big.NewRat(1, 8))))
	require.Equal(t, proofbound.RatFloatUp(big.NewRat(4, 3)), tangentEnergyOf(t, ratSpan([][2]float64{{0, 0}, {1, 1}, {2, 0}})))

	require.Zero(t, tangentEnergyOf(t, ratSpan([][2]float64{{0.25, -3}, {7, 1.5}})), "a degree-1 span's tangent IS its chord")
	require.Zero(t, tangentEnergyOf(t, ratSpan([][2]float64{{0, 0}, {1, 0}, {2, 0}})), "a uniformly spaced straight span runs at its chord's own velocity")
	require.Zero(t, tangentEnergyOf(t, ratSpan([][2]float64{{4, 4}})), "a one-point span has no chord")

	spans := append(quarterCircleFitSpans(t),
		ratSpan([][2]float64{{0, 0}, {-3, 0.01}, {4, 0.01}, {1, 0}}),
		ratSpan([][2]float64{{0, 0}, {1, 2}, {3, -1}, {5, 3}, {6, 0}}))
	for i, span := range spans {
		got := tangentEnergyOf(t, span)
		ref := denseTangentEnergy(span)
		require.Positive(t, ref, "span %d must carry a tangent deviation for this comparison to mean anything", i)
		require.InEpsilon(t, ref, got, 1e-12, "span %d: the published energy must be the integral itself", i)

		// The premise-free reading CellChordCurveAreaAllow falls back to is
		// (speed + chord)², which must never be below the exact energy.
		chord := math.Hypot(floatOfRatPointDiff(t, span[len(span)-1], span[0]))
		free := (speedOf(t, span) + chord) * (speedOf(t, span) + chord)
		require.LessOrEqual(t, got, free, "span %d", i)
	}
}

// floatOfRatPointDiff is b − a in float64, for a chord length a test compares
// loosely.
func floatOfRatPointDiff(t *testing.T, b, a freeform.RatPoint) (float64, float64) {
	t.Helper()
	bu, bv := floatOfRatPoint(t, b)
	au, av := floatOfRatPoint(t, a)
	return bu - au, bv - av
}

// TestSpanTangentEnergyUpperChargesItsOwnCodePath pins the energy's charge,
// written from the literals rather than the constants under test, on a span of
// one-word integers: the chord vector (2), one coefficient pass over three
// points (14 each), the product integral over q = 1 (four pairs at 12, three
// product coefficients at 8, and the closing 3), and one outward rounding (4).
// A counter one unit short refuses as R7, and one that covers it exactly ends
// at the limit.
//
// Shown to fail first: with BernsteinSquaredNormIntegral's own charge removed,
// the total drops by 75 and the equality fails.
func TestSpanTangentEnergyUpperChargesItsOwnCodePath(t *testing.T) {
	t.Parallel()
	span := ratSpan([][2]float64{{0, 0}, {1, 1}, {2, 0}})
	const want = 2 + 3*14 + (4*12 + 3*8 + 3) + 4

	work := freeform.NewFreeformWork()
	_, err := freeform.SpanTangentEnergyUpper(work, span)
	require.NoError(t, err)
	require.Equal(t, uint64(want), work.Spent)

	exact := &freeform.FreeformWork{Spent: freeform.FreeformWorkLimit - want}
	_, err = freeform.SpanTangentEnergyUpper(exact, span)
	require.NoError(t, err)
	require.Equal(t, freeform.FreeformWorkLimit, exact.Spent)

	short := &freeform.FreeformWork{Spent: freeform.FreeformWorkLimit - want + 1}
	_, err = freeform.SpanTangentEnergyUpper(short, span)
	require.ErrorIs(t, err, ErrUnsupported)
}
