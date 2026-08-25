package decad

import (
	"errors"
	"math"
	"math/big"
	"testing"

	"github.com/stretchr/testify/require"
)

// This file asserts docs/spline-design.md §6.2.1's chord-sagitta bound
// (spanSagittaUpper) and the shared dyadic station generator built on top of
// it (pairStations), both in spline_sagitta.go.
//
// FALSIFICATION LEDGER — a10-plan.md Part 3 PR 8's own mandatory protocol.
// Every leg below was broken IN spline_sagitta.go, `go test` was run against
// the fixture that exists to catch it, the fixture was watched go RED, and
// the file was then restored (`git diff` confirmed clean before this file was
// committed). A leg with no red run has its redundancy argued instead, never
// silently skipped.
//
//   - Remove the [0,1] clamp in chordSegmentSquaredDistance (project onto the
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
//   - Swap ratSqrtUp for ratSqrtDown in dyadicSpanSagittaUpper's final
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
//     (count every walkCell entry against maxChordsPerWalk):
//     TestPairStationsAcceptsTheStatedChordCap went red — a walk needing
//     exactly maxChordsPerWalk chords was refused with errTooManyChords
//     barely halfway through, because a binary refinement visits 2L−m cells
//     for L chords over m spans and so trips a visit-charged ceiling at
//     roughly half the chord count that ceiling's own message names.
//   - Drop pairStations' span-count entry guard:
//     TestPairStationsSpanCountPastTheCapRefusesUpFront went red — a chain of
//     maxChordsPerWalk+1 straight spans returned that many chords and no
//     error, one past the count errTooManyChords names.
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
//   - A1: chordSegmentSquaredDistance's d==0 branch (a degenerate CHORD, e.g.
//     a closed free-form loop whose first and last control point coincide —
//     not a collapsed SPAN) returning 0 instead of |p-a|^2:
//     TestSpanSagittaUpperClosedLoopChordIsAPointNotZero went red — the
//     reported bound fell from ~5.099 to 0 on a net whose farthest control
//     point sits exactly sqrt(26) from the shared chord point. This gap
//     existed because the test file's own independent oracle,
//     independentMaxChordSquaredDistance, asserted abLenSq.Sign() nonzero and
//     so could not exercise this branch at all; it is now fixed to answer
//     |p-a|^2 directly for a degenerate chord, the same derivation
//     chordSegmentSquaredDistance's own doc comment states.
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
//   - A6: sagittaMeasureCostPerPoint narrowed from 8 to 1:
//     TestSagittaMeasureCostMatchesDocumentedFormula went red, while
//     TestPairStationsChargesBothCountersSeparately — which pins only the
//     RATIO between two differently-sized sides' own charges — stayed green,
//     confirming the audit's own finding that a uniform rescaling of the cost
//     constant is invisible to a ratio-only fixture.
//   - B1: dyadicSpanSagittaUpper's own n==0 guard was already dead-ended —
//     nothing in pairStations prevented a zero-control-point span from
//     reaching walkCell, whose accept branch then panicked on
//     c0.ratPointAt(0)'s empty slice. Removing the new entry-level guard and
//     running TestPairStationsRefusesAZeroControlSpanInsteadOfPanicking
//     reproduced exactly that panic (index out of range on
//     dyadicSpan.ratPointAt); pairStations now refuses a zero-control span on
//     either side with ErrDegenerate before any cell is walked, so the
//     dead-ended guard's own 0 answer is no longer reachable from this walk.
//   - B2: the final station's own append reverted from a fresh
//     ratPoint{u: new(big.Rat).Set(...), v: new(big.Rat).Set(...)} copy back
//     to appending the caller's own last control point ratPoint directly:
//     TestPairStationsFinalStationDoesNotAliasTheInputSpan went red —
//     mutating the returned station's own *big.Rat in place changed the
//     caller's input span, violating ratPointAt's own non-aliasing contract
//     every OTHER station in the two returned lists already carries.
//
// Part C's three new primitives (spanHodographGapUpper, spanMatchedDeltaUpper,
// spanSpeedUpper) are new proofs, not fixes to existing broken legs, so they
// carry no mutation-of-existing-code entry here. Their own soundness rests on
// TestSpanMatchedDeltaUpperEnclosesWhatTheSagittaMisses (the decisive
// zigzag-hugging fixture proving the sagitta is NOT a substitute) and
// TestHodographBoundsAreExactlyZeroOnCollapsedAndStraightUniformSpans, which a
// quick sanity check confirmed IS sensitive to a broken derivation: dropping
// the hodograph's own "- Delta" term (silencing it via a x0 multiply so the
// build still compiles) left the dense-sample enclosure tests green — a wider
// bound still encloses — but turned the straight-uniform-span's own EXACT
// zero reading into 2, which the zero-reading fixture caught immediately.

// ratSpan is spline_extreme_internal_test.go's own helper (same package): it
// builds a bezierSpan directly from plane-local float coordinates, for tests
// that exercise this file's machinery without going through a recorded
// segment.

// quarterCircleFitSpans converts a 5-point Tier A FitSplineSeg through the
// same radius-5 quarter circle loft_chord_calibration_internal_test.go's own
// wedgeFitSpline builds via sketch (k*pi/8 for k = 0..4) — built directly on
// the record here, with no sketch dependency, since fitSplineBezierSpans
// (spline_fit.go) takes a FitSplineSeg's Fit points on their own.
func quarterCircleFitSpans(t *testing.T) []bezierSpan {
	t.Helper()
	const radius = 5.0
	fit := make([]Point2, 5)
	for k := range fit {
		theta := float64(k) * math.Pi / 8
		fit[k] = Point2{U: radius * math.Cos(theta), V: radius * math.Sin(theta)}
	}
	spans, err := fitSplineBezierSpans(FitSplineSeg{Fit: fit, TStart: 0, TEnd: 1}, newFreeformWork())
	require.NoError(t, err)
	require.NotEmpty(t, spans)
	return spans
}

// scaleSpans returns a new chain with every control point's coordinate
// multiplied by the exact integer factor — used to build a second "side" of
// genuinely different absolute sagitta from the same shape, over exact
// rationals so the relationship pairStations must preserve (station1 = factor
// * station0 at every shared dyadic parameter) is checkable bit-for-bit.
func scaleSpans(spans []bezierSpan, factor int64) []bezierSpan {
	f := big.NewRat(factor, 1)
	out := make([]bezierSpan, len(spans))
	for i, span := range spans {
		s := make(bezierSpan, len(span))
		for j, p := range span {
			s[j] = ratPoint{u: new(big.Rat).Mul(p.u, f), v: new(big.Rat).Mul(p.v, f)}
		}
		out[i] = s
	}
	return out
}

// maxSagittaAtDepth measures a single span's own §6.2.1 sagitta at a FIXED
// uniform dyadic depth, over the same production primitives pairStations
// itself bisects with (dyadicSpanOf, dyadicSpan.split, dyadicSpanSagittaUpper)
// — a different (uniform, non-adaptive) traversal of the real machinery,
// never a parallel reimplementation of the bound it measures.
func maxSagittaAtDepth(s dyadicSpan, depth int) float64 {
	if depth == 0 {
		return dyadicSpanSagittaUpper(s)
	}
	left, right := s.split()
	return math.Max(maxSagittaAtDepth(left, depth-1), maxSagittaAtDepth(right, depth-1))
}

func levelSagitta(span bezierSpan, depth int) float64 {
	return maxSagittaAtDepth(dyadicSpanOf(span), depth)
}

// denseChordSegmentDeviation samples a single-span chain densely and returns
// the maximum true distance from a sampled curve point to the chord SEGMENT
// joining the chain's own first and last control point — the falsifier a
// bound built from the chord's carrier LINE, or from the parametric deviation
// |C(t) - L(t)|, cannot survive. The cached float de Casteljau oracle is the
// independent evaluator used for this large sample count; the exact-rational
// oracle remains the conversion check in spline_bezier_internal_test.go.
func denseChordSegmentDeviation(t *testing.T, span bezierSpan, samples int) float64 {
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
func carrierLineDistanceUpper(t *testing.T, span bezierSpan) float64 {
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

func floatOfRatPoint(t *testing.T, p ratPoint) (float64, float64) {
	t.Helper()
	pt, ok := point2Of(p)
	require.True(t, ok, "a test fixture's control point must be representable")
	return pt.U, pt.V
}

// --- 1. the overshooting net (docs/spline-design.md §6.2.1, §11) ---

func TestSpanSagittaUpperEnclosesOvershootingChordSegment(t *testing.T) {
	span := ratSpan([][2]float64{{0, 0}, {-3, 0.01}, {4, 0.01}, {1, 0}})

	dense := denseChordSegmentDeviation(t, span, 200_000)
	require.InDelta(t, 0.76, dense, 0.01, "the dense-sample deviation must match §6.2.1's own stated ~0.76")

	bound := spanSagittaUpper(span)
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
	span := ratSpan([][2]float64{{0, 0}, {0.1, 0}, {0.1, 0}, {1, 0}})

	bound := spanSagittaUpper(span)
	require.Zero(t, bound, "every control point already sits on the chord segment, so the sagitta is exactly 0")

	cx, cy := evalSpans(t, []bezierSpan{span}, 0.5)
	require.Zero(t, cy, "the curve never leaves the line v=0")
	lx := 0.5 // L(0.5) on the naive uniform-rate interpolant between (0,0) and (1,0)
	require.Greater(t, math.Abs(cx-lx), 0.2,
		"the parametric deviation |C(0.5)-L(0.5)| is well over 0.2 even though the sagitta the curve actually commits is exactly 0")
}

// --- 3. a genuinely collapsed span reports sagitta exactly 0 ---

func TestSpanSagittaUpperCollapsedSpanIsExactZero(t *testing.T) {
	span := ratSpan([][2]float64{{2, 3}, {2, 3}, {2, 3}, {2, 3}})
	require.Equal(t, 0.0, spanSagittaUpper(span), "a span whose control points all coincide reports sagitta exactly 0, by exact float equality")
}

// --- 4. station determinism ---

func TestPairStationsStationDeterminism(t *testing.T) {
	spans := quarterCircleFitSpans(t)
	const target = 1e-4

	s0a, s1a, sagA, err := pairStations(spans, spans, target, nil, nil)
	require.NoError(t, err)
	s0b, s1b, sagB, err := pairStations(spans, spans, target, nil, nil)
	require.NoError(t, err)

	require.Equal(t, sagA, sagB, "the achieved sagittaUpper must be bit-identical across calls")
	require.Len(t, s0b, len(s0a))
	require.Len(t, s1b, len(s1a))
	for i := range s0a {
		require.Zero(t, s0a[i].u.Cmp(s0b[i].u), "station %d side0 U must be bit-identical", i)
		require.Zero(t, s0a[i].v.Cmp(s0b[i].v), "station %d side0 V must be bit-identical", i)
		require.Zero(t, s1a[i].u.Cmp(s1b[i].u), "station %d side1 U must be bit-identical", i)
		require.Zero(t, s1a[i].v.Cmp(s1b[i].v), "station %d side1 V must be bit-identical", i)
	}
}

// --- 5. sagitta vs level: strictly decreasing, and settling on the smallest ---

func TestPairStationsSagittaStrictlyDecreasesWithLevel(t *testing.T) {
	spans := quarterCircleFitSpans(t)
	span := spans[0]

	previous := math.Inf(1)
	for depth := range 7 {
		s := levelSagitta(span, depth)
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
	spans := quarterCircleFitSpans(t)
	span := spans[0]

	// Find the smallest depth d whose UNIFORM bisection already meets a
	// target set just above level d's own measured value — d itself is
	// discovered from the level sequence, never a hard-coded number.
	d := 0
	for levelSagitta(span, d) > levelSagitta(span, 4) {
		d++
	}
	require.LessOrEqual(t, d, 4)
	require.Positive(t, d, "the fixture needs at least one level of refinement for this test to exercise anything")
	targetFine := levelSagitta(span, d) * (1 + 1e-9)
	targetCoarse := levelSagitta(span, d-1) * (1 + 1e-9)
	require.Greater(t, targetCoarse, targetFine, "level d-1's own target must be strictly laxer than level d's")

	single := []bezierSpan{span}

	s0Fine, _, sagFine, err := pairStations(single, single, targetFine, nil, nil)
	require.NoError(t, err)
	require.LessOrEqual(t, sagFine, targetFine, "the achieved sagitta must honor the fine target")
	leavesFine := len(s0Fine) - 1
	require.LessOrEqual(t, leavesFine, 1<<d,
		"the generator must never need more cells than uniform refinement to depth %d already guarantees suffices", d)

	s0Coarse, _, sagCoarse, err := pairStations(single, single, targetCoarse, nil, nil)
	require.NoError(t, err)
	require.LessOrEqual(t, sagCoarse, targetCoarse, "the achieved sagitta must honor the coarse target")
	leavesCoarse := len(s0Coarse) - 1
	require.LessOrEqual(t, leavesCoarse, 1<<(d-1),
		"the generator must never need more cells than uniform refinement to depth %d already guarantees suffices", d-1)

	require.Less(t, leavesCoarse, leavesFine,
		"a strictly laxer target must settle on strictly fewer cells, proving the walk tracks the target rather than a fixed depth")
}

// --- 6. over-cap refuses errTooManyChords ---

func TestPairStationsOverCapRefuses(t *testing.T) {
	spans := quarterCircleFitSpans(t)
	_, _, _, err := pairStations(spans, spans, 1e-20, nil, nil) //nolint:dogsled // stations/sagitta discarded; only the refusal is under test
	require.Error(t, err)
	require.ErrorIs(t, err, errTooManyChords)
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
func parabolaSpan() bezierSpan {
	return ratSpan([][2]float64{{0, 0}, {1, 1}, {2, 0}})
}

// straightSpanFrom is a collinear, evenly spaced quadratic span: every control
// point lies ON its own chord segment, so its sagitta is exactly 0, and it is
// accepted whole as ONE chord at depth 0 against any target. It contributes a
// chord to a chain without contributing a bisection.
func straightSpanFrom(u float64) bezierSpan {
	return ratSpan([][2]float64{{u, 0}, {u + 1, 0}, {u + 2, 0}})
}

// capDepth is the uniform bisection depth whose leaves number exactly
// maxChordsPerWalk, read off the cap itself rather than written down as a
// tuned number.
func capDepth(t *testing.T) int {
	t.Helper()
	d := 0
	for 1<<d < maxChordsPerWalk {
		d++
	}
	require.Equal(t, maxChordsPerWalk, 1<<d,
		"these fixtures read their refinement depth off the cap; a cap that is not a power of two needs a different construction")
	return d
}

// TestPairStationsAcceptsTheStatedChordCap puts the cap where its own message
// puts it. errTooManyChords reads "more than maxChordsPerWalk chords on one
// curve", so a walk that settles on EXACTLY maxChordsPerWalk chords is inside
// the ceiling and must be built, not refused.
//
// This is the fixture that goes red if the cap ever binds below the count it
// names. A binary refinement visits nearly two cells for every chord it
// accepts (2L−m cells for L chords over m spans), so a ceiling charged per
// cell VISITED rather than per chord ACCEPTED refuses this walk barely halfway
// through it, at a chord count under half the one the message states.
func TestPairStationsAcceptsTheStatedChordCap(t *testing.T) {
	span := parabolaSpan()
	chain := []bezierSpan{span}
	target := levelSagitta(span, capDepth(t))

	s0, s1, sag, err := pairStations(chain, chain, target, nil, nil)
	require.NoError(t, err, "a walk needing exactly the chord count the cap names must be built")
	require.Len(t, s0, maxChordsPerWalk+1,
		"a chain of exactly maxChordsPerWalk chords carries one more station than that")
	require.Len(t, s1, len(s0), "both sides share one station set by construction")
	require.LessOrEqual(t, sag, target, "the achieved sagitta must honor the target it settled on")
}

// TestPairStationsRefusesOneChordPastTheStatedCap is the other half of the
// same boundary. The identical parabola walk gains ONE straight span, which is
// accepted whole at depth 0, so the chain needs maxChordsPerWalk+1 chords —
// the first count the message's "more than" covers — and the walk refuses.
func TestPairStationsRefusesOneChordPastTheStatedCap(t *testing.T) {
	span := parabolaSpan()
	chain := []bezierSpan{span, straightSpanFrom(2)}
	target := levelSagitta(span, capDepth(t))

	_, _, _, err := pairStations(chain, chain, target, nil, nil) //nolint:dogsled // stations/sagitta discarded; only the refusal is under test
	require.ErrorIs(t, err, errTooManyChords)
	require.ErrorIs(t, err, ErrUnsupported)
}

// TestPairStationsSpanCountPastTheCapRefusesUpFront pins the entry guard: each
// span carries at least one chord even when it needs no bisection at all, so a
// chain of more spans than the cap admits already exceeds the chord count the
// message names, and refuses before a single cell is measured. The target here
// is deliberately lax enough that every span would otherwise be accepted whole.
func TestPairStationsSpanCountPastTheCapRefusesUpFront(t *testing.T) {
	chain := make([]bezierSpan, maxChordsPerWalk+1)
	for i := range chain {
		chain[i] = straightSpanFrom(float64(2 * i))
	}

	_, _, _, err := pairStations(chain, chain, 1, nil, nil) //nolint:dogsled // stations/sagitta discarded; only the refusal is under test
	require.ErrorIs(t, err, errTooManyChords)
	require.ErrorIs(t, err, ErrUnsupported)
}

// --- 7. over-budget refuses Table R row R7 ---

func TestPairStationsOverBudgetRefusesR7(t *testing.T) {
	spans := quarterCircleFitSpans(t)
	exhausted := &freeformWork{spent: freeformWorkLimit}
	_, _, _, err := pairStations(spans, spans, 1e-9, exhausted, newFreeformWork()) //nolint:dogsled // stations/sagitta discarded; only the refusal is under test
	require.Error(t, err)
	require.ErrorIs(t, err, ErrUnsupported)
}

// --- 8. shared station set: same length AND same dyadic parameter fractions ---

// TestPairStationsSharedStationSetAcrossDifferentScale pairs a span with an
// exact integer scaling of itself (factor 5): the two curves have
// GENUINELY different absolute sagitta at every cell (scaling multiplies
// every squared distance by 25 and every sagitta by 5 exactly), yet
// pairStations always bisects both sides from the SAME cell together
// (spline_sagitta.go's own doc comment), so the two station lists must land
// on the identical dyadic parameter fractions. That is checked directly, not
// merely by length: because de Casteljau blending is linear, station1[k]
// must equal EXACTLY 5*station0[k] for every k, which only holds if index k
// on both sides names the same parameter — a mismatched tree would break the
// exact ratio at whatever index the trees first diverged.
func TestPairStationsSharedStationSetAcrossDifferentScale(t *testing.T) {
	base := quarterCircleFitSpans(t)
	scaled := scaleSpans(base, 5)

	target := levelSagitta(base[0], 2) // fine enough to force several splits
	s0, s1, _, err := pairStations(base, scaled, target, nil, nil)
	require.NoError(t, err)
	require.Equal(t, len(s0), len(s1))
	require.Greater(t, len(s0), len(base)+1, "the target must force genuine subdivision for this test to exercise the shared-cell claim")

	five := big.NewRat(5, 1)
	for k := range s0 {
		wantU := new(big.Rat).Mul(s0[k].u, five)
		wantV := new(big.Rat).Mul(s0[k].v, five)
		require.Zero(t, wantU.Cmp(s1[k].u), "station %d: side1 must be the exact 5x scaling of side0 at the same dyadic parameter fraction", k)
		require.Zero(t, wantV.Cmp(s1[k].v), "station %d: side1 must be the exact 5x scaling of side0 at the same dyadic parameter fraction", k)
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
	small := ratSpan([][2]float64{{0, 0}, {1, 3}, {2, -3}, {3, 0}})
	big8 := ratSpan([][2]float64{
		{0, 0}, {0.4, 3}, {0.9, -2}, {1.3, 3},
		{1.7, -3}, {2.1, 2}, {2.6, -3}, {3, 0},
	})

	target := math.Min(levelSagitta(small, 2), levelSagitta(big8, 2))
	work0, work1 := newFreeformWork(), newFreeformWork()
	_, _, _, err := pairStations([]bezierSpan{small}, []bezierSpan{big8}, target, work0, work1) //nolint:dogsled // stations/sagitta discarded; only the counter split is under test
	require.NoError(t, err)

	require.Positive(t, work0.spent, "side 0's own counter must be charged")
	require.Positive(t, work1.spent, "side 1's own counter must be charged")
	require.Greater(t, work1.spent, work0.spent,
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
	bulge := ratSpan([][2]float64{{0, 0}, {0, 5}, {1, 5}, {1, 0}})
	flat := ratSpan([][2]float64{{0, 0}, {0.33, 0.0001}, {0.66, 0.0001}, {1, 0}})

	bulgeSag := spanSagittaUpper(bulge)
	flatSag := spanSagittaUpper(flat)
	require.Greater(t, bulgeSag, flatSag*100, "the fixture needs a large gap between the two spans' own readings")

	target := bulgeSag * (1 + 1e-9)
	spans0 := []bezierSpan{bulge, flat}
	spans1 := []bezierSpan{bulge, flat}
	_, _, sagUp, err := pairStations(spans0, spans1, target, nil, nil)
	require.NoError(t, err)
	require.InEpsilon(t, bulgeSag, sagUp, 1e-9, "sagittaUpper must be the running MAXIMUM (the first, bulging cell), never the last cell's own tiny reading")
}

// --- extra: pairStations refuses a span-count mismatch defensively ---

func TestPairStationsSpanCountMismatchRefuses(t *testing.T) {
	spans := quarterCircleFitSpans(t)
	_, _, _, err := pairStations(spans, spans[:len(spans)-1], 1e-6, nil, nil) //nolint:dogsled // stations/sagitta discarded; only the refusal is under test
	require.Error(t, err)
	require.ErrorIs(t, err, ErrUnsupported)
	require.False(t, errors.Is(err, ErrDegenerate))
}

// --- extra: the final rounding is genuinely outward, not merely close ---

// TestSpanSagittaUpperRoundsOutward proves the outward-rounding contract
// directly over exact rationals, at a precision no dense-sample test could
// resolve: the exact maximum squared distance is computed independently (a
// different formula, not chordSegmentSquaredDistance's own code path — the
// single interior control point's own clamped-projection distance, worked out
// by hand for this fixture) and its square root is bracketed to 200 bits with
// math/big.Float.Sqrt. spanSagittaUpper's returned float64 must sit AT OR
// ABOVE that bracket, never below it — the property that distinguishes
// ratSqrtUp from ratSqrtDown, and one dense sampling at float64 precision is
// too coarse (both round within a handful of ulps of the true root) to catch
// on its own.
func TestSpanSagittaUpperRoundsOutward(t *testing.T) {
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

	bound := spanSagittaUpper(span)
	require.False(t, math.IsNaN(bound))
	boundFloat := new(big.Float).SetPrec(200).SetFloat64(bound)
	require.GreaterOrEqual(t, boundFloat.Cmp(ref), 0,
		"the reported bound must round OUTWARD of the true root, never below it")
}

// independentMaxChordSquaredDistance is TestSpanSagittaUpperRoundsOutward's
// own independent oracle: it re-derives the same maximum-squared-distance-to-
// segment quantity chordSegmentSquaredDistance computes, but via a separately
// written clamped-projection formula (Cramer-style, no shared helper with
// spline_sagitta.go), so agreement between the two is the §1 falsifier
// working rather than one call site echoing the other's arithmetic.
func independentMaxChordSquaredDistance(t *testing.T, span bezierSpan) *big.Rat {
	t.Helper()
	a, b := span[0], span[len(span)-1]
	abx := new(big.Rat).Sub(b.u, a.u)
	aby := new(big.Rat).Sub(b.v, a.v)
	abLenSq := new(big.Rat).Add(new(big.Rat).Mul(abx, abx), new(big.Rat).Mul(aby, aby))

	var maxSq *big.Rat
	for _, p := range span {
		apx := new(big.Rat).Sub(p.u, a.u)
		apy := new(big.Rat).Sub(p.v, a.v)
		var sq *big.Rat
		if abLenSq.Sign() == 0 {
			// The chord's own two ends coincide (a == b), so the segment
			// degenerates to the single point a — §6.2.1's own explicit
			// "distance to a one-point set" case (spline_sagitta.go's own
			// chordSegmentSquaredDistance doc comment). There is nothing to
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
			qx := new(big.Rat).Add(a.u, new(big.Rat).Mul(s, abx))
			qy := new(big.Rat).Add(a.v, new(big.Rat).Mul(s, aby))
			ex := new(big.Rat).Sub(p.u, qx)
			ey := new(big.Rat).Sub(p.v, qy)
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
// shape a loft cap can contribute, and chordSegmentSquaredDistance's own
// d==0 branch must still report the true |p-a|^2 for it, never a bolted-on
// 0. The net (0,0) (1,5) (-1,5) (0,0) is a non-collapsed control polygon (the
// FIRST and LAST points coincide; the interior two do not) whose farthest
// control point sits exactly sqrt(26) from the shared chord point.
func TestSpanSagittaUpperClosedLoopChordIsAPointNotZero(t *testing.T) {
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

	bound := spanSagittaUpper(span)
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
	small := ratSpan([][2]float64{{0, 0}, {0.33, 0.0001}, {0.66, 0.0001}, {1, 0}})
	large := ratSpan([][2]float64{{0, 0}, {0, 5}, {1, 5}, {1, 0}})

	const target = 0.01
	require.LessOrEqual(t, spanSagittaUpper(small), target,
		"side 0 alone must already sit inside the target, or the accept-test bug this fixture targets has nothing to catch")
	require.Greater(t, spanSagittaUpper(large), target,
		"side 1 alone must sit outside the target, forcing real subdivision under a correct accept test")

	spans0 := []bezierSpan{small}
	spans1 := []bezierSpan{large}
	_, _, sagUp, err := pairStations(spans0, spans1, target, nil, nil)
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
	small := ratSpan([][2]float64{{0, 0}, {0.33, 0.0001}, {0.66, 0.0001}, {1, 0}})
	large := ratSpan([][2]float64{{0, 0}, {0, 5}, {1, 5}, {1, 0}})

	smallSag := spanSagittaUpper(small)
	largeSag := spanSagittaUpper(large)
	require.Greater(t, largeSag, smallSag*100, "the fixture needs a large gap between the two sides' own readings")

	// Just above the LARGER side's own reading, so the single cell accepts
	// whole with no bisection at all — the returned value is then a direct
	// readout of whatever the fold computed, not an artifact of subdivision.
	target := largeSag * (1 + 1e-9)
	spans0 := []bezierSpan{small}
	spans1 := []bezierSpan{large}
	_, _, sagUp, err := pairStations(spans0, spans1, target, nil, nil)
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
	spans := quarterCircleFitSpans(t)
	target := levelSagitta(spans[0], 3)

	s0, _, _, err := pairStations(spans, spans, target, nil, nil)
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
	spans := quarterCircleFitSpans(t)
	target := levelSagitta(spans[0], 3)

	s0, s1, _, err := pairStations(spans, spans, target, nil, nil)
	require.NoError(t, err)
	require.Greater(t, len(s0), len(spans)+1, "the target must force genuine subdivision for this test to exercise anything")

	requireExactRatPoint := func(t *testing.T, want, got ratPoint, msg string) {
		t.Helper()
		require.Zero(t, want.u.Cmp(got.u), "%s: U", msg)
		require.Zero(t, want.v.Cmp(got.v), "%s: V", msg)
	}

	requireExactRatPoint(t, spans[0][0], s0[0], "first station side0 must be the chain's own start control point")
	requireExactRatPoint(t, spans[0][0], s1[0], "first station side1 must be the chain's own start control point")
	last := spans[len(spans)-1]
	requireExactRatPoint(t, last[len(last)-1], s0[len(s0)-1], "final station side0 must be the chain's own end control point")
	requireExactRatPoint(t, last[len(last)-1], s1[len(s1)-1], "final station side1 must be the chain's own end control point")
}

// --- A6: charged work MAGNITUDE, not merely its cross-side ratio ---

// TestSagittaMeasureCostMatchesDocumentedFormula pins sagittaMeasureCost's
// own closed form directly: sagittaMeasureCostPerPoint (8) units per control
// point, per its own doc comment. TestPairStationsChargesBothCountersSeparately
// only pins the RATIO between two differently-sized sides' own charges, which
// any uniform rescaling of sagittaMeasureCostPerPoint preserves; this pins the
// constant's own value.
func TestSagittaMeasureCostMatchesDocumentedFormula(t *testing.T) {
	// The expected multiplier is the LITERAL 8 the doc comment states, never
	// sagittaMeasureCostPerPoint itself — comparing against that constant would
	// be circular (it would still pass after the constant were mutated to 1),
	// which is exactly the vulnerability this fixture exists to close.
	const perPointCost = 8
	require.Equal(t, perPointCost, sagittaMeasureCostPerPoint,
		"this test's own literal must match the constant it pins; update both together on a deliberate cost-model change")
	for _, n := range []int{1, 2, 4, 8} {
		require.Equal(t, uint64(perPointCost*n), sagittaMeasureCost(n), "n=%d", n)
	}
}

// TestSagittaSplitCostMatchesDocumentedFormula pins sagittaSplitCost's own
// closed form: n(n-1) exact dyadicMidpoint blends, per its own doc comment.
func TestSagittaSplitCostMatchesDocumentedFormula(t *testing.T) {
	for _, n := range []int{2, 3, 4, 8} {
		require.Equal(t, uint64(n*(n-1)), sagittaSplitCost(n), "n=%d", n)
	}
}

// --- B1: a zero-control-point span must refuse, never panic ---

// TestPairStationsRefusesAZeroControlSpanInsteadOfPanicking pins the fix for
// dyadicSpanSagittaUpper's own dead-ended n==0 guard: with no entry-level
// gate, a zero-control span reaches walkCell, both sides' sagitta reads 0
// (dyadicSpanSagittaUpper's own guard), the accept test passes trivially,
// and the accept branch's own c0.ratPointAt(0) panics with an index out of
// range on the empty points slice. pairStations must refuse this input
// cleanly, on either side, before any cell is ever walked.
func TestPairStationsRefusesAZeroControlSpanInsteadOfPanicking(t *testing.T) {
	empty := bezierSpan{}
	line := ratSpan([][2]float64{{0, 0}, {1, 0}})

	t.Run("side0", func(t *testing.T) {
		_, _, _, err := pairStations([]bezierSpan{empty}, []bezierSpan{line}, 1, nil, nil)
		require.Error(t, err)
		require.ErrorIs(t, err, ErrDegenerate)
	})
	t.Run("side1", func(t *testing.T) {
		_, _, _, err := pairStations([]bezierSpan{line}, []bezierSpan{empty}, 1, nil, nil)
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
	span := ratSpan([][2]float64{{0, 0}, {1, 0}})
	spans := []bezierSpan{span}

	// target=1 is far above this straight span's own (zero) sagitta, so the
	// single cell accepts whole with no bisection — the final station is
	// exactly the code path under test.
	s0, _, _, err := pairStations(spans, spans, 1, nil, nil)
	require.NoError(t, err)

	wantU := new(big.Rat).Set(span[len(span)-1].u) // the input's own value, before any mutation
	last := s0[len(s0)-1]
	last.u.Add(last.u, big.NewRat(1, 1)) // mutate the RETURNED station in place

	require.Zero(t, wantU.Cmp(span[len(span)-1].u),
		"mutating a returned station must never change the caller's own input span")
}

// --- C: matched-delta primitives (bounds.go's cellChordCurveAreaUpper
// matchedDeltaUpper obligation) — spanHodographGapUpper, spanMatchedDeltaUpper,
// spanSpeedUpper ---

// denseMatchedDeviation samples a single-span chain densely and returns the
// maximum true |C(t) - (P_0 + t*Delta)| over the span's own NATIVE parameter
// t — the parameter-matched deviation spanMatchedDeltaUpper bounds, sampled
// through the same independent de Casteljau oracle (evalSpans) the sagitta
// tests already use, never through any of spline_sagitta.go's own machinery.
func denseMatchedDeviation(t *testing.T, span bezierSpan, samples int) float64 {
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
// spanHodographGapUpper's own exact-rational hodograph.
func denseSpeedSample(t *testing.T, span bezierSpan, samples int) float64 {
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
func zigzagHuggingSpan() bezierSpan {
	return ratSpan([][2]float64{{0, 0}, {0.001, 0}, {0.001, 0}, {1, 0}})
}

// TestSpanMatchedDeltaUpperEnclosesWhatTheSagittaMisses is the decisive C4
// fixture: it densely proves the sagitta of 0 FAILS to bound the true
// parameter-matched deviation on zigzagHuggingSpan, and that
// spanMatchedDeltaUpper (d/2) DOES bound it. This is F1's rule made
// concrete: cellChordCurveAreaUpper's matchedDeltaUpper obligation is a
// strictly stronger claim than the sagitta, and confusing the two is exactly
// the unsoundness this function exists to prevent a downstream caller from
// committing.
func TestSpanMatchedDeltaUpperEnclosesWhatTheSagittaMisses(t *testing.T) {
	span := zigzagHuggingSpan()

	sagitta := spanSagittaUpper(span)
	require.Zero(t, sagitta, "every control point sits exactly on the chord segment, so the sagitta is exactly 0")

	dense := denseMatchedDeviation(t, span, 200_000)
	require.Greater(t, dense, 0.3, "the fixture needs a substantial true parameter-matched deviation for this test to mean anything")

	require.Less(t, sagitta, dense,
		"pinning F1's own violation: a sagitta of 0 must FAIL to bound the true parameter-matched deviation of %.6g", dense)

	matched := spanMatchedDeltaUpper(span)
	require.GreaterOrEqual(t, matched, dense,
		"spanMatchedDeltaUpper must ENCLOSE the true parameter-matched deviation, where the sagitta above does not")
}

// TestSpanMatchedDeltaUpperEnclosesDenseSampleOnOrdinarySpans checks the
// bound holds — not merely on the decisive counterexample above — on an
// ordinary curved chain.
func TestSpanMatchedDeltaUpperEnclosesDenseSampleOnOrdinarySpans(t *testing.T) {
	spans := quarterCircleFitSpans(t)
	for i, span := range spans {
		dense := denseMatchedDeviation(t, span, 20_000)
		matched := spanMatchedDeltaUpper(span)
		require.GreaterOrEqual(t, matched, dense,
			"span %d: matchedDeltaUpper must enclose the dense-sampled parameter-matched deviation", i)
	}
}

// TestSpanSpeedUpperEnclosesDenseSampleAndNeverFallsBelowChordLength checks
// both of spanSpeedUpper's own obligations: it encloses a dense finite-
// difference sample of ||C'(t)||, and it never reads below the span's own
// chord length — the floor cellChordCurveAreaUpper's own tangent-magnitude
// argument depends on.
func TestSpanSpeedUpperEnclosesDenseSampleAndNeverFallsBelowChordLength(t *testing.T) {
	spans := quarterCircleFitSpans(t)
	for i, span := range spans {
		dense := denseSpeedSample(t, span, 2000)
		speed := spanSpeedUpper(span)
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
// POSITIVE length), the two GAP terms (spanHodographGapUpper,
// spanMatchedDeltaUpper) still read exactly 0, because a uniformly-spaced
// collinear net's hodograph is the CONSTANT vector Delta itself at every
// control point; the speed bound is NOT zero there — it reads exactly the
// span's own chord length, since d=0 leaves nothing to widen it by.
func TestHodographBoundsAreExactlyZeroOnCollapsedAndStraightUniformSpans(t *testing.T) {
	collapsed := ratSpan([][2]float64{{2, 3}, {2, 3}, {2, 3}, {2, 3}})
	require.Zero(t, spanHodographGapUpper(collapsed))
	require.Zero(t, spanMatchedDeltaUpper(collapsed))
	require.Zero(t, spanSpeedUpper(collapsed), "a fully collapsed span (chord length 0 too) has zero speed as well as zero deviation")

	straight := ratSpan([][2]float64{{0, 0}, {1, 0}, {2, 0}}) // uniformly-spaced collinear controls: constant-speed line
	require.Zero(t, spanHodographGapUpper(straight))
	require.Zero(t, spanMatchedDeltaUpper(straight))
	chordLen := math.Hypot(2, 0)
	require.InEpsilon(t, chordLen, spanSpeedUpper(straight), 1e-12,
		"a straight uniformly-spaced span's speed bound must equal its own chord length exactly (d=0), never merely enclose it")
}
