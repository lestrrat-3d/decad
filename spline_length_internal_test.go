package decad

import (
	"math"
	"runtime"
	"strconv"
	"testing"
	"time"

	"github.com/lestrrat-3d/sketch/geom"
	"github.com/stretchr/testify/require"
)

// The bracket is only useful if it both ENCLOSES the true length and NARROWS
// with subdivision. A dense polyline is the falsifier for the first: it must
// land inside every depth's interval, and it must not be used as the answer.

func denseSplineLength(t *testing.T, coords [][2]float64) float64 {
	t.Helper()
	ring, err := geom.SampleCubicBSpline(coords, 400000)
	require.NoError(t, err)
	total := 0.0
	for i := 0; i+1 < len(ring); i++ {
		total += math.Hypot(ring[i+1][0]-ring[i][0], ring[i+1][1]-ring[i][1])
	}
	return total
}

func TestFreeformArcLengthBracketEnclosesAndNarrows(t *testing.T) {
	control := []Point2{{U: 0, V: 0}, {U: 1, V: 2}, {U: 3, V: 2}, {U: 4, V: 0}, {U: 6, V: 1}, {U: 7, V: -2}}
	coords := make([][2]float64, len(control))
	for i, point := range control {
		coords[i] = [2]float64{point.U, point.V}
	}
	spans, err := splineBezierSpans(SplineSeg{Control: control, TStart: 0, TEnd: 1}, &freeformWork{})
	require.NoError(t, err)

	reference := denseSplineLength(t, coords)

	previous := math.Inf(1)
	for _, depth := range []int{0, 2, 4, 6, 8, 10} {
		lo, hi := 0.0, 0.0
		for _, span := range spans {
			spanLo, spanHi := spanLengthBracket(span, depth)
			lo += spanLo
			hi += spanHi
		}
		require.LessOrEqual(t, lo, reference, "depth %d lower bound must not exceed the true length", depth)
		require.GreaterOrEqual(t, hi, reference, "depth %d upper bound must not fall below the true length", depth)
		width := hi - lo
		require.Less(t, width, previous, "depth %d must narrow the bracket", depth)
		previous = width
	}
	// THIS fixture's own numbers, not a promise of the depth: an ordinary
	// six-control spline of 9.6 mm closes to ~1.5e-6 mm at full depth, a relative
	// width of 1.5e-7. The depth is fixed and carries no relative-width promise
	// at all, and a span two orders of magnitude coarser is pinned below in
	// TestFreeformArcLengthRelativeWidthVariesWithTheSpan.
	require.Less(t, previous, 1e-5, "this ordinary spline closes to well under 10 nm at full depth")
	require.Less(t, previous/reference, 2e-7, "this ordinary spline's relative bracket width")
}

// The reported bound is the enclosure MEASURED at the fixed depth, and a fixed
// depth promises no relative width: what a span reaches varies with the span.
// Both ends of that spread are pinned here, because a single figure stated for
// the depth is false for one of them. The ordinary cubic measures 1.92e-07. The
// widest span the bracket's own preflight admits — 32 controls, a degree-31
// Bézier whose control points wind eight times around a 10 mm circle — measures
// 2.20e-05, over a hundred times coarser, and it narrows by as little as 1.37x
// at one of its ten levels, so no per-level rate sizes it either.
func TestFreeformArcLengthRelativeWidthVariesWithTheSpan(t *testing.T) {
	t.Parallel()
	require.LessOrEqual(t, freeformBracketCost(32), freeformWorkLimit,
		"a 32-control span's bracket is affordable")
	require.Greater(t, freeformBracketCost(33), freeformWorkLimit,
		"33 controls is past the ceiling, so 32 is the widest span measured here")

	cubic := []Point2{{U: 0, V: 0}, {U: 1, V: 2}, {U: 3, V: 2}, {U: 4, V: 0}}
	spans, err := splineBezierSpans(SplineSeg{Control: cubic, TStart: 0, TEnd: 1}, &freeformWork{})
	require.NoError(t, err)
	value, bound, err := freeformArcLength(spans, &freeformWork{})
	require.NoError(t, err)
	require.InDelta(t, 1.9213e-07, bound/value, 1e-11,
		"an ordinary cubic's measured relative half width")

	wide := windingControlNet(32, 8, 10)
	seg := equalWeightNURBS(wide)
	require.NoError(t, validateNURBSSegment(seg), "the widest span is a well-formed record")
	wideSpans, _, err := freeformBezierSpans(seg, &freeformWork{})
	require.NoError(t, err)
	require.Len(t, wideSpans, 1, "no interior knot, so the record IS one Bézier span")

	value, bound, err = freeformArcLength(wideSpans, &freeformWork{})
	require.NoError(t, err)
	require.Greater(t, bound/value, 1e-05,
		"the widest admitted span's width is nowhere near an ordinary curve's")
	require.InDelta(t, 2.1985e-05, bound/value, 1e-08,
		"the widest admitted span's measured relative half width")

	// The width is coarse and the enclosure still holds: a dense polyline over
	// the same control net, evaluated in floats by an independent de Casteljau,
	// lands inside the reported interval.
	reference := denseBezierLength(wide, 200000)
	require.GreaterOrEqual(t, value+bound, reference, "the interval's top encloses the true length")
	require.LessOrEqual(t, value-bound, reference, "the interval's bottom encloses the true length")
}

// windingControlNet places n control points on a circle of radius r, walking
// turns full revolutions from the first to the last. The winding is what buys
// the span its curvature: an alternating net of the same degree is damped by the
// Bernstein weighting into an almost straight curve.
func windingControlNet(n, turns int, r float64) []Point2 {
	control := make([]Point2, n)
	for i := range control {
		a := 2 * math.Pi * float64(turns) * float64(i) / float64(n-1)
		control[i] = Point2{U: r * math.Cos(a), V: r * math.Sin(a)}
	}
	return control
}

// denseBezierLength is the falsifier for the widest span: a float de Casteljau
// polyline over the recorded control net, sharing no code with the rational
// bracket. A polyline undercuts the arc, so it is a reference the interval must
// contain, never the answer.
func denseBezierLength(control []Point2, samples int) float64 {
	net := make([][2]float64, len(control))
	for i, p := range control {
		net[i] = [2]float64{p.U, p.V}
	}
	work := make([][2]float64, len(net))
	at := func(t float64) [2]float64 {
		copy(work, net)
		for round := len(work) - 1; round > 0; round-- {
			for i := range round {
				work[i][0] += t * (work[i+1][0] - work[i][0])
				work[i][1] += t * (work[i+1][1] - work[i][1])
			}
		}
		return work[0]
	}
	previous := at(0)
	total := 0.0
	for i := 1; i <= samples; i++ {
		current := at(float64(i) / float64(samples))
		total += math.Hypot(current[0]-previous[0], current[1]-previous[1])
		previous = current
	}
	return total
}

func TestFreeformArcLengthReportsPositiveBound(t *testing.T) {
	control := []Point2{{U: 0, V: 0}, {U: 1, V: 2}, {U: 3, V: 2}, {U: 4, V: 0}}
	spans, err := splineBezierSpans(SplineSeg{Control: control, TStart: 0, TEnd: 1}, &freeformWork{})
	require.NoError(t, err)

	value, bound, err := freeformArcLength(spans, &freeformWork{})
	require.NoError(t, err)
	require.Positive(t, value)
	require.Positive(t, bound, "an arc length is never exact, so its bound is never zero")

	coords := [][2]float64{{0, 0}, {1, 2}, {3, 2}, {4, 0}}
	reference := denseSplineLength(t, coords)
	require.InDelta(t, reference, value, bound+1e-9, "the reported interval encloses the true length")
}

// A control net that has collapsed to a single point brackets to [0, 0], and a
// zero half width IS the Exact zero §6.1 forbids. It must refuse instead — with
// the same sentinel the moments path already gives the identical record, so the
// two paths agree on what the record is.
func TestFreeformCoincidentControlNetRefused(t *testing.T) {
	for _, tc := range []struct {
		name    string
		control []Point2
	}{
		{name: "at the origin", control: []Point2{{}, {}, {}, {}}},
		{
			name: "away from the origin",
			control: []Point2{
				{U: 5, V: 7}, {U: 5, V: 7}, {U: 5, V: 7}, {U: 5, V: 7},
			},
		},
	} {
		t.Run(tc.name, func(t *testing.T) {
			seg := SplineSeg{Control: tc.control, TStart: 0, TEnd: 1}

			spans, err := splineBezierSpans(seg, &freeformWork{})
			require.NoError(t, err, "the record itself converts")
			_, _, err = freeformArcLength(spans, &freeformWork{})
			require.ErrorIs(t, err, ErrDegenerate)
			require.Contains(t, err.Error(), "coincide")

			_, err = walkOf(seg, newFreeformWork())
			require.ErrorIs(t, err, ErrDegenerate, "no walk carries a zero length bound")

			_, _, _, err = validateFreeformMomentSegment(seg, &freeformWork{})
			require.ErrorIs(t, err, ErrDegenerate, "the moments path refuses the same record")
		})
	}
}

// A straight-line degenerate case pins the directed rounding: the exact length
// of the unit-diagonal chord is irrational, so the lower bound must sit at or
// below it and the upper at or above.
func TestDirectedSqrtBracketsIrrationalLength(t *testing.T) {
	a := ratPoint{u: mustRatOf(0), v: mustRatOf(0)}
	b := ratPoint{u: mustRatOf(1), v: mustRatOf(1)}
	squared := ratSquaredDistance(a, b)

	lo := ratSqrtDown(squared)
	hi := ratSqrtUp(squared)
	require.LessOrEqual(t, lo*lo, 2.0)
	require.GreaterOrEqual(t, hi*hi, 2.0)
	require.LessOrEqual(t, lo, hi)
	require.InDelta(t, math.Sqrt2, lo, 1e-15)
}

// The square-root seed must survive every scale a finite coordinate reaches.
// Rounding the exact squared distance to a float64 first does not: a leg small
// enough to make it subnormal loses most of its significand and one large
// enough to make it overflow loses it entirely, so the seed lands far outside
// the eight-step adjustment walk and the bound escapes to its outward extreme.
// Both ends of the range must bracket, and both bounds must be finite.
func TestDirectedSqrtBracketsAtExtremeScale(t *testing.T) {
	for _, leg := range []float64{
		math.SmallestNonzeroFloat64, // the exact bottom of the range
		1e-320,                      // subnormal leg, subnormal square
		1e-200,                      // the square underflows to zero
		5.840241926400845e-156,      // the square is subnormal and nonzero
		1e-100,
		10,
		1e150,
		1.34e154,            // the square overflows
		1e300,               // the square overflows by orders of magnitude
		math.MaxFloat64 / 2, // the largest leg whose root still fits
	} {
		t.Run(strconv.FormatFloat(leg, 'g', -1, 64), func(t *testing.T) {
			q := ratSquaredDistance(
				ratPoint{u: mustRatOf(0), v: mustRatOf(0)},
				ratPoint{u: mustRatOf(leg), v: mustRatOf(0)},
			)

			lo := ratSqrtDown(q)
			hi := ratSqrtUp(q)
			require.False(t, isNonFinite(lo), "the lower bound stays finite")
			require.False(t, isNonFinite(hi), "the upper bound stays finite")
			require.True(t, ratSquareAtMost(lo, q), "the lower bound's square does not exceed the exact distance")
			require.False(t, ratSquareAtMost(hi, q) && !ratSquareEquals(hi, q),
				"the upper bound's square is not below the exact distance")
			require.LessOrEqual(t, lo, hi)
			// The leg is its own exact root here, so a seed that works at this
			// scale brackets it to a point rather than to a decade-wide gap.
			require.Equal(t, leg, lo)
			require.Equal(t, leg, hi)
		})
	}
}

// The ordinary curve, rescaled to each end of the float64 range. Every one of
// these is a valid record with finite coordinates, so every one must MEASURE:
// 4.572036268370589e-153 and 1.3729595320261218e+157 are the two control
// scales at which a seed read off the ROUNDED square stops bracketing, one at
// each end. The curve is a pure scaling of the ordinary one, so the bracket
// must also keep its relative width — a bound that survives only by widening
// is no fix.
func TestFreeformArcLengthBracketsAtExtremeScale(t *testing.T) {
	t.Parallel()
	for _, scale := range []float64{
		4.572036268370589e-153,
		1e-200,
		1e-300,
		1.3729595320261218e+157,
		1e300,
	} {
		t.Run(strconv.FormatFloat(scale, 'g', -1, 64), func(t *testing.T) {
			coords := [][2]float64{{0, 0}, {scale, 2 * scale}, {3 * scale, 2 * scale}, {4 * scale, 0}}
			control := make([]Point2, len(coords))
			for i, c := range coords {
				control[i] = Point2{U: c[0], V: c[1]}
			}
			spans, err := splineBezierSpans(SplineSeg{Control: control, TStart: 0, TEnd: 1}, &freeformWork{})
			require.NoError(t, err)

			value, bound, err := freeformArcLength(spans, &freeformWork{})
			require.NoError(t, err, "a valid curve is never refused for its scale alone")
			require.Positive(t, value)
			require.Positive(t, bound, "an arc length is never exact")

			reference := denseSplineLength(t, coords)
			require.GreaterOrEqual(t, value+bound, reference, "the interval's top encloses the true length")
			require.LessOrEqual(t, value-bound, reference, "the interval's bottom encloses the true length")
			require.InDelta(t, 1.9213e-07, bound/value, 1e-11,
				"the relative width is the unscaled ordinary cubic's own, to the ulp")
		})
	}
}

// A single near-duplicate control pair used to poison an otherwise ordinary
// 10 mm curve: that one leg's squared distance underflowed, its upper bound
// escaped to +Inf, and the whole walk's length went with it.
func TestFreeformArcLengthNearDuplicateControlPair(t *testing.T) {
	t.Parallel()
	for _, gap := range []float64{4.572036268370589e-153, 1e-200, 1e-300} {
		t.Run(strconv.FormatFloat(gap, 'g', -1, 64), func(t *testing.T) {
			coords := [][2]float64{{0, 0}, {gap, 0}, {5, 3}, {10, 0}}
			control := make([]Point2, len(coords))
			for i, c := range coords {
				control[i] = Point2{U: c[0], V: c[1]}
			}
			spans, err := splineBezierSpans(SplineSeg{Control: control, TStart: 0, TEnd: 1}, &freeformWork{})
			require.NoError(t, err)

			value, bound, err := freeformArcLength(spans, &freeformWork{})
			require.NoError(t, err, "a near-duplicate pair is a valid curve, not a refusal")

			reference := denseSplineLength(t, coords)
			require.InDelta(t, reference, value, bound+1e-9, "the reported interval encloses the true length")
		})
	}
}

// The one length that genuinely has no float64 interval: a curve whose proven
// upper bound runs past MaxFloat64. Refusing is right, and the sentinel is
// R15's ErrUnsupported — the curve exists and this evaluator cannot state its
// length — never ErrNotFinite, which would assert something false about an
// input whose every coordinate is finite.
func TestFreeformArcLengthAboveFloat64RangeRefused(t *testing.T) {
	const m = math.MaxFloat64
	seg := SplineSeg{
		Control: []Point2{{U: -m, V: -m}, {U: -m, V: m}, {U: m, V: -m}, {U: m, V: m}},
		TStart:  0,
		TEnd:    1,
	}

	spans, err := splineBezierSpans(seg, &freeformWork{})
	require.NoError(t, err, "the record itself is finite and converts")

	_, _, err = freeformArcLength(spans, &freeformWork{})
	require.ErrorIs(t, err, ErrUnsupported)
	require.NotErrorIs(t, err, ErrNotFinite, "every coordinate here is finite")
	require.Contains(t, err.Error(), "representable float64 range")

	_, err = walkOf(seg, newFreeformWork())
	require.ErrorIs(t, err, ErrUnsupported, "the walk carries the same refusal")
	require.NotErrorIs(t, err, ErrNotFinite)
}

// Every consumer that has no free-form construction must refuse a free-form
// walk rather than take its line branch. walkElem is the shared 2D conversion,
// so its refusal covers the wall survey, the section audit and the clearance
// trims at once.
func TestFreeformWalkRefusedByAnalyticConsumers(t *testing.T) {
	control := []Point2{{U: 0, V: 0}, {U: 1, V: 2}, {U: 3, V: 2}, {U: 4, V: 0}}
	walk, err := walkOf(SplineSeg{Control: control, TStart: 0, TEnd: 1}, newFreeformWork())
	require.NoError(t, err, "a Tier A segment resolves into a walk")
	require.Equal(t, walkFreeform, walk.kind)
	require.False(t, walk.isLine(), "a free-form walk is not a line")
	require.False(t, walk.isCircular(), "a free-form walk is not circular")

	_, ok := walkElem(walk)
	require.False(t, ok, "there is no 2D boundary element for a free-form walk yet")

	err = requireAnalyticWalk(walk, "the test consumer")
	require.Error(t, err)
	require.ErrorIs(t, err, ErrUnsupported)
}

// The bracket's charge must grow with the span DEGREE, not only with the fixed
// subdivision depth: one split blends every de Casteljau pair, so a charge that
// counts leaves alone admits a span whose splits run for hours.
func TestFreeformBracketCostGrowsWithDegree(t *testing.T) {
	require.Less(t, freeformBracketCost(4), freeformWorkLimit, "a cubic span's bracket is affordable")
	require.Greater(t, freeformBracketCost(8), 3*freeformBracketCost(4),
		"doubling the degree more than triples the charge")
	require.Equal(t, freeformCostCeiling, freeformBracketCost(1025), "the finding's degree-1024 span")
	require.Equal(t, freeformCostCeiling, freeformBracketCost(1<<20), "an absurd degree saturates, never wraps")
	require.Less(t, uint64(1)<<freeformLengthDepth, freeformWorkLimit,
		"a charge counting only the leaves is what let that degree through")
}

// The measured defect: a validator-accepted single-span NURBS of high degree
// subdivided 1023 times, each split blending half a million rational pairs,
// with no charge past the leaf count. The preflight must refuse it before the
// first split — and public Extrude reaches this through walkOf, so the walk must
// refuse it too rather than run the bracket on the way to its staging refusal.
func TestWideSpanBracketRefusesBeforeSubdividing(t *testing.T) {
	const degree = 1024
	seg := oneSpanNURBS(degree)
	require.NoError(t, validateNURBSSegment(seg), "the record itself is well formed")

	work := &freeformWork{}
	spans, _, err := freeformBezierSpans(seg, work)
	require.NoError(t, err, "a single span needs no knot insertion")
	require.Len(t, spans, 1)

	start := time.Now()
	_, _, err = freeformArcLength(spans, work)
	require.ErrorIs(t, err, ErrUnsupported)
	require.Contains(t, err.Error(), "work budget")
	require.Less(t, time.Since(start), 10*time.Second, "the refusal precedes the subdivision")

	start = time.Now()
	_, err = walkOf(seg, newFreeformWork())
	require.ErrorIs(t, err, ErrUnsupported)
	require.Contains(t, err.Error(), "work budget")
	require.Less(t, time.Since(start), 10*time.Second, "the walk refuses on the same preflight")
}

// oneSpanNURBS is a valid non-rational clamped NURBS of the given degree over
// degree+1 control points: no interior knot, so it converts to exactly one
// Bézier span and the conversion pass itself charges almost nothing.
func oneSpanNURBS(degree int) NURBSSeg {
	control := make([]Point2, degree+1)
	for i := range control {
		control[i] = Point2{U: float64(i), V: float64(i % 7)}
	}
	return equalWeightNURBS(control)
}

// equalWeightNURBS clamps the given control net into a single-span, equal-weight
// NURBSSeg: degree len-1, no interior knot.
func equalWeightNURBS(control []Point2) NURBSSeg {
	n := len(control)
	weights := make([]float64, n)
	knots := make([]float64, 0, 2*n)
	for i := range weights {
		weights[i] = 1
	}
	for range n {
		knots = append(knots, 0)
	}
	for range n {
		knots = append(knots, 1)
	}
	return NURBSSeg{Degree: n - 1, Control: control, Knots: knots, Weights: weights, TStart: 0, TEnd: 1}
}

// The R7 ceiling belongs to the RECORD, across every phase of one operation
// (docs/spline-design.md §5.2), and the length bracket is bound to that rule
// (§6.1). A moments preflight and the walk resolution that follows it are two
// phases over the same record, so the walk spends what the preflight left; a
// counter minted inside the walk hands the same record a second full ceiling and
// subdivides for seconds over work the first phase already proved unaffordable.
//
// The assertion MEASURES the second phase, because a second counter is invisible
// to any unit count: both counters read well under the limit, and only the cost
// tells them apart.
func TestWalkSpendsTheRecordsRemainingCeiling(t *testing.T) {
	seg := ringSplineSeg(45)
	record := ProfileRecord{Outer: LoopRecord{Segments: []CurveSegment{seg}}}

	pre, err := validateMomentFields(record)
	require.NoError(t, err, "the preflight admits this record")
	require.Greater(t, pre.work.spent, freeformWorkLimit/2,
		"the fixture is sized so the preflight alone spends most of the record's ceiling")

	var before, after runtime.MemStats
	runtime.GC()
	runtime.ReadMemStats(&before)
	start := time.Now()
	_, err = walkOf(seg, pre.work)
	elapsed := time.Since(start)
	runtime.ReadMemStats(&after)

	require.ErrorIs(t, err, ErrUnsupported)
	require.Contains(t, err.Error(), "work budget",
		"the walk refuses on the record's own ceiling, not on a fresh one")
	// A second ceiling runs the whole bracket over this chain: 2.17 s and
	// 1.46 GB measured. What is left of the record's ceiling cannot pay for one
	// span of it, so the refusal costs neither.
	require.Less(t, after.TotalAlloc-before.TotalAlloc, uint64(64)<<20,
		"a second ceiling would allocate gigabytes of rational subdivision here")
	require.Less(t, elapsed, 2*time.Second,
		"a second ceiling would subdivide for seconds before refusing")
}

// walkOf mints no counter of its own: successive resolutions charge the counter
// they were handed. A per-call counter spends the same units twice from zero,
// which is exactly what leaves the ceiling unable to bound a whole record.
func TestWalkChargesTheCounterItIsGiven(t *testing.T) {
	seg := SplineSeg{
		Control: []Point2{{U: 0, V: 0}, {U: 1, V: 2}, {U: 3, V: 2}, {U: 4, V: 0}},
		TStart:  0,
		TEnd:    1,
	}
	shared := newFreeformWork()

	_, err := walkOf(seg, shared)
	require.NoError(t, err)
	first := shared.spent
	require.Positive(t, first, "a free-form walk charges its conversion and its bracket")

	_, err = walkOf(seg, shared)
	require.NoError(t, err)
	require.Equal(t, 2*first, shared.spent,
		"the second resolution accumulates onto the same counter")
}

// A free-form resolution handed no counter has no ceiling at all, so it refuses
// rather than quietly open one — the silent mint is the defect this rule closes.
func TestFreeformWalkWithoutCounterRefuses(t *testing.T) {
	seg := SplineSeg{
		Control: []Point2{{U: 0, V: 0}, {U: 1, V: 2}, {U: 3, V: 2}, {U: 4, V: 0}},
		TStart:  0,
		TEnd:    1,
	}
	_, err := walkOf(seg, nil)
	require.ErrorIs(t, err, ErrUnsupported)
	require.Contains(t, err.Error(), "work counter")

	// An analytic walk charges nothing, so it is unaffected.
	_, err = walkOf(LineSeg{Start: Point2{}, End: Point2{U: 1, V: 1}, TEnd: 1}, nil)
	require.NoError(t, err)
}

// ringSplineSeg is a cubic spline whose control points ride a circle. Its
// conversion is the widest single Tier A segment the record preflight admits at
// 45 controls, which is what makes the remaining ceiling small enough to observe.
func ringSplineSeg(controls int) SplineSeg {
	control := make([]Point2, controls)
	for i := range control {
		a := 2 * math.Pi * float64(i) / float64(controls)
		control[i] = Point2{U: 10 * math.Cos(a), V: 10 * math.Sin(a)}
	}
	return SplineSeg{Control: control, TStart: 0, TEnd: 1}
}
