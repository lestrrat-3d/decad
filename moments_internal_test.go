package decad

import (
	"context"
	"math"
	"math/big"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/freeform"

	"github.com/lestrrat-3d/decad/internal/survey2d"

	"github.com/lestrrat-3d/decad/internal/proofbound"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestFirstOrderMomentsMatchFullAreaCentroidAndBounds(t *testing.T) {
	t.Parallel()
	check := func(t *testing.T, first, full regionIntegrals) {
		t.Helper()
		first.publishExact()
		full.publishExact()
		require.Equal(t, full.area, first.area)
		require.Equal(t, full.areaBound, first.areaBound)
		require.Equal(t, full.mu, first.mu)
		require.Equal(t, full.muBound, first.muBound)
		require.Equal(t, full.mv, first.mv)
		require.Equal(t, full.mvBound, first.mvBound)
		require.Zero(t, first.muu)
		require.Zero(t, first.muv)
		require.Zero(t, first.mvv)
		require.True(t, full.muu != 0 || full.muv != 0 || full.mvv != 0)
	}

	for _, tc := range []struct {
		name    string
		segment CurveSegment
	}{
		{"line", LineSeg{Start: Point2{U: 2, V: 1}, End: Point2{U: 5, V: 4}, TEnd: 1}},
		{"whole circle", CircleSeg{Center: Point2{U: 4, V: 3}, Radius: units.Millimeters(2), CCW: true, TEnd: 1}},
		{"circle fragment", CircleSeg{Center: Point2{U: 4, V: 3}, Radius: units.Millimeters(2), CCW: true, TStart: 0.125, TEnd: 0.625}},
		{"arc", ArcSeg{Center: Point2{U: 4, V: 3}, Start: Point2{U: 5, V: 3}, End: Point2{U: 4, V: 4}, TEnd: 1}},
	} {
		t.Run(tc.name, func(t *testing.T) {
			var first, full regionIntegrals
			require.NoError(t, first.addFor(tc.segment, freeformPlan{}, Point2{}, freeform.MomentFirstOrder))
			require.NoError(t, full.addFor(tc.segment, freeformPlan{}, Point2{}, freeform.MomentSecondOrder))
			check(t, first, full)
			var area regionIntegrals
			require.NoError(t, area.addFor(tc.segment, freeformPlan{}, Point2{}, freeform.MomentAreaOrder))
			area.publishExact()
			require.Equal(t, full.area, area.area)
			require.Equal(t, full.areaBound, area.areaBound)
			require.Zero(t, area.muu)
			require.Zero(t, area.muv)
			require.Zero(t, area.mvv)
		})
	}

	t.Run("freeform", func(t *testing.T) {
		spans := []survey2d.BezierSpan{{
			{U: big.NewRat(0, 1), V: big.NewRat(0, 1)},
			{U: big.NewRat(1, 1), V: big.NewRat(2, 1)},
			{U: big.NewRat(3, 1), V: big.NewRat(0, 1)},
		}}
		var first, full regionIntegrals
		first.addFreeformTo(spans, false, freeform.MomentFirstOrder)
		full.addFreeformTo(spans, false, freeform.MomentSecondOrder)
		check(t, first, full)
		var area regionIntegrals
		area.addFreeformTo(spans, false, freeform.MomentAreaOrder)
		area.publishExact()
		require.Equal(t, full.area, area.area)
		require.Equal(t, full.areaBound, area.areaBound)
		require.Zero(t, area.muu)
		require.Zero(t, area.muv)
		require.Zero(t, area.mvv)
	})

	t.Run("offset rectangle through evaluator", func(t *testing.T) {
		record := ProfileRecord{Outer: LoopRecord{Segments: []CurveSegment{
			LineSeg{Start: Point2{U: 100.25, V: -50.5}, End: Point2{U: 120.25, V: -50.5}, TEnd: 1},
			LineSeg{Start: Point2{U: 120.25, V: -50.5}, End: Point2{U: 120.25, V: -45.5}, TEnd: 1},
			LineSeg{Start: Point2{U: 120.25, V: -45.5}, End: Point2{U: 100.25, V: -45.5}, TEnd: 1},
			LineSeg{Start: Point2{U: 100.25, V: -45.5}, End: Point2{U: 100.25, V: -50.5}, TEnd: 1},
		}}}
		first, err := record.evaluatorIntegralsUncheckedContext(t.Context(), freeform.MomentFirstOrder, freeform.NewFreeformWork())
		require.NoError(t, err)
		full, err := record.evaluatorIntegralsUncheckedContext(t.Context(), freeform.MomentSecondOrder, freeform.NewFreeformWork())
		require.NoError(t, err)
		check(t, first, full)
		area, err := record.evaluatorIntegrals(freeform.MomentAreaOrder, freeform.NewFreeformWork())
		require.NoError(t, err)
		require.Equal(t, full.area, area.area)
		require.Equal(t, full.areaBound, area.areaBound)
		require.Zero(t, area.muu)
		require.Zero(t, area.muv)
		require.Zero(t, area.mvv)
		firstCentroid, firstExact := first.exactCentroid()
		fullCentroid, fullExact := full.exactCentroid()
		require.True(t, firstExact)
		require.True(t, fullExact)
		require.Equal(t, fullCentroid, firstCentroid)
	})

	t.Run("area with overflowing higher moments", func(t *testing.T) {
		seg := LineSeg{Start: Point2{}, End: Point2{U: 1e120, V: 1e120}, TEnd: 1}
		var area regionIntegrals
		require.NoError(t, area.addFor(seg, freeformPlan{}, Point2{}, freeform.MomentAreaOrder))
		require.True(t, area.isFinite(freeform.MomentAreaOrder))
		require.False(t, area.isFinite(freeform.MomentFirstOrder))
		require.Zero(t, area.muu)
		require.Zero(t, area.muv)
		require.Zero(t, area.mvv)
	})

	t.Run("freeform work charges", func(t *testing.T) {
		profile := involuteFitProfile()
		areaWork := freeform.NewFreeformWork()
		_, err := profile.evaluatorIntegrals(freeform.MomentAreaOrder, areaWork)
		require.NoError(t, err)
		fullWork := freeform.NewFreeformWork()
		_, err = profile.evaluatorIntegrals(freeform.MomentSecondOrder, fullWork)
		require.NoError(t, err)
		require.Positive(t, areaWork.Spent)
		require.Positive(t, areaWork.ReconstructionSpent)
		require.Equal(t, *fullWork, *areaWork)
	})
}

// The positive-area gate reads the region's own exact rational wherever there is
// one, and the float accumulator only where there is not. Underflow is why: a
// strictly positive rational can round to a float zero, and refusing it would
// deny a measurement the accumulator already holds. Every non-positive region
// must still refuse, on either arithmetic.
func TestPositiveAreaGateConsultsExactRational(t *testing.T) {
	t.Parallel()
	withExact := func(area *big.Rat, held float64) *regionIntegrals {
		ig := &regionIntegrals{area: held, exact: newExactMoments()}
		ig.exact.Area.Set(area)
		return ig
	}
	for _, tc := range []struct {
		name    string
		ig      *regionIntegrals
		refuses bool
	}{
		{
			name: "positive rational underflowing to zero",
			ig:   withExact(big.NewRat(1, 1<<62), 0),
		},
		{
			name: "positive rational and positive float",
			ig:   withExact(big.NewRat(3, 2), 1.5),
		},
		{
			name:    "zero rational",
			ig:      withExact(new(big.Rat), 0),
			refuses: true,
		},
		{
			name:    "negative rational held as a positive float",
			ig:      withExact(big.NewRat(-3, 2), 1.5),
			refuses: true,
		},
		{
			name:    "retired accumulator with a non-positive float",
			ig:      &regionIntegrals{area: 0, exactDead: true},
			refuses: true,
		},
		{
			name: "retired accumulator with a positive float",
			ig:   &regionIntegrals{area: 2, exactDead: true},
		},
	} {
		t.Run(tc.name, func(t *testing.T) {
			err := tc.ig.requirePositiveArea()
			if !tc.refuses {
				require.NoError(t, err)
				return
			}
			require.ErrorIs(t, err, ErrDegenerate)
			require.Contains(t, err.Error(), "no positive net area")
		})
	}
}

func TestMomentValidationCancellationIsBounded(t *testing.T) {
	t.Parallel()
	segments := make([]CurveSegment, proofbound.WorkPollInterval+64)
	for i := range segments {
		start := Point2{U: float64(i), V: 0}
		segments[i] = LineSeg{Start: start, End: Point2{U: start.U + 1, V: math.Sin(float64(i))}, TEnd: 1}
	}
	record := ProfileRecord{Outer: LoopRecord{Segments: segments}}
	ctx := &internalFrameCancelContext{Context: t.Context(), target: "validateMomentFieldsBudget"}

	_, err := record.integralsBudget(proofbound.NewWorkBudget(ctx))

	require.ErrorIs(t, err, context.Canceled)
	require.True(t, ctx.entered, `moment field validation must poll inside its segment scan`)
}

// ratLerpGeneral holds ratLerp's pre-fast-path body verbatim, so the
// endpoint case added to ratLerp can be checked against the general formula
// it now bypasses at t == 0 and t == 1.
func ratLerpGeneral(start, end, t float64) *big.Rat {
	rs, re, rt := proofarith.FloatRat(start), proofarith.FloatRat(end), proofarith.FloatRat(t)
	if rs == nil || re == nil || rt == nil {
		return nil
	}
	return new(big.Rat).Add(rs, new(big.Rat).Mul(rt, new(big.Rat).Sub(re, rs)))
}

// TestRatLerpEndpointsMatchTheGeneralPath is the correctness proof for
// ratLerp's endpoint fast path: over the cross product of a fixed value set
// (including negative zero, both infinities and NaN), plus seeded raw-bit
// pairs, and a fixed parameter set, ratLerp must return exactly what
// ratLerpGeneral returns — nil for nil, and an exact rational equal by Cmp
// otherwise. This is what settles the claim, not a tolerance: a sampled
// near-endpoint parameter belongs to the general path, never this one.
func TestRatLerpEndpointsMatchTheGeneralPath(t *testing.T) {
	t.Parallel()
	negZero := math.Copysign(0, -1)
	values := []float64{
		0, negZero, 1, -1, 1e-300, 1e300,
		math.MaxFloat64, math.SmallestNonzeroFloat64,
		math.Inf(1), math.Inf(-1), math.NaN(),
	}
	params := []float64{0, negZero, 1, 0.5, 0.25, 1.0 / 3.0, -0.5, 2, math.NaN(), math.Inf(1)}

	checked := 0
	checkPair := func(start, end float64) {
		for _, tParam := range params {
			want := ratLerpGeneral(start, end, tParam)
			got := ratLerp(start, end, tParam)
			checked++
			if want == nil {
				require.Nil(t, got, "start=%v end=%v t=%v", start, end, tParam)
				continue
			}
			require.NotNil(t, got, "start=%v end=%v t=%v", start, end, tParam)
			require.Zero(t, got.Cmp(want),
				"start=%v end=%v t=%v got=%v want=%v", start, end, tParam, got, want)
		}
	}
	for _, start := range values {
		for _, end := range values {
			checkPair(start, end)
		}
	}
	rng := rand.New(rand.NewPCG(41, 43))
	for range 200 {
		checkPair(math.Float64frombits(rng.Uint64()), math.Float64frombits(rng.Uint64()))
	}
	require.Equal(t, 3210, checked, "the fixture must exercise every special pair and 200 raw-bit pairs")

	// A degenerate TStart == TEnd == 0 record defeats "the other endpoint is
	// checked anyway": the far operand here is never read by the general
	// path's own Sub/Mul, yet the fast path must still refuse it.
	require.Nil(t, ratLerp(3, math.NaN(), 0), "a NaN far endpoint must still refuse at t == 0")
	require.Nil(t, ratLerp(math.Inf(1), 7, 1), "an infinite far endpoint must still refuse at t == 1")
}

// TestRatLerpEndpointReturnsAFreshRational pins the "never memoize" rule:
// exactLineMoments mutates its ratLerp results in place, so a cached or
// shared rational at the endpoint case would corrupt the next call.
func TestRatLerpEndpointReturnsAFreshRational(t *testing.T) {
	t.Parallel()
	a := ratLerp(2, 5, 0)
	require.NotNil(t, a)
	a.Sub(a, big.NewRat(1, 1))

	b := ratLerp(2, 5, 0)
	require.NotNil(t, b)
	require.Zero(t, b.Cmp(big.NewRat(2, 1)))
}

// BenchmarkRatLerpWholeEdge measures the recorded-whole-edge call shape:
// exactLineMoments and lineWalkBounds each run one ratLerp(start, end, 0)
// and one ratLerp(start, end, 1) per whole LineSeg, the fast path's target.
func BenchmarkRatLerpWholeEdge(b *testing.B) {
	for b.Loop() {
		ratLerp(100, 60, 0)
		ratLerp(100, 60, 1)
	}
}

// BenchmarkRatLerpTrimmed is the guard, not a target: it measures the
// interior-parameter shape a Partial fragment produces, which must take the
// unchanged general path. Its allocs/op must not move.
func BenchmarkRatLerpTrimmed(b *testing.B) {
	for b.Loop() {
		ratLerp(100, 60, 0.25)
		ratLerp(100, 60, 0.75)
	}
}

// exactAtanSeries is proofbound.AtanSmallInterval's pre-port body, kept here verbatim as
// the oracle TestAtanSmallIntervalContainsExactSeries checks the fixed-point
// port against: the same 64-term alternating series and x^129/129 remainder,
// evaluated over exact big.Rat instead of the fixed-point grid.
func exactAtanSeries(x *big.Rat) proofbound.RatInterval {
	if x.Sign() < 0 {
		return proofbound.IntervalNeg(exactAtanSeries(new(big.Rat).Neg(x)))
	}
	x2 := new(big.Rat).Mul(x, x)
	power := new(big.Rat).Set(x)
	sum := new(big.Rat)
	for n := range 64 {
		term := new(big.Rat).Quo(power, big.NewRat(int64(2*n+1), 1))
		if n%2 == 0 {
			sum.Add(sum, term)
		} else {
			sum.Sub(sum, term)
		}
		power.Mul(power, x2)
	}
	remainder := new(big.Rat).Quo(power, big.NewRat(129, 1))
	return proofbound.Interval(sum, new(big.Rat).Add(sum, remainder))
}

// TestAtanSmallIntervalContainsExactSeries is the fast-path/slow-path
// equivalence proof the fixed-point port turns on: the new grid-evaluated
// enclosure must CONTAIN the old exact-rational one (never narrower, since a
// narrower bound would mean some rounding direction turned inward), and the
// gap the grid's extra truncation opens up must stay far under float64
// resolution.
func TestAtanSmallIntervalContainsExactSeries(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(1, 2))

	var args []*big.Rat
	for range 20 {
		x := proofarith.FloatRat(rng.Float64() * 0.5)
		args = append(args, x, new(big.Rat).Neg(x))
	}
	for range 20 {
		a := proofarith.FloatRat(rng.Float64())
		b := proofarith.FloatRat(float64(2 + rng.IntN(999))) // [2, 1000]
		q := new(big.Rat).Quo(a, b)
		args = append(args, q, new(big.Rat).Neg(q))
	}
	tiny := proofarith.FloatRat(1e-9)
	args = append(args,
		tiny, new(big.Rat).Neg(tiny),
		proofarith.FloatRat(1.0009765625e-9), new(big.Rat).Neg(proofarith.FloatRat(1.0009765625e-9)),
	)
	require.GreaterOrEqualf(t, len(args), 60, "need at least 60 arguments")

	const widthCeiling = 0x1p-120 // measured worst case during the investigation: ~2^-137.6

	for _, x := range args {
		got := proofbound.AtanSmallInterval(x)
		want := exactAtanSeries(x)

		require.LessOrEqualf(t, got.Lo.Cmp(want.Lo), 0, "x=%v: new lower bound narrower than the exact series", x)
		require.GreaterOrEqualf(t, got.Hi.Cmp(want.Hi), 0, "x=%v: new upper bound narrower than the exact series", x)

		width := new(big.Rat).Sub(got.Hi, got.Lo)
		widthF, _ := width.Float64()
		require.Lessf(t, widthF, widthCeiling, "x=%v: enclosure too wide", x)
	}
}

// TestAtanSmallIntervalEnclosesMathAtan is the independent-oracle check,
// modelled on TestTurnSinCosIntervalEnclosesMathSincos: math.Atan's own
// float64 answer must land inside the returned enclosure, widened by two ulps
// on each side to absorb math.Atan's own (undocumented, but necessarily tiny)
// rounding.
//
// The comparison stays entirely in big.Rat: converting an exact rational
// bound to float64 for comparison can round it the wrong way (this happened
// during the investigation at x ~ -0.2287, where an exact lower bound rounded
// UP past the true value), so every comparison here is a big.Rat Cmp against
// a big.Rat slack built from math.Nextafter, never a float64 <=.
func TestAtanSmallIntervalEnclosesMathAtan(t *testing.T) {
	t.Parallel()
	args := []float64{0, 0.5, -0.5, 1.0 / 3, 1.0 / 1024, math.Ldexp(1, -400)}
	for i := 1; i <= 50; i++ {
		args = append(args, float64(i)/100)
	}

	for _, x64 := range args {
		x := proofarith.FloatRat(x64)
		require.NotNilf(t, x, "x=%v", x64)
		got := proofbound.AtanSmallInterval(x)
		require.LessOrEqualf(t, got.Lo.Cmp(got.Hi), 0, "x=%v: interval inverted", x64)

		truth := math.Atan(x64)
		truthRat := proofarith.FloatRat(truth)
		ulp := new(big.Rat).Sub(proofarith.FloatRat(math.Nextafter(truth, math.Inf(1))), truthRat)
		if ulp.Sign() < 0 {
			ulp.Neg(ulp)
		}
		slack := new(big.Rat).Mul(ulp, big.NewRat(2, 1))
		upper := new(big.Rat).Add(truthRat, slack)
		lower := new(big.Rat).Sub(truthRat, slack)

		require.LessOrEqualf(t, got.Lo.Cmp(upper), 0, "x=%v: lower bound above truth+2ulp", x64)
		require.GreaterOrEqualf(t, got.Hi.Cmp(lower), 0, "x=%v: upper bound below truth-2ulp", x64)
	}
}

// TestAtanSmallIntervalDegenerateArguments checks the two edge cases the
// fixed-point grid's outward rounding must never mishandle: an argument that
// lands exactly on zero, and one so small it underflows the grid entirely.
func TestAtanSmallIntervalDegenerateArguments(t *testing.T) {
	t.Parallel()
	t.Run("zero", func(t *testing.T) {
		got := proofbound.AtanSmallInterval(new(big.Rat))
		gridUnit := new(big.Rat).SetFrac(big.NewInt(1), new(big.Int).Lsh(big.NewInt(1), proofbound.TrigFixedBits))
		negGridUnit := new(big.Rat).Neg(gridUnit)
		require.LessOrEqualf(t, got.Lo.Cmp(gridUnit), 0, "lower bound too far above zero")
		require.GreaterOrEqualf(t, got.Lo.Cmp(negGridUnit), 0, "lower bound too far below zero")
		require.LessOrEqualf(t, got.Hi.Cmp(gridUnit), 0, "upper bound too far above zero")
		require.GreaterOrEqualf(t, got.Hi.Cmp(negGridUnit), 0, "upper bound too far below zero")
	})

	t.Run("underflowing tiny argument", func(t *testing.T) {
		tiny := new(big.Rat).SetFrac(big.NewInt(1), new(big.Int).Lsh(big.NewInt(1), 400))
		got := proofbound.AtanSmallInterval(tiny)
		require.LessOrEqualf(t, got.Lo.Sign(), 0, "lower bound must not exceed zero")
		require.Greaterf(t, got.Hi.Sign(), 0, "upper bound must be strictly positive")
	})
}

// BenchmarkAtanSmallInterval is task fu159 §9's per-call cost guard: 20
// full-53-bit-dyadic arguments, the shape proofbound.Atan2Interval actually passes down
// (a ratio of two recorded coordinate deltas) and the shape whose powers blow
// up a big.Rat numerator.
func BenchmarkAtanSmallInterval(b *testing.B) {
	args := make([]*big.Rat, 20)
	for i := range args {
		args[i] = proofarith.FloatRat(0.5 * float64(i+1) / 21.0)
	}
	b.ReportAllocs()
	for b.Loop() {
		for _, x := range args {
			proofbound.AtanSmallInterval(x)
		}
	}
}

// BenchmarkAtan2Interval measures the whole bracket a circular segment pays,
// over the nine (y, x) pairs exercising both the quadrant switch and both
// argument-reduction arms.
func BenchmarkAtan2Interval(b *testing.B) {
	ys := []float64{3, -7.25, 0.125}
	xs := []float64{11, 2.5, -4.75}
	type pair struct{ y, x *big.Rat }
	var pairs []pair
	for _, y := range ys {
		for _, x := range xs {
			pairs = append(pairs, pair{proofarith.FloatRat(y), proofarith.FloatRat(x)})
		}
	}
	b.ReportAllocs()
	for b.Loop() {
		for _, p := range pairs {
			proofbound.Atan2Interval(p.y, p.x, false)
		}
	}
}

// BenchmarkPiInterval pins task fu159's caching of the pi multiples. The
// cached arm measures the three accessors callers now reach the multiples
// through; the rebuild arm measures the proofbound.IntervalScale-over-proofbound.PiLower/proofbound.PiUpper
// work those accessors avoid. Both arms produce the same three multiples per
// iteration, so their ns/op are directly comparable.
func BenchmarkPiInterval(b *testing.B) {
	// Both arms park their results in piIntervalSink rather than discarding
	// them: the accessors are thin enough to inline, and a dead result could
	// otherwise let the compiler drop the work being measured. Each multiple
	// gets its own slot, so no assignment overwrites a live one.
	b.Run("cached", func(b *testing.B) {
		b.ReportAllocs()
		for b.Loop() {
			piIntervalSink[0] = proofbound.QuarterPiInterval()
			piIntervalSink[1] = proofbound.HalfPiInterval()
			piIntervalSink[2] = proofbound.TwoPiInterval()
		}
	})

	b.Run("rebuild", func(b *testing.B) {
		b.ReportAllocs()
		for b.Loop() {
			piIntervalSink[0] = proofbound.IntervalScale(proofbound.Interval(proofbound.PiLower, proofbound.PiUpper), big.NewRat(1, 4))
			piIntervalSink[1] = proofbound.IntervalScale(proofbound.Interval(proofbound.PiLower, proofbound.PiUpper), big.NewRat(1, 2))
			piIntervalSink[2] = proofbound.IntervalScale(proofbound.Interval(proofbound.PiLower, proofbound.PiUpper), big.NewRat(2, 1))
		}
	})
}

// piIntervalSink holds BenchmarkPiInterval's three pi multiples, one slot per
// multiple, so neither arm's measured work is dead on arrival.
var piIntervalSink [3]proofbound.RatInterval

// TestTurnSinCosIntervalEnclosesMathSincos is proofbound.TurnSinCosInterval's own
// enclosure proof (design A7 §5.1): every turn checked must have
// math.Sincos's own float64 answer land inside the returned interval, widened
// by a few ulps to absorb the reference libm call's own (undocumented, but
// necessarily tiny) rounding — and the interval itself must be far tighter
// than float64 resolution, so the fixed-point series is not merely correct
// but not the bottleneck of the bracket it serves.
func TestTurnSinCosIntervalEnclosesMathSincos(t *testing.T) {
	t.Parallel()
	const widthCeiling = 1e-40 // proofbound.TrigFixedSeries's own margin measures in the 1e-58 range
	const ulpSlack = 1e-15

	turns := make([]float64, 0, 400)
	turns = append(turns, 0, 1)
	for k := range 8 {
		boundary := float64(k) / 8
		turns = append(turns, boundary)
		turns = append(turns, math.Nextafter(boundary, math.Inf(1)))
		turns = append(turns, math.Nextafter(boundary, math.Inf(-1)))
	}
	for i := range 400 {
		turns = append(turns, float64(i)/400)
	}

	for _, turn := range turns {
		tr := new(big.Rat).SetFloat64(turn)
		require.NotNil(t, tr, "turn=%g", turn)
		sinIv, cosIv := proofbound.TurnSinCosInterval(tr)

		sinLo, _ := sinIv.Lo.Float64()
		sinHi, _ := sinIv.Hi.Float64()
		cosLo, _ := cosIv.Lo.Float64()
		cosHi, _ := cosIv.Hi.Float64()
		require.LessOrEqualf(t, sinLo, sinHi, "turn=%g: sin interval inverted", turn)
		require.LessOrEqualf(t, cosLo, cosHi, "turn=%g: cos interval inverted", turn)

		wantSin, wantCos := math.Sincos(2 * math.Pi * turn)
		require.GreaterOrEqualf(t, wantSin, sinLo-ulpSlack, "turn=%g: sin below the enclosure", turn)
		require.LessOrEqualf(t, wantSin, sinHi+ulpSlack, "turn=%g: sin above the enclosure", turn)
		require.GreaterOrEqualf(t, wantCos, cosLo-ulpSlack, "turn=%g: cos below the enclosure", turn)
		require.LessOrEqualf(t, wantCos, cosHi+ulpSlack, "turn=%g: cos above the enclosure", turn)

		sinWidth := new(big.Rat).Sub(sinIv.Hi, sinIv.Lo)
		cosWidth := new(big.Rat).Sub(cosIv.Hi, cosIv.Lo)
		sinWidthF, _ := sinWidth.Float64()
		cosWidthF, _ := cosWidth.Float64()
		require.LessOrEqualf(t, sinWidthF, widthCeiling, "turn=%g: sin interval too wide", turn)
		require.LessOrEqualf(t, cosWidthF, widthCeiling, "turn=%g: cos interval too wide", turn)

		// sin^2+cos^2 must enclose 1: the Pythagorean identity holds exactly
		// for the true value, so a sound enclosure of both factors must
		// enclose their sum of squares at 1.
		one := big.NewRat(1, 1)
		sq := func(iv proofbound.RatInterval) proofbound.RatInterval { return proofbound.IntervalMul(iv, iv) }
		pyth := proofbound.IntervalAdd(sq(sinIv), sq(cosIv))
		require.LessOrEqualf(t, pyth.Lo.Cmp(one), 0, "turn=%g: sin^2+cos^2 lower bound above 1", turn)
		require.GreaterOrEqualf(t, pyth.Hi.Cmp(one), 0, "turn=%g: sin^2+cos^2 upper bound below 1", turn)
	}
}

// TestTurnSinCosIntervalMatchesKnownTurns checks a handful of turns whose
// sine/cosine are exactly representable rationals or simple radicals, against
// their own closed forms rather than against math.Sincos, so this test does
// not merely check proofbound.TurnSinCosInterval against the same libm call it is meant
// to replace.
func TestTurnSinCosIntervalMatchesKnownTurns(t *testing.T) {
	t.Parallel()
	sqrt2over2 := math.Sqrt2 / 2
	sqrt3over2 := math.Sqrt(3) / 2
	for _, tc := range []struct {
		turn     float64
		sin, cos float64
	}{
		{0, 0, 1},
		{0.25, 1, 0},
		{0.5, 0, -1},
		{0.75, -1, 0},
		{1.0 / 8, sqrt2over2, sqrt2over2},
		{1.0 / 12, 0.5, sqrt3over2},
		{5.0 / 6, -sqrt3over2, 0.5},
	} {
		tr := new(big.Rat).SetFloat64(tc.turn)
		require.NotNil(t, tr)
		sinIv, cosIv := proofbound.TurnSinCosInterval(tr)
		sinLo, _ := sinIv.Lo.Float64()
		sinHi, _ := sinIv.Hi.Float64()
		cosLo, _ := cosIv.Lo.Float64()
		cosHi, _ := cosIv.Hi.Float64()
		require.InDeltaf(t, tc.sin, (sinLo+sinHi)/2, 1e-12, "turn=%g sin", tc.turn)
		require.InDeltaf(t, tc.cos, (cosLo+cosHi)/2, 1e-12, "turn=%g cos", tc.turn)
		require.LessOrEqualf(t, sinLo, tc.sin+1e-12, "turn=%g sin below expected", tc.turn)
		require.GreaterOrEqualf(t, sinHi, tc.sin-1e-12, "turn=%g sin above expected", tc.turn)
	}
}

// BenchmarkTurnSinCosInterval is the design's own cost guard (A7 §5.6):
// ~34us/turn was the prototype's own figure with a 12-term series later
// found to under-charge the alternating-series remainder (proofbound.TrigSeriesTerms's
// own comment); this benchmark is this file's live measurement of the
// corrected 24-term series' real cost, in place of a number asserted in a
// comment. A fractional CircleSeg charges two calls (one per endpoint) per
// area or first-moment bracket.
func BenchmarkTurnSinCosInterval(b *testing.B) {
	turns := make([]*big.Rat, 64)
	for i := range turns {
		turns[i] = big.NewRat(int64(i), int64(len(turns)))
	}
	b.ResetTimer()
	for i := 0; b.Loop(); i++ {
		proofbound.TurnSinCosInterval(turns[i%len(turns)])
	}
}

// TestThirdOrderMomentsOfASector integrates the sector of radius 5 about the
// origin between the angles of (4, 3) and (0, 5) — two lines through the
// origin and one ArcSeg — at freeform.MomentThirdOrder. In polar form
// ∫u^p·v^q dA = (5⁵/5)·∫cos^p θ·sin^q θ dθ for p + q = 3, and every odd trig
// power has a polynomial antiderivative, so the expected values are exact
// rationals from the endpoints' own cos/sin (4/5, 3/5) and (0, 1). The lines
// contribute through the same dv form as the arc, so only the region's sum is
// comparable, not a single segment.
//
// Shown-to-fail: flipping the sign of circularMonomials' b ≥ 2 boundary term
// separates the third-order sum from its values.
func TestThirdOrderMomentsOfASector(t *testing.T) {
	t.Parallel()
	record := ProfileRecord{Outer: LoopRecord{Segments: []CurveSegment{
		LineSeg{Start: Point2{}, End: Point2{U: 4, V: 3}, TEnd: 1},
		ArcSeg{Center: Point2{}, Start: Point2{U: 4, V: 3}, End: Point2{U: 0, V: 5}, TEnd: 1},
		LineSeg{Start: Point2{U: 0, V: 5}, End: Point2{}, TEnd: 1},
	}}}
	ig, err := record.evaluatorIntegralsContext(t.Context(), freeform.MomentThirdOrder, freeform.NewFreeformWork())
	require.NoError(t, err)
	got, ok := ig.thirdMoments()
	require.True(t, ok)

	c0, s0 := big.NewRat(4, 5), big.NewRat(3, 5)
	c1, s1 := new(big.Rat), big.NewRat(1, 1)
	cube := func(x *big.Rat) *big.Rat { return proofbound.RatMul(x, x, x) }
	third := func(x *big.Rat) *big.Rat { return ratScale(x, 1, 3) }
	trig := [4]*big.Rat{
		new(big.Rat).Sub(new(big.Rat).Sub(s1, third(cube(s1))), new(big.Rat).Sub(s0, third(cube(s0)))),
		third(new(big.Rat).Sub(cube(c0), cube(c1))),
		third(new(big.Rat).Sub(cube(s1), cube(s0))),
		new(big.Rat).Sub(new(big.Rat).Sub(third(cube(c1)), c1), new(big.Rat).Sub(third(cube(c0)), c0)),
	}
	for i, factor := range trig {
		name := []string{"u³", "u²v", "uv²", "v³"}[i]
		want := proofbound.RatMul(big.NewRat(625, 1), factor)
		requireIntervalsOverlap(t, name, got[i], proofbound.PointInterval(want))
		requireIntervalWidthAtMost(t, name, got[i], 1e-9)
	}

	second, err := record.evaluatorIntegralsContext(t.Context(), freeform.MomentSecondOrder, freeform.NewFreeformWork())
	require.NoError(t, err)
	_, ok = second.thirdMoments()
	require.False(t, ok, `a second-order integration carries no third-order sum`)
}
