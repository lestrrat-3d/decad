package decad

import (
	"context"
	"math"
	"math/big"
	"math/rand/v2"
	"testing"

	"github.com/stretchr/testify/require"
)

// The positive-area gate reads the region's own exact rational wherever there is
// one, and the float accumulator only where there is not. Underflow is why: a
// strictly positive rational can round to a float zero, and refusing it would
// deny a measurement the accumulator already holds. Every non-positive region
// must still refuse, on either arithmetic.
func TestPositiveAreaGateConsultsExactRational(t *testing.T) {
	withExact := func(area *big.Rat, held float64) *regionIntegrals {
		ig := &regionIntegrals{area: held, exact: newExactMoments()}
		ig.exact.area.Set(area)
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
	segments := make([]CurveSegment, workPollInterval+64)
	for i := range segments {
		start := Point2{U: float64(i), V: 0}
		segments[i] = LineSeg{Start: start, End: Point2{U: start.U + 1, V: math.Sin(float64(i))}, TEnd: 1}
	}
	record := ProfileRecord{Outer: LoopRecord{Segments: segments}}
	ctx := &internalFrameCancelContext{Context: t.Context(), target: "validateMomentFieldsBudget"}

	_, err := record.integralsBudget(newWorkBudget(ctx))

	require.ErrorIs(t, err, context.Canceled)
	require.True(t, ctx.entered, `moment field validation must poll inside its segment scan`)
}

// ratLerpGeneral holds ratLerp's pre-fast-path body verbatim, so the
// endpoint case added to ratLerp can be checked against the general formula
// it now bypasses at t == 0 and t == 1.
func ratLerpGeneral(start, end, t float64) *big.Rat {
	rs, re, rt := floatRat(start), floatRat(end), floatRat(t)
	if rs == nil || re == nil || rt == nil {
		return nil
	}
	return new(big.Rat).Add(rs, new(big.Rat).Mul(rt, new(big.Rat).Sub(re, rs)))
}

// TestRatLerpEndpointsMatchTheGeneralPath is the correctness proof for
// ratLerp's endpoint fast path: over the cross product of a fixed value set
// (including negative zero, both infinities and NaN) and a fixed parameter
// set, at t == 0 and t == 1 alike, ratLerp must return exactly what
// ratLerpGeneral returns — nil for nil, and an exact rational equal by
// Cmp otherwise. This is what settles the claim, not a tolerance: a sampled
// near-endpoint parameter belongs to the general path, never this one.
func TestRatLerpEndpointsMatchTheGeneralPath(t *testing.T) {
	negZero := math.Copysign(0, -1)
	values := []float64{
		0, negZero, 1, -1, 1e-300, 1e300,
		math.MaxFloat64, math.SmallestNonzeroFloat64,
		math.Inf(1), math.Inf(-1), math.NaN(),
	}
	rng := rand.New(rand.NewPCG(41, 43))
	for range 200 {
		// Raw bit patterns cover every float64 class deterministically:
		// normals across every magnitude, subnormals, both zeros, both
		// infinities and a spread of NaN payloads.
		values = append(values, math.Float64frombits(rng.Uint64()))
	}
	params := []float64{0, negZero, 1, 0.5, 0.25, 1.0 / 3.0, -0.5, 2, math.NaN(), math.Inf(1)}

	checked := 0
	for _, start := range values {
		for _, end := range values {
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
	}
	require.Greater(t, checked, 440000, "the fixture must exercise the full cross product")

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

// exactAtanSeries is atanSmallInterval's pre-port body, kept here verbatim as
// the oracle TestAtanSmallIntervalContainsExactSeries checks the fixed-point
// port against: the same 64-term alternating series and x^129/129 remainder,
// evaluated over exact big.Rat instead of the fixed-point grid.
func exactAtanSeries(x *big.Rat) ratInterval {
	if x.Sign() < 0 {
		return intervalNeg(exactAtanSeries(new(big.Rat).Neg(x)))
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
	return interval(sum, new(big.Rat).Add(sum, remainder))
}

// TestAtanSmallIntervalContainsExactSeries is the fast-path/slow-path
// equivalence proof the fixed-point port turns on: the new grid-evaluated
// enclosure must CONTAIN the old exact-rational one (never narrower, since a
// narrower bound would mean some rounding direction turned inward), and the
// gap the grid's extra truncation opens up must stay far under float64
// resolution.
func TestAtanSmallIntervalContainsExactSeries(t *testing.T) {
	rng := rand.New(rand.NewPCG(1, 2))

	var args []*big.Rat
	for range 20 {
		x := floatRat(rng.Float64() * 0.5)
		args = append(args, x, new(big.Rat).Neg(x))
	}
	for range 20 {
		a := floatRat(rng.Float64())
		b := floatRat(float64(2 + rng.IntN(999))) // [2, 1000]
		q := new(big.Rat).Quo(a, b)
		args = append(args, q, new(big.Rat).Neg(q))
	}
	tiny := floatRat(1e-9)
	args = append(args,
		tiny, new(big.Rat).Neg(tiny),
		floatRat(1.0009765625e-9), new(big.Rat).Neg(floatRat(1.0009765625e-9)),
	)
	require.GreaterOrEqualf(t, len(args), 60, "need at least 60 arguments")

	const widthCeiling = 0x1p-120 // measured worst case during the investigation: ~2^-137.6

	for _, x := range args {
		got := atanSmallInterval(x)
		want := exactAtanSeries(x)

		require.LessOrEqualf(t, got.lo.Cmp(want.lo), 0, "x=%v: new lower bound narrower than the exact series", x)
		require.GreaterOrEqualf(t, got.hi.Cmp(want.hi), 0, "x=%v: new upper bound narrower than the exact series", x)

		width := new(big.Rat).Sub(got.hi, got.lo)
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
	args := []float64{0, 0.5, -0.5, 1.0 / 3, 1.0 / 1024, math.Ldexp(1, -400)}
	for i := 1; i <= 50; i++ {
		args = append(args, float64(i)/100)
	}

	for _, x64 := range args {
		x := floatRat(x64)
		require.NotNilf(t, x, "x=%v", x64)
		got := atanSmallInterval(x)
		require.LessOrEqualf(t, got.lo.Cmp(got.hi), 0, "x=%v: interval inverted", x64)

		truth := math.Atan(x64)
		truthRat := floatRat(truth)
		ulp := new(big.Rat).Sub(floatRat(math.Nextafter(truth, math.Inf(1))), truthRat)
		if ulp.Sign() < 0 {
			ulp.Neg(ulp)
		}
		slack := new(big.Rat).Mul(ulp, big.NewRat(2, 1))
		upper := new(big.Rat).Add(truthRat, slack)
		lower := new(big.Rat).Sub(truthRat, slack)

		require.LessOrEqualf(t, got.lo.Cmp(upper), 0, "x=%v: lower bound above truth+2ulp", x64)
		require.GreaterOrEqualf(t, got.hi.Cmp(lower), 0, "x=%v: upper bound below truth-2ulp", x64)
	}
}

// TestAtanSmallIntervalDegenerateArguments checks the two edge cases the
// fixed-point grid's outward rounding must never mishandle: an argument that
// lands exactly on zero, and one so small it underflows the grid entirely.
func TestAtanSmallIntervalDegenerateArguments(t *testing.T) {
	t.Run("zero", func(t *testing.T) {
		got := atanSmallInterval(new(big.Rat))
		gridUnit := new(big.Rat).SetFrac(big.NewInt(1), new(big.Int).Lsh(big.NewInt(1), trigFixedBits))
		negGridUnit := new(big.Rat).Neg(gridUnit)
		require.LessOrEqualf(t, got.lo.Cmp(gridUnit), 0, "lower bound too far above zero")
		require.GreaterOrEqualf(t, got.lo.Cmp(negGridUnit), 0, "lower bound too far below zero")
		require.LessOrEqualf(t, got.hi.Cmp(gridUnit), 0, "upper bound too far above zero")
		require.GreaterOrEqualf(t, got.hi.Cmp(negGridUnit), 0, "upper bound too far below zero")
	})

	t.Run("underflowing tiny argument", func(t *testing.T) {
		tiny := new(big.Rat).SetFrac(big.NewInt(1), new(big.Int).Lsh(big.NewInt(1), 400))
		got := atanSmallInterval(tiny)
		require.LessOrEqualf(t, got.lo.Sign(), 0, "lower bound must not exceed zero")
		require.Greaterf(t, got.hi.Sign(), 0, "upper bound must be strictly positive")
	})
}

// BenchmarkAtanSmallInterval is task fu159 §9's per-call cost guard: 20
// full-53-bit-dyadic arguments, the shape atan2Interval actually passes down
// (a ratio of two recorded coordinate deltas) and the shape whose powers blow
// up a big.Rat numerator.
func BenchmarkAtanSmallInterval(b *testing.B) {
	args := make([]*big.Rat, 20)
	for i := range args {
		args[i] = floatRat(0.5 * float64(i+1) / 21.0)
	}
	b.ReportAllocs()
	for b.Loop() {
		for _, x := range args {
			atanSmallInterval(x)
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
			pairs = append(pairs, pair{floatRat(y), floatRat(x)})
		}
	}
	b.ReportAllocs()
	for b.Loop() {
		for _, p := range pairs {
			atan2Interval(p.y, p.x, false)
		}
	}
}

// BenchmarkPiInterval pins task fu159's caching of the pi multiples. The
// cached arm measures the three accessors callers now reach the multiples
// through; the rebuild arm measures the intervalScale-over-piLower/piUpper
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
			piIntervalSink[0] = quarterPiInterval()
			piIntervalSink[1] = halfPiInterval()
			piIntervalSink[2] = twoPiInterval()
		}
	})

	b.Run("rebuild", func(b *testing.B) {
		b.ReportAllocs()
		for b.Loop() {
			piIntervalSink[0] = intervalScale(interval(piLower, piUpper), big.NewRat(1, 4))
			piIntervalSink[1] = intervalScale(interval(piLower, piUpper), big.NewRat(1, 2))
			piIntervalSink[2] = intervalScale(interval(piLower, piUpper), big.NewRat(2, 1))
		}
	})
}

// piIntervalSink holds BenchmarkPiInterval's three pi multiples, one slot per
// multiple, so neither arm's measured work is dead on arrival.
var piIntervalSink [3]ratInterval
