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
