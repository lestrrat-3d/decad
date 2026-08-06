package decad

import (
	"math/big"
	"runtime"
	"testing"
	"time"

	"github.com/lestrrat-3d/sketch/geom"
	"github.com/stretchr/testify/require"
)

// The conversion is only sound if the spans ARE the curve sketch's own
// interpolant defines. FitSpan is a DIFFERENT (rounded) polynomial from the
// one decad's closed form produces (spline_fit.go's own doc comment), so this
// is an INDEPENDENT-oracle cross-check rather than a bit-identity assertion:
// decad's exact rational control values, converted to monomial form and
// rounded to float64 once, must agree with FitSpan.X/Y to a few ulps.

// monomialFromBezierCubic restates one coordinate's cubic Bezier control
// values b0..b3 in monomial form c0..c3 over the SAME normalized span
// parameter u in [0, 1] that FitSpan itself is stated in (spline_fit.go's own
// doc comment): c0=b0, c1=3(b1-b0), c2=3(b0-2b1+b2), c3=-b0+3b1-3b2+b3. This
// is a monomial CONVERSION used only as a test oracle, never as a production
// path (spline_fit.go never routes through Spans() or through this form).
func monomialFromBezierCubic(b0, b1, b2, b3 *big.Rat) [4]float64 {
	two := big.NewRat(2, 1)
	three := big.NewRat(3, 1)

	c0 := new(big.Rat).Set(b0)

	c1 := new(big.Rat).Sub(b1, b0)
	c1.Mul(c1, three)

	c2 := new(big.Rat).Mul(two, b1)
	c2.Sub(b0, c2)
	c2.Add(c2, b2)
	c2.Mul(c2, three)

	c3 := new(big.Rat).Neg(b0)
	c3.Add(c3, new(big.Rat).Mul(three, b1))
	c3.Sub(c3, new(big.Rat).Mul(three, b2))
	c3.Add(c3, b3)

	f0, _ := c0.Float64()
	f1, _ := c1.Float64()
	f2, _ := c2.Float64()
	f3, _ := c3.Float64()
	return [4]float64{f0, f1, f2, f3}
}

func TestFitSplineBezierMatchesSpansToAFewULPs(t *testing.T) {
	fit := []Point2{{U: 0, V: 0}, {U: 4, V: 3}, {U: 9, V: -1}, {U: 12, V: 2}, {U: 15, V: 0}}
	seg := FitSplineSeg{Fit: fit, TStart: 0, TEnd: 1}

	spans, err := fitSplineBezierSpans(seg, &freeformWork{})
	require.NoError(t, err)

	interp, err := geom.NewFitInterpolant(fitCoords(fit))
	require.NoError(t, err)
	geomSpans := interp.Spans()
	require.Len(t, spans, len(geomSpans))
	require.NotEmpty(t, spans)

	almostEqual := func(t *testing.T, want, got float64, msg string) {
		t.Helper()
		if want == 0 {
			require.InDelta(t, 0, got, 1e-9, msg)
			return
		}
		require.InEpsilon(t, want, got, 1e-9, msg)
	}

	for i, span := range spans {
		gotX := monomialFromBezierCubic(span[0].u, span[1].u, span[2].u, span[3].u)
		gotY := monomialFromBezierCubic(span[0].v, span[1].v, span[2].v, span[3].v)
		want := geomSpans[i]
		for k := range 4 {
			almostEqual(t, want.X[k], gotX[k], "span %d X[%d]")
			almostEqual(t, want.Y[k], gotY[k], "span %d Y[%d]")
		}
	}
}

// TestFitSplineEndpointsAreFitZeroAndActiveLast pins finding #2 directly on
// the converted chain, without depending on any full-loop closure gate: the
// chain's first control point is Fit[0] exactly, and its last is
// FitInterpolant.Points[len-1] — the last ACTIVE (deduplicated) fit point —
// which is NOT Fit[len(Fit)-1] whenever the last two recorded fit points
// coincide within geom's absolute 1e-12 threshold.
func TestFitSplineEndpointsAreFitZeroAndActiveLast(t *testing.T) {
	fit := []Point2{
		{U: 0, V: 0}, {U: 10, V: 0}, {U: 10, V: 10},
		{U: 10 + 3e-13, V: 10}, // collapses into the point before it
	}
	seg := FitSplineSeg{Fit: fit, TStart: 0, TEnd: 1}

	spans, err := fitSplineBezierSpans(seg, &freeformWork{})
	require.NoError(t, err)

	start, end, err := freeformEndpoints(spans, false)
	require.NoError(t, err)
	require.Equal(t, fit[0], start, "the chain's first control point is Fit[0] exactly")
	require.NotEqual(t, fit[len(fit)-1], end,
		"the last two fit points coincide within 1e-12, so the active end is not the raw Fit[len-1]")
	require.Equal(t, fit[2], end, "the active end is the FIRST of the collapsed run, Points[k-1]")

	// Cross-checked against geom's own interpolant directly.
	interp, err := geom.NewFitInterpolant(fitCoords(fit))
	require.NoError(t, err)
	last := interp.Points[len(interp.Points)-1]
	require.Equal(t, Point2{U: last[0], V: last[1]}, end)
}

// allocatedByFit reports how many bytes a call allocates in total — the only
// way to tell a charge levied BEFORE geom.NewFitInterpolant allocates from
// one levied after it: both refuse, and only the measurement distinguishes
// them (mirrors spline_moments_test.go's external allocatedBy).
func allocatedByFit(call func()) uint64 {
	runtime.GC()
	var before, after runtime.MemStats
	runtime.ReadMemStats(&before)
	call()
	runtime.ReadMemStats(&after)
	return after.TotalAlloc - before.TotalAlloc
}

// fitInterpolantCost(n) = 64n has to be reserved BEFORE geom.NewFitInterpolant
// runs its dedup pass, chord accumulation and tridiagonal solve, not after: a
// record past the ceiling must refuse allocating on the order of its own Fit
// slice, never on the order of the interpolant it would have solved.
func TestFitInterpolantChargeRefusesBeforeSolving(t *testing.T) {
	const points = 20000 // 64*20000 = 1,280,000 > 2^20
	fit := make([]Point2, points)
	for i := range fit {
		fit[i] = Point2{U: float64(i), V: float64(i % 7)}
	}
	seg := FitSplineSeg{Fit: fit, TStart: 0, TEnd: 1}

	var err error
	start := time.Now()
	allocated := allocatedByFit(func() { _, err = fitSplineBezierSpans(seg, &freeformWork{}) })
	elapsed := time.Since(start)

	require.Error(t, err)
	require.ErrorIs(t, err, ErrUnsupported)
	require.Contains(t, err.Error(), "work budget")
	// fit itself is already allocated (16 bytes/point) before the call starts;
	// a refusal that solved nothing allocates on that order, not on the many
	// float64 slices (Params/Points/SecondDerivs/Thomas intermediates) a real
	// solve over 20,000 points would need.
	require.Less(t, allocated, uint64(points)*16)
	require.Less(t, elapsed, 2*time.Second)
}

// fitInterpolantCost is linear (no quadratic term): unlike an open spline's
// knot insertion, a natural cubic gives one span per interval directly.
func TestFitInterpolantCostIsLinear(t *testing.T) {
	require.Equal(t, uint64(0), fitInterpolantCost(0))
	require.Equal(t, uint64(64*10), fitInterpolantCost(10))
	require.Equal(t, uint64(64*1000), fitInterpolantCost(1000))
	// Doubling n exactly doubles the charge — the linear-not-quadratic shape.
	require.Equal(t, 2*fitInterpolantCost(500), fitInterpolantCost(1000))
}

// TestFitInterpolantNonFiniteMapsToR16 pins the sentinel choice directly on
// the conversion: an interpolant sketch's own solve declines to describe
// (chord parameter or span coefficients running off float64) is
// ErrUnsupported and mentions the float64 range, never ErrNotFinite — every
// input coordinate here is finite (Table R row R16).
func TestFitInterpolantNonFiniteMapsToR16(t *testing.T) {
	seg := FitSplineSeg{
		Fit:    []Point2{{U: -1e308, V: 0}, {U: 1e308, V: 1}},
		TStart: 0, TEnd: 1,
	}
	_, err := fitSplineBezierSpans(seg, &freeformWork{})
	require.Error(t, err)
	require.ErrorIs(t, err, ErrUnsupported)
	require.NotErrorIs(t, err, ErrNotFinite)
	require.Contains(t, err.Error(), "float64 range")
}

// A record whose Fit holds fewer than 2 points is an O(1) size refusal ahead
// of any content read — the caller-built / decoded-record guard behind
// record.go's own >= 2 floor.
func TestFitSplineTooFewPointsRefuses(t *testing.T) {
	seg := FitSplineSeg{Fit: []Point2{{U: 1}}, TStart: 0, TEnd: 1}
	_, err := fitSplineBezierSpans(seg, &freeformWork{})
	require.Error(t, err)
	require.ErrorIs(t, err, ErrDegenerate)
	require.Contains(t, err.Error(), "at least 2 fit points")
}

// An all-coincident fit set collapses to ONE active point and zero spans —
// Table R row R14 is decided by the caller (freeformEndpoints /
// freeformDegenerate), not here, but this pins that the conversion itself
// returns the empty chain rather than erroring on its own.
func TestFitSplineAllCoincidentReturnsNoSpans(t *testing.T) {
	seg := FitSplineSeg{
		Fit:    []Point2{{U: 3, V: 4}, {U: 3, V: 4}, {U: 3, V: 4}},
		TStart: 0, TEnd: 1,
	}
	spans, err := fitSplineBezierSpans(seg, &freeformWork{})
	require.NoError(t, err)
	require.Empty(t, spans)
}

// A trimmed fit-spline range refuses through requireFullFreeformRange, the
// same "full domain" cause every other Tier A kind reports for a trimmed
// range (spline design §2) — never the interpolant conversion's own reason.
func TestFitSplineTrimmedRangeRefusesAtFullDomainGate(t *testing.T) {
	seg := FitSplineSeg{
		Fit:    []Point2{{U: 0}, {U: 1, V: 1}, {U: 2}},
		TStart: 0.25, TEnd: 0.75,
	}
	_, err := fitSplineBezierSpans(seg, &freeformWork{})
	require.Error(t, err)
	require.ErrorIs(t, err, ErrUnsupported)
	require.Contains(t, err.Error(), "full domain")
}
