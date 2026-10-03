package decad

import (
	"math"
	"math/big"
	"testing"

	"github.com/stretchr/testify/require"
)

// driftedArcFixture is an ArcSeg whose End sits one ulp of its radius off
// Start's circle: Start is on the unit circle about (5, −3), End on the
// circle of radius nextafter(1, +Inf) at angle atan2(0.8, 0.6). The exact
// squared radii differ, so every circular bracket has to read End's radial
// ratio rather than trust it to be 1.
func driftedArcFixture(t *testing.T) (ArcSeg, *big.Rat, *big.Rat) {
	t.Helper()
	d := math.Nextafter(1, math.Inf(1))
	seg := ArcSeg{
		Center: Point2{U: 5, V: -3},
		Start:  Point2{U: 6, V: -3},
		End:    Point2{U: 5 + 0.6*d, V: -3 + 0.8*d},
		TStart: 0,
		TEnd:   1,
	}
	dx0 := exactCoordinateDelta(seg.Start.U, seg.Center.U)
	dy0 := exactCoordinateDelta(seg.Start.V, seg.Center.V)
	dx1 := exactCoordinateDelta(seg.End.U, seg.Center.U)
	dy1 := exactCoordinateDelta(seg.End.V, seg.Center.V)
	r2 := ratAdd(ratMul(dx0, dx0), ratMul(dy0, dy0))
	endR2 := ratAdd(ratMul(dx1, dx1), ratMul(dy1, dy1))
	require.NotEqual(t, 0, endR2.Cmp(r2), `the fixture's two exact squared radii must differ`)
	q := new(big.Rat).Quo(r2, endR2)
	require.NotEqual(t, ratSqrtDown(q), ratSqrtUp(q), `the radial ratio's bracket must have width`)
	return seg, r2, endR2
}

func requireIntervalWidthAtMost(t *testing.T, name string, iv ratInterval, ceiling float64) {
	t.Helper()
	width, _ := new(big.Rat).Sub(iv.hi, iv.lo).Float64()
	require.GreaterOrEqual(t, width, 0.0, "%s: an interval's hi must not sit below its lo", name)
	require.LessOrEqual(t, width, ceiling, "%s: interval width %g exceeds %g", name, width, ceiling)
}

func requireIntervalNegates(t *testing.T, name string, fwd, rev ratInterval) {
	t.Helper()
	require.Zero(t, rev.lo.Cmp(new(big.Rat).Neg(fwd.hi)), "%s: reversed lo must be the forward hi negated", name)
	require.Zero(t, rev.hi.Cmp(new(big.Rat).Neg(fwd.lo)), "%s: reversed hi must be the forward lo negated", name)
}

func requirePointInterval(t *testing.T, name string, iv ratInterval) {
	t.Helper()
	require.Zero(t, iv.lo.Cmp(iv.hi), "%s: expected a point interval, got [%s, %s]",
		name, iv.lo.FloatString(20), iv.hi.FloatString(20))
}

// Shown-to-fail: swapping the ratSqrtDown/ratSqrtUp calls in
// arcEndRadialRatio inverts the bracket, and the containment leg goes red.
func TestArcEndRadialRatioBracketsTheRatio(t *testing.T) {
	t.Parallel()
	_, r2, endR2 := driftedArcFixture(t)
	q := new(big.Rat).Quo(r2, endR2)

	rho, ok := arcEndRadialRatio(r2, endR2)
	require.True(t, ok)
	require.LessOrEqual(t, new(big.Rat).Mul(rho.lo, rho.lo).Cmp(q), 0, `lo² must not exceed r²/endR²`)
	require.GreaterOrEqual(t, new(big.Rat).Mul(rho.hi, rho.hi).Cmp(q), 0, `hi² must not fall below r²/endR²`)
	requireIntervalWidthAtMost(t, "rho", rho, math.Ldexp(1, -48))

	equal, ok := arcEndRadialRatio(r2, r2)
	require.True(t, ok)
	require.Zero(t, equal.lo.Cmp(big.NewRat(1, 1)), `equal radii give exactly 1`)
	require.Zero(t, equal.hi.Cmp(big.NewRat(1, 1)), `equal radii give exactly 1`)

	_, ok = arcEndRadialRatio(r2, new(big.Rat))
	require.False(t, ok, `an End on the Center has no radial ratio`)
}

// Shown-to-fail: restoring `if endR2.Cmp(r2) != 0 { return …, false }` in
// circularFirstMomentInterval's ArcSeg arm answers ok == false for the drifted
// fixture, and the first leg goes red.
func TestCircularFirstMomentIntervalChargesEndpointRadiusDrift(t *testing.T) {
	t.Parallel()
	seg, _, _ := driftedArcFixture(t)
	anchor := Point2{}

	mu, mv, ok := circularFirstMomentInterval(seg, anchor)
	require.True(t, ok, `a drifted End is charged into the bracket, not refused`)
	requireIntervalWidthAtMost(t, "mu", mu, 1e-12)
	requireIntervalWidthAtMost(t, "mv", mv, 1e-12)

	reversed := seg
	reversed.TStart, reversed.TEnd = 1, 0
	revMU, revMV, ok := circularFirstMomentInterval(reversed, anchor)
	require.True(t, ok)
	requireIntervalNegates(t, "mu", mu, revMU)
	requireIntervalNegates(t, "mv", mv, revMV)
}

// Shown-to-fail: restoring `if endR2.Cmp(r2) != 0 { return …, false }` in
// circularSecondMomentInterval's ArcSeg arm answers ok == false for the
// drifted fixture, and the first leg goes red.
func TestCircularSecondMomentIntervalChargesEndpointRadiusDrift(t *testing.T) {
	t.Parallel()
	seg, _, _ := driftedArcFixture(t)
	anchor := Point2{}

	muu, muv, mvv, ok := circularSecondMomentInterval(seg, anchor)
	require.True(t, ok, `a drifted End is charged into the bracket, not refused`)
	requireIntervalWidthAtMost(t, "muu", muu, 1e-12)
	requireIntervalWidthAtMost(t, "muv", muv, 1e-12)
	requireIntervalWidthAtMost(t, "mvv", mvv, 1e-12)

	reversed := seg
	reversed.TStart, reversed.TEnd = 1, 0
	revMUU, revMUV, revMVV, ok := circularSecondMomentInterval(reversed, anchor)
	require.True(t, ok)
	requireIntervalNegates(t, "muu", muu, revMUU)
	requireIntervalNegates(t, "muv", muv, revMUV)
	requireIntervalNegates(t, "mvv", mvv, revMVV)
}

// With the centre on the anchor every swept-angle coefficient of mu, mv and
// muv is zero and every other term is rational, so any width these intervals
// carry could only come from End's radial ratio. Equal radii (End = (12, 16)
// on the radius-20 circle, both of its deltas nonzero so every leg reads ρ)
// must keep it a point.
//
// Shown-to-fail: replacing the equal-radii fast path's pointInterval(1) in
// arcEndRadialRatio by interval(1, 1 + 2^-52) widens mu, mv and muv, and each
// point-interval leg goes red.
func TestCircularMomentIntervalsExactArcKeepPointRadialRatio(t *testing.T) {
	t.Parallel()
	seg := ArcSeg{
		Center: Point2{U: 0, V: 0},
		Start:  Point2{U: 20, V: 0},
		End:    Point2{U: 12, V: 16},
		TStart: 0,
		TEnd:   1,
	}
	anchor := Point2{}

	mu, mv, ok := circularFirstMomentInterval(seg, anchor)
	require.True(t, ok)
	requirePointInterval(t, "mu", mu)
	requirePointInterval(t, "mv", mv)

	_, muv, _, ok := circularSecondMomentInterval(seg, anchor)
	require.True(t, ok)
	requirePointInterval(t, "muv", muv)
}
