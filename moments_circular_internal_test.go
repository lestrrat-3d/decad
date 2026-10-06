package decad

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"

	"github.com/lestrrat-3d/units"
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
	r2 := proofbound.RatAdd(proofbound.RatMul(dx0, dx0), proofbound.RatMul(dy0, dy0))
	endR2 := proofbound.RatAdd(proofbound.RatMul(dx1, dx1), proofbound.RatMul(dy1, dy1))
	require.NotEqual(t, 0, endR2.Cmp(r2), `the fixture's two exact squared radii must differ`)
	q := new(big.Rat).Quo(r2, endR2)
	require.NotEqual(t, proofbound.RatSqrtDown(q), proofbound.RatSqrtUp(q), `the radial ratio's bracket must have width`)
	return seg, r2, endR2
}

func requireIntervalWidthAtMost(t *testing.T, name string, iv proofbound.RatInterval, ceiling float64) {
	t.Helper()
	width, _ := new(big.Rat).Sub(iv.Hi, iv.Lo).Float64()
	require.GreaterOrEqual(t, width, 0.0, "%s: an interval's hi must not sit below its lo", name)
	require.LessOrEqual(t, width, ceiling, "%s: interval width %g exceeds %g", name, width, ceiling)
}

func requireIntervalNegates(t *testing.T, name string, fwd, rev proofbound.RatInterval) {
	t.Helper()
	require.Zero(t, rev.Lo.Cmp(new(big.Rat).Neg(fwd.Hi)), "%s: reversed lo must be the forward hi negated", name)
	require.Zero(t, rev.Hi.Cmp(new(big.Rat).Neg(fwd.Lo)), "%s: reversed hi must be the forward lo negated", name)
}

func requirePointInterval(t *testing.T, name string, iv proofbound.RatInterval) {
	t.Helper()
	require.Zero(t, iv.Lo.Cmp(iv.Hi), "%s: expected a point interval, got [%s, %s]",
		name, iv.Lo.FloatString(20), iv.Hi.FloatString(20))
}

// Shown-to-fail: swapping the proofbound.RatSqrtDown/proofbound.RatSqrtUp calls in
// arcEndRadialRatio inverts the bracket, and the containment leg goes red.
func TestArcEndRadialRatioBracketsTheRatio(t *testing.T) {
	t.Parallel()
	_, r2, endR2 := driftedArcFixture(t)
	q := new(big.Rat).Quo(r2, endR2)

	rho, ok := arcEndRadialRatio(r2, endR2)
	require.True(t, ok)
	require.LessOrEqual(t, new(big.Rat).Mul(rho.Lo, rho.Lo).Cmp(q), 0, `lo² must not exceed r²/endR²`)
	require.GreaterOrEqual(t, new(big.Rat).Mul(rho.Hi, rho.Hi).Cmp(q), 0, `hi² must not fall below r²/endR²`)
	requireIntervalWidthAtMost(t, "rho", rho, math.Ldexp(1, -48))

	equal, ok := arcEndRadialRatio(r2, r2)
	require.True(t, ok)
	require.Zero(t, equal.Lo.Cmp(big.NewRat(1, 1)), `equal radii give exactly 1`)
	require.Zero(t, equal.Hi.Cmp(big.NewRat(1, 1)), `equal radii give exactly 1`)

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
// Shown-to-fail: replacing the equal-radii fast path's proofbound.PointInterval(1) in
// arcEndRadialRatio by proofbound.Interval(1, 1 + 2^-52) widens mu, mv and muv, and each
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

func requireIntervalsOverlap(t *testing.T, name string, a, b proofbound.RatInterval) {
	t.Helper()
	require.LessOrEqual(t, a.Lo.Cmp(b.Hi), 0, "%s: [%s, %s] lies above [%s, %s]", name,
		a.Lo.FloatString(20), a.Hi.FloatString(20), b.Lo.FloatString(20), b.Hi.FloatString(20))
	require.LessOrEqual(t, b.Lo.Cmp(a.Hi), 0, "%s: [%s, %s] lies below [%s, %s]", name,
		a.Lo.FloatString(20), a.Hi.FloatString(20), b.Lo.FloatString(20), b.Hi.FloatString(20))
}

// TestCircularMonomialsAgreeWithClosedForms cross-checks circularMonomials'
// generic trig-power reduction against the hand-expanded first- and
// second-moment enclosures, over the forms both evaluate the same way
// (∫u dA = ½∮u²dv, ∫u² dA = ⅓∮u³dv, ∫uv dA = ½∮u²v dv): a fractional
// CircleSeg, a whole one, and the drifted ArcSeg walked both ways. Two sound
// enclosures of one value must overlap.
//
// Shown-to-fail: dropping the a ≥ 2 reduction's (a−1)·r² lower term
// separates every case. These forms never reach the b ≥ 2 reduction;
// TestThirdOrderMomentsOfASector covers it.
func TestCircularMonomialsAgreeWithClosedForms(t *testing.T) {
	t.Parallel()
	drifted, _, _ := driftedArcFixture(t)
	reversed := drifted
	reversed.TStart, reversed.TEnd = 1, 0
	for name, seg := range map[string]CurveSegment{
		"fractional circle": CircleSeg{Center: Point2{U: 5, V: -3}, Radius: units.Millimeters(2),
			TStart: 0.125, TEnd: 0.4375, CCW: true},
		"whole circle": CircleSeg{Center: Point2{U: 5, V: -3}, Radius: units.Millimeters(2),
			TStart: 0, TEnd: 1, CCW: true},
		"forward arc": drifted,
		"reverse arc": reversed,
	} {
		t.Run(name, func(t *testing.T) {
			t.Parallel()
			walk, ok := circularMomentWalkOf(seg)
			require.True(t, ok)
			j := circularMonomials(walk, 5)
			mu, _, ok := circularFirstMomentInterval(seg, Point2{})
			require.True(t, ok)
			muu, muv, _, ok := circularSecondMomentInterval(seg, Point2{})
			require.True(t, ok)
			for _, check := range []struct {
				name string
				p, q int
				want proofbound.RatInterval
			}{{"mu", 1, 0, mu}, {"muu", 2, 0, muu}, {"muv", 1, 1, muv}} {
				got := circularGreenMoment(walk, j, check.p, check.q)
				requireIntervalsOverlap(t, check.name, got, check.want)
				requireIntervalWidthAtMost(t, check.name, got, 1e-9)
			}
		})
	}
}

// TestCircularThirdMomentWholeCircle checks a whole CCW circle's third-order
// contributions against the disc's own: about its centre ∫x² dA = πr⁴/4 and
// every odd moment vanishes, so ∫u³ = π(cU³r² + 3cU·r⁴/4),
// ∫u²v = π(cU²cV·r² + cV·r⁴/4), ∫uv² = π(cU·cV²·r² + cU·r⁴/4) and
// ∫v³ = π(cV³r² + 3cV·r⁴/4). The turn starting at 1/8 has endpoint sine and
// cosine enclosures of series width; the whole-turn closure is what keeps
// them out of the result, leaving only 2π's enclosure.
//
// Shown-to-fail: always clearing circularMomentWalk.closed widens the
// 1/8-turn walk's intervals past the width ceiling; dropping J(0,0)'s sweep
// separates all four intervals from the closed form.
func TestCircularThirdMomentWholeCircle(t *testing.T) {
	t.Parallel()
	cU, cV := big.NewRat(3, 1), big.NewRat(-2, 1)
	r2, r4 := big.NewRat(4, 1), big.NewRat(16, 1)
	quarterR4 := ratScale(r4, 1, 4)
	pi := proofbound.Interval(proofbound.PiLower, proofbound.PiUpper)
	want := [4]*big.Rat{
		proofbound.RatAdd(proofbound.RatMul(cU, cU, cU, r2), proofbound.RatMul(big.NewRat(3, 1), cU, quarterR4)),
		proofbound.RatAdd(proofbound.RatMul(cU, cU, cV, r2), proofbound.RatMul(cV, quarterR4)),
		proofbound.RatAdd(proofbound.RatMul(cU, cV, cV, r2), proofbound.RatMul(cU, quarterR4)),
		proofbound.RatAdd(proofbound.RatMul(cV, cV, cV, r2), proofbound.RatMul(big.NewRat(3, 1), cV, quarterR4)),
	}
	for _, start := range []float64{0, 0.125} {
		seg := CircleSeg{Center: Point2{U: 3, V: -2}, Radius: units.Millimeters(2),
			TStart: start, TEnd: start + 1, CCW: true}
		got, ok := circularThirdMomentInterval(seg)
		require.True(t, ok)
		for i, coefficient := range want {
			name := []string{"u³", "u²v", "uv²", "v³"}[i]
			requireIntervalsOverlap(t, name, got[i], proofbound.IntervalScale(pi, coefficient))
			requireIntervalWidthAtMost(t, name, got[i], 1e-65)
		}
	}

	trimmed, _, _ := driftedArcFixture(t)
	trimmed.TEnd = 0.5
	_, ok := circularThirdMomentInterval(trimmed)
	require.False(t, ok, `a trimmed ArcSeg fragment has no third-order enclosure`)
}
