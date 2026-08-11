package decad

import (
	"math"
	"testing"

	"github.com/stretchr/testify/require"
)

// TestLineCircleLocusSpeedUpperRefusesMomentaryFold pins fu144's own refusal
// path for a line meeting a circle: lineCircleLocusSpeedUpper answers false
// when the corner's own discriminant Δ(t) is zero AT the corner (t = 0, a
// genuine tangency there) but its own slope Δ1 is NOT — a momentary fold,
// where the two offset carriers touch only at the corner itself and then
// immediately stop meeting at all as the offset grows, so no real locus
// exists to bound a speed over. This is the "Δ0 = 0 but Δ1 != 0" case
// lineCircleLocusSpeedUpper's own doc comment sets apart from the tangent
// join's PERSISTENT Δ ≡ 0, which returns a finite answer instead
// (TestLineCircleLocusSpeedUpperExactAtPersistentTangency below).
//
// The public Chamfer path never reaches this: every tangent join this
// evaluator itself builds (a Fillet corner, fillet.go) lands on the
// persistent branch, bit for bit, and a corner recorded through sketch's own
// solver is never quite so exactly tangent that it can hit this one narrow
// non-persistent case — the same reason capblend_normal_internal_test.go
// covers its own certified enclosures directly rather than hunting for a
// public fixture that reaches them.
//
// The construction: a straight wall along the U axis (material-side normal
// (0, 1)) meets a circular wall centred at (0, -R), radius R, so the corner
// (0, 0) is the circle's own point of tangency with the line. Offsetting the
// line INTO material moves it to y = t (anchor + t·n), which moves AWAY from
// the centre; offsetting this circle with th1 > th0 shrinks its own radius
// to R - t. A line moving away from a shrinking circle's centre separates
// from it immediately once t > 0, so Δ0 = 0 but Δ1 = -4R != 0.
func TestLineCircleLocusSpeedUpperRefusesMomentaryFold(t *testing.T) {
	const radius = 1000.0
	line := sideWalk{segmentWalk: segmentWalk{
		startU: 0, startV: 0, endU: 10, endV: 0,
		tanInU: 1, tanInV: 0, tanOutU: 1, tanOutV: 0,
	}}
	circle := sideWalk{segmentWalk: segmentWalk{
		kind: walkCircular,
		cU:   0, cV: -radius, radius: radius,
		th0: math.Pi / 2, th1: math.Pi/2 + 0.5,
	}}

	for _, d := range []float64{0.001, 0.1, 1, 3} {
		speed, ok := lineCircleLocusSpeedUpper(line, circle, 0, d)
		require.False(t, ok, "d=%v: a momentary fold must refuse, not publish a speed", d)
		require.Zero(t, speed)
	}
}

// TestLineCircleLocusSpeedUpperExactAtPersistentTangency pins the closed
// form's other branch: a line permanently tangent to a circle under a
// consistent offset (every Fillet-built corner in this codebase) publishes
// the EXACT unit speed 1, not a refusal and not circleCircleLocusSpeedUpper's
// own looser bound.
//
// The construction mirrors the refusal case above but offsets the circle the
// OTHER way (th1 < th0, growing radius R + t): a line moving away from the
// centre at rate 1 then matches the circle's own radius growing at rate 1,
// so the two stay tangent — at (0, t) — for every offset amount, and the
// locus is that same straight line, ridden at unit speed.
func TestLineCircleLocusSpeedUpperExactAtPersistentTangency(t *testing.T) {
	const radius = 1000.0
	line := sideWalk{segmentWalk: segmentWalk{
		startU: 0, startV: 0, endU: 10, endV: 0,
		tanInU: 1, tanInV: 0, tanOutU: 1, tanOutV: 0,
	}}
	circle := sideWalk{segmentWalk: segmentWalk{
		kind: walkCircular,
		cU:   0, cV: -radius, radius: radius,
		th0: math.Pi/2 + 0.5, th1: math.Pi / 2, // th1 < th0: material outside, radius grows
	}}

	for _, d := range []float64{0.001, 0.1, 1, 3} {
		speed, ok := lineCircleLocusSpeedUpper(line, circle, 0, d)
		require.True(t, ok, "d=%v: a persistent tangency must publish a finite speed", d)
		require.InDelta(t, 1.0, speed, 1e-9)
	}
}

// TestCircleCircleLocusSpeedUpperRefusesNearParallelCorner covers
// circleCircleLocusSpeedUpper's own decorrelated Cramer solve directly,
// since no public two-circular-wall miter fixture reaches its refusal
// either (circleCircleLocusSpeedUpper's own doc comment states why: it
// cannot tell a persistent circle-circle tangency from a momentary one, and
// refuses on both).
//
// The construction: a circular wall centred at the origin meets a second
// circular wall centred twice its radius further down the same axis, so the
// two circles are EXTERNALLY tangent at (0, -radius) — the shared corner —
// and their own radial directions there are anti-parallel (still a zero
// cross product, the same singular configuration a parallel pair is).
func TestCircleCircleLocusSpeedUpperRefusesNearParallelCorner(t *testing.T) {
	const radius = 1000.0
	prev := sideWalk{segmentWalk: segmentWalk{
		kind: walkCircular,
		cU:   0, cV: 0, radius: radius,
		th0: -math.Pi / 2, th1: -math.Pi / 4,
	}}
	cur := sideWalk{segmentWalk: segmentWalk{
		kind: walkCircular,
		cU:   0, cV: -2 * radius, radius: radius,
		th0: math.Pi / 2, th1: math.Pi/2 + 0.3,
	}}

	for _, d := range []float64{0.001, 0.1, 1, 3} {
		speed, ok := circleCircleLocusSpeedUpper(prev, cur, 0, d, 0, -radius)
		require.False(t, ok, "d=%v: a near-parallel corner must refuse, not publish a speed", d)
		require.Zero(t, speed)
	}
}
