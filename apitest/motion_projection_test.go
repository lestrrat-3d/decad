package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file holds the tests of docs/motion-check-design.md §5.2's projection
// bound: VerifyMotion reads its motion as one joint and takes the larger of
// the travel bound and the projection bound over every interval.

// requireMotionIntervalsBelow asserts every IntervalClear interval's
// Clearance sits at or below gap at the interval's far end, the true minimum
// over it for a gap that falls along the whole path.
func requireMotionIntervalsBelow(t *testing.T, report *decad.MotionReport, gap func(at float64) float64) {
	t.Helper()
	certified := 0
	for _, iv := range report.Intervals {
		if iv.Outcome != decad.IntervalClear {
			continue
		}
		certified++
		require.NotNil(t, iv.Clearance)
		require.LessOrEqual(t, iv.Clearance.Value.Mag(), gap(iv.To.Mag())+1e-9, "[%v, %v]", iv.From.Mag(), iv.To.Mag())
	}
	require.Positive(t, certified)
}

// TestVerifyMotionProjectionAlongAWall: a 10 mm cube slides 0 → 100 mm along
// +X under a wall whose face is 10 mm above it, so the gap is 10 mm at every
// parameter. The travel bound charges the whole slide and certifies only
// intervals shorter than 20 mm; the projection bound's segment term charges
// the cube's motion along the wall's normal, which is nothing, so the one
// interval of the path certifies at the endpoints with the gap itself. Assert
// Sound with two poses, the interval's Clearance within 1e-9 of 10 and not
// above it, and the reading enclosing 10 inside the gate.
//
// Leg seen to fail when deleted: the projection bound (the path is bisected
// to the reading floor).
func TestVerifyMotionProjectionAlongAWall(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	cube := boxBody(t, doc, 0, 0, 10, 10, 10)
	boxBodyAtZ(t, doc, -10, 20, 120, 30, -5, 20)
	report := verifyMotion(t, doc, []*decad.Body{cube}, alongX(100))
	require.Equal(t, decad.Sound, report.Status)
	require.Empty(t, report.Diagnostics)
	require.Len(t, report.Poses, 2)
	require.Len(t, report.Intervals, 1)
	iv := report.Intervals[0]
	require.Equal(t, decad.IntervalClear, iv.Outcome)
	require.LessOrEqual(t, iv.Clearance.Value.Mag(), 10.0)
	require.InDelta(t, 10, iv.Clearance.Value.Mag(), 1e-9)
	require.NotNil(t, report.Clearance)
	require.Equal(t, decad.ToleranceSatisfied, report.Clearance.Tolerance.State)
	require.LessOrEqual(t, report.Clearance.Value.Mag()-report.Clearance.Bound.Mag(), 10.0)
	require.GreaterOrEqual(t, report.Clearance.Value.Mag()+report.Clearance.Bound.Mag(), 10.0)
}

// pendulumScene is docs/linkage-check-design.md §5.8's scene 13 under
// VerifyMotion: a block x ∈ [−5, 5], y ∈ [−50, −40], z ∈ [0, 10] swinging
// about Z through the origin toward a wall y ∈ [20, 40] above the pivot.
func pendulumScene(t *testing.T) (*decad.Document, *decad.Body) {
	t.Helper()
	doc := decad.New()
	block := boxBodyAtZ(t, doc, -5, -50, 5, -40, 0, 10)
	boxBodyAtZ(t, doc, -100, 20, 100, 40, -10, 30)
	return doc, block
}

// pendulumGap is the gap g(θ) = 20 − 5·sin θ + 40·cos θ under the block's
// highest corner (5, −40); it falls from 60 to 15 over 0 → 90°.
func pendulumGap(th float64) float64 { return 20 - 5*math.Sin(th) + 40*math.Cos(th) }

// TestVerifyMotionProjectionPendulum is the remainder's test: the pendulum
// swung 0° → 90° as a Revolute and as a Between to the quarter turn, each at
// a resolution of a sixteenth of its path. The gap is concave over the
// swing, so the expansion from an interval's near end without its remainder
// exceeds the minimum at its far end by about ½·|g”|·Δθ², with |g”| ≤ 40;
// ρ_max = √(50² + 5²) ≈ 50.25 per radian², or θ² times it per unit fraction
// for the Between, covers it. Assert every IntervalClear interval's Clearance
// at or below g at its far end.
//
// Legs seen to fail when deleted: the remainder (both); the factor θ² in the
// Between's remainder; and the factor θ in the Between's velocity, which then
// reads per radian rather than per unit fraction.
func TestVerifyMotionProjectionPendulum(t *testing.T) {
	t.Parallel()
	t.Run("revolute", func(t *testing.T) {
		t.Parallel()
		doc, block := pendulumScene(t)
		swing := decad.Revolute{Axis: r3.NewVec(0, 0, 1), From: units.Degrees(0), To: units.Degrees(90)}
		report := verifyMotion(t, doc, []*decad.Body{block}, swing, decad.WithResolution(units.Degrees(90.0/16)))
		requireMotionIntervalsBelow(t, report, func(at float64) float64 { return pendulumGap(at * math.Pi / 180) })
	})
	t.Run("between", func(t *testing.T) {
		t.Parallel()
		doc, block := pendulumScene(t)
		to, err := r3.RotationAround(r3.Vec{}, r3.NewVec(0, 0, 1), units.Degrees(90))
		require.NoError(t, err)
		report := verifyMotion(t, doc, []*decad.Body{block}, decad.Between{From: r3.Identity(), To: to}, fraction(1.0/16))
		requireMotionIntervalsBelow(t, report, func(at float64) float64 { return pendulumGap(at * math.Pi / 2) })
	})
}

// TestVerifyMotionProjectionScrewSlide is the Between's slide term: the cube
// turned 30° about the line through (5, ·, 5) along +Y while it slides 12 mm
// along it, toward a wall whose face is 15 mm above it. A turn about an axis
// along Y moves no point along Y, so the gap is 15 − 12·s at every s, and its
// minimum over an interval is at its far end. Assert every IntervalClear
// interval's Clearance at or below it, and the reading enclosing 3.
//
// Leg seen to fail when deleted: the slide term d·k of the Between's
// velocity (the expansion then holds the cube at its height, and an interval
// reads the gap at its near end).
func TestVerifyMotionProjectionScrewSlide(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	cube := boxBody(t, doc, 0, 0, 10, 10, 10)
	boxBodyAtZ(t, doc, -10, 25, 20, 35, -5, 20)
	turn, err := r3.RotationAround(r3.NewVec(5, 0, 5), r3.NewVec(0, 1, 0), units.Degrees(30))
	require.NoError(t, err)
	slide, err := r3.Translation(r3.NewVec(0, 12, 0))
	require.NoError(t, err)
	to, err := turn.Then(slide)
	require.NoError(t, err)
	report := verifyMotion(t, doc, []*decad.Body{cube}, decad.Between{From: r3.Identity(), To: to})
	requireMotionIntervalsBelow(t, report, func(s float64) float64 { return 15 - 12*s })
	require.NotNil(t, report.Clearance)
	require.LessOrEqual(t, report.Clearance.Value.Mag()-report.Clearance.Bound.Mag(), 3.0)
	require.GreaterOrEqual(t, report.Clearance.Value.Mag()+report.Clearance.Bound.Mag(), 3.0)
}
