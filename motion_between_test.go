package decad_test

import (
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file holds the Between tests of docs/motion-check-design.md §9,
// numbered 12-20 as §9 numbers them; test 21 is examples/'s
// Example_decad_motionBetween, and the internal tests sit in
// motion_internal_test.go. Bounds are asserted against geometric truths or as
// small, never pinned to a measured literal: FMA contraction moves the last
// ulp between amd64 and arm64.
//
// The screw arm of tests 12, 13, 17 and 20 is the arm of tests 1-2 under a
// quarter turn about Z with a 20 mm rise. The rise leaves every y unchanged,
// so every angle tests 1 and 2 work out holds at the fraction s = θ/(π/2), and
// the walls are raised to z ∈ [−10, 40] to reach past the arm's caps at every
// height the rise visits.

// transformOK unwraps an r3 constructor's result, failing the test on error.
func transformOK(t *testing.T) func(r3.Transform, error) r3.Transform {
	return func(tr r3.Transform, err error) r3.Transform {
		t.Helper()
		require.NoError(t, err)
		return tr
	}
}

func quarterTurnZ(t *testing.T, center r3.Vec) r3.Transform {
	t.Helper()
	return transformOK(t)(r3.RotationAround(center, r3.NewVec(0, 0, 1), units.Degrees(90)))
}

func shiftBy(t *testing.T, v r3.Vec) r3.Transform {
	t.Helper()
	return transformOK(t)(r3.Translation(v))
}

func then(t *testing.T, a, b r3.Transform) r3.Transform {
	t.Helper()
	return transformOK(t)(a.Then(b))
}

// screwTo is the quarter turn about Z through the origin followed by a rise
// of dz along Z.
func screwTo(t *testing.T, dz float64) r3.Transform {
	t.Helper()
	return then(t, quarterTurnZ(t, r3.Vec{}), shiftBy(t, r3.NewVec(0, 0, dz)))
}

func screwArm(t *testing.T) decad.Between {
	t.Helper()
	return decad.Between{From: r3.Identity(), To: screwTo(t, 20)}
}

func fraction(f float64) decad.MotionOption {
	return decad.WithResolution(units.Scalar(f))
}

// requireOnsetAbove asserts §9 tests 12's and 18's onset claims: every
// collision sits strictly past the contact parameter star, every certified
// interval ends at or before it, the interval holding it is never clear, the
// first collision lies within two grid steps of 1/256 above it, and every
// published volume is a proven positive lower bound.
func requireOnsetAbove(t *testing.T, report *decad.MotionReport, star float64) {
	t.Helper()
	require.Equal(t, decad.Interfering, report.Status)
	require.NotEmpty(t, report.Collisions)
	for _, c := range report.Collisions {
		require.Equal(t, units.Dimensionless, c.At.Kind())
		require.Greater(t, c.At.Mag(), star)
		require.Greater(t, c.Volume.Value.Mag(), 0.0)
		require.Less(t, c.Volume.Bound.Mag(), c.Volume.Value.Mag())
	}
	require.LessOrEqual(t, report.Collisions[0].At.Mag()-star, 2.0/256, `the onset is bracketed to the resolution`)
	for _, iv := range report.Intervals {
		if iv.Outcome == decad.IntervalClear {
			require.LessOrEqual(t, iv.To.Mag(), star)
		}
		if iv.From.Mag() <= star && star <= iv.To.Mag() {
			require.NotEqual(t, decad.IntervalClear, iv.Outcome, `the interval holding the onset is never clear`)
		}
	}
}

// TestBetweenPoseAt checks the screw path's poses and PoseAt's own refusals
// (§2): the ends are From and To as stated, the midpoint of the screw arm is
// the eighth turn half-risen, and a wrong-kind or non-finite fraction is
// refused. Two reflections are a legal pair.
func TestBetweenPoseAt(t *testing.T) {
	t.Parallel()
	m := screwArm(t)
	start, err := m.PoseAt(units.Scalar(0))
	require.NoError(t, err)
	require.Equal(t, m.From, start)
	end, err := m.PoseAt(units.Scalar(1))
	require.NoError(t, err)
	require.Equal(t, m.To, end)
	mid, err := m.PoseAt(units.Scalar(0.5))
	require.NoError(t, err)
	got := mid.Apply(r3.NewVec(10, 0, 0))
	require.InDelta(t, 10*math.Sqrt(0.5), got.X, 1e-12)
	require.InDelta(t, 10*math.Sqrt(0.5), got.Y, 1e-12)
	require.InDelta(t, 10, got.Z, 1e-12)

	mirror, err := r3.NewFrame(r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	reflect := transformOK(t)(r3.Reflection(mirror))
	pair := decad.Between{From: reflect, To: then(t, reflect, shiftBy(t, r3.NewVec(0, 0, 5)))}
	_, err = pair.PoseAt(units.Scalar(0.5))
	require.NoError(t, err, `two reflections are a legal pair`)

	for _, tc := range []struct {
		name string
		at   units.Value
		want error
	}{
		{"an angle", units.Degrees(10), decad.ErrUnitKind},
		{"a length", units.Millimeters(1), decad.ErrUnitKind},
		{"NaN", units.Scalar(math.NaN()), decad.ErrNotFinite},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			_, err := m.PoseAt(tc.at)
			require.ErrorIs(t, err, tc.want)
		})
	}
}

// TestVerifyMotionBetweenKnownCollision is §9 test 12.
//
// Rotation-led: the screw arm against test 1's wall raised. The corner
// (48, 14) reaches y = 40 at θ* = atan(3/4), the fraction
// s* = (2/π)·atan(3/4) ≈ 0.40967. At s = 1 the arm spans x ∈ [−14, 14],
// y ∈ [0, 48], z ∈ [20, 30], overlapping the wall over 28 × 8 × 10 = 2240 mm³.
//
// Slide-led: the arm descending 20 mm onto a floor whose top is z = −15. Its
// underside z = 0 reaches the floor at the slide −15, s = 3/4 exactly, a grid
// point where the two share a face plane and touch without overlapping.
//
// Legs seen to fail when deleted: the slide term s·d·n of the ideal pose
// (both subtests: η grows to |s·d|, 15 mm at the floor, no overlap clears the
// allowance, and no collision is published).
func TestVerifyMotionBetweenKnownCollision(t *testing.T) {
	t.Parallel()
	t.Run("rotation-led", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		arm := motionArm(t, doc)
		wall := boxBodyAtZ(t, doc, -100, 40, 100, 60, -10, 50)
		m := screwArm(t)
		report := verifyMotion(t, doc, []*decad.Body{arm}, m, fraction(1.0/256))

		requireOnsetAbove(t, report, 2/math.Pi*math.Atan(3.0/4.0))
		for _, c := range report.Collisions {
			require.Same(t, arm, c.Moving)
			require.Same(t, wall, c.Static)
		}
		last := report.Poses[len(report.Poses)-1]
		require.Equal(t, units.Scalar(1), last.At)
		require.True(t, last.Pose == m.To, `the s = 1 pose is the stated To`)
		require.Len(t, last.Interferences, 1)
		require.InDelta(t, 2240, last.Interferences[0].Volume.Value.Mag(), 1e-6)
	})
	t.Run("slide-led", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		arm := motionArm(t, doc)
		boxBodyAtZ(t, doc, -100, -100, 100, 100, -40, 25)
		m := decad.Between{From: r3.Identity(), To: screwTo(t, -20)}
		report := verifyMotion(t, doc, []*decad.Body{arm}, m, fraction(1.0/256))
		requireOnsetAbove(t, report, 0.75)
	})
}

// TestVerifyMotionBetweenClearPath is §9 test 13: the screw arm past test 2's
// wall raised. The minimum gap is 10 mm, at s₁ = 1 − (2/π)·atan(7/24); the
// endpoint gaps, 46 mm at s = 0 and 12 mm at s = 1, sum to 58 mm, under the
// whole path's travel 50·π/2 + 20 ≈ 98.54 mm, so the endpoints alone certify
// nothing. A certified interval's Clearance dips below the true gap by up to
// (50·θ + 20)/2 × Δs ≈ 49.27 × Δs mm, so the path reading's half-width is
// about 24.6 × Δs: Δs = 1/4096 gives 6.0e-3 mm, inside the 0.01 mm gate on a
// 10 mm gap at rel = 1e-3, while the default 1/1024 gives 0.024 mm, outside.
//
// Legs seen to fail when deleted: the slide term s·d·n of the ideal pose, and
// θ read as degrees instead of radians (each fails every subtest but the
// exact 10 mm margin: η then exceeds the gaps, so the endpoints-only s = 1
// row's bound is no longer ulp-scale and no bisected interval certifies).
func TestVerifyMotionBetweenClearPath(t *testing.T) {
	t.Parallel()
	run := func(t *testing.T, opts ...decad.MotionOption) *decad.MotionReport {
		t.Helper()
		doc := decad.New()
		arm := motionArm(t, doc)
		boxBodyAtZ(t, doc, -100, 60, 100, 80, -10, 50)
		return verifyMotion(t, doc, []*decad.Body{arm}, screwArm(t), opts...)
	}
	fine := fraction(1.0 / 4096)
	t.Run("endpoints only", func(t *testing.T) {
		t.Parallel()
		report := run(t, fraction(1))
		require.Equal(t, decad.Suspect, report.Status)
		require.Equal(t, units.Scalar(1), report.Request.Resolution)
		require.Empty(t, report.Collisions)
		require.Nil(t, report.Clearance)
		require.Len(t, report.Intervals, 1)
		require.Equal(t, decad.IntervalUndecided, report.Intervals[0].Outcome)
		require.Len(t, report.Poses, 2)
		startGap := report.Poses[0].Clearances[0].Gap
		require.Equal(t, decad.Exact, startGap.Exactness, `From is the identity and the arm unplaced, so η_0 is zero`)
		require.Equal(t, 46.0, startGap.Value.Mag())
		require.Zero(t, startGap.Bound.Mag())
		endGap := report.Poses[1].Clearances[0].Gap
		require.Equal(t, decad.Approximate, endGap.Exactness)
		require.Greater(t, endGap.Bound.Mag(), 0.0)
		require.Less(t, endGap.Bound.Mag(), 1e-9, `η_1 is of enclosure-width scale`)
		require.LessOrEqual(t, endGap.Value.Mag()-endGap.Bound.Mag(), 12.0)
		require.GreaterOrEqual(t, endGap.Value.Mag()+endGap.Bound.Mag(), 12.0)
	})
	t.Run("bisected to the gate", func(t *testing.T) {
		t.Parallel()
		report := run(t, fine)
		require.Equal(t, decad.Sound, report.Status)
		require.Empty(t, report.Diagnostics)
		// Each interval's Clearance bounds the gap over that interval from
		// below, so it exceeds neither endpoint's proven upper end, and the
		// smallest of them cannot exceed the path's true minimum.
		upper := func(p decad.PoseResult) float64 {
			require.Len(t, p.Clearances, 1)
			return p.Clearances[0].Gap.Value.Mag() + p.Clearances[0].Gap.Bound.Mag()
		}
		lowest := math.Inf(1)
		for k, iv := range report.Intervals {
			require.Equal(t, decad.IntervalClear, iv.Outcome)
			require.LessOrEqual(t, iv.Clearance.Value.Mag(), min(upper(report.Poses[k]), upper(report.Poses[k+1])))
			lowest = min(lowest, iv.Clearance.Value.Mag())
		}
		require.LessOrEqual(t, lowest, 10.0, `a proven lower bound never exceeds the true minimum`)
		require.NotNil(t, report.Clearance)
		require.InDelta(t, 10, report.Clearance.Value.Mag(), 0.1)
		require.LessOrEqual(t, report.Clearance.Value.Mag()-report.Clearance.Bound.Mag(), 10.0)
		require.GreaterOrEqual(t, report.Clearance.Value.Mag()+report.Clearance.Bound.Mag(), 10.0)
		require.Equal(t, decad.ToleranceSatisfied, report.Clearance.Tolerance.State)
	})
	t.Run("a 9 mm margin is met", func(t *testing.T) {
		t.Parallel()
		report := run(t, fine, decad.WithMinClearance(units.Millimeters(9)))
		require.Equal(t, decad.AssessmentMet, report.Assessment)
		require.Equal(t, decad.Sound, report.Status)
	})
	t.Run("an 11 mm margin is violated at a proving pose", func(t *testing.T) {
		t.Parallel()
		report := run(t, fine, decad.WithMinClearance(units.Millimeters(11)))
		require.Equal(t, decad.AssessmentViolated, report.Assessment)
		require.Equal(t, decad.Violating, report.Status)
		violations := motionDiagnostics(report, decad.DiagMotionClearanceViolated)
		require.NotEmpty(t, violations)
		for _, d := range violations {
			require.NotNil(t, d.At)
			require.Equal(t, units.Dimensionless, d.At.Kind())
			require.Less(t, d.Observed.Value.Mag()+d.Observed.Bound.Mag(), 11.0)
			found := false
			for _, pose := range report.Poses {
				if pose.At == *d.At {
					found = true
					require.Equal(t, *d.Observed, pose.Clearances[0].Gap, `the diagnostic names the pose of the proving gap`)
				}
			}
			require.True(t, found)
		}
	})
	t.Run("an exact 10 mm margin stays undecided", func(t *testing.T) {
		t.Parallel()
		report := run(t, fine, decad.WithMinClearance(units.Millimeters(10)))
		require.Equal(t, decad.AssessmentUndecided, report.Assessment)
		require.Equal(t, decad.Suspect, report.Status)
	})
	t.Run("the default floor leaves the reading beyond tolerance", func(t *testing.T) {
		t.Parallel()
		report := run(t)
		require.Equal(t, units.Scalar(1.0/1024), report.Request.Resolution)
		for _, iv := range report.Intervals {
			require.Equal(t, decad.IntervalClear, iv.Outcome, `Sound in verdict`)
		}
		require.Equal(t, decad.Suspect, report.Status, `but the reading is coarse`)
		require.NotNil(t, report.Clearance)
		require.Equal(t, decad.ToleranceExceeded, report.Clearance.Tolerance.State)
		beyond := motionDiagnostics(report, decad.DiagMeasurementBeyondTolerance)
		require.Len(t, beyond, 1)
		require.Equal(t, decad.ReadingGap, beyond[0].Reading)
		require.Nil(t, beyond[0].At, `the whole-path reading concerns no one parameter`)
		require.Equal(t, report.Diagnostics, beyond)
	})
}

// requireUndecidedHolds asserts a coarse report never reads a pass through a
// pin clear: no collision, Suspect, and an undecided interval holding s.
func requireUndecidedHolds(t *testing.T, report *decad.MotionReport, s float64) {
	t.Helper()
	require.Empty(t, report.Collisions)
	require.Equal(t, decad.Suspect, report.Status)
	found := false
	for _, iv := range report.Intervals {
		if iv.From.Mag() <= s && s <= iv.To.Mag() {
			found = true
			require.Equal(t, decad.IntervalUndecided, iv.Outcome)
		}
	}
	require.True(t, found)
}

// TestVerifyMotionBetweenNearMissBetweenSamples is §9 test 14: test 3's blade
// and pin under a quarter turn stated as a Between. The pin's angle
// α = 90·31/64° is the fraction 31/64, and test 3's contact window ±1.247° is
// ±0.013856 in s. At WithResolution(1/30) refinement stops at width 1/32,
// whose grid points 15/32 and 16/32 each sit 1.40625° from α, outside the
// window; at WithResolution(1/900) the floor is 1/1024, whose grid includes
// 31/64 itself.
//
// Legs seen to fail when deleted: the ρ_max·θ term of a Between's τ (a pure
// rotation's τ collapses to zero, the endpoints alone certify the whole path
// past the pin, and the report reads Sound at either resolution).
func TestVerifyMotionBetweenNearMissBetweenSamples(t *testing.T) {
	t.Parallel()
	alpha := 90.0 * 31 / 64
	sAlpha := 31.0 / 64
	build := func(t *testing.T) (*decad.Document, *decad.Body) {
		t.Helper()
		doc := decad.New()
		blade := boxBody(t, doc, 0, -0.5, 50, 0.5, 10)
		a := alpha * math.Pi / 180
		cx, cy := 49*math.Cos(a), 49*math.Sin(a)
		boxBodyAtZ(t, doc, cx-0.4, cy-0.4, cx+0.4, cy+0.4, 4.6, 0.8)
		return doc, blade
	}
	m := func(t *testing.T) decad.Between {
		return decad.Between{From: r3.Identity(), To: quarterTurnZ(t, r3.Vec{})}
	}
	t.Run("a coarse resolution never reads the pass clear", func(t *testing.T) {
		t.Parallel()
		doc, blade := build(t)
		report := verifyMotion(t, doc, []*decad.Body{blade}, m(t), fraction(1.0/30))
		requireUndecidedHolds(t, report, sAlpha)
	})
	t.Run("a fine resolution finds the pin", func(t *testing.T) {
		t.Parallel()
		doc, blade := build(t)
		report := verifyMotion(t, doc, []*decad.Body{blade}, m(t), fraction(1.0/900))
		require.Equal(t, decad.Interfering, report.Status)
		require.NotEmpty(t, report.Collisions)
		require.InDelta(t, sAlpha, report.Collisions[0].At.Mag(), 0.013856)
	})
}

// TestVerifyMotionBetweenOffsetAxis is §9 test 15: test 3's blade starting at
// x ∈ [50, 100] and swinging a quarter turn about the Z axis through
// (−50, 0, 0), so its tip is 150 mm from the screw axis, past a 0.8 mm pin
// 149 mm from it at α = 90·31/64°. The contact window is
// |θ − α| ≤ asin(1.066/149) ≈ 0.41°; the depth-5 grid points 15/32 and 16/32
// clear the pin by about 2.59 mm each, and over a width-1/32 interval the
// travel with the true ρ_max ≈ 150 is 7.36 mm, above the gaps' sum 5.18.
//
// Legs seen to fail when deleted: ρ_max read from the rest box instead of the
// From-placed corners, and ρ_max taken about the axis through the origin
// instead of through Point (each reads 100 mm, so the whole path's travel is
// 157 mm against endpoint gaps summing to about 209 mm, the endpoints alone
// certify the path past the pin, and both subtests fail); and the ideal
// frame composing the screw before From instead of after it (η grows to
// tens of millimetres and the fine subtest finds no collision).
func TestVerifyMotionBetweenOffsetAxis(t *testing.T) {
	t.Parallel()
	alpha := 90.0 * 31 / 64
	sAlpha := 31.0 / 64
	pivot := r3.NewVec(-50, 0, 0)
	from := shiftBy(t, r3.NewVec(50, 0, 0))
	m := decad.Between{From: from, To: then(t, from, quarterTurnZ(t, pivot))}

	inv := transformOK(t)(m.From.Inverse())
	sc, err := transformOK(t)(inv.Then(m.To)).Screw()
	require.NoError(t, err)
	require.InDelta(t, -50, sc.Point.X, 1e-9)
	require.InDelta(t, 0, sc.Point.Y, 1e-9)
	require.InDelta(t, 0, sc.Point.Z, 1e-9)
	require.InDelta(t, math.Pi/2, sc.Angle.Mag(), 1e-15)
	require.Zero(t, sc.Slide)

	build := func(t *testing.T) (*decad.Document, *decad.Body) {
		t.Helper()
		doc := decad.New()
		blade := boxBody(t, doc, 0, -0.5, 50, 0.5, 10)
		a := alpha * math.Pi / 180
		cx, cy := -50+149*math.Cos(a), 149*math.Sin(a)
		boxBodyAtZ(t, doc, cx-0.4, cy-0.4, cx+0.4, cy+0.4, 4.6, 0.8)
		return doc, blade
	}
	t.Run("a coarse resolution never reads the pass clear", func(t *testing.T) {
		t.Parallel()
		doc, blade := build(t)
		report := verifyMotion(t, doc, []*decad.Body{blade}, m, fraction(1.0/30))
		requireUndecidedHolds(t, report, sAlpha)
	})
	t.Run("a fine resolution finds the pin", func(t *testing.T) {
		t.Parallel()
		doc, blade := build(t)
		report := verifyMotion(t, doc, []*decad.Body{blade}, m, fraction(1.0/900))
		require.Equal(t, decad.Interfering, report.Status)
		require.NotEmpty(t, report.Collisions)
		require.InDelta(t, sAlpha, report.Collisions[0].At.Mag(), 0.41/90)
	})
}

// TestVerifyMotionBetweenTranslationAgreesWithPrismatic is §9 test 16: test
// 5's cube under Between{Identity(), Translation(30, 0, 0)} and under
// Prismatic{X, 0 mm, 30 mm}, on test 5's three static fixtures. The read
// screw is Axis (1, 0, 0), Angle 0, Slide 30, Point 0, and screw.At(s) builds
// the translation (30·s, 0, 0) exactly for every dyadic s, so the float
// poses, the kernel results and η (zero on both sides) coincide.
//
// Legs seen to fail when deleted: the slide term |d| of a Between's τ (the
// read θ is 0, τ collapses to zero, and the pin's endpoints falsely certify
// the interval, so the Between reads Sound where the Prismatic reads Suspect;
// the other two subtests fail too, their interval clearances no longer the
// Prismatic's).
func TestVerifyMotionBetweenTranslationAgreesWithPrismatic(t *testing.T) {
	t.Parallel()
	between := decad.Between{From: r3.Identity(), To: shiftBy(t, r3.NewVec(30, 0, 0))}
	sc, err := between.To.Screw()
	require.NoError(t, err)
	require.Equal(t, r3.NewVec(1, 0, 0), sc.Axis)
	require.Equal(t, r3.Vec{}, sc.Point)
	require.Zero(t, sc.Angle.Mag())
	require.Equal(t, 30.0, sc.Slide)

	cases := []struct {
		name             string
		static           func(t *testing.T, doc *decad.Document)
		between, prism   []decad.MotionOption
		wantStatus       decad.Status
		wantCollisions   bool
		wantPathReading  bool
		wantIntervalOnly decad.IntervalOutcome
	}{
		{name: "wall at x = 25", static: func(t *testing.T, doc *decad.Document) { boxBodyAtZ(t, doc, 25, -10, 35, 20, -10, 30) },
			wantStatus: decad.Interfering, wantCollisions: true},
		{name: "L-shaped clear body", static: func(t *testing.T, doc *decad.Document) { lWall(t, doc) },
			wantStatus: decad.Sound, wantPathReading: true},
		{name: "pin between samples", static: func(t *testing.T, doc *decad.Document) { boxBodyAtZ(t, doc, 14, 4, 16, 6, 4, 2) },
			between: []decad.MotionOption{fraction(1)}, prism: []decad.MotionOption{decad.WithResolution(units.Millimeters(30))},
			wantStatus: decad.Suspect, wantIntervalOnly: decad.IntervalUndecided},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			reportFor := func(m decad.Motion, opts []decad.MotionOption) *decad.MotionReport {
				doc := decad.New()
				cube := boxBody(t, doc, 0, 0, 10, 10, 10)
				tc.static(t, doc)
				return verifyMotion(t, doc, []*decad.Body{cube}, m, opts...)
			}
			b := reportFor(between, tc.between)
			p := reportFor(alongX(30), tc.prism)

			require.Equal(t, tc.wantStatus, p.Status)
			require.Equal(t, p.Status, b.Status)
			require.Len(t, b.Poses, len(p.Poses))
			require.Len(t, b.Intervals, len(p.Intervals))
			for k := range p.Intervals {
				require.Equal(t, p.Intervals[k].Outcome, b.Intervals[k].Outcome)
				if tc.wantIntervalOnly != decad.IntervalNotEvaluated {
					require.Equal(t, tc.wantIntervalOnly, b.Intervals[k].Outcome)
				}
				if p.Intervals[k].Clearance == nil {
					require.Nil(t, b.Intervals[k].Clearance)
					continue
				}
				require.NotNil(t, b.Intervals[k].Clearance)
				require.InDelta(t, p.Intervals[k].Clearance.Value.Mag(), b.Intervals[k].Clearance.Value.Mag(), 1e-9)
			}
			require.Len(t, b.Collisions, len(p.Collisions))
			require.Equal(t, tc.wantCollisions, len(p.Collisions) > 0)
			for k := range p.Collisions {
				require.InDelta(t, p.Collisions[k].At.Mag(), 30*b.Collisions[k].At.Mag(), 1e-9)
				require.InDelta(t, p.Collisions[k].Volume.Value.Mag(), b.Collisions[k].Volume.Value.Mag(), 1e-9)
			}
			require.Equal(t, tc.wantPathReading, p.Clearance != nil)
			if p.Clearance == nil {
				require.Nil(t, b.Clearance)
				return
			}
			require.NotNil(t, b.Clearance)
			require.InDelta(t, p.Clearance.Value.Mag(), b.Clearance.Value.Mag(), 1e-9)
		})
	}
}

// TestVerifyMotionBetweenRotationAgreesWithRevolute is §9 test 17: the arm
// under Between{Identity(), RotationAround(origin, Z, 90°)} and under test
// 1's Revolute, on test 1's wall, on test 2's wall bisected (both floors stop
// the dyadic grid at depth 14) and on test 4's stop. The read screw's angle is
// the float nearest π/2, not the exact quarter turn the Revolute denotes, so
// the two ideal paths differ by about 6e-17 rad, twelve orders below the
// fixtures' margins.
func TestVerifyMotionBetweenRotationAgreesWithRevolute(t *testing.T) {
	t.Parallel()
	cases := []struct {
		name           string
		static         func(t *testing.T, doc *decad.Document)
		between, revol []decad.MotionOption
	}{
		{name: "test 1's wall", static: func(t *testing.T, doc *decad.Document) { boxBodyAtZ(t, doc, -100, 40, 100, 60, -10, 40) },
			between: []decad.MotionOption{fraction(1.0 / 256)}, revol: []decad.MotionOption{decad.WithResolution(units.Degrees(90.0 / 256))}},
		{name: "test 2's wall bisected", static: func(t *testing.T, doc *decad.Document) { boxBody(t, doc, -100, 60, 100, 80, 10) },
			between: []decad.MotionOption{fraction(0.01 / 90)}, revol: []decad.MotionOption{decad.WithResolution(units.Degrees(0.01))}},
		{name: "test 4's stop", static: func(t *testing.T, doc *decad.Document) { boxBody(t, doc, 0, -24, 48, -14, 10) }},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			reportFor := func(m decad.Motion, opts []decad.MotionOption) *decad.MotionReport {
				doc := decad.New()
				arm := motionArm(t, doc)
				tc.static(t, doc)
				return verifyMotion(t, doc, []*decad.Body{arm}, m, opts...)
			}
			b := reportFor(decad.Between{From: r3.Identity(), To: quarterTurnZ(t, r3.Vec{})}, tc.between)
			r := reportFor(armSwing(), tc.revol)

			require.Equal(t, r.Status, b.Status)
			require.Len(t, b.Intervals, len(r.Intervals))
			for k := range r.Intervals {
				require.Equal(t, r.Intervals[k].Outcome, b.Intervals[k].Outcome)
			}
			require.Len(t, b.Collisions, len(r.Collisions))
			for k := range r.Collisions {
				require.InDelta(t, r.Collisions[k].At.Mag(), 90*b.Collisions[k].At.Mag(), 1e-9)
			}
			if r.Clearance == nil {
				require.Nil(t, b.Clearance)
			} else {
				require.NotNil(t, b.Clearance)
				require.InDelta(t, r.Clearance.Value.Mag(), b.Clearance.Value.Mag(), 1e-9)
			}
			if tc.name != "test 4's stop" {
				return
			}
			first := b.Poses[0].Clearances
			require.Len(t, first, 1)
			require.Equal(t, decad.Exact, first[0].Gap.Exactness)
			require.Zero(t, first[0].Gap.Value.Mag())
			require.Zero(t, first[0].Gap.Bound.Mag())
		})
	}
}

// TestVerifyMotionBetweenSweptBoxIsFromPlaced is §9 test 18: a cube resting at
// x ∈ [0, 10] carried from Translation(100, 0, 0) to Translation(130, 0, 0),
// so it travels x ∈ [100, 140]. Its leading face reaches the wall at
// x ∈ [125, 135] at s = 1/2 exactly, a grid point where the two touch across
// a shared plane. The near slab beside the cube's RESTING place and the far
// slab are both excluded by the From-placed box [100, 110] grown by the
// travel 30 to [70, 140].
//
// Legs seen to fail when deleted: the From-placed swept box (the rest box
// [0, 10] grown by 30 excludes the wall, and the report reads Sound).
func TestVerifyMotionBetweenSweptBoxIsFromPlaced(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	cube := boxBody(t, doc, 0, 0, 10, 10, 10)
	wall := boxBodyAtZ(t, doc, 125, -10, 135, 20, -10, 30)
	near := boxBodyAtZ(t, doc, 20, -10, 30, 20, -10, 30)
	far := boxBodyAtZ(t, doc, 500, -10, 510, 20, -10, 30)
	m := decad.Between{From: shiftBy(t, r3.NewVec(100, 0, 0)), To: shiftBy(t, r3.NewVec(130, 0, 0))}
	report := verifyMotion(t, doc, []*decad.Body{cube}, m, fraction(1.0/256))

	requireOnsetAbove(t, report, 0.5)
	last := report.Collisions[len(report.Collisions)-1]
	require.Same(t, wall, last.Static)
	require.InDelta(t, 500, last.Volume.Value.Mag(), 1e-6, `the cube at [130, 140] overlaps the wall's [125, 135] over 5 × 10 × 10`)
	require.Equal(t, []*decad.Body{wall, near, far}, report.Against)
	for _, pose := range report.Poses {
		for _, row := range pose.Clearances {
			require.Same(t, wall, row.B)
		}
		for _, row := range pose.Interferences {
			require.Same(t, wall, row.B)
		}
		for _, d := range pose.Diagnostics {
			require.Same(t, wall, d.Pair.B)
		}
	}
}

// TestVerifyMotionBetweenCoversTheStatedTo is §9 test 19: the arm starting
// about 1.5e6 mm away, under the inverse of a quarter turn about a far pivot,
// and arriving at s = 1 exactly where it sits, 46 mm from test 2's wall. The
// read screw rebuilds the relative motion only to about ε·|t| ≈ 1e-10 mm, so
// the ideal end misses the identity by that much, while the kernel's own
// s = 1 reading is the exact axis-aligned box gap; the charge η_1 covers both.
// With the pivot at the origin the s = 1 bound is strictly smaller and still
// positive, since the angle's enclosure has nonzero width.
//
// Legs seen to fail when deleted: η_ideal at s = 1, charging the pose against
// the stated To alone (η_To is exactly zero for the unplaced arm, and the row
// comes back Exact with a zero bound). Proven redundant: the η_To leg of the
// maximum, since max(η_ideal, η_To) ≥ η_To by definition and η_To is one Then
// rounding, far below η_ideal on any fixture whose ideal end is not To.
func TestVerifyMotionBetweenCoversTheStatedTo(t *testing.T) {
	t.Parallel()
	endGap := func(t *testing.T, pivot r3.Vec) decad.Measurement {
		t.Helper()
		doc := decad.New()
		arm := motionArm(t, doc)
		boxBody(t, doc, -100, 60, 100, 80, 10)
		m := decad.Between{From: transformOK(t)(quarterTurnZ(t, pivot).Inverse()), To: r3.Identity()}
		report := verifyMotion(t, doc, []*decad.Body{arm}, m, fraction(1))
		end := report.Poses[len(report.Poses)-1]
		require.True(t, end.Pose == r3.Identity(), `the s = 1 pose is the stated To`)
		require.Len(t, end.Clearances, 1)
		for _, d := range motionDiagnostics(report, decad.DiagMeasurementBeyondTolerance) {
			// A nil At is the whole-path reading's own finding, not the pose's.
			require.False(t, d.At != nil && *d.At == units.Scalar(1), `the s = 1 bound passes the default gate`)
		}
		gap := end.Clearances[0].Gap
		require.InDelta(t, 46, gap.Value.Mag(), 1e-6)
		require.Equal(t, decad.Approximate, gap.Exactness)
		require.Greater(t, gap.Bound.Mag(), 0.0)
		require.Less(t, gap.Bound.Mag(), 1e-6)
		return gap
	}
	far := endGap(t, r3.NewVec(1e6, 3e5, 0))
	origin := endGap(t, r3.Vec{})
	require.Less(t, origin.Bound.Mag(), far.Bound.Mag())
}

// TestVerifyMotionBetweenErrors is §9 test 20's error rows: one subtest per
// Between row of §8's table, each asserting the sentinel, no report and an
// unchanged document. The non-finite-component row has no subtest: every r3
// producer validates what it builds, so no r3.Transform with a non-finite
// component can be constructed through r3's public API.
func TestVerifyMotionBetweenErrors(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	arm := motionArm(t, doc)
	boxBodyAtZ(t, doc, -100, 60, 100, 80, -10, 50)
	mirror, err := r3.NewFrame(r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	reflect := transformOK(t)(r3.Reflection(mirror))
	tiny := transformOK(t)(r3.Rotation(r3.NewVec(0, 0, 1), units.Radians(1e-300)))

	cases := []struct {
		name   string
		motion decad.Motion
		opts   []decad.MotionOption
		want   error
	}{
		{name: "zero From", motion: decad.Between{To: r3.Identity()}, want: decad.ErrDegenerate},
		{name: "zero To", motion: decad.Between{From: r3.Identity()}, want: decad.ErrDegenerate},
		{name: "reflection against the identity", motion: decad.Between{From: r3.Identity(), To: reflect}, want: decad.ErrDegenerate},
		{name: "From equals To", motion: decad.Between{From: screwTo(t, 20), To: screwTo(t, 20)}, want: decad.ErrDegenerate},
		{name: "unrepresentable screw point", motion: decad.Between{From: r3.Identity(), To: then(t, tiny, shiftBy(t, r3.NewVec(1e10, 0, 0)))}, want: decad.ErrNotFinite},
		{name: "nil between pointer", motion: (*decad.Between)(nil), want: decad.ErrDegenerate},
		{name: "resolution as a length", motion: screwArm(t), opts: []decad.MotionOption{decad.WithResolution(units.Millimeters(1))}, want: decad.ErrUnitKind},
		{name: "resolution as an angle", motion: screwArm(t), opts: []decad.MotionOption{decad.WithResolution(units.Degrees(1))}, want: decad.ErrUnitKind},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			before := doc.Bodies()
			report, err := doc.VerifyMotion(t.Context(), []*decad.Body{arm}, tc.motion, tc.opts...)
			require.ErrorIs(t, err, tc.want)
			require.Nil(t, report)
			require.Equal(t, before, doc.Bodies())
		})
	}
}

// TestVerifyMotionBetweenNonMutationAndDeterminism is §9 test 7 run on the
// screw arm (§9 test 20): the live body set and its order survive the call,
// and two calls on the same inputs return reports equal in every field. A
// canceled context returns ctx.Err() and no report before any pose.
func TestVerifyMotionBetweenNonMutationAndDeterminism(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	arm := motionArm(t, doc)
	boxBodyAtZ(t, doc, -100, 40, 100, 60, -10, 50)
	m := screwArm(t)
	before := doc.Bodies()
	first := verifyMotion(t, doc, []*decad.Body{arm}, m, fraction(1.0/64))
	second := verifyMotion(t, doc, []*decad.Body{arm}, &m, fraction(1.0/64))
	require.Equal(t, before, doc.Bodies())
	require.Equal(t, decad.Interfering, first.Status)
	second.Motion = first.Motion
	require.Equal(t, first, second)

	canceled, cancel := context.WithCancel(t.Context())
	cancel()
	report, err := doc.VerifyMotion(canceled, []*decad.Body{arm}, m)
	require.ErrorIs(t, err, context.Canceled)
	require.Nil(t, report)
	require.Equal(t, before, doc.Bodies())
}
