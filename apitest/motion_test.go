package apitest_test

import (
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file holds the required tests of docs/motion-check-design.md §9,
// numbered as §9 numbers them. A resolution wider than the whole path
// (endpointsOnly) evaluates the two endpoints alone. Bounds are
// asserted against geometric truths or as small, never pinned to a measured
// literal: FMA contraction moves the last ulp between amd64 and arm64.
//
// The arm shared by the revolute fixtures is the prism extruded 10 mm from
// the XY-plane rectangle x ∈ [0, 48], y ∈ [−14, 14], swinging about the Z
// axis through the origin from 0° to 90°. Its farthest corner (48, ±14) sits
// exactly 50 mm from the axis, so ρ_max is 50 and the whole swing's travel
// bound τ is 50·π/2 ≈ 78.54 mm.

func motionArm(t *testing.T, doc *decad.Document) *decad.Body {
	t.Helper()
	return boxBody(t, doc, 0, -14, 48, 14, 10)
}

func armSwing() decad.Revolute {
	return decad.Revolute{Axis: r3.NewVec(0, 0, 1), From: units.Degrees(0), To: units.Degrees(90)}
}

// alongX slides from rest along +X to the displacement to.
// endpointsOnly states a resolution wider than any path these tests use, so
// the check evaluates From and To alone (§3).
func endpointsOnly(motion decad.Motion) decad.MotionOption {
	if _, ok := motion.(decad.Prismatic); ok {
		return decad.WithResolution(units.Millimeters(1000))
	}
	return decad.WithResolution(units.Degrees(360))
}

func alongX(to float64) decad.Prismatic {
	return decad.Prismatic{Dir: r3.NewVec(1, 0, 0), From: units.Millimeters(0), To: units.Millimeters(to)}
}

// lWall is a static L-shaped prism 10 mm tall: a bar whose face x = 45
// stands across the +X path of a cube resting at [0, 10]², joined to an arm
// along y ∈ [50, 60] that reaches back over the cube's resting place. The arm
// puts the wall's bounding box within the cube's swept box, so the pair is
// evaluated rather than excluded, while its own gap to the cube, 40 mm, never
// becomes the minimum.
func lWall(t *testing.T, doc *decad.Document) *decad.Body {
	t.Helper()
	s, profile := polygonSketch(t, [][2]float64{{45, -10}, {55, -10}, {55, 60}, {0, 60}, {0, 50}, {45, 50}})
	body, err := doc.Extrude(s, profile, decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	require.NoError(t, err)
	return body
}

func verifyMotion(t *testing.T, doc *decad.Document, moving []*decad.Body, m decad.Motion, opts ...decad.MotionOption) *decad.MotionReport {
	t.Helper()
	before := doc.Bodies()
	report, err := doc.VerifyMotion(t.Context(), moving, m, opts...)
	require.NoError(t, err)
	require.Equal(t, before, doc.Bodies(), `VerifyMotion must leave the live body set and its order unchanged`)
	return report
}

func motionDiagnostics(report *decad.MotionReport, code decad.DiagnosticCode) []decad.Diagnostic {
	var out []decad.Diagnostic
	for _, d := range report.Diagnostics {
		if d.Code == code {
			out = append(out, d)
		}
	}
	return out
}

func TestMotionPoseAt(t *testing.T) {
	t.Parallel()
	t.Run("revolute rotates about its center", func(t *testing.T) {
		t.Parallel()
		m := decad.Revolute{Center: r3.NewVec(10, 0, 0), Axis: r3.NewVec(0, 0, 2), From: units.Degrees(0), To: units.Degrees(90)}
		pose, err := m.PoseAt(units.Degrees(90))
		require.NoError(t, err)
		got := pose.Apply(r3.NewVec(11, 0, 0))
		require.InDelta(t, 10, got.X, 1e-12)
		require.InDelta(t, 1, got.Y, 1e-12)
		require.InDelta(t, 0, got.Z, 1e-12)
		fixed := pose.Apply(r3.NewVec(10, 0, 0))
		require.InDelta(t, 10, fixed.X, 1e-12, `the center is fixed`)
		require.InDelta(t, 0, fixed.Y, 1e-12, `the center is fixed`)
	})
	t.Run("prismatic translates along its unit direction", func(t *testing.T) {
		t.Parallel()
		m := decad.Prismatic{Dir: r3.NewVec(2, 0, 0), From: units.Millimeters(0), To: units.Millimeters(30)}
		pose, err := m.PoseAt(units.Meters(0.005))
		require.NoError(t, err)
		require.Equal(t, r3.NewVec(5, 0, 0), pose.Translation())
	})
	refusals := []struct {
		name string
		m    decad.Motion
		at   units.Value
		want error
	}{
		{"revolute From of the wrong kind", decad.Revolute{Axis: r3.NewVec(0, 0, 1), From: units.Millimeters(0), To: units.Degrees(90)}, units.Degrees(1), decad.ErrUnitKind},
		{"revolute To a bare scalar", decad.Revolute{Axis: r3.NewVec(0, 0, 1), From: units.Degrees(0), To: units.Scalar(1)}, units.Degrees(1), decad.ErrUnitKind},
		{"revolute pose of the wrong kind", armSwing(), units.Millimeters(1), decad.ErrUnitKind},
		{"prismatic From of the wrong kind", decad.Prismatic{Dir: r3.NewVec(1, 0, 0), From: units.Degrees(0), To: units.Millimeters(1)}, units.Millimeters(1), decad.ErrUnitKind},
		{"prismatic pose of the wrong kind", alongX(1), units.Degrees(1), decad.ErrUnitKind},
		{"non-finite From", decad.Revolute{Axis: r3.NewVec(0, 0, 1), From: units.Degrees(math.NaN()), To: units.Degrees(90)}, units.Degrees(1), decad.ErrNotFinite},
		{"non-finite pose", armSwing(), units.Degrees(math.Inf(1)), decad.ErrNotFinite},
		{"non-finite center", decad.Revolute{Center: r3.NewVec(math.Inf(1), 0, 0), Axis: r3.NewVec(0, 0, 1), From: units.Degrees(0), To: units.Degrees(90)}, units.Degrees(1), decad.ErrNotFinite},
		{"non-finite axis", decad.Revolute{Axis: r3.NewVec(0, math.NaN(), 1), From: units.Degrees(0), To: units.Degrees(90)}, units.Degrees(1), decad.ErrNotFinite},
		{"non-finite direction", decad.Prismatic{Dir: r3.NewVec(math.Inf(-1), 0, 0), From: units.Millimeters(0), To: units.Millimeters(1)}, units.Millimeters(1), decad.ErrNotFinite},
		{"zero axis", decad.Revolute{From: units.Degrees(0), To: units.Degrees(90)}, units.Degrees(1), decad.ErrDegenerate},
		{"zero direction", decad.Prismatic{From: units.Millimeters(0), To: units.Millimeters(1)}, units.Millimeters(1), decad.ErrDegenerate},
		{"overflowing pivot offset", decad.Revolute{Center: r3.NewVec(math.MaxFloat64, math.MaxFloat64, 0), Axis: r3.NewVec(0, 0, 1), From: units.Degrees(0), To: units.Degrees(90)}, units.Degrees(90), decad.ErrNotFinite},
		{"overflowing displacement", alongX(1), units.Meters(1e306), decad.ErrNotFinite},
	}
	for _, tc := range refusals {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			_, err := tc.m.PoseAt(tc.at)
			require.ErrorIs(t, err, tc.want)
		})
	}
}

// TestVerifyMotionKnownCollisionAngle is §9 test 1: the arm swinging into a
// wall at y ∈ [40, 60] whose z ∈ [−10, 30] reaches past the arm's caps, so
// the two bodies share no face plane and the read-only proof measures their
// overlap. The corner (48, 14) reaches y = 40 first, at θ* = atan(3/4) ≈
// 36.87°. The 90° endpoint collides, so the colliding interval is halved
// toward the onset, and at WithResolution(0.25°) — a grid step of
// 90°/512 ≈ 0.176° — the first collision lands within 0.5° above θ*. The same
// holds for the swing stated in radians.
//
// Legs seen to fail when deleted: the onset bisection of a colliding interval
// with a collision-free end (the first collision stays at the 90° endpoint).
func TestVerifyMotionKnownCollisionAngle(t *testing.T) {
	t.Parallel()
	thetaStar := math.Atan(3.0 / 4.0)
	cases := []struct {
		name       string
		motion     decad.Motion
		resolution units.Value
		toRadians  func(units.Value) float64
	}{
		{"degrees", armSwing(), units.Degrees(0.25), func(v units.Value) float64 { return v.Mag() * math.Pi / 180 }},
		{"radians", decad.Revolute{Axis: r3.NewVec(0, 0, 1), From: units.Radians(0), To: units.Radians(math.Pi / 2)},
			units.Radians(0.25 * math.Pi / 180), func(v units.Value) float64 { return v.Mag() }},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc := decad.New()
			arm := motionArm(t, doc)
			wall := boxBodyAtZ(t, doc, -100, 40, 100, 60, -10, 40)
			report := verifyMotion(t, doc, []*decad.Body{arm}, tc.motion, decad.WithResolution(tc.resolution))

			require.Equal(t, decad.Interfering, report.Status)
			require.NotEmpty(t, report.Collisions)
			for _, c := range report.Collisions {
				require.Greater(t, tc.toRadians(c.At), thetaStar)
				require.Same(t, arm, c.Moving)
				require.Same(t, wall, c.Static)
				require.Greater(t, c.Volume.Value.Mag(), 0.0)
				require.Less(t, c.Volume.Bound.Mag(), c.Volume.Value.Mag())
			}
			first := tc.toRadians(report.Collisions[0].At)
			require.Less(t, first-thetaStar, 0.5*math.Pi/180, `the onset is bracketed to the resolution`)
			for _, iv := range report.Intervals {
				from, to := tc.toRadians(iv.From), tc.toRadians(iv.To)
				if iv.Outcome == decad.IntervalClear {
					require.LessOrEqual(t, to, thetaStar)
				}
				if from <= thetaStar && thetaStar <= to {
					require.NotEqual(t, decad.IntervalClear, iv.Outcome, `the interval holding θ* is never clear`)
				}
			}
		})
	}
}

// TestVerifyMotionNearMissBetweenSamples is §9 test 3: a 1 mm blade swinging
// through a 0.8 mm pin at polar angle α = 90·31/64 = 43.59375°, radius 49.
// Contact needs 49·|sin(θ − α)| ≤ 0.5 + 0.4·√2, so the contact window lies
// within |θ − α| ≤ 1.247°. At WithResolution(3°) refinement stops at
// 90°/32 = 2.8125°, and the reachable poses nearest α, 42.1875° and 45°, sit
// 1.40625° from it — outside the window — so the blade passes through the pin
// between samples and the interval spanning it reads undecided, never clear.
// At WithResolution(0.1°) the grid reaches α itself and the collision is found.
//
// Legs seen to fail when deleted: the ρ_max·Δθ travel term, and ρ_max read
// from the blade's centroid instead of its box (each lets the coarse interval
// spanning the pin certify, and the 3° report reads Sound). Proven redundant:
// a one-sided certificate, lo_k > τ or lo_{k+1} > τ alone, implies the
// two-sided lo_k + lo_{k+1} > τ, so it certifies fewer intervals, never more,
// and no fixture can fail on it.
func TestVerifyMotionNearMissBetweenSamples(t *testing.T) {
	t.Parallel()
	alpha := 90.0 * 31 / 64
	window := 1.247
	build := func(t *testing.T) (*decad.Document, *decad.Body) {
		t.Helper()
		doc := decad.New()
		blade := boxBody(t, doc, 0, -0.5, 50, 0.5, 10)
		a := alpha * math.Pi / 180
		cx, cy := 49*math.Cos(a), 49*math.Sin(a)
		boxBodyAtZ(t, doc, cx-0.4, cy-0.4, cx+0.4, cy+0.4, 4.6, 0.8)
		return doc, blade
	}
	t.Run("a coarse resolution never reads the pass clear", func(t *testing.T) {
		t.Parallel()
		doc, blade := build(t)
		report := verifyMotion(t, doc, []*decad.Body{blade}, armSwing(), decad.WithResolution(units.Degrees(3)))
		require.Empty(t, report.Collisions)
		require.Equal(t, decad.Suspect, report.Status)
		found := false
		for _, iv := range report.Intervals {
			if iv.From.Mag() <= alpha && alpha <= iv.To.Mag() {
				found = true
				require.Equal(t, decad.IntervalUndecided, iv.Outcome)
			}
		}
		require.True(t, found)
	})
	t.Run("a fine resolution finds the pin", func(t *testing.T) {
		t.Parallel()
		doc, blade := build(t)
		report := verifyMotion(t, doc, []*decad.Body{blade}, armSwing(), decad.WithResolution(units.Degrees(0.1)))
		require.Equal(t, decad.Interfering, report.Status)
		require.NotEmpty(t, report.Collisions)
		require.InDelta(t, alpha, report.Collisions[0].At.Mag(), window)
	})
}

// TestVerifyMotionClearSwingEndpoints is §9 test 2's endpoints-only part: the
// arm swinging 0°→90° past a wall at
// y ∈ [60, 80]. The true minimum gap is 10 mm, at θ = 90° − atan(7/24), and
// the arm never touches the wall — but the two endpoint gaps, 46 mm at 0° and
// 12 mm at 90°, sum to 58 mm, under the 78.54 mm the arm's far corner can
// travel. The certificate therefore cannot close, the interval is
// IntervalUndecided, and the report is Suspect, never Sound: two clear
// samples prove nothing about the swing between them.
//
// Legs seen to fail when deleted: the revolute travel term ρ_max·Δθ (τ = 0
// certifies the interval clear); ρ_max read from the arm's centroid (24, 0)
// instead of its box (τ = 37.7 mm < 58 mm certifies it); and the swept-box
// exclusion's travel inflation (the wall, outside the arm's resting box,
// is excluded unevaluated and the report reads Sound).
func TestVerifyMotionClearSwingEndpoints(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	arm := motionArm(t, doc)
	wall := boxBody(t, doc, -100, 60, 100, 80, 10)
	report := verifyMotion(t, doc, []*decad.Body{arm}, armSwing(), endpointsOnly(armSwing()))

	require.Equal(t, decad.Suspect, report.Status)
	require.False(t, report.Passed())
	require.Equal(t, units.Degrees(360), report.Request.Resolution)
	require.Nil(t, report.Request.MinClearance)
	require.Equal(t, decad.AssessmentNotEvaluated, report.Assessment)
	require.Equal(t, []*decad.Body{arm}, report.Moving)
	require.Equal(t, []*decad.Body{wall}, report.Against)
	require.Empty(t, report.Collisions)
	require.Nil(t, report.Clearance, `an undecided stretch leaves the path minimum unknown`)

	require.Len(t, report.Poses, 2)
	start, end := report.Poses[0], report.Poses[1]
	require.Equal(t, units.Degrees(0), start.At)
	require.Equal(t, units.Degrees(90), end.At)
	require.Len(t, start.Clearances, 1)
	require.Same(t, arm, start.Clearances[0].A)
	require.Same(t, wall, start.Clearances[0].B)
	require.Equal(t, decad.Exact, start.Clearances[0].Gap.Exactness, `the resting arm is its own unplaced body`)
	require.Equal(t, 46.0, start.Clearances[0].Gap.Value.Mag())
	require.Len(t, end.Clearances, 1)
	endGap := end.Clearances[0].Gap
	require.Equal(t, decad.Approximate, endGap.Exactness)
	require.Greater(t, endGap.Bound.Mag(), 0.0)
	require.Less(t, endGap.Bound.Mag(), 1e-9, `a placement's rounding is ulp-scale`)
	require.LessOrEqual(t, endGap.Value.Mag()-endGap.Bound.Mag(), 12.0)
	require.GreaterOrEqual(t, endGap.Value.Mag()+endGap.Bound.Mag(), 12.0)

	require.Len(t, report.Intervals, 1)
	require.Equal(t, decad.IntervalUndecided, report.Intervals[0].Outcome)
	require.Nil(t, report.Intervals[0].Clearance)
	undecided := motionDiagnostics(report, decad.DiagMotionUndecidedInterval)
	require.Len(t, undecided, 1)
	require.Equal(t, report.Diagnostics, undecided, `the undecided interval is the only finding`)
	require.Nil(t, undecided[0].Pair)
	require.NotNil(t, undecided[0].At)
	require.Equal(t, units.Degrees(0), *undecided[0].At)
	require.Equal(t, decad.ReadingNone, undecided[0].Reading)
}

// TestVerifyMotionClearSwingBisected is §9 test 2's bisected part: the same
// swing refined on the dyadic grid of 90°. Under the travel bound a certified
// interval's Clearance sits below the true gap by up to τ/2 = 25 mm × Δθ, so
// the whole-path reading's half-width is about 12.5 mm × Δθ against a gate of
// 0.01 mm at rel = 1e-3 on the 10 mm minimum: WithResolution(0.01°) reaches
// it. The projection bound (§5.2) sits about ½·50·Δθ² below the flat minimum,
// so the default floor of 90°/1024 reaches it too, and a gate of 1e-6 mm at
// rel = 1e-7 is past it.
//
// Legs seen to fail when deleted: the projection bound (the default floor's
// reading stays beyond tolerance); refinement for the reading (the 0.01° run's
// Clearance stays coarse and fails the gate); refinement for the margin (under
// the loose gate the 9 mm margin stays undecided); and the halving of the
// lower envelope's minimum (interval lower bounds rise above their endpoints'
// proven gaps).
func TestVerifyMotionClearSwingBisected(t *testing.T) {
	t.Parallel()
	run := func(t *testing.T, opts ...decad.MotionOption) *decad.MotionReport {
		t.Helper()
		doc := decad.New()
		arm := motionArm(t, doc)
		boxBody(t, doc, -100, 60, 100, 80, 10)
		return verifyMotion(t, doc, []*decad.Body{arm}, armSwing(), opts...)
	}
	fine := decad.WithResolution(units.Degrees(0.01))
	// Each interval's Clearance bounds the gap over that interval from below,
	// so it can exceed neither endpoint's proven upper end, and the smallest
	// of them cannot exceed the path's true minimum.
	requireLowerBounds := func(t *testing.T, report *decad.MotionReport) {
		t.Helper()
		upper := func(p decad.PoseResult) float64 {
			require.Len(t, p.Clearances, 1)
			return p.Clearances[0].Gap.Value.Mag() + p.Clearances[0].Gap.Bound.Mag()
		}
		lowest := math.Inf(1)
		for k, iv := range report.Intervals {
			require.Equal(t, decad.IntervalClear, iv.Outcome)
			require.NotNil(t, iv.Clearance)
			require.LessOrEqual(t, iv.Clearance.Value.Mag(), min(upper(report.Poses[k]), upper(report.Poses[k+1])))
			lowest = min(lowest, iv.Clearance.Value.Mag())
		}
		require.LessOrEqual(t, lowest, 10.0, `a proven lower bound never exceeds the true minimum`)
	}
	t.Run("the reading reaches the gate at a fine resolution", func(t *testing.T) {
		t.Parallel()
		report := run(t, fine)
		require.Equal(t, decad.Sound, report.Status)
		require.Empty(t, report.Diagnostics)
		require.Equal(t, units.Degrees(0.01), report.Request.Resolution)
		requireLowerBounds(t, report)
		require.NotNil(t, report.Clearance)
		require.InDelta(t, 10, report.Clearance.Value.Mag(), 0.1)
		require.LessOrEqual(t, report.Clearance.Value.Mag()-report.Clearance.Bound.Mag(), 10.0)
		require.GreaterOrEqual(t, report.Clearance.Value.Mag()+report.Clearance.Bound.Mag(), 10.0)
		require.Equal(t, decad.ToleranceSatisfied, report.Clearance.Tolerance.State)
		require.Equal(t, decad.AssessmentNotEvaluated, report.Assessment)
	})
	t.Run("a 9 mm margin is met", func(t *testing.T) {
		t.Parallel()
		minimum := units.Millimeters(9)
		report := run(t, fine, decad.WithMinClearance(minimum))
		require.Equal(t, decad.AssessmentMet, report.Assessment)
		require.Equal(t, decad.Sound, report.Status)
		require.Equal(t, &minimum, report.Request.MinClearance)
	})
	t.Run("a margin alone drives refinement under a loose gate", func(t *testing.T) {
		t.Parallel()
		// At rel = 0.5 the whole-path reading passes the gate as soon as the
		// swing certifies, so only the margin keeps the check refining until
		// every interval's lower bound reaches 9 mm.
		report := run(t, fine, decad.WithMotionTolerance(units.Scalar(0.5)), decad.WithMinClearance(units.Millimeters(9)))
		require.Equal(t, decad.AssessmentMet, report.Assessment)
		require.Equal(t, decad.Sound, report.Status)
		requireLowerBounds(t, report)
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
			require.Equal(t, decad.ReadingGap, d.Reading)
			require.Equal(t, units.Millimeters(11), *d.Required)
			require.Less(t, d.Observed.Value.Mag()+d.Observed.Bound.Mag(), 11.0, `the gap's whole proven interval lies below the minimum`)
			found := false
			for _, pose := range report.Poses {
				if pose.At == *d.At {
					found = true
					require.Equal(t, *d.Observed, pose.Clearances[0].Gap, `the diagnostic names the pose of the proving gap`)
				}
			}
			require.True(t, found)
		}
		require.Empty(t, motionDiagnostics(report, decad.DiagMotionUndecidedClearance), `a disproven margin is decided`)
	})
	t.Run("an exact 10 mm margin stays undecided", func(t *testing.T) {
		t.Parallel()
		report := run(t, fine, decad.WithMinClearance(units.Millimeters(10)))
		require.Equal(t, decad.AssessmentUndecided, report.Assessment, `no proven lower bound reaches an exact minimum`)
		require.Equal(t, decad.Suspect, report.Status)
		require.Empty(t, motionDiagnostics(report, decad.DiagMotionClearanceViolated))
		undecided := motionDiagnostics(report, decad.DiagMotionUndecidedClearance)
		require.NotEmpty(t, undecided)
		require.Equal(t, decad.ReadingGap, undecided[0].Reading)
		require.Less(t, undecided[0].Observed.Value.Mag(), 10.0)
	})
	t.Run("the default floor closes the reading", func(t *testing.T) {
		t.Parallel()
		report := run(t)
		require.Equal(t, units.Degrees(90.0/1024), report.Request.Resolution)
		requireLowerBounds(t, report)
		require.Equal(t, decad.Sound, report.Status)
		require.Empty(t, report.Diagnostics)
		require.NotNil(t, report.Clearance)
		require.Equal(t, decad.ToleranceSatisfied, report.Clearance.Tolerance.State)
		require.LessOrEqual(t, report.Clearance.Value.Mag()-report.Clearance.Bound.Mag(), 10.0)
		require.GreaterOrEqual(t, report.Clearance.Value.Mag()+report.Clearance.Bound.Mag(), 10.0)
	})
	t.Run("a tolerance finer than the floor's reading leaves it beyond tolerance", func(t *testing.T) {
		t.Parallel()
		report := run(t, decad.WithMotionTolerance(units.Scalar(1e-7)))
		requireLowerBounds(t, report)
		require.Equal(t, decad.Suspect, report.Status, `Sound in verdict, but the reading is coarse`)
		require.NotNil(t, report.Clearance)
		require.Equal(t, decad.ToleranceExceeded, report.Clearance.Tolerance.State)
		beyond := motionDiagnostics(report, decad.DiagMeasurementBeyondTolerance)
		require.Len(t, beyond, 1)
		require.Equal(t, decad.ReadingGap, beyond[0].Reading)
		require.Nil(t, beyond[0].At, `the whole-path reading concerns no one parameter`)
		require.Equal(t, report.Diagnostics, beyond)
	})
}

// TestVerifyMotionTouchingStart is §9 test 4's fixture: the arm resting on a
// stop prism that shares its y = −14 face plane at θ = 0, swinging away.
//
// The first pose's row is the coplanar certificate's Exact zero: the pose at
// 0° is the identity motion, so the transient arm is placed exactly where it
// rests and carries no placement rounding.
//
// The first interval is IntervalUndecided however far bisection goes (§5.2):
// certifying it needs lo_0 + lo_1 > τ with lo_0 = 0, but the far pose's gap is
// at most the start gap, 0, plus the farthest any arm point travels, so
// lo_1 ≤ τ always — the closed interval contains a pose at zero distance, and
// IntervalClear claims a positive one everywhere.
//
// Leg seen to fail when deleted: the exact sine and cosine of a whole number
// of quarter turns (the identity pose then carries an enclosure-width η, and
// its row is no longer an Exact zero).
func TestVerifyMotionTouchingStart(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	arm := motionArm(t, doc)
	stop := boxBody(t, doc, 0, -24, 48, -14, 10)
	report := verifyMotion(t, doc, []*decad.Body{arm}, armSwing())

	require.Greater(t, len(report.Poses), 2, `the check bisects toward the touch down to the resolution`)
	first := report.Poses[0]
	require.Len(t, first.Clearances, 1)
	require.Same(t, stop, first.Clearances[0].B)
	require.Equal(t, decad.Exact, first.Clearances[0].Gap.Exactness)
	require.Zero(t, first.Clearances[0].Gap.Value.Mag())
	require.Zero(t, first.Clearances[0].Gap.Bound.Mag())
	require.Empty(t, report.Collisions, `touching is not overlap`)

	require.Equal(t, decad.IntervalUndecided, report.Intervals[0].Outcome, `an interval holding a zero-distance pose never certifies`)
	require.Equal(t, decad.Suspect, report.Status)
}

// TestVerifyMotionPrismatic is §9 test 5: a 10 mm cube translating along +X
// from 0 to 30 mm.
//
// The cube's gap to a wall ahead shrinks at exactly the travel rate, so the
// certificate's lower envelope meets the far endpoint's gap exactly and the
// clear case's whole-path reading lands on the true minimum, 5 mm.
//
// Legs seen to fail when deleted: the prismatic travel term |Δd| (the "pin
// between samples" interval certifies clear through the pin, and the clear
// case's interval lower bound rises to 20 mm, above the true minimum).
// A plain slab ahead of the cube, excluded by the swept box, is covered in
// TestVerifyMotionSweptBoxExclusion.
func TestVerifyMotionPrismatic(t *testing.T) {
	t.Parallel()
	t.Run("a wall face at x = 25 is hit past 15 mm", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		cube := boxBody(t, doc, 0, 0, 10, 10, 10)
		wall := boxBodyAtZ(t, doc, 25, -10, 35, 20, -10, 30)
		report := verifyMotion(t, doc, []*decad.Body{cube}, alongX(30))

		require.Equal(t, decad.Interfering, report.Status)
		require.NotEmpty(t, report.Collisions)
		for _, c := range report.Collisions {
			require.Greater(t, c.At.Mag(), 15.0)
			require.Same(t, cube, c.Moving)
			require.Same(t, wall, c.Static)
			require.Greater(t, c.Volume.Value.Mag(), 0.0)
			require.Less(t, c.Volume.Bound.Mag(), c.Volume.Value.Mag())
		}
		last := report.Collisions[len(report.Collisions)-1]
		require.InDelta(t, 500, last.Volume.Value.Mag(), 1e-6, `the cube at [30, 40] overlaps the wall's [25, 35] over a 5 × 10 × 10 slab`)
		for _, iv := range report.Intervals {
			if iv.Outcome == decad.IntervalClear {
				require.LessOrEqual(t, iv.To.Mag(), 15.0)
			}
		}
		require.Equal(t, decad.IntervalColliding, report.Intervals[len(report.Intervals)-1].Outcome)
		collisions := motionDiagnostics(report, decad.DiagMotionCollision)
		require.Len(t, collisions, len(report.Collisions))
		require.Equal(t, decad.Interfering, collisions[0].Status)
		require.Equal(t, decad.ReadingOverlapVolume, collisions[0].Reading)
		require.Equal(t, report.Collisions[0].At, *collisions[0].At)
		require.Equal(t, &decad.DiagnosticPair{A: cube, B: wall}, collisions[0].Pair)
	})
	t.Run("a wall face at x = 45 is cleared by 5 mm", func(t *testing.T) {
		t.Parallel()
		// lWall, not a plain slab: a slab ahead of the cube is settled by the
		// swept-box exclusion alone, which proves the path clear but measures
		// no gap (§5.3), so it would carry no whole-path reading to check.
		doc := decad.New()
		cube := boxBody(t, doc, 0, 0, 10, 10, 10)
		lWall(t, doc)
		report := verifyMotion(t, doc, []*decad.Body{cube}, alongX(30))

		require.Equal(t, decad.Sound, report.Status)
		require.True(t, report.Passed())
		require.Empty(t, report.Diagnostics)
		require.Len(t, report.Intervals, 1)
		iv := report.Intervals[0]
		require.Equal(t, decad.IntervalClear, iv.Outcome)
		require.NotNil(t, iv.Clearance)
		require.LessOrEqual(t, iv.Clearance.Value.Mag(), 5.0, `a proven lower bound never exceeds the true minimum`)
		require.InDelta(t, 5, iv.Clearance.Value.Mag(), 0.01)
		require.NotNil(t, report.Clearance)
		require.InDelta(t, 5, report.Clearance.Value.Mag(), 0.01)
		require.Equal(t, decad.Approximate, report.Clearance.Exactness)
		require.LessOrEqual(t, report.Clearance.Value.Mag()-report.Clearance.Bound.Mag(), 5.0)
		require.GreaterOrEqual(t, report.Clearance.Value.Mag()+report.Clearance.Bound.Mag(), 5.0)
		require.Equal(t, decad.ToleranceSatisfied, report.Clearance.Tolerance.State)
	})
	t.Run("a pin between samples is never certified clear from the endpoints", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		cube := boxBody(t, doc, 0, 0, 10, 10, 10)
		boxBodyAtZ(t, doc, 14, 4, 16, 6, 4, 2)
		report := verifyMotion(t, doc, []*decad.Body{cube}, alongX(30), endpointsOnly(alongX(30)))

		// The pin sits 4 mm ahead of the cube at rest and 14 mm behind it at
		// the end, and the cube passes straight through it in between.
		require.Empty(t, report.Collisions)
		require.Equal(t, decad.IntervalUndecided, report.Intervals[0].Outcome)
		require.Equal(t, decad.Suspect, report.Status)
	})
}

// TestVerifyMotionSweptBoxExclusion is §9 test 6: a far body changes nothing
// but the static set. It appears in Against and in no pose row, and the
// report's Status and Clearance match the same fixture without it — for the
// arm swing of test 2 and for the clear prismatic case of test 5.
func TestVerifyMotionSweptBoxExclusion(t *testing.T) {
	t.Parallel()
	cases := []struct {
		name   string
		build  func(t *testing.T, doc *decad.Document) *decad.Body
		motion decad.Motion
	}{
		{"arm swing", func(t *testing.T, doc *decad.Document) *decad.Body {
			arm := motionArm(t, doc)
			boxBody(t, doc, -100, 60, 100, 80, 10)
			return arm
		}, armSwing()},
		{"cube slide", func(t *testing.T, doc *decad.Document) *decad.Body {
			cube := boxBody(t, doc, 0, 0, 10, 10, 10)
			lWall(t, doc)
			return cube
		}, alongX(30)},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			plainDoc := decad.New()
			plain := verifyMotion(t, plainDoc, []*decad.Body{tc.build(t, plainDoc)}, tc.motion)

			doc := decad.New()
			mover := tc.build(t, doc)
			far := boxBody(t, doc, 500, -14, 510, 14, 10)
			report := verifyMotion(t, doc, []*decad.Body{mover}, tc.motion)

			require.Contains(t, report.Against, far)
			for _, pose := range report.Poses {
				for _, row := range pose.Clearances {
					require.NotSame(t, far, row.B)
				}
				for _, row := range pose.Interferences {
					require.NotSame(t, far, row.B)
				}
				for _, d := range pose.Diagnostics {
					require.Nil(t, d.Pair)
				}
			}
			require.Equal(t, plain.Status, report.Status)
			if plain.Clearance == nil {
				require.Nil(t, report.Clearance)
				return
			}
			require.NotNil(t, report.Clearance)
			require.Equal(t, plain.Clearance.Measurement, report.Clearance.Measurement)
		})
	}
}

// TestVerifyMotionSweptBoxSettlesASlab: a plain slab ahead of the sliding
// cube lies outside the cube's resting box grown by its whole travel, so the
// swept-box exclusion proves the path clear without evaluating the pair at
// any pose. The interval's Clearance is the exclusion's own lower bound, the
// gap between the two grown boxes, and the path carries no whole-path
// reading because no pose measured an upper bound (§5.3).
//
// Leg seen to fail when deleted: the travel inflation of the mover's box (the
// lower bound rises to the resting gap, 35 mm, above the true minimum 5 mm).
func TestVerifyMotionSweptBoxSettlesASlab(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	cube := boxBody(t, doc, 0, 0, 10, 10, 10)
	boxBodyAtZ(t, doc, 45, -10, 55, 20, -10, 30)
	report := verifyMotion(t, doc, []*decad.Body{cube}, alongX(30))

	require.Equal(t, decad.Sound, report.Status)
	for _, pose := range report.Poses {
		require.Empty(t, pose.Clearances)
		require.Empty(t, pose.Interferences)
		require.Empty(t, pose.Diagnostics)
	}
	require.Len(t, report.Intervals, 1)
	iv := report.Intervals[0]
	require.Equal(t, decad.IntervalClear, iv.Outcome)
	require.NotNil(t, iv.Clearance)
	require.Greater(t, iv.Clearance.Value.Mag(), 0.0)
	require.LessOrEqual(t, iv.Clearance.Value.Mag(), 5.0, `a proven lower bound never exceeds the true minimum`)
	require.Nil(t, report.Clearance)
}

// TestVerifyMotionSweptBoxCoversTravelFromRest pins where the swept box's
// travel is measured from. The arm swings from 80° to 90° into a wall at
// y ∈ [40, 60]: it overlaps the wall at both ends. Inflating the arm's
// resting box by the travel across [From, To] alone — 50 mm × 10° ≈ 8.7 mm —
// would leave it short of the wall and exclude the pair unevaluated, reading
// Sound over two collisions; the box must grow by the travel from where the
// arm rests, 50 mm × 90°.
//
// Leg seen to fail when deleted: measuring the swept box's travel across
// [From, To] instead of from the resting placement.
func TestVerifyMotionSweptBoxCoversTravelFromRest(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	arm := motionArm(t, doc)
	// The wall spans z ∈ [−10, 20], past the arm's own caps at z = 0 and 10:
	// the read-only overlap proof refuses operands sharing a face plane, and
	// an overlap it cannot measure is not a collision (§5.1).
	boxBodyAtZ(t, doc, -20, 40, 20, 60, -10, 30)
	swing := decad.Revolute{Axis: r3.NewVec(0, 0, 1), From: units.Degrees(80), To: units.Degrees(90)}
	report := verifyMotion(t, doc, []*decad.Body{arm}, swing)
	require.Equal(t, decad.Interfering, report.Status)
	require.Len(t, report.Collisions, 2)
}

// TestVerifyMotionNonMutationAndDeterminism is §9 test 7 (interference
// §10.1) for a colliding and an undecided motion: the live body set and its
// order survive the call, and two calls on the same inputs return reports
// equal in every field. The producer identity half lives in
// motion_internal_test.go, which can read it.
func TestVerifyMotionNonMutationAndDeterminism(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	cube := boxBody(t, doc, 0, 0, 10, 10, 10)
	boxBodyAtZ(t, doc, 25, -10, 35, 20, -10, 30)
	arm := boxBody(t, doc, 0, -114, 48, -86, 10)
	boxBody(t, doc, -100, -40, 100, -20, 10)

	for _, run := range []struct {
		moving []*decad.Body
		motion decad.Motion
	}{
		{[]*decad.Body{cube}, alongX(30)},
		{[]*decad.Body{arm}, decad.Revolute{Center: r3.NewVec(0, -100, 0), Axis: r3.NewVec(0, 0, 1), From: units.Degrees(0), To: units.Degrees(90)}},
	} {
		before := doc.Bodies()
		first := verifyMotion(t, doc, run.moving, run.motion)
		second := verifyMotion(t, doc, run.moving, run.motion)
		require.Equal(t, before, doc.Bodies())
		require.Equal(t, first, second)
	}
}

// TestVerifyMotionPoseDeviationIsCharged is §9 test 8: the arm pose at 90°
// stated in degrees, swung once about the origin and once about a pivot a
// million millimetres away, each 10 mm from a wall placed for that pose. The
// far pivot's pose rounds at its magnitude, so its gap bound exceeds the
// origin pivot's, and both stay far inside the tolerance gate for a 50 mm
// body. That the deviation η itself reaches the published gap is pinned by
// motion_internal_test.go, which can compare the pose's gap against the
// kernel's own.
func TestVerifyMotionPoseDeviationIsCharged(t *testing.T) {
	t.Parallel()
	endBound := func(t *testing.T, cx float64) float64 {
		t.Helper()
		doc := decad.New()
		arm := motionArm(t, doc)
		// At 90° about (cx, 0, 0) the arm spans x ∈ [cx−14, cx+14],
		// y ∈ [−cx, −cx+48]; the wall stands 10 mm beyond its far end.
		boxBody(t, doc, cx-20, -cx+58, cx+20, -cx+68, 10)
		swing := decad.Revolute{Center: r3.NewVec(cx, 0, 0), Axis: r3.NewVec(0, 0, 1), From: units.Degrees(0), To: units.Degrees(90)}
		report := verifyMotion(t, doc, []*decad.Body{arm}, swing, endpointsOnly(swing))
		require.Empty(t, motionDiagnostics(report, decad.DiagMeasurementBeyondTolerance))
		end := report.Poses[len(report.Poses)-1]
		require.Len(t, end.Clearances, 1)
		gap := end.Clearances[0].Gap
		require.LessOrEqual(t, gap.Value.Mag()-gap.Bound.Mag(), 10.0)
		require.GreaterOrEqual(t, gap.Value.Mag()+gap.Bound.Mag(), 10.0)
		require.Less(t, gap.Bound.Mag(), 1e-3*10, `the bound passes the default gate at a 10 mm gap`)
		return gap.Bound.Mag()
	}
	near := endBound(t, 0)
	far := endBound(t, 1e6)
	require.Greater(t, near, 0.0)
	require.Greater(t, far, near)
}

// TestVerifyMotionErrors is §9 test 9: one subtest per row of
// docs/motion-check-design.md §8's table the public API can reach, each
// asserting the sentinel, no report, and an unchanged document. The row for
// a body this evaluator did not build is unreachable through the public API
// and lives in motion_internal_test.go.
func TestVerifyMotionErrors(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	arm := motionArm(t, doc)
	boxBody(t, doc, -100, 60, 100, 80, 10)
	retiredSource := boxBody(t, doc, 200, 200, 210, 210, 10)
	translated(t, retiredSource, 0, 0, 100)
	foreign := boxBody(t, decad.New(), 0, 0, 10, 10, 10)
	canceled, cancel := context.WithCancel(t.Context())
	cancel()

	cases := []struct {
		name   string
		ctx    context.Context //nolint:containedctx // a table of per-case contexts.
		moving []*decad.Body
		motion decad.Motion
		opts   []decad.MotionOption
		want   error
	}{
		{name: "empty moving set", motion: armSwing(), want: decad.ErrDegenerate},
		{name: "nil mover", moving: []*decad.Body{nil}, motion: armSwing(), want: decad.ErrDegenerate},
		{name: "mover listed twice", moving: []*decad.Body{arm, arm}, motion: armSwing(), want: decad.ErrDegenerate},
		{name: "retired mover", moving: []*decad.Body{retiredSource}, motion: armSwing(), want: decad.ErrRetiredBody},
		{name: "foreign mover", moving: []*decad.Body{foreign}, motion: armSwing(), want: decad.ErrForeignBody},
		{name: "nil motion", moving: []*decad.Body{arm}, want: decad.ErrDegenerate},
		{name: "nil revolute pointer", moving: []*decad.Body{arm}, motion: (*decad.Revolute)(nil), want: decad.ErrDegenerate},
		{name: "From equals To", moving: []*decad.Body{arm}, motion: decad.Revolute{Axis: r3.NewVec(0, 0, 1), From: units.Degrees(30), To: units.Degrees(30)}, want: decad.ErrDegenerate},
		{name: "From equals To across units", moving: []*decad.Body{arm}, motion: decad.Prismatic{Dir: r3.NewVec(1, 0, 0), From: units.Millimeters(500), To: units.Meters(0.5)}, want: decad.ErrDegenerate},
		{name: "zero degrees equals zero radians", moving: []*decad.Body{arm}, motion: decad.Revolute{Axis: r3.NewVec(0, 0, 1), From: units.Degrees(0), To: units.Radians(0)}, want: decad.ErrDegenerate},
		{name: "axis with no direction", moving: []*decad.Body{arm}, motion: decad.Revolute{From: units.Degrees(0), To: units.Degrees(90)}, want: decad.ErrDegenerate},
		{name: "direction with no direction", moving: []*decad.Body{arm}, motion: decad.Prismatic{From: units.Millimeters(0), To: units.Millimeters(5)}, want: decad.ErrDegenerate},
		{name: "wrong-kind From", moving: []*decad.Body{arm}, motion: decad.Revolute{Axis: r3.NewVec(0, 0, 1), From: units.Millimeters(0), To: units.Degrees(90)}, want: decad.ErrUnitKind},
		{name: "wrong-kind To", moving: []*decad.Body{arm}, motion: decad.Prismatic{Dir: r3.NewVec(1, 0, 0), From: units.Millimeters(0), To: units.Degrees(5)}, want: decad.ErrUnitKind},
		{name: "wrong-kind tolerance", moving: []*decad.Body{arm}, motion: armSwing(), opts: []decad.MotionOption{decad.WithMotionTolerance(units.Millimeters(1))}, want: decad.ErrUnitKind},
		{name: "non-finite parameter", moving: []*decad.Body{arm}, motion: decad.Revolute{Axis: r3.NewVec(0, 0, 1), From: units.Degrees(0), To: units.Degrees(math.Inf(1))}, want: decad.ErrNotFinite},
		{name: "non-finite vector component", moving: []*decad.Body{arm}, motion: decad.Revolute{Center: r3.NewVec(0, math.NaN(), 0), Axis: r3.NewVec(0, 0, 1), From: units.Degrees(0), To: units.Degrees(90)}, want: decad.ErrNotFinite},
		{name: "non-finite tolerance", moving: []*decad.Body{arm}, motion: armSwing(), opts: []decad.MotionOption{decad.WithMotionTolerance(units.Scalar(math.NaN()))}, want: decad.ErrNotFinite},
		{name: "negative tolerance", moving: []*decad.Body{arm}, motion: armSwing(), opts: []decad.MotionOption{decad.WithMotionTolerance(units.Scalar(-1e-3))}, want: decad.ErrNegativeMagnitude},
		{name: "wrong-kind resolution", moving: []*decad.Body{arm}, motion: armSwing(), opts: []decad.MotionOption{decad.WithResolution(units.Millimeters(1))}, want: decad.ErrUnitKind},
		{name: "wrong-kind minimum", moving: []*decad.Body{arm}, motion: armSwing(), opts: []decad.MotionOption{decad.WithMinClearance(units.Degrees(1))}, want: decad.ErrUnitKind},
		{name: "non-finite resolution", moving: []*decad.Body{arm}, motion: armSwing(), opts: []decad.MotionOption{decad.WithResolution(units.Degrees(math.Inf(1)))}, want: decad.ErrNotFinite},
		{name: "non-finite minimum", moving: []*decad.Body{arm}, motion: armSwing(), opts: []decad.MotionOption{decad.WithMinClearance(units.Millimeters(math.NaN()))}, want: decad.ErrNotFinite},
		{name: "negative resolution", moving: []*decad.Body{arm}, motion: armSwing(), opts: []decad.MotionOption{decad.WithResolution(units.Degrees(-1))}, want: decad.ErrNegativeMagnitude},
		{name: "zero resolution", moving: []*decad.Body{arm}, motion: armSwing(), opts: []decad.MotionOption{decad.WithResolution(units.Degrees(0))}, want: decad.ErrNegativeMagnitude},
		{name: "negative minimum", moving: []*decad.Body{arm}, motion: armSwing(), opts: []decad.MotionOption{decad.WithMinClearance(units.Millimeters(-1))}, want: decad.ErrNegativeMagnitude},
		{name: "zero minimum", moving: []*decad.Body{arm}, motion: armSwing(), opts: []decad.MotionOption{decad.WithMinClearance(units.Millimeters(0))}, want: decad.ErrDegenerate},
		{name: "nil motion option", moving: []*decad.Body{arm}, motion: armSwing(), opts: []decad.MotionOption{nil}, want: decad.ErrDegenerate},
		{name: "validation wins over a canceled context", ctx: canceled, moving: []*decad.Body{arm}, motion: decad.Revolute{From: units.Degrees(0), To: units.Degrees(90)}, want: decad.ErrDegenerate},
		{name: "canceled after validation", ctx: canceled, moving: []*decad.Body{arm}, motion: armSwing(), want: context.Canceled},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			ctx := tc.ctx
			if ctx == nil {
				ctx = t.Context()
			}
			before := doc.Bodies()
			report, err := doc.VerifyMotion(ctx, tc.moving, tc.motion, tc.opts...)
			require.ErrorIs(t, err, tc.want)
			require.Nil(t, report)
			require.Equal(t, before, doc.Bodies())
		})
	}
	t.Run("nil document", func(t *testing.T) {
		var nilDoc *decad.Document
		report, err := nilDoc.VerifyMotion(t.Context(), []*decad.Body{arm}, armSwing())
		require.ErrorIs(t, err, decad.ErrDegenerate)
		require.Nil(t, report)
	})
}

// TestVerifyMotionCancellation is §9 test 10: a context canceled at
// successively later checks after validation — through the endpoints and on
// through the bisection of test 2's swing, the step growing by half each
// time to keep the race build quick — returns ctx.Err() and no report, and
// leaves the document unchanged.
func TestVerifyMotionCancellation(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	arm := motionArm(t, doc)
	boxBody(t, doc, -100, 60, 100, 80, 10)
	before := doc.Bodies()
	var completed *decad.MotionReport
	canceled := 0
	for limit := int32(1); limit <= 1<<24; limit += 1 + limit/2 {
		ctx := newCancelAfterContext(t.Context(), limit)
		report, err := doc.VerifyMotion(ctx, []*decad.Body{arm}, armSwing())
		require.Equal(t, before, doc.Bodies())
		if err == nil {
			completed = report
			break
		}
		require.ErrorIs(t, err, context.Canceled)
		require.Nil(t, report)
		canceled++
	}
	require.NotNil(t, completed, `the check finishes once the context outlasts it`)
	require.Greater(t, len(completed.Poses), 2, `the probed checks reach into bisection`)
	require.Greater(t, canceled, 10)
}
