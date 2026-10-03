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

// This file holds the required tests of docs/motion-check-design.md §9 that
// the endpoint-only check can carry, numbered as §9 numbers them. Bounds are
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

func verifyMotion(t *testing.T, doc *decad.Document, moving []*decad.Body, m decad.Motion) *decad.MotionReport {
	t.Helper()
	before := doc.Bodies()
	report, err := doc.VerifyMotion(t.Context(), moving, m)
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

// TestVerifyMotionClearSwingEndpoints is §9 test 2's fixture at the scope the
// endpoint-only check reaches: the arm swinging 0°→90° past a wall at
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
	report := verifyMotion(t, doc, []*decad.Body{arm}, armSwing())

	require.Equal(t, decad.Suspect, report.Status)
	require.False(t, report.Passed())
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

// TestVerifyMotionTouchingStart is §9 test 4's fixture: the arm resting on a
// stop prism that shares its y = −14 face plane at θ = 0, swinging away.
//
// The first pose's row is the coplanar certificate's Exact zero: the pose at
// 0° is the identity motion, so the transient arm is placed exactly where it
// rests and carries no placement rounding.
//
// §9 also asks the first interval to read IntervalClear and the report Sound.
// No sound check can give that answer, so this test pins the opposite. The
// interval would have to certify lo_0 + lo_1 > τ with lo_0 = 0, that is
// lo_1 > τ; but lo_1 is a lower bound on the gap at the far pose, and that gap
// is at most the start gap, 0, plus the farthest any arm point travels, which
// is at most τ. So lo_1 ≤ τ always, and an interval that starts touching can
// never certify — the closed interval contains a pose at zero distance, and
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

	require.Len(t, report.Poses, 2)
	first := report.Poses[0]
	require.Len(t, first.Clearances, 1)
	require.Same(t, stop, first.Clearances[0].B)
	require.Equal(t, decad.Exact, first.Clearances[0].Gap.Exactness)
	require.Zero(t, first.Clearances[0].Gap.Value.Mag())
	require.Zero(t, first.Clearances[0].Gap.Bound.Mag())
	require.Empty(t, report.Collisions, `touching is not overlap`)

	require.Len(t, report.Intervals, 1)
	require.Equal(t, decad.IntervalUndecided, report.Intervals[0].Outcome)
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
			require.NotNil(t, c.Volume)
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
	t.Run("a pin between samples is never certified clear", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		cube := boxBody(t, doc, 0, 0, 10, 10, 10)
		boxBodyAtZ(t, doc, 14, 4, 16, 6, 4, 2)
		report := verifyMotion(t, doc, []*decad.Body{cube}, alongX(30))

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
	boxBody(t, doc, -20, 40, 20, 60, 10)
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
		report := verifyMotion(t, doc, []*decad.Body{arm}, swing)
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

// TestVerifyMotionCancellation is §9 test 10 at the endpoint-only scope: a
// context canceled at successively later checks after validation — through
// both poses' placements, kernels and overlap proofs, the step growing by a
// quarter each time to keep the race build quick — returns ctx.Err() and no
// report, and leaves the document unchanged.
func TestVerifyMotionCancellation(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	cube := boxBody(t, doc, 0, 0, 10, 10, 10)
	boxBodyAtZ(t, doc, 25, -10, 35, 20, -10, 30)
	before := doc.Bodies()
	completed := false
	for limit := int32(1); limit <= 1<<20; limit += 1 + limit/4 {
		ctx := newCancelAfterContext(t.Context(), limit)
		report, err := doc.VerifyMotion(ctx, []*decad.Body{cube}, alongX(30))
		require.Equal(t, before, doc.Bodies())
		if err == nil {
			require.NotNil(t, report)
			completed = true
			break
		}
		require.ErrorIs(t, err, context.Canceled)
		require.Nil(t, report)
	}
	require.True(t, completed, `the check finishes once the context outlasts it`)
}
