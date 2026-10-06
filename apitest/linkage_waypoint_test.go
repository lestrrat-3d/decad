package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file holds scene 5 of docs/linkage-check-design.md §11: drives that
// pass through waypoints (§2.3). A drive whose sweeps carry n − 1 Via values
// has n segments of equal share, segment j covering s ∈ [j/n, (j+1)/n], and
// each joint runs linearly between consecutive waypoints. The internal test
// of the travel bound across a waypoint sits in linkage_internal_test.go.

// requireNoClearIntervalHolds asserts no IntervalClear interval of the
// report holds s.
func requireNoClearIntervalHolds(t *testing.T, report *decad.LinkageReport, s float64) {
	t.Helper()
	for _, iv := range report.Intervals {
		if iv.From.Mag() <= s && s <= iv.To.Mag() {
			require.NotEqual(t, decad.IntervalClear, iv.Outcome, `an interval holding a colliding pose is never clear`)
		}
	}
}

// liftSwingLower is scene 5's arm. A hub x, y ∈ [−5, 5], z ∈ [−20, −12]
// turns about Z through the origin; under it an arm x ∈ [0, 50],
// y ∈ [−5, 5], z ∈ [0, 10] slides along +Z. The drive has three segments:
// the arm lifts 0 → 20 mm (s ∈ [0, 1/3]), swings 0° → 90° (s ∈ [1/3, 2/3])
// and lowers 20 → 0 mm (s ∈ [2/3, 1]). A post x, y ∈ [17, 25],
// z ∈ [−10, 15] stands in the swing's path 5 mm below the lifted arm, and a
// landing block x ∈ [−20, 20], y ∈ [30, 40], z ∈ [−10, 5] sits under the
// arm's final place.
type liftSwingLower struct {
	doc                 *decad.Document
	hub, arm            *decad.Body
	post, landing       *decad.Body
	linkage             *decad.Linkage
	swing, lift         *decad.Link
	swingSweep, liftDue decad.JointSweep
}

func buildLiftSwingLower(t *testing.T) liftSwingLower {
	t.Helper()
	c := liftSwingLower{doc: decad.New()}
	c.hub = boxBodyAtZ(t, c.doc, -5, -5, 5, 5, -20, 8)
	c.arm = boxBody(t, c.doc, 0, -5, 50, 5, 10)
	c.post = boxBodyAtZ(t, c.doc, 17, 17, 25, 25, -10, 25)
	c.landing = boxBodyAtZ(t, c.doc, -20, 30, 20, 40, -10, 15)
	c.linkage = decad.NewLinkage()
	var err error
	c.swing, err = c.linkage.Ground().Revolute(r3.Vec{}, zAxis, []*decad.Body{c.hub})
	require.NoError(t, err)
	c.lift, err = c.swing.Prismatic(zAxis, []*decad.Body{c.arm})
	require.NoError(t, err)
	c.swingSweep = decad.JointSweep{Link: c.swing, From: units.Degrees(0), Via: []units.Value{units.Degrees(0), units.Degrees(90)}, To: units.Degrees(90)}
	c.liftDue = decad.JointSweep{Link: c.lift, From: units.Millimeters(0), Via: []units.Value{units.Millimeters(20), units.Millimeters(20)}, To: units.Millimeters(0)}
	return c
}

// liftSwingLowerValues is the closed-form schedule of scene 5 at s: the
// swing angle in degrees and the lift in millimetres.
func liftSwingLowerValues(s float64) (float64, float64) {
	switch {
	case s <= 1.0/3:
		return 0, 60 * s
	case s <= 2.0/3:
		return 270 * (s - 1.0/3), 20
	}
	return 90, 60 * (1 - s)
}

// TestVerifyLinkageWaypoints is scene 5 of docs/linkage-check-design.md §11.
//
// Lift, swing, lower: the lowering arm's underside stands at z = 60·(1 − s)
// over the landing block's top face z = 5, which it reaches at s* = 11/12;
// the arm then overlaps the block in its 10 × 10 mm footprint, so the overlap
// is 100·(5 − 60·(1 − s)) mm³. The post stays 5 mm below the lifted arm. The
// same arm swinging unlifted strikes the post.
//
// Collinear waypoints: Via values on the straight line from From to To leave
// the path, every exact joint value, every label at a dyadic fraction and
// every travel bound what the one-segment drive has, so the reports agree in
// every field but Drive.
//
// A joint turning back: a blade x ∈ [0, 50], y ∈ [−0.5, 0.5], z ∈ [0, 10]
// turns about Z through 0° → 60° → 10° → 20°, and a 0.8 mm pin at radius 49
// and polar angle 59.5° sits in its path only near the 60° corner at
// s = 1/3, which no dyadic grid point reaches. Contact needs
// |q − 59.5°| ≤ asin((0.5 + 0.4·√2)/49) < 1.25°, so every collision lies in
// s ∈ [58.25/180, 1/3 + 1.75/150], and the corner pose collides.
//
// Legs seen to fail when deleted: summing the joint's travel over the pieces
// a waypoint cuts an interval into (in the endpoint-only subtest the blade's
// ends differ by 20°, [0, 1] certifies from them and the report reads Sound
// with two poses); and the Via values in the joint's reach (the corner
// subtest's swept box, grown by 20°, excludes the pin and reads Sound).
func TestVerifyLinkageWaypoints(t *testing.T) {
	t.Parallel()
	t.Run("lift swing lower reaches the landing block at eleven twelfths", func(t *testing.T) {
		t.Parallel()
		c := buildLiftSwingLower(t)
		drive := decad.Drive{c.swingSweep, c.liftDue}
		report := verifyLinkage(t, c.doc, c.linkage, drive, decad.WithResolution(units.Scalar(1.0/256)))
		sStar := 11.0 / 12
		requireFirstCollisionAbove(t, report, sStar, c.arm, c.landing)
		requirePosesArePoseAt(t, report)
		first := report.Collisions[0]
		require.Equal(t, units.Scalar(235.0/256), first.At, `the first grid point above s* = 11/12`)
		require.InDelta(t, 100*(5-60*(1-first.At.Mag())), first.Volume.Value.Mag(), 1e-6)
		for _, hit := range report.Collisions {
			require.Same(t, c.landing, hit.B, `the lifted arm clears the post`)
		}
		// The travel bound sums each joint's motion on both sides of a
		// waypoint, and an interval across one still certifies where the
		// gaps cover it.
		straddled := 0
		for _, iv := range report.Intervals {
			for _, w := range []float64{1.0 / 3, 2.0 / 3} {
				if iv.From.Mag() < w && w < iv.To.Mag() && iv.Outcome == decad.IntervalClear {
					straddled++
				}
			}
		}
		require.Equal(t, 2, straddled, `each interior waypoint lies inside a certified interval`)
		for _, p := range report.Poses {
			swing, lift := liftSwingLowerValues(p.Pose.At.Mag())
			require.InDelta(t, swing, p.Pose.Values[0].Mag(), 1e-12)
			require.InDelta(t, lift, p.Pose.Values[1].Mag(), 1e-12)
		}
	})
	t.Run("the same swing unlifted strikes the post", func(t *testing.T) {
		t.Parallel()
		c := buildLiftSwingLower(t)
		report := verifyLinkage(t, c.doc, c.linkage, decad.Drive{c.swingSweep}, decad.WithResolution(units.Scalar(1.0/64)))
		require.Equal(t, decad.Interfering, report.Status)
		require.NotEmpty(t, report.Collisions)
		require.Same(t, c.arm, report.Collisions[0].A)
		require.Same(t, c.post, report.Collisions[0].B)
	})
	t.Run("collinear waypoints reproduce the one-segment drive", func(t *testing.T) {
		t.Parallel()
		a := buildFoldingArm(t, true)
		opt := decad.WithResolution(units.Scalar(1.0 / 256))
		plain := verifyLinkage(t, a.doc, a.linkage, a.drive(), opt)
		for _, tc := range []struct {
			name            string
			shoulder, elbow []units.Value
		}{
			{"one interior waypoint", []units.Value{units.Degrees(45)}, []units.Value{units.Degrees(-45)}},
			{"two interior waypoints", []units.Value{units.Degrees(30), units.Degrees(60)}, []units.Value{units.Degrees(-30), units.Degrees(-60)}},
		} {
			drive := a.drive()
			drive[0].Via, drive[1].Via = tc.shoulder, tc.elbow
			got := verifyLinkage(t, a.doc, a.linkage, drive, opt)
			require.Equal(t, drive, got.Drive, tc.name)
			got.Drive = plain.Drive
			require.Equal(t, plain, got, tc.name)
		}
	})
	turningBlade := func(t *testing.T) (*decad.Document, *decad.Linkage, decad.Drive, *decad.Body, *decad.Body) {
		t.Helper()
		doc := decad.New()
		blade := boxBody(t, doc, 0, -0.5, 50, 0.5, 10)
		alpha := 59.5 * math.Pi / 180
		cx, cy := 49*math.Cos(alpha), 49*math.Sin(alpha)
		pin := boxBodyAtZ(t, doc, cx-0.4, cy-0.4, cx+0.4, cy+0.4, 4.6, 0.8)
		l := decad.NewLinkage()
		turn, err := l.Ground().Revolute(r3.Vec{}, zAxis, []*decad.Body{blade})
		require.NoError(t, err)
		return doc, l, decad.Drive{{
			Link: turn, From: units.Degrees(0), Via: []units.Value{units.Degrees(60), units.Degrees(10)}, To: units.Degrees(20),
		}}, blade, pin
	}
	t.Run("a joint turning back at a waypoint is never certified across the corner", func(t *testing.T) {
		t.Parallel()
		doc, l, drive, blade, pin := turningBlade(t)
		corner, err := l.PoseAt(drive, units.Scalar(1.0/3))
		require.NoError(t, err)
		require.InDelta(t, 60, corner.Values[0].Mag(), 1e-12)

		report := verifyLinkage(t, doc, l, drive, decad.WithResolution(units.Scalar(1.0/128)))
		require.Equal(t, decad.Interfering, report.Status)
		require.NotEmpty(t, report.Collisions)
		for _, hit := range report.Collisions {
			require.Same(t, blade, hit.A)
			require.Same(t, pin, hit.B)
			require.GreaterOrEqual(t, hit.At.Mag(), 58.25/180)
			require.LessOrEqual(t, hit.At.Mag(), 1.0/3+1.75/150)
		}
		requireNoClearIntervalHolds(t, report, 1.0/3)
	})
	t.Run("an endpoint-only reading is never enough", func(t *testing.T) {
		t.Parallel()
		// The two ends of the drive are clear of the pin and 20° apart; only
		// the travel through both waypoints keeps [0, 1] from certifying. At
		// the floor 1/2, [1/2, 1] turns 35° → 10° → 20° far from the pin and
		// certifies, while [0, 1/2] holds the corner and stays undecided.
		doc, l, drive, _, _ := turningBlade(t)
		report := verifyLinkage(t, doc, l, drive, decad.WithResolution(units.Scalar(1.0/2)))
		require.Equal(t, decad.Suspect, report.Status)
		require.Empty(t, report.Collisions)
		require.Len(t, report.Poses, 3)
		require.Equal(t, decad.IntervalUndecided, report.Intervals[0].Outcome)
		require.Equal(t, decad.IntervalClear, report.Intervals[1].Outcome)
	})
}

// TestVerifyLinkageWaypointReach pins the two readings a waypoint enters
// besides the travel bound. The motion arm, x ∈ [0, 48], y ∈ [−14, 14],
// z ∈ [0, 10], turns about Z through the origin; a wall x ∈ [−20, 20],
// y ∈ [30, 40], z ∈ [−10, 20] stands where the arm points at 90°, and the
// arm's ends, at 20° or less, stay clear of it.
//
//   - 0° → 90° → 10°: the swept box grows by the farthest waypoint's reach,
//     90°, so the wall is evaluated and struck at s = 1/2. Reading the reach
//     from From and To alone grows the box by 10°'s 8.7 mm, excludes the wall
//     and reads Sound.
//   - 0° → 90° → 0°: both ends are 0, yet the joint moves; reading a link as
//     held at 0 from From and To alone leaves its pair with the wall
//     unformed and reads Sound, and reading the drive as a hold from them
//     refuses it.
//   - 20° → 90° → 20°: both ends are 20°; reading the link as one constant
//     pose from From and To alone measures it at 20° at every s and reads
//     Sound.
//
// Legs seen to fail when deleted: the Via values in the joint's reach, in the
// held-at-zero test, in the drive's hold test and in the link's constant
// test, each in its subtest.
func TestVerifyLinkageWaypointReach(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name     string
		from, to units.Value
	}{
		{"a waypoint beyond both ends grows the swept box", units.Degrees(0), units.Degrees(10)},
		{"a joint out and back from zero is not held", units.Degrees(0), units.Degrees(0)},
		{"a joint out and back from 20° is not one constant pose", units.Degrees(20), units.Degrees(20)},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc := decad.New()
			arm := boxBody(t, doc, 0, -14, 48, 14, 10)
			wall := boxBodyAtZ(t, doc, -20, 30, 20, 40, -10, 30)
			l := decad.NewLinkage()
			turn, err := l.Ground().Revolute(r3.Vec{}, zAxis, []*decad.Body{arm})
			require.NoError(t, err)
			drive := decad.Drive{{Link: turn, From: tc.from, Via: []units.Value{units.Degrees(90)}, To: tc.to}}
			report := verifyLinkage(t, doc, l, drive, decad.WithResolution(units.Scalar(1.0/2)))
			require.Equal(t, decad.Interfering, report.Status)
			require.Len(t, report.Poses, 3)
			require.Equal(t, units.Degrees(90), report.Poses[1].Pose.Values[0])
			require.Len(t, report.Collisions, 1)
			hit := report.Collisions[0]
			require.Same(t, arm, hit.A)
			require.Same(t, wall, hit.B)
			require.Equal(t, units.Scalar(0.5), hit.At)
			// At 90° the arm covers x ∈ [−14, 14], y ∈ [0, 48], z ∈ [0, 10],
			// and the wall's slab x ∈ [−14, 14], y ∈ [30, 40] of it.
			require.InDelta(t, 28*10*10, hit.Volume.Value.Mag(), 1e-6)
		})
	}
}

// TestLinkagePoseAtWaypoints pins PoseAt's schedule on a four-segment drive,
// whose waypoints sit at the dyadic fractions 1/4, 1/2 and 3/4: the scene 5
// arm lifts 0 → 20 mm, holds, swings 0° → 90°, then holds both. At a
// waypoint's fraction each value is the waypoint as stated; inside a segment
// it is the segment's linear interpolation; outside [0, 1] the first and last
// segments' lines extend.
func TestLinkagePoseAtWaypoints(t *testing.T) {
	t.Parallel()
	c := buildLiftSwingLower(t)
	drive := decad.Drive{
		{Link: c.swing, From: units.Degrees(0), Via: []units.Value{units.Degrees(0), units.Degrees(0), units.Degrees(90)}, To: units.Degrees(90)},
		{Link: c.lift, From: units.Millimeters(0), Via: []units.Value{units.Millimeters(20), units.Millimeters(20), units.Millimeters(20)}, To: units.Centimeters(2)},
	}
	for _, tc := range []struct {
		at          float64
		swing, lift units.Value
	}{
		{0.25, units.Degrees(0), units.Millimeters(20)},
		{0.5, units.Degrees(0), units.Millimeters(20)},
		{0.625, units.Degrees(45), units.Millimeters(20)},
		{0.75, units.Degrees(90), units.Millimeters(20)},
		{0.125, units.Degrees(0), units.Millimeters(10)},
		{1, units.Degrees(90), units.Centimeters(2)},
		{-0.25, units.Degrees(0), units.Millimeters(-20)},
		// Past 1 the last segment's line, 20 mm to 2 cm, is carried in its
		// starting waypoint's unit.
		{1.25, units.Degrees(90), units.Millimeters(20)},
	} {
		pose, err := c.linkage.PoseAt(drive, units.Scalar(tc.at))
		require.NoError(t, err)
		require.Equal(t, tc.swing, pose.Values[0], "swing at %v", tc.at)
		require.Equal(t, tc.lift, pose.Values[1], "lift at %v", tc.at)
	}
	pose, err := c.linkage.PoseAt(drive, units.Scalar(0.625))
	require.NoError(t, err)
	tip := pose.Poses[1].Apply(r3.NewVec(50, 0, 10))
	h := 50 * math.Sqrt2 / 2
	require.InDelta(t, h, tip.X, 1e-12)
	require.InDelta(t, h, tip.Y, 1e-12)
	require.InDelta(t, 30, tip.Z, 1e-12)
}
