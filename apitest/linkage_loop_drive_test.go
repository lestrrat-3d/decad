package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file holds the loop-driver tests of docs/linkage-check-design.md
// §15.10 that L3 lands: waypoints on a loop's driver, a driver crossing 0, a
// held driver, limits on a dependent joint, and scene 9 driven out and back
// through its fold. Every closed form is the four-bar's two-circle
// construction or the slider-crank's, in float64.

// requireRockerValues asserts every pose's dependent values are scene 7's
// closed forms at the crank angle th2(s) the drive states.
func requireRockerValues(t *testing.T, poses []decad.LinkagePose, th2 func(s float64) float64) {
	t.Helper()
	theta40, theta30 := rockerTheta4(0), rockerCouplerAngle(0)
	for _, p := range poses {
		th := th2(p.At.Mag())
		require.InDelta(t, rockerTheta4(th)-theta40, p.Values[2].Mag(), 1e-9, `the follower at s = %v`, p.At)
		require.InDelta(t, rockerCouplerAngle(th)-th-theta30, p.Values[1].Mag(), 1e-9, `the coupler at s = %v`, p.At)
		require.Less(t, p.Bounds[2].Mag(), 1e-9)
	}
}

// schedulePoses is a schedule's pose at every fraction given.
func schedulePoses(t *testing.T, l *decad.Linkage, drive decad.Drive, at ...float64) []decad.LinkagePose {
	t.Helper()
	sched, err := l.Schedule(t.Context(), drive)
	require.NoError(t, err)
	out := make([]decad.LinkagePose, len(at))
	for n, s := range at {
		out[n], err = sched.PoseAt(t.Context(), units.Scalar(s))
		require.NoError(t, err)
	}
	return out
}

// sixteenths is every fraction k/16 of a drive, and extra.
func sixteenths(extra ...float64) []float64 {
	out := make([]float64, 0, 17+len(extra))
	for k := range 17 {
		out = append(out, float64(k)/16)
	}
	return append(out, extra...)
}

// reportPoses is a report's evaluated poses.
func reportPoses(report *decad.LinkageReport) []decad.LinkagePose {
	out := make([]decad.LinkagePose, len(report.Poses))
	for n, p := range report.Poses {
		out[n] = p.Pose
	}
	return out
}

// TestVerifyLinkageLoopWaypoints drives scene 7's crank through a waypoint:
// 0° → 60° → 20°. The second segment's driver value falls toward 0, so its
// chain starts at its far end, s = 1, and the two segments meet at the
// waypoint s = 1/2. Every pose, the waypoint's included, reads the closed form
// at its own crank angle.
//
// Legs seen to fail when deleted: each sub-segment's own near end — a chain
// started at the segment's first waypoint asks the falling segment's cells
// downward, sketch refuses each, and the poses past 1/2 are unbuildable.
func TestVerifyLinkageLoopWaypoints(t *testing.T) {
	t.Parallel()
	fb, _ := buildRocker(t, false)
	drive := decad.Drive{{Link: fb.crank, From: units.Degrees(0), Via: []units.Value{units.Degrees(60)}, To: units.Degrees(20)}}
	report := verifyLinkage(t, fb.doc, fb.linkage, drive, decad.WithResolution(units.Scalar(1.0/16)))
	require.Equal(t, decad.Sound, report.Status, `the layer exclusion settles every pair`)
	th2 := func(s float64) float64 {
		if s <= 0.5 {
			return 120 * s * math.Pi / 180
		}
		return (60 - 80*(s-0.5)) * math.Pi / 180
	}
	requireRockerValues(t, reportPoses(report), th2)
	requireRockerValues(t, schedulePoses(t, fb.linkage, drive, sixteenths(1.0/3, 2.0/3)...), th2)
}

// TestVerifyLinkageLoopOutAndBack drives scene 7's crank 0° → 30° → 0°, with a
// 0.8 mm pin in the follower's layer centred on the follower's top corner at
// the waypoint, θ2 = 30°. Evaluated at its endpoints alone, both at the zero
// pose, the one interval holds the waypoint, and the follower's travel over
// it is the sum over its two pieces, each read in its own sub-segment's chain:
// about twice the corner's 10.2 mm arc against the two endpoint gaps' sum of
// about 19.2 mm, so it stays undecided. With the waypoint evaluated the pin is
// struck there.
//
// Leg seen to fail when deleted: summing the dependent's travel over every
// piece — one piece alone halves it, the interval certifies, and the report
// reads Sound past a collision.
func TestVerifyLinkageLoopOutAndBack(t *testing.T) {
	t.Parallel()
	build := func(t *testing.T) (fourBar, *decad.Body, decad.Drive) {
		fb, _ := buildRocker(t, false)
		cx, cy := rockerCorner(30 * math.Pi / 180)
		pin := boxBodyAtZ(t, fb.doc, cx-0.4, cy-0.4, cx+0.4, cy+0.4, 23.6, 0.8)
		drive := decad.Drive{{Link: fb.crank, From: units.Degrees(0), Via: []units.Value{units.Degrees(30)}, To: units.Degrees(0)}}
		return fb, pin, drive
	}
	t.Run("endpoints alone", func(t *testing.T) {
		t.Parallel()
		fb, _, drive := build(t)
		report := verifyLinkage(t, fb.doc, fb.linkage, drive, decad.WithResolution(units.Scalar(1)))
		require.Equal(t, decad.Suspect, report.Status)
		require.Len(t, report.Poses, 2)
		require.Equal(t, decad.IntervalUndecided, report.Intervals[0].Outcome)
		for _, p := range report.Poses {
			require.InDelta(t, 0, p.Pose.Values[2].Mag(), 1e-9, `both ends stand at the zero pose`)
		}
	})
	t.Run("the waypoint evaluated", func(t *testing.T) {
		t.Parallel()
		fb, pin, drive := build(t)
		report := verifyLinkage(t, fb.doc, fb.linkage, drive, decad.WithResolution(units.Scalar(1.0/2)))
		require.Equal(t, decad.Interfering, report.Status)
		require.Equal(t, units.Scalar(0.5), report.Collisions[0].At)
		require.Same(t, pin, report.Collisions[0].B)
		requireRockerValues(t, reportPoses(report), func(s float64) float64 { return (30 - 60*math.Abs(s-0.5)) * math.Pi / 180 })
	})
}

// TestVerifyLinkageLoopCrossingZero drives scene 7's crank −30° → 60°, across
// 0 at the non-dyadic s = 1/3. The stretch below reads on the mirrored scene,
// the stretch above on the scene's own side, each chain starting at the
// crossing's zero pose; every pose reads the closed form on its side.
//
// Leg seen to fail when deleted: reading each stretch on its own side — the
// negative stretch is then asked on the scene's own side as |q|, and reads the
// follower at +30° where the crank stands at −30°.
func TestVerifyLinkageLoopCrossingZero(t *testing.T) {
	t.Parallel()
	fb, _ := buildRocker(t, false)
	drive := decad.Drive{{Link: fb.crank, From: units.Degrees(-30), To: units.Degrees(60)}}
	report := verifyLinkage(t, fb.doc, fb.linkage, drive, decad.WithResolution(units.Scalar(1.0/16)))
	require.Equal(t, decad.Sound, report.Status)
	th2 := func(s float64) float64 { return (-30 + 90*s) * math.Pi / 180 }
	requireRockerValues(t, reportPoses(report), th2)
	poses := schedulePoses(t, fb.linkage, drive, sixteenths(1.0/3)...)
	requireRockerValues(t, poses, th2)
	require.InDelta(t, 14.583238, poses[0].Values[2].Mag()*180/math.Pi, 1e-6)

	third, err := fb.linkage.PoseAt(drive, units.Scalar(1.0/3))
	require.NoError(t, err)
	require.InDelta(t, 0, third.Values[2].Mag(), 1e-12, `the crossing is the zero pose`)
}

// TestVerifyLinkageLoopCrossingZeroMixed drives scene 7's crank from −30°, a
// whole-turn fraction, to 1 rad, across 0 at the irrational
// s₀ = (π/6)/(1 + π/6). The segment is cut at two rationals bracketing s₀:
// the stretch below reads on the mirrored scene, the one above on the scene's
// own side, each chain starting at its cut, and the straddle between the cuts
// holds both sides' values near the zero pose. Every pose reads the closed
// form at its own crank angle.
//
// Leg seen to fail when deleted: reading each stretch on its own side — the
// stretch below s₀ then reads the follower at +30° where the crank stands at
// −30°.
func TestVerifyLinkageLoopCrossingZeroMixed(t *testing.T) {
	t.Parallel()
	fb, _ := buildRocker(t, false)
	drive := decad.Drive{{Link: fb.crank, From: units.Degrees(-30), To: units.Radians(1)}}
	report := verifyLinkage(t, fb.doc, fb.linkage, drive, decad.WithResolution(units.Scalar(1.0/16)))
	require.Equal(t, decad.Sound, report.Status)
	th2 := func(s float64) float64 { return -math.Pi/6 + s*(1+math.Pi/6) }
	requireRockerValues(t, reportPoses(report), th2)
	s0 := (math.Pi / 6) / (1 + math.Pi/6)
	poses := schedulePoses(t, fb.linkage, drive, sixteenths(s0)...)
	requireRockerValues(t, poses, th2)
	require.InDelta(t, 14.583238, poses[0].Values[2].Mag()*180/math.Pi, 1e-6)
	require.InDelta(t, 0, poses[len(poses)-1].Values[2].Mag(), 1e-12, `the float nearest s₀ stands at the zero pose`)
}

// TestVerifyLinkageLoopSlideCrossingZero drives TestVerifyLinkageLoopSlideDriven's
// slider-crank at its slide from −5 to +5 mm, across 0 at s = 1/2: the
// backward stretch on the half-turned scene, the forward one on the scene's
// own side.
//
// Leg seen to fail when deleted: the half-turned side for the backward
// stretch.
func TestVerifyLinkageLoopSlideCrossingZero(t *testing.T) {
	t.Parallel()
	px := math.Sqrt(5500)
	doc := decad.New()
	l := decad.NewLinkage()
	z := r3.NewVec(0, 0, 1)
	crank, err := l.Ground().Revolute(r3.Vec{}, z, []*decad.Body{boxBodyAtZ(t, doc, -3, 0, 3, 30, 0, 8)})
	require.NoError(t, err)
	rod, err := crank.Revolute(r3.NewVec(0, 30, 0), z, []*decad.Body{barBody(t, doc, [2]float64{0, 30}, [2]float64{px, 0}, 10)})
	require.NoError(t, err)
	slider, err := l.Ground().Prismatic(r3.NewVec(1, 0, 0), []*decad.Body{boxBodyAtZ(t, doc, px-5, -5, px+5, 5, 20, 8)})
	require.NoError(t, err)
	_, err = l.Close(rod, slider, r3.NewVec(px, 0, 0), z)
	require.NoError(t, err)
	drive := decad.Drive{{Link: slider, From: units.Millimeters(-5), To: units.Millimeters(5)}}
	sched, err := l.Schedule(t.Context(), drive)
	require.NoError(t, err)
	for _, at := range []float64{0, 0.25, 0.5, 0.75, 1} {
		pose, err := sched.PoseAt(t.Context(), units.Scalar(at))
		require.NoError(t, err)
		x := px - 5 + 10*at
		phi := bisectRoot(func(phi float64) float64 {
			return 30*math.Cos(phi) + math.Sqrt(6400-900*math.Sin(phi)*math.Sin(phi)) - x
		}, math.Pi/3, 2*math.Pi/3)
		require.InDelta(t, phi-math.Pi/2, pose.Values[0].Mag(), 1e-9, `the crank at s = %v`, at)
	}
}

// TestVerifyLinkageLoopHeldDriver: a loop whose driver holds a nonzero value
// is one point ask after the approach. Scene 7's crank held at 30° while a
// separate arm off the loop turns: every pose reads the closed form at 30°,
// and the loop's links stand at one placement. A drive that holds the crank
// over its middle segment, 0° → 30° → 30° → 0°, reads 30° throughout that
// segment.
//
// Leg seen to fail when deleted: one point per held stretch — the stretch's
// cells otherwise ask an empty range, which sketch refuses, and its poses are
// unbuildable.
func TestVerifyLinkageLoopHeldDriver(t *testing.T) {
	t.Parallel()
	thirty := 30 * math.Pi / 180
	t.Run("held for the whole drive", func(t *testing.T) {
		t.Parallel()
		fb, _ := buildRocker(t, false)
		arm, err := fb.linkage.Ground().Revolute(r3.NewVec(300, 0, 0), r3.NewVec(0, 0, 1), []*decad.Body{boxBodyAtZ(t, fb.doc, 300, -4, 340, 4, 40, 8)})
		require.NoError(t, err)
		drive := decad.Drive{
			{Link: fb.crank, From: units.Degrees(30), To: units.Degrees(30)},
			{Link: arm, From: units.Degrees(0), To: units.Degrees(90)},
		}
		report := verifyLinkage(t, fb.doc, fb.linkage, drive, decad.WithResolution(units.Scalar(1.0/4)))
		require.Equal(t, decad.Sound, report.Status)
		poses := schedulePoses(t, fb.linkage, drive, sixteenths()...)
		requireRockerValues(t, append(poses, reportPoses(report)...), func(float64) float64 { return thirty })
		for _, p := range poses {
			require.Equal(t, poses[0].Poses[:3], p.Poses[:3], `the loop's links stand at one placement`)
			require.Positive(t, p.Bounds[2].Mag())
		}
	})
	t.Run("held over a segment", func(t *testing.T) {
		t.Parallel()
		fb, _ := buildRocker(t, false)
		drive := decad.Drive{{Link: fb.crank, From: units.Degrees(0), Via: []units.Value{units.Degrees(30), units.Degrees(30)}, To: units.Degrees(0)}}
		report := verifyLinkage(t, fb.doc, fb.linkage, drive, decad.WithResolution(units.Scalar(1.0/16)))
		require.Equal(t, decad.Sound, report.Status)
		requireRockerValues(t, schedulePoses(t, fb.linkage, drive, sixteenths(1.0/3, 2.0/3)...), func(s float64) float64 {
			switch {
			case s <= 1.0/3:
				return 90 * s * math.Pi / 180
			case s <= 2.0/3:
				return thirty
			}
			return 90 * (1 - s) * math.Pi / 180
		})
	})
}

// TestVerifyLinkageLoopDependentLimits: a dependent joint's limits are held to
// its whole-drive hull. Under scene 7's crank 0° → 90° the follower turns
// over [−8.763°, 3.025°]: limits of [−5°, 5°] refuse the drive naming the
// follower, [−10°, 5°] admit it. Limits that exclude the zero pose,
// [−10°, −1°], admit the crank 20° → 50°, over which the follower stays in
// [−8.763°, −6.690°]: a dependent is never held at 0.
//
// Leg seen to fail when deleted: the hull check — the [−5°, 5°] drive is then
// admitted.
func TestVerifyLinkageLoopDependentLimits(t *testing.T) {
	t.Parallel()
	limited := func(t *testing.T, lo, hi float64) fourBar {
		doc := decad.New()
		z := r3.NewVec(0, 0, 1)
		fb := buildFourBarOpen(t, doc, z, decad.WithJointLimits(units.Degrees(lo), units.Degrees(hi)))
		var err error
		fb.loop, err = fb.linkage.Close(fb.couplerLk, fb.follow, r3.NewVec(fb.b[0], fb.b[1], 0), z)
		require.NoError(t, err)
		return fb
	}
	t.Run("outside", func(t *testing.T) {
		t.Parallel()
		fb := limited(t, -5, 5)
		_, err := fb.doc.VerifyLinkage(t.Context(), fb.linkage, fb.crankDrive(units.Degrees(90)))
		require.ErrorIs(t, err, decad.ErrDegenerate)
		require.ErrorContains(t, err, `link 2's dependent joint`)
		_, err = fb.linkage.Schedule(t.Context(), fb.crankDrive(units.Degrees(90)))
		require.ErrorIs(t, err, decad.ErrDegenerate)
	})
	t.Run("inside", func(t *testing.T) {
		t.Parallel()
		fb := limited(t, -10, 5)
		verifyLinkage(t, fb.doc, fb.linkage, fb.crankDrive(units.Degrees(90)), decad.WithResolution(units.Scalar(1.0/4)))
	})
	t.Run("excluding the zero pose", func(t *testing.T) {
		t.Parallel()
		fb := limited(t, -10, -1)
		drive := decad.Drive{{Link: fb.crank, From: units.Degrees(20), To: units.Degrees(50)}}
		verifyLinkage(t, fb.doc, fb.linkage, drive, decad.WithResolution(units.Scalar(1.0/4)))
		requireRockerValues(t, schedulePoses(t, fb.linkage, drive, sixteenths()...), func(s float64) float64 { return (20 + 30*s) * math.Pi / 180 })
	})
}

// TestVerifyLinkageLoopFoldOutAndBack is scene 9 driven 0° → 100° → 0° at the
// defaults: the crank enters the fold at 87.707557° on the way out and leaves
// it on the way back, so no pose lies in [0.438538, 0.561462], the two ends
// read the zero pose, and one undecided interval holds that stretch with every
// other interval clear.
func TestVerifyLinkageLoopFoldOutAndBack(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	fb := buildFourBar(t, doc, foldCrank, foldCoupler, foldFollower)
	sIn := math.Acos(0.04) * 180 / math.Pi / 200
	require.InDelta(t, 0.438538, sIn, 1e-6)
	sOut := 1 - sIn
	drive := decad.Drive{{Link: fb.crank, From: units.Degrees(0), Via: []units.Value{units.Degrees(100)}, To: units.Degrees(0)}}
	report := verifyLinkage(t, doc, fb.linkage, drive)
	require.Equal(t, decad.Suspect, report.Status)
	for _, end := range []decad.LinkagePose{report.Poses[0].Pose, report.Poses[len(report.Poses)-1].Pose} {
		require.InDelta(t, 0, end.Values[1].Mag(), 1e-9)
		require.InDelta(t, 0, end.Values[2].Mag(), 1e-9)
	}
	for _, p := range report.Poses {
		s := p.Pose.At.Mag()
		require.True(t, s < sIn || s > sOut, `no pose inside the fold at s = %v`, s)
	}
	undecided := 0
	for _, iv := range report.Intervals {
		if iv.Outcome == decad.IntervalClear {
			continue
		}
		require.Equal(t, decad.IntervalUndecided, iv.Outcome)
		require.LessOrEqual(t, iv.From.Mag(), sIn)
		require.GreaterOrEqual(t, iv.To.Mag(), sOut)
		undecided++
	}
	require.Equal(t, 1, undecided)
}
