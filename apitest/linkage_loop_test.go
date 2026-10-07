package apitest_test

import (
	"context"
	"math"
	"sync"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file holds the closed-loop tests of docs/linkage-check-design.md
// §15.10: scene 7 (the crank-rocker against a wall) with its pin and
// turn-back legs, scene 9 (the fold), and the loop's standing tests. Every
// closed form is the four-bar's two-circle construction in float64; an angle
// that is an inverse is bracketed to 1e-12 by bisection. Bounds are
// asserted small, never pinned: FMA contraction moves the last ulp between
// amd64 and arm64.
//
// Scene 7's crank-rocker has ground 100, crank 30, coupler 80 and follower
// 70; the coupler pin B sits above the ground line. The follower's top
// corner B − 4·n4, n4 = (−sin θ4, cos θ4), stands at y = 70·sin θ4 − 4·cos θ4,
// highest at the follower's minimum θ4 = 101.5370° (θ2 = 38.5727°). A wall
// x ∈ [−50, 150], y ∈ [68.5, 78.5], z ∈ [19, 29] sits in the follower's layer
// alone.

const (
	rockerGround, rockerCrank, rockerCoupler, rockerFollower = 100.0, 30.0, 80.0, 70.0
	// rockerMinCrank is the crank angle at the follower's minimum, where crank
	// and coupler lie along one line (|O2 B| = 110); it only separates the
	// two roots below.
	rockerMinCrank = 38.5727 * math.Pi / 180
)

func rockerTheta4(th2 float64) float64 {
	return fourBarTheta4(th2, rockerGround, rockerCrank, rockerCoupler, rockerFollower)
}

// rockerCouplerAngle is the coupler's angle from the ground line at crank
// angle th2.
func rockerCouplerAngle(th2 float64) float64 {
	t4 := rockerTheta4(th2)
	ax, ay := rockerCrank*math.Cos(th2), rockerCrank*math.Sin(th2)
	bx, by := rockerGround+rockerFollower*math.Cos(t4), rockerFollower*math.Sin(t4)
	return math.Atan2(by-ay, bx-ax)
}

// rockerCorner is the follower's top corner at crank angle th2.
func rockerCorner(th2 float64) (float64, float64) {
	t4 := rockerTheta4(th2)
	return rockerGround + rockerFollower*math.Cos(t4) + 4*math.Sin(t4), rockerFollower*math.Sin(t4) - 4*math.Cos(t4)
}

// rockerWallHits is where the follower's corner enters and leaves the wall's
// face y = 68.5, as fractions of a 0° → 90° drive.
func rockerWallHits() (float64, float64) {
	above := func(th2 float64) float64 { _, y := rockerCorner(th2); return y - 68.5 }
	enter := bisectRoot(above, 0, rockerMinCrank)
	leave := bisectRoot(above, rockerMinCrank, math.Pi/2)
	return enter / (math.Pi / 2), leave / (math.Pi / 2)
}

// buildRocker is scene 7's linkage, with the wall when asked for.
func buildRocker(t *testing.T, withWall bool) (fourBar, *decad.Body) {
	t.Helper()
	doc := decad.New()
	fb := buildFourBar(t, doc, rockerGround, rockerCrank, rockerCoupler, rockerFollower)
	var wall *decad.Body
	if withWall {
		wall = boxBodyAtZ(t, doc, -50, 68.5, 150, 78.5, 19, 10)
	}
	return fb, wall
}

// requireNoLinkRows asserts the layer exclusion settled every link-link pair:
// no pose carries a row naming two link bodies.
func requireNoLinkRows(t *testing.T, fb fourBar, report *decad.LinkageReport) {
	t.Helper()
	links := map[*decad.Body]struct{}{fb.crankBody: {}, fb.coupler: {}, fb.foll: {}}
	for _, p := range report.Poses {
		for _, row := range p.Clearances {
			_, isLink := links[row.B]
			require.False(t, isLink, `the layer exclusion settles every link-link pair`)
		}
		for _, row := range p.Interferences {
			_, isLink := links[row.B]
			require.False(t, isLink, `the layer exclusion settles every link-link pair`)
		}
	}
}

// TestVerifyLinkageLoopCrankRocker is scene 7 of docs/linkage-check-design.md
// §15.10, the closed loop's acceptance target.
//
// Legs seen to fail when deleted: subtracting the zero-pose reading (the
// follower then reads its angle from the ground line, 113.3250° at s = 1);
// the mirrored scene's signs (the drive 0° → −90° then reads the follower's
// turn under +90°, 3.0248°).
func TestVerifyLinkageLoopCrankRocker(t *testing.T) {
	t.Parallel()
	t.Run("the bars are the document's pins' exact distances", func(t *testing.T) {
		t.Parallel()
		fb, _ := buildRocker(t, false)
		bars := fb.loop.Bars()
		require.Len(t, bars, 4)
		require.Same(t, fb.linkage.Ground(), fb.loop.Common())
		require.Equal(t, []*decad.Link{fb.crank, fb.couplerLk, fb.follow}, fb.loop.Links())
		require.Equal(t, []*decad.LinkageLoop{fb.loop}, fb.linkage.Loops())
		for n, want := range []struct {
			link   *decad.Link
			length float64
		}{{fb.linkage.Ground(), rockerGround}, {fb.crank, rockerCrank}, {fb.couplerLk, rockerCoupler}, {fb.follow, rockerFollower}} {
			require.Same(t, want.link, bars[n].Link)
			require.InDelta(t, want.length, bars[n].Length.Value.Mag(), 1e-9)
			require.Less(t, bars[n].Length.Bound.Mag(), 1e-12)
		}
		for _, n := range []int{0, 1} {
			require.Equal(t, decad.Exact, bars[n].Length.Exactness, `a float length is stated exactly`)
			require.Zero(t, bars[n].Length.Bound.Mag())
		}
		closure := fb.loop.Closure()
		require.Equal(t, r3.NewVec(fb.b[0], fb.b[1], 0), closure.Center)
		require.Nil(t, closure.Limits)
	})
	t.Run("the follower reaches the wall", func(t *testing.T) {
		t.Parallel()
		fb, wall := buildRocker(t, true)
		report := verifyLinkage(t, fb.doc, fb.linkage, fb.crankDrive(units.Degrees(90)), decad.WithResolution(units.Scalar(1.0/256)))
		s1, s2 := rockerWallHits()
		require.InDelta(t, 0.140278, s1, 1e-6)
		require.InDelta(t, 0.742137, s2, 1e-6)

		require.Equal(t, decad.Interfering, report.Status)
		require.NotEmpty(t, report.Collisions)
		first := report.Collisions[0]
		require.Same(t, fb.foll, first.A)
		require.Same(t, wall, first.B)
		require.Equal(t, units.Scalar(36.0/256), first.At, `the first grid point above s₁`)
		for _, c := range report.Collisions {
			require.Greater(t, c.At.Mag(), s1)
			require.Less(t, c.At.Mag(), s2)
			require.Less(t, c.Volume.Bound.Mag(), c.Volume.Value.Mag())
		}
		for _, iv := range report.Intervals {
			if iv.Outcome == decad.IntervalClear {
				require.True(t, iv.To.Mag() <= s1 || iv.From.Mag() >= s2, `a clear interval holds no contact`)
			}
		}
		require.Equal(t, decad.IntervalClear, report.Intervals[len(report.Intervals)-1].Outcome)
		requireNoLinkRows(t, fb, report)

		// Every dependent value is the closed form's turn from the zero pose.
		theta40, theta30 := rockerTheta4(0), rockerCouplerAngle(0)
		for _, p := range report.Poses {
			th2 := p.Pose.At.Mag() * math.Pi / 2
			require.InDelta(t, rockerTheta4(th2)-theta40, p.Pose.Values[2].Mag(), 1e-9)
			coupler := rockerCouplerAngle(th2) - th2 - theta30
			require.InDelta(t, coupler, p.Pose.Values[1].Mag(), 1e-9)
			require.Equal(t, units.Radian, p.Pose.Values[2].Unit())
			require.Zero(t, p.Pose.Bounds[0].Mag(), `the stated crank carries no half-width`)
			for _, k := range []int{1, 2} {
				require.Positive(t, p.Pose.Bounds[k].Mag())
				require.Less(t, p.Pose.Bounds[k].Mag(), 1e-9)
			}
		}
		start, end := report.Poses[0].Pose, report.Poses[len(report.Poses)-1].Pose
		require.InDelta(t, 0, start.Values[1].Mag(), 1e-9)
		require.InDelta(t, 0, start.Values[2].Mag(), 1e-9)
		require.InDelta(t, 3.0248, end.Values[2].Mag()*180/math.Pi, 1e-4)
	})
	t.Run("the overlap at the follower's minimum is the corner's prism", func(t *testing.T) {
		t.Parallel()
		// The grid point 110/256 of the 0° → 90° drive, θ2 = 38.671875°, as
		// the end of a drive evaluated at its endpoints alone.
		fb, wall := buildRocker(t, true)
		th2 := 38.671875 * math.Pi / 180
		report := verifyLinkage(t, fb.doc, fb.linkage, fb.crankDrive(units.Degrees(38.671875)), decad.WithResolution(units.Scalar(1)))
		t4 := rockerTheta4(th2)
		_, y := rockerCorner(th2)
		depth := y - 68.5
		require.Less(t, depth, 8*math.Abs(math.Cos(t4)), `only the one corner has crossed`)
		var hit *decad.LinkCollision
		for n := range report.Collisions {
			if report.Collisions[n].At.Mag() == 1 && report.Collisions[n].B == wall {
				hit = &report.Collisions[n]
			}
		}
		require.NotNil(t, hit)
		require.InDelta(t, 8*depth*depth/(2*math.Sin(t4)*(-math.Cos(t4))), hit.Volume.Value.Mag(), 1e-6)
		require.Less(t, hit.Volume.Bound.Mag(), hit.Volume.Value.Mag())
	})
	t.Run("the crank turned the other way reads the mirrored scene", func(t *testing.T) {
		t.Parallel()
		fb, _ := buildRocker(t, false)
		report := verifyLinkage(t, fb.doc, fb.linkage, fb.crankDrive(units.Degrees(-90)), decad.WithResolution(units.Scalar(1)))
		end := report.Poses[len(report.Poses)-1].Pose
		want := rockerTheta4(-math.Pi/2) - rockerTheta4(0)
		require.InDelta(t, 36.4233, want*180/math.Pi, 1e-4)
		require.InDelta(t, want, end.Values[2].Mag(), 1e-9)
	})
	t.Run("a schedule poses exactly what the report evaluated", func(t *testing.T) {
		t.Parallel()
		fb, _ := buildRocker(t, true)
		drive := fb.crankDrive(units.Degrees(90))
		report := verifyLinkage(t, fb.doc, fb.linkage, drive, decad.WithResolution(units.Scalar(1.0/64)))
		one, err := fb.linkage.Schedule(t.Context(), drive)
		require.NoError(t, err)
		two, err := fb.linkage.Schedule(t.Context(), drive)
		require.NoError(t, err)
		require.Equal(t, drive, one.Drive())
		for _, p := range report.Poses {
			got, err := one.PoseAt(t.Context(), p.Pose.At)
			require.NoError(t, err)
			require.Equal(t, p.Pose, got, `the verifier's pose is the schedule's, bit for bit`)
			again, err := two.PoseAt(t.Context(), p.Pose.At)
			require.NoError(t, err)
			require.Equal(t, got, again)
		}
		third := units.Scalar(1.0 / 3)
		want, err := one.PoseAt(t.Context(), third)
		require.NoError(t, err)
		once, err := fb.linkage.PoseAt(drive, third)
		require.NoError(t, err)
		require.Equal(t, want, once)

		fractions := make([]units.Value, 64)
		for n := range fractions {
			fractions[n] = units.Scalar(float64((n*37)%64) / 63)
		}
		serial := make([]decad.LinkagePose, len(fractions))
		for n, at := range fractions {
			serial[n], err = two.PoseAt(t.Context(), at)
			require.NoError(t, err)
		}
		fresh, err := fb.linkage.Schedule(t.Context(), drive)
		require.NoError(t, err)
		concurrent := make([]decad.LinkagePose, len(fractions))
		errs := make([]error, len(fractions))
		var wg sync.WaitGroup
		for g := range 8 {
			wg.Go(func() {
				for n := g; n < len(fractions); n += 8 {
					concurrent[n], errs[n] = fresh.PoseAt(context.Background(), fractions[n])
				}
			})
		}
		wg.Wait()
		for n := range fractions {
			require.NoError(t, errs[n])
			require.Equal(t, serial[n], concurrent[n])
		}
	})
}

// TestVerifyLinkageLoopPinBetweenSamples: scene 7 without the wall and a
// 0.8 mm pin in the follower's layer that only the follower's top corner
// grazes. The follower's end face sweeps every radius from 70 to 70.114 mm
// about O4, so a pin on the corner's own path would stay inside the bar for a
// wide stretch of the drive; this pin is centred 70.55 mm from O4, 0.2° past
// the corner's polar angle at θ2 = 16.875°, where the corner reaches it from
// θ2 = 16.551652° to 19.24° — the onset bracketed by bisection of the
// separating-axis gap between the follower's outline and the pin's — between
// the grid points 1/8 and 1/4 of WithResolution(1/8), across which the corner
// moves 3.6 mm.
//
// Leg seen to fail when deleted: a dependent joint's term in τ — the follower
// is a child of the ground, so its travel is its own joint's alone, and the
// coarse interval then certifies from its ends.
func TestVerifyLinkageLoopPinBetweenSamples(t *testing.T) {
	t.Parallel()
	const onset = 16.551652 / 90
	build := func(t *testing.T) (fourBar, *decad.Body) {
		fb, _ := buildRocker(t, false)
		p := rockerTheta4(16.875*math.Pi/180) - math.Atan2(4, 70) + 0.2*math.Pi/180
		cx, cy := rockerGround+70.55*math.Cos(p), 70.55*math.Sin(p)
		pin := boxBodyAtZ(t, fb.doc, cx-0.4, cy-0.4, cx+0.4, cy+0.4, 23.6, 0.8)
		return fb, pin
	}
	t.Run("coarse", func(t *testing.T) {
		t.Parallel()
		fb, _ := build(t)
		report := verifyLinkage(t, fb.doc, fb.linkage, fb.crankDrive(units.Degrees(90)), decad.WithResolution(units.Scalar(1.0/8)))
		require.Equal(t, decad.Suspect, report.Status)
		require.Empty(t, report.Collisions)
		var holding *decad.MotionInterval
		for n, iv := range report.Intervals {
			if iv.From.Mag() == 1.0/8 && iv.To.Mag() == 1.0/4 {
				holding = &report.Intervals[n]
			}
		}
		require.NotNil(t, holding)
		require.Equal(t, decad.IntervalUndecided, holding.Outcome)
	})
	t.Run("fine", func(t *testing.T) {
		t.Parallel()
		fb, pin := build(t)
		report := verifyLinkage(t, fb.doc, fb.linkage, fb.crankDrive(units.Degrees(90)), decad.WithResolution(units.Scalar(1.0/1024)))
		require.Equal(t, decad.Interfering, report.Status)
		first := report.Collisions[0]
		require.Same(t, fb.foll, first.A)
		require.Same(t, pin, first.B)
		require.Greater(t, first.At.Mag(), onset)
		require.LessOrEqual(t, first.At.Mag(), onset+2.0/1024)
	})
}

// TestVerifyLinkageLoopTurnBack: scene 7 without the wall, the crank driven
// to where the follower returns to its starting angle after dipping to its
// minimum, and a 0.8 mm pin centred on the corner at that minimum. The
// follower's two ends agree, so only the two-sided hull bound of §15.5 keeps
// the one interval of the endpoints-only check undecided.
//
// Leg seen to fail when deleted: reading a dependent's |Δq| as the span of
// the interval's ends — the interval then certifies and the report reads
// Sound with two poses.
func TestVerifyLinkageLoopTurnBack(t *testing.T) {
	t.Parallel()
	fb, _ := buildRocker(t, false)
	back := bisectRoot(func(th2 float64) float64 { return rockerTheta4(th2) - rockerTheta4(0) }, rockerMinCrank, 2*math.Pi/3)
	require.InDelta(t, 81.857366, back*180/math.Pi, 1e-6)
	cx, cy := rockerCorner(rockerMinCrank)
	pin := boxBodyAtZ(t, fb.doc, cx-0.4, cy-0.4, cx+0.4, cy+0.4, 23.6, 0.8)
	report := verifyLinkage(t, fb.doc, fb.linkage, fb.crankDrive(units.Radians(back)), decad.WithResolution(units.Scalar(1)))
	require.Equal(t, decad.Suspect, report.Status)
	require.Len(t, report.Poses, 2)
	require.Len(t, report.Intervals, 1)
	require.Equal(t, decad.IntervalUndecided, report.Intervals[0].Outcome)
	require.InDelta(t, report.Poses[0].Pose.Values[2].Mag(), report.Poses[1].Pose.Values[2].Mag(), 1e-6, `the follower's ends agree`)
	require.Contains(t, report.Against, pin)
}

// TestVerifyLinkageLoopMirroredAxes: scene 7 stated with every joint and the
// closure about −Z. A crank turning −90° about −Z is scene 7's +90° turn, and
// each dependent reads its turn about its own −Z axis: the negation of scene
// 7's.
func TestVerifyLinkageLoopMirroredAxes(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	fb := fourBar{doc: doc}
	t4 := rockerTheta4(0)
	fb.b = [2]float64{rockerGround + rockerFollower*math.Cos(t4), rockerFollower * math.Sin(t4)}
	fb.crankBody = boxBodyAtZ(t, doc, 0, -4, rockerCrank, 4, 0, 8)
	fb.coupler = barBody(t, doc, [2]float64{rockerCrank, 0}, fb.b, 10)
	fb.foll = barBody(t, doc, [2]float64{rockerGround, 0}, fb.b, 20)
	fb.linkage = decad.NewLinkage()
	down := r3.NewVec(0, 0, -1)
	var err error
	fb.crank, err = fb.linkage.Ground().Revolute(r3.Vec{}, down, []*decad.Body{fb.crankBody})
	require.NoError(t, err)
	fb.couplerLk, err = fb.crank.Revolute(r3.NewVec(rockerCrank, 0, 0), down, []*decad.Body{fb.coupler})
	require.NoError(t, err)
	fb.follow, err = fb.linkage.Ground().Revolute(r3.NewVec(rockerGround, 0, 0), down, []*decad.Body{fb.foll})
	require.NoError(t, err)
	fb.loop, err = fb.linkage.Close(fb.couplerLk, fb.follow, r3.NewVec(fb.b[0], fb.b[1], 0), down)
	require.NoError(t, err)
	pose, err := fb.linkage.PoseAt(fb.crankDrive(units.Degrees(-90)), units.Scalar(1))
	require.NoError(t, err)
	require.InDelta(t, -(rockerTheta4(math.Pi/2) - t4), pose.Values[2].Mag(), 1e-9)
	coupler := rockerCouplerAngle(math.Pi/2) - math.Pi/2 -
		rockerCouplerAngle(0)
	require.InDelta(t, -coupler, pose.Values[1].Mag(), 1e-9)
}

// The non-Grashof four-bar of scene 9: ground 100, crank 50, coupler 60,
// follower 50. Its loop folds at cos θ2 = 0.04: past θ2 = 87.707557° no
// configuration exists.
const foldGround, foldCrank, foldCoupler, foldFollower = 100.0, 50.0, 60.0, 50.0

// TestVerifyLinkageLoopFold is scene 9 of docs/linkage-check-design.md §15.10.
// No static body stands in the document and the layer exclusion settles every
// pair, so every interval's outcome is the loop's own.
//
// Measured at the defaults: the last pose sits at 997/1024 = 0.973633, under
// s_fold = 0.974528 by less than one verdict floor.
//
// Legs seen to fail when deleted: refusing an interval whose cell sketch
// refused or whose end it could not enclose — the last interval then reads
// IntervalClear; and treating an unbuildable pose as not built — the run then
// publishes a pose past the fold.
func TestVerifyLinkageLoopFold(t *testing.T) {
	t.Parallel()
	sFold := math.Acos(0.04) / (math.Pi / 2)
	require.InDelta(t, 0.974528, sFold, 1e-6)
	t.Run("the drive into the fold", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		fb := buildFourBar(t, doc, foldGround, foldCrank, foldCoupler, foldFollower)
		drive := fb.crankDrive(units.Degrees(90))
		report := verifyLinkage(t, doc, fb.linkage, drive)
		require.Equal(t, decad.Suspect, report.Status)
		require.Empty(t, report.Collisions)
		for _, p := range report.Poses {
			require.Less(t, p.Pose.At.Mag(), sFold, `no pose past the fold`)
		}
		last := report.Poses[len(report.Poses)-1].Pose.At.Mag()
		require.Greater(t, last, sFold-1.0/1024, `the last pose sits within one verdict floor of the fold`)
		tail := report.Intervals[len(report.Intervals)-1]
		require.Equal(t, decad.IntervalUndecided, tail.Outcome)
		require.Equal(t, units.Scalar(1), tail.To, `the last interval ends at 1, which has no pose`)
		require.Equal(t, last, tail.From.Mag())
		for _, iv := range report.Intervals[:len(report.Intervals)-1] {
			require.Equal(t, decad.IntervalClear, iv.Outcome)
		}
		require.Len(t, report.Diagnostics, 1)
		diag := report.Diagnostics[0]
		require.Equal(t, decad.DiagMotionUndecidedInterval, diag.Code)
		require.Equal(t, tail.From, *diag.At)
		require.Contains(t, diag.Message, `the loop closing links 1 and 2`)
		require.Contains(t, diag.Message, `not certified`)

		sched, err := fb.linkage.Schedule(t.Context(), drive)
		require.NoError(t, err)
		_, err = sched.PoseAt(t.Context(), units.Scalar(1))
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.ErrorIs(t, err, sketch.ErrNotCertified)
		_, err = fb.linkage.PoseAt(drive, units.Scalar(1))
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.ErrorIs(t, err, sketch.ErrNotCertified)
	})
	t.Run("the flat four-bar refuses its zero pose", func(t *testing.T) {
		t.Parallel()
		// Coupler and follower lie along the ground line at the zero pose,
		// where the loop's Jacobian is singular.
		doc := decad.New()
		crank := boxBodyAtZ(t, doc, 0, -4, 30, 4, 0, 8)
		coupler := boxBodyAtZ(t, doc, 30, -4, 70, 4, 10, 8)
		follower := boxBodyAtZ(t, doc, 70, -4, 100, 4, 20, 8)
		l := decad.NewLinkage()
		z := r3.NewVec(0, 0, 1)
		ck, err := l.Ground().Revolute(r3.Vec{}, z, []*decad.Body{crank})
		require.NoError(t, err)
		cp, err := ck.Revolute(r3.NewVec(30, 0, 0), z, []*decad.Body{coupler})
		require.NoError(t, err)
		fl, err := l.Ground().Revolute(r3.NewVec(100, 0, 0), z, []*decad.Body{follower})
		require.NoError(t, err)
		_, err = l.Close(cp, fl, r3.NewVec(70, 0, 0), z)
		require.NoError(t, err)
		drive := decad.Drive{{Link: ck, From: units.Degrees(0), To: units.Degrees(90)}}
		_, err = doc.VerifyLinkage(t.Context(), l, drive)
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.ErrorIs(t, err, sketch.ErrUnderconstrained)
		_, err = l.Schedule(t.Context(), drive)
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.ErrorIs(t, err, sketch.ErrUnderconstrained)
		_, err = l.PoseAt(drive, units.Scalar(0.5))
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.ErrorIs(t, err, sketch.ErrUnderconstrained)
	})
}

// TestLinkageLoopRefusals is one subtest per row of
// docs/linkage-check-design.md §15.1's and §15.6's tables, and per refusal L1
// holds back for a later increment.
func TestLinkageLoopRefusals(t *testing.T) {
	t.Parallel()
	z := r3.NewVec(0, 0, 1)
	closeRows := []struct {
		name string
		mod  func(t *testing.T, fb fourBar) error
		want error
	}{
		{"a nil link", func(_ *testing.T, fb fourBar) error {
			_, err := fb.linkage.Close(nil, fb.follow, r3.Vec{}, z)
			return err
		}, decad.ErrDegenerate},
		{"the ground", func(_ *testing.T, fb fourBar) error {
			_, err := fb.linkage.Close(fb.linkage.Ground(), fb.follow, r3.Vec{}, z)
			return err
		}, decad.ErrDegenerate},
		{"one link twice", func(_ *testing.T, fb fourBar) error {
			_, err := fb.linkage.Close(fb.follow, fb.follow, r3.Vec{}, z)
			return err
		}, decad.ErrDegenerate},
		{"a link of another linkage", func(t *testing.T, fb fourBar) error {
			other := decad.NewLinkage()
			k, err := other.Ground().Revolute(r3.Vec{}, z, []*decad.Body{boxBody(t, fb.doc, 0, 0, 1, 1, 1)})
			require.NoError(t, err)
			_, err = fb.linkage.Close(k, fb.follow, r3.Vec{}, z)
			return err
		}, decad.ErrDegenerate},
		{"a non-finite center", func(_ *testing.T, fb fourBar) error {
			_, err := fb.linkage.Close(fb.couplerLk, fb.follow, r3.NewVec(math.NaN(), 0, 0), z)
			return err
		}, decad.ErrNotFinite},
		{"a zero axis", func(_ *testing.T, fb fourBar) error {
			_, err := fb.linkage.Close(fb.couplerLk, fb.follow, r3.NewVec(fb.b[0], fb.b[1], 0), r3.Vec{})
			return err
		}, decad.ErrDegenerate},
		{"an axis tilted 1e-9 from Z", func(_ *testing.T, fb fourBar) error {
			_, err := fb.linkage.Close(fb.couplerLk, fb.follow, r3.NewVec(fb.b[0], fb.b[1], 0), r3.NewVec(1e-9, 0, 1))
			return err
		}, decad.ErrUnsupported},
		{"coincident pins", func(_ *testing.T, fb fourBar) error {
			_, err := fb.linkage.Close(fb.couplerLk, fb.follow, r3.NewVec(rockerGround, 0, 5), z)
			return err
		}, decad.ErrDegenerate},
	}
	for _, row := range closeRows {
		t.Run(row.name, func(t *testing.T) {
			t.Parallel()
			doc := decad.New()
			fb := buildFourBarOpen(t, doc, rockerGround, rockerCrank, rockerCoupler, rockerFollower, z)
			require.ErrorIs(t, row.mod(t, fb), row.want)
			require.Empty(t, fb.linkage.Loops())
		})
	}
	t.Run("a loop revolute not parallel to the closure", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		fb := buildFourBarOpen(t, doc, rockerGround, rockerCrank, rockerCoupler, rockerFollower, r3.NewVec(0, 1e-12, 1))
		_, err := fb.linkage.Close(fb.couplerLk, fb.follow, r3.NewVec(fb.b[0], fb.b[1], 0), z)
		require.ErrorIs(t, err, decad.ErrUnsupported)
	})
	slideRows := []struct {
		name  string
		build func(t *testing.T, doc *decad.Document, l *decad.Linkage) (*decad.Link, *decad.Link)
	}{
		{"a slide along the closure axis", func(t *testing.T, doc *decad.Document, l *decad.Linkage) (*decad.Link, *decad.Link) {
			return sliderCrankOpen(t, doc, l, r3.NewVec(0, 0, 1))
		}},
		{"a slide off every coordinate axis", func(t *testing.T, doc *decad.Document, l *decad.Linkage) (*decad.Link, *decad.Link) {
			return sliderCrankOpen(t, doc, l, r3.NewVec(1, 1e-12, 0))
		}},
		{"a slide under a link that is not the common one", func(t *testing.T, doc *decad.Document, l *decad.Linkage) (*decad.Link, *decad.Link) {
			crank, err := l.Ground().Revolute(r3.Vec{}, z, []*decad.Body{boxBodyAtZ(t, doc, 0, -3, 30, 3, 0, 8)})
			require.NoError(t, err)
			slide, err := crank.Prismatic(r3.NewVec(1, 0, 0), []*decad.Body{boxBodyAtZ(t, doc, 25, -3, 35, 3, 10, 8)})
			require.NoError(t, err)
			follower, err := l.Ground().Revolute(r3.NewVec(100, 0, 0), z, []*decad.Body{boxBodyAtZ(t, doc, 30, -3, 100, 3, 20, 8)})
			require.NoError(t, err)
			return slide, follower
		}},
		{"two slides on the loop", func(t *testing.T, doc *decad.Document, l *decad.Linkage) (*decad.Link, *decad.Link) {
			first, err := l.Ground().Prismatic(r3.NewVec(1, 0, 0), []*decad.Body{boxBodyAtZ(t, doc, -5, -5, 5, 5, 0, 8)})
			require.NoError(t, err)
			rod, err := first.Revolute(r3.Vec{}, z, []*decad.Body{boxBodyAtZ(t, doc, 0, -3, 60, 3, 10, 8)})
			require.NoError(t, err)
			second, err := l.Ground().Prismatic(r3.NewVec(0, 1, 0), []*decad.Body{boxBodyAtZ(t, doc, 55, -5, 65, 5, 20, 8)})
			require.NoError(t, err)
			return rod, second
		}},
	}
	for _, row := range slideRows {
		t.Run(row.name, func(t *testing.T) {
			t.Parallel()
			doc := decad.New()
			l := decad.NewLinkage()
			a, b := row.build(t, doc, l)
			_, err := l.Close(a, b, r3.NewVec(60, 0, 0), z)
			require.ErrorIs(t, err, decad.ErrUnsupported)
			require.Empty(t, l.Loops())
		})
	}
	t.Run("a second closure on the coupler", func(t *testing.T) {
		t.Parallel()
		fb, _ := buildRocker(t, false)
		extra, err := fb.linkage.Ground().Revolute(r3.NewVec(200, 0, 0), z, []*decad.Body{boxBodyAtZ(t, fb.doc, 200, -4, 230, 4, 30, 8)})
		require.NoError(t, err)
		_, err = fb.linkage.Close(fb.couplerLk, extra, r3.NewVec(150, 50, 0), z)
		require.ErrorIs(t, err, decad.ErrUnsupported)
	})

	driveRows := []struct {
		name  string
		drive func(fb fourBar) decad.Drive
		want  error
	}{
		{"crank and follower both stated", func(fb fourBar) decad.Drive {
			return decad.Drive{
				{Link: fb.crank, From: units.Degrees(0), To: units.Degrees(90)},
				{Link: fb.follow, From: units.Degrees(0), To: units.Degrees(3)},
			}
		}, decad.ErrDegenerate},
		{"the coupler stated", func(fb fourBar) decad.Drive {
			return decad.Drive{{Link: fb.couplerLk, From: units.Degrees(0), To: units.Degrees(10)}}
		}, decad.ErrUnsupported},
		{"a driver with Via", func(fb fourBar) decad.Drive {
			return decad.Drive{{Link: fb.crank, From: units.Degrees(0), Via: []units.Value{units.Degrees(45)}, To: units.Degrees(90)}}
		}, decad.ErrUnsupported},
		{"a driver held off zero", func(fb fourBar) decad.Drive {
			return decad.Drive{{Link: fb.crank, From: units.Degrees(10), To: units.Degrees(10)}}
		}, decad.ErrUnsupported},
		{"a driver crossing zero", func(fb fourBar) decad.Drive {
			return decad.Drive{{Link: fb.crank, From: units.Degrees(-10), To: units.Degrees(10)}}
		}, decad.ErrUnsupported},
	}
	for _, row := range driveRows {
		t.Run(row.name, func(t *testing.T) {
			t.Parallel()
			fb, _ := buildRocker(t, false)
			before := fb.doc.Bodies()
			_, err := fb.doc.VerifyLinkage(t.Context(), fb.linkage, row.drive(fb))
			require.ErrorIs(t, err, row.want)
			_, err = fb.linkage.Schedule(t.Context(), row.drive(fb))
			require.ErrorIs(t, err, row.want)
			_, err = fb.linkage.PoseAt(row.drive(fb), units.Scalar(0.5))
			require.ErrorIs(t, err, row.want)
			require.Equal(t, before, fb.doc.Bodies())
		})
	}
	t.Run("a dependent with limits", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		fb := buildFourBarOpen(t, doc, rockerGround, rockerCrank, rockerCoupler, rockerFollower, z, decad.WithJointLimits(units.Degrees(-5), units.Degrees(5)))
		_, err := fb.linkage.Close(fb.couplerLk, fb.follow, r3.NewVec(fb.b[0], fb.b[1], 0), z)
		require.NoError(t, err)
		_, err = doc.VerifyLinkage(t.Context(), fb.linkage, fb.crankDrive(units.Degrees(90)))
		require.ErrorIs(t, err, decad.ErrUnsupported)
	})
	t.Run("a schedule outside the drive", func(t *testing.T) {
		t.Parallel()
		fb, _ := buildRocker(t, false)
		sched, err := fb.linkage.Schedule(t.Context(), fb.crankDrive(units.Degrees(90)))
		require.NoError(t, err)
		_, err = sched.PoseAt(t.Context(), units.Scalar(1.5))
		require.ErrorIs(t, err, decad.ErrUnsupported)
		_, err = fb.linkage.PoseAt(fb.crankDrive(units.Degrees(90)), units.Scalar(-0.5))
		require.ErrorIs(t, err, decad.ErrUnsupported)
		_, err = sched.PoseAt(t.Context(), units.Degrees(1))
		require.ErrorIs(t, err, decad.ErrUnitKind)
	})
	t.Run("the joint box and a configuration", func(t *testing.T) {
		t.Parallel()
		fb, _ := buildRocker(t, false)
		_, err := fb.doc.VerifyJointBox(t.Context(), fb.linkage, decad.JointBox{{Link: fb.crank, Min: units.Degrees(0), Max: units.Degrees(90)}})
		require.ErrorIs(t, err, decad.ErrUnsupported)
		_, err = fb.linkage.Configuration([]units.Value{units.Degrees(0), units.Degrees(0), units.Degrees(0)})
		require.ErrorIs(t, err, decad.ErrUnsupported)
	})
	t.Run("a loop the drive does not move stands at the zero pose", func(t *testing.T) {
		t.Parallel()
		fb, _ := buildRocker(t, false)
		pose, err := fb.linkage.PoseAt(decad.Drive{}, units.Scalar(0.5))
		require.NoError(t, err)
		for k := range 3 {
			require.Zero(t, pose.Values[k].Mag())
			require.Zero(t, pose.Bounds[k].Mag())
			require.Equal(t, r3.Identity(), pose.Poses[k])
		}
	})
}

// buildFourBarOpen is buildFourBar with every joint about axis and no
// closure yet; opts go to the follower's joint.
func buildFourBarOpen(t *testing.T, doc *decad.Document, g, r, l, f float64, axis r3.Vec, opts ...decad.JointOption) fourBar {
	t.Helper()
	fb := fourBar{doc: doc, g: g, r: r, l: l, f: f}
	t4 := fourBarTheta4(0, g, r, l, f)
	fb.b = [2]float64{g + f*math.Cos(t4), f * math.Sin(t4)}
	fb.crankBody = boxBodyAtZ(t, doc, 0, -4, r, 4, 0, 8)
	fb.coupler = barBody(t, doc, [2]float64{r, 0}, fb.b, 10)
	fb.foll = barBody(t, doc, [2]float64{g, 0}, fb.b, 20)
	fb.linkage = decad.NewLinkage()
	z := r3.NewVec(0, 0, 1)
	var err error
	fb.crank, err = fb.linkage.Ground().Revolute(r3.Vec{}, z, []*decad.Body{fb.crankBody})
	require.NoError(t, err)
	fb.couplerLk, err = fb.crank.Revolute(r3.NewVec(r, 0, 0), axis, []*decad.Body{fb.coupler})
	require.NoError(t, err)
	fb.follow, err = fb.linkage.Ground().Revolute(r3.NewVec(g, 0, 0), z, []*decad.Body{fb.foll}, opts...)
	require.NoError(t, err)
	return fb
}

// TestVerifyLinkageLoopNonMutationAndDeterminism: two checks of scene 7 leave
// the document as it was and return reports equal in every field.
func TestVerifyLinkageLoopNonMutationAndDeterminism(t *testing.T) {
	t.Parallel()
	fb, _ := buildRocker(t, true)
	drive := fb.crankDrive(units.Degrees(90))
	before := fb.doc.Bodies()
	first := verifyLinkage(t, fb.doc, fb.linkage, drive, decad.WithResolution(units.Scalar(1.0/32)))
	second := verifyLinkage(t, fb.doc, fb.linkage, drive, decad.WithResolution(units.Scalar(1.0/32)))
	require.Equal(t, first, second)
	require.Equal(t, before, fb.doc.Bodies())
}

// TestVerifyLinkageLoopCancellation: a context canceled at every depth of the
// check, inside a sketch.Enclose call among them, returns its error and no
// report, and the check finishes once the context outlasts it.
func TestVerifyLinkageLoopCancellation(t *testing.T) {
	t.Parallel()
	fb, _ := buildRocker(t, true)
	drive := fb.crankDrive(units.Degrees(90))
	before := fb.doc.Bodies()
	var completed *decad.LinkageReport
	canceled := 0
	for limit := int32(1); limit <= 1<<24; limit += 1 + limit/2 {
		ctx := newCancelAfterContext(t.Context(), limit)
		report, err := fb.doc.VerifyLinkage(ctx, fb.linkage, drive, decad.WithResolution(units.Scalar(1.0/16)))
		require.Equal(t, before, fb.doc.Bodies())
		if err == nil {
			completed = report
			break
		}
		require.ErrorIs(t, err, context.Canceled)
		require.Nil(t, report)
		canceled++
	}
	require.NotNil(t, completed)
	require.Greater(t, canceled, 10)
}
