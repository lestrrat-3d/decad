package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file holds the tilted-loop tests of docs/linkage-check-design.md
// §15.10: scenes 7 and 8 with every body, pin and axis carried by one
// rotation that takes Z to (1, 1, 1)/√3, so the loop's plane lies along no
// coordinate plane and every pin's plane position is irrational. Each scene's
// first collision is asserted equal to the untilted scene's.

// tiltToDiagonal is the rotation that takes X to (1, −1, 0)/√2 and Z to
// (1, 1, 1)/√3, so a slide along scene 8's X runs along (1, −1, 0), exactly
// perpendicular to the tilted axis (1, 1, 1).
func tiltToDiagonal(t *testing.T) r3.Transform {
	t.Helper()
	frame, err := r3.NewFrame(r3.Vec{}, r3.NewVec(1, -1, 0), r3.NewVec(1, 1, -2))
	require.NoError(t, err)
	tilt, err := r3.FromFrame(frame)
	require.NoError(t, err)
	return tilt
}

// tiltedScene places bodies and pins of an untilted scene under
// tiltToDiagonal.
type tiltedScene struct {
	t    *testing.T
	tilt r3.Transform
}

func newTiltedScene(t *testing.T) tiltedScene {
	t.Helper()
	return tiltedScene{t: t, tilt: tiltToDiagonal(t)}
}

// place carries a body built where the untilted scene has it.
func (ts tiltedScene) place(b *decad.Body) *decad.Body {
	ts.t.Helper()
	placed, err := b.Placed(ts.t.Context(), ts.tilt)
	require.NoError(ts.t, err)
	return placed
}

// pin carries the untilted scene's pin (x, y, 0).
func (ts tiltedScene) pin(x, y float64) r3.Vec { return ts.tilt.Apply(r3.NewVec(x, y, 0)) }

// loopTiltAxis is the axis every tilted joint and closure turns about.
var loopTiltAxis = r3.NewVec(1, 1, 1)

// buildTiltedRocker is scene 7 carried by tiltToDiagonal: each body is built
// as scene 7 builds it and placed by the tilt, each pin is the tilt of scene
// 7's, and every joint and the closure turn about (1, 1, 1).
func buildTiltedRocker(t *testing.T, withWall bool) (fourBar, *decad.Body) {
	t.Helper()
	ts := newTiltedScene(t)
	doc := decad.New()
	fb := fourBar{doc: doc, g: rockerGround, r: rockerCrank, l: rockerCoupler, f: rockerFollower}
	t4 := rockerTheta4(0)
	fb.b = [2]float64{rockerGround + rockerFollower*math.Cos(t4), rockerFollower * math.Sin(t4)}
	fb.crankBody = ts.place(boxBodyAtZ(t, doc, 0, -4, rockerCrank, 4, 0, 8))
	fb.coupler = ts.place(barBody(t, doc, [2]float64{rockerCrank, 0}, fb.b, 10))
	fb.foll = ts.place(barBody(t, doc, [2]float64{rockerGround, 0}, fb.b, 20))
	fb.linkage = decad.NewLinkage()
	var err error
	fb.crank, err = fb.linkage.Ground().Revolute(ts.pin(0, 0), loopTiltAxis, []*decad.Body{fb.crankBody})
	require.NoError(t, err)
	fb.couplerLk, err = fb.crank.Revolute(ts.pin(rockerCrank, 0), loopTiltAxis, []*decad.Body{fb.coupler})
	require.NoError(t, err)
	fb.follow, err = fb.linkage.Ground().Revolute(ts.pin(rockerGround, 0), loopTiltAxis, []*decad.Body{fb.foll})
	require.NoError(t, err)
	fb.loop, err = fb.linkage.Close(fb.couplerLk, fb.follow, ts.pin(fb.b[0], fb.b[1]), loopTiltAxis)
	require.NoError(t, err)
	var wall *decad.Body
	if withWall {
		wall = ts.place(boxBodyAtZ(t, doc, -50, 68.5, 150, 78.5, 19, 10))
	}
	return fb, wall
}

// TestVerifyLinkageLoopTilted is scene 7 about the tilted axis (1, 1, 1),
// against its wall carried by the same tilt. The bars are the tilted pins'
// exact distances, every dependent value is scene 7's closed form, and the
// first collision is the untilted scene's, at the grid point 36/256.
//
// Leg seen to fail when deleted: the fixed boxes — each of Common's pins is
// then fixed at its float plane position, which E0 reports as its box and
// which does not hold the pin's exact enclosure, so the zero-pose falsifier
// refuses the loop.
func TestVerifyLinkageLoopTilted(t *testing.T) {
	t.Parallel()
	t.Run("the bars are the tilted pins' exact distances", func(t *testing.T) {
		t.Parallel()
		fb, _ := buildTiltedRocker(t, false)
		bars := fb.loop.Bars()
		require.Len(t, bars, 4)
		for n, want := range []float64{rockerGround, rockerCrank, rockerCoupler, rockerFollower} {
			require.InDelta(t, want, bars[n].Length.Value.Mag(), 1e-9)
			require.Less(t, bars[n].Length.Bound.Mag(), 1e-12)
		}
		require.Equal(t, loopTiltAxis, fb.loop.Closure().Axis)
	})
	t.Run("the follower reaches the tilted wall where scene 7's does", func(t *testing.T) {
		t.Parallel()
		drive := func(fb fourBar) decad.Drive { return fb.crankDrive(units.Degrees(90)) }
		flat, _ := buildRocker(t, true)
		want := verifyLinkage(t, flat.doc, flat.linkage, drive(flat), decad.WithResolution(units.Scalar(1.0/256)))
		fb, wall := buildTiltedRocker(t, true)
		report := verifyLinkage(t, fb.doc, fb.linkage, drive(fb), decad.WithResolution(units.Scalar(1.0/256)))
		s1, s2 := rockerWallHits()
		require.Equal(t, decad.Interfering, report.Status)
		require.NotEmpty(t, report.Collisions)
		first := report.Collisions[0]
		require.Same(t, fb.foll, first.A)
		require.Same(t, wall, first.B)
		require.Equal(t, want.Collisions[0].At, first.At, `the untilted scene's first collision`)
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
		theta40, theta30 := rockerTheta4(0), rockerCouplerAngle(0)
		for _, p := range report.Poses {
			th2 := p.Pose.At.Mag() * math.Pi / 2
			require.InDelta(t, rockerTheta4(th2)-theta40, p.Pose.Values[2].Mag(), 1e-9)
			require.InDelta(t, rockerCouplerAngle(th2)-th2-theta30, p.Pose.Values[1].Mag(), 1e-9)
			for _, k := range []int{1, 2} {
				require.Positive(t, p.Pose.Bounds[k].Mag())
				require.Less(t, p.Pose.Bounds[k].Mag(), 1e-9)
			}
		}
	})
}

// TestVerifyLinkageLoopTiltedSlider is scene 8 carried by tiltToDiagonal: the
// slide runs along (1, −1, 0), exactly perpendicular to the closure axis
// (1, 1, 1), so the rail is stated along the slide's own direction, through
// irrational plane positions. The first collision is the untilted scene's,
// at the grid point 252/256 with the overlap 80·(75 − x) mm³, and every pose
// reads scene 8's closed forms.
func TestVerifyLinkageLoopTiltedSlider(t *testing.T) {
	t.Parallel()
	ts := newTiltedScene(t)
	doc := decad.New()
	l := decad.NewLinkage()
	crank, err := l.Ground().Revolute(ts.pin(0, 0), loopTiltAxis, []*decad.Body{ts.place(boxBodyAtZ(t, doc, 0, -3, 30, 3, 0, 8))})
	require.NoError(t, err)
	rod, err := crank.Revolute(ts.pin(30, 0), loopTiltAxis, []*decad.Body{ts.place(boxBodyAtZ(t, doc, 30, -3, 110, 3, 10, 8))})
	require.NoError(t, err)
	sliderBody := ts.place(boxBodyAtZ(t, doc, 105, -5, 115, 5, 20, 8))
	slider, err := l.Ground().Prismatic(r3.NewVec(1, -1, 0), []*decad.Body{sliderBody})
	require.NoError(t, err)
	_, err = l.Close(rod, slider, ts.pin(110, 0), loopTiltAxis)
	require.NoError(t, err)
	stop := ts.place(boxBodyAtZ(t, doc, 60, -20, 70, 20, 19, 10))
	drive := decad.Drive{{Link: crank, From: units.Degrees(0), To: units.Degrees(90)}}
	report := verifyLinkage(t, doc, l, drive, decad.WithResolution(units.Scalar(1.0/256)))

	flat := buildSliderCrank(t)
	boxBodyAtZ(t, flat.doc, 60, -20, 70, 20, 19, 10)
	want := verifyLinkage(t, flat.doc, flat.linkage, flat.drive(units.Degrees(90)), decad.WithResolution(units.Scalar(1.0/256)))

	sStar := math.Acos(1.0/36) / (math.Pi / 2)
	require.Equal(t, decad.Interfering, report.Status)
	first := report.Collisions[0]
	require.Same(t, sliderBody, first.A)
	require.Same(t, stop, first.B)
	require.Equal(t, want.Collisions[0].At, first.At, `the untilted scene's first collision`)
	require.Equal(t, units.Scalar(252.0/256), first.At, `the first grid point above s*`)
	require.InDelta(t, 80*(75-sliderPin(252.0/256*math.Pi/2)), first.Volume.Value.Mag(), 1e-6)
	require.Less(t, first.Volume.Bound.Mag(), first.Volume.Value.Mag())
	for _, c := range report.Collisions {
		require.Greater(t, c.At.Mag(), sStar)
	}
	for _, p := range report.Poses {
		th := p.Pose.At.Mag() * math.Pi / 2
		require.Equal(t, units.Millimeter, p.Pose.Values[2].Unit())
		require.InDelta(t, sliderPin(th)-110, p.Pose.Values[2].Mag(), 1e-9)
		require.Less(t, p.Pose.Bounds[2].Mag(), 1e-9)
		require.InDelta(t, -th-math.Asin(30*math.Sin(th)/80), p.Pose.Values[1].Mag(), 1e-9)
	}
}
