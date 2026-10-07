package main

import (
	"math"
	"slices"
	"testing"
	"time"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// These tests run the crank-rocker, a closed loop, through the real
// producers and the real viewer: decad's VerifyLinkage and Schedule,
// kinetograph's schedule tracks (Scene.AddSchedule), and four rendered
// frames.
//
// Each leg was seen red by breaking what it guards: the clip driven to one
// grid step past the turnaround fails the proven-clear leg; the nodes driven
// by a linear channel while the clip reports the eased one fails the
// bit-identical pose leg; a hit colour of
// gold fails the pixel leg at frame 192; the stop's angle moved to 144.6°
// fails the volume leg; and the coupler pins set on their bores' centres fail
// the joint-contact leg.

// TestLinkageLoopClipMatchesSchedule asserts that the crank-rocker's clip
// shows only poses the check proves clear, each exactly as the scene's
// Schedule builds it: every frame's drive fraction lies in [0, s], s the end
// of the stretch from 0 that VerifyLinkage proves clear, and every link's
// part takes the schedule's transform at that fraction bit for bit. The
// follower — a dependent joint — turns as the four-bar's construction says:
// θ4(θ2) − θ4(0), with θ2 = 360°·s, within 1e-9 rad of its pose's own
// rotation.
func TestLinkageLoopClipMatchesSchedule(t *testing.T) {
	t.Parallel()
	scene, err := crankRockerScene(t.Context())
	require.NoError(t, err)
	report, err := scene.verify(t.Context())
	require.NoError(t, err)
	turn, err := turnaroundOf(report)
	require.NoError(t, err)
	clip, _, fraction, err := scene.clip(turn, linkageClipLength, smokeWidth, smokeHeight)
	require.NoError(t, err)
	poseAt := func(s float64) (decad.LinkagePose, error) {
		return scene.schedule.PoseAt(t.Context(), units.Scalar(s))
	}
	requireClipPoses(t, scene, clip, fraction, turn, provenClear(report), poseAt)

	require.Equal(t, "follower", scene.parts[2].name)
	follower := slices.Index(scene.linkage.Links(), scene.parts[2].link)
	require.GreaterOrEqual(t, follower, 0)
	for _, i := range []int{0, 100, int(linkageHoldStart / (time.Second / linkageFPS)), 300} {
		at, err := fraction.At(clip.FrameTime(i))
		require.NoError(t, err)
		want, err := poseAt(at.Mag())
		require.NoError(t, err)
		ex := want.Poses[follower].Basis().EX
		th2 := at.Mag() * 2 * math.Pi
		require.InDelta(t, rockerTheta4(th2)-rockerTheta4(0), math.Atan2(ex.Y, ex.X), 1e-9, "frame %d", i)
	}
}

// TestLinkageLoopClipTurnsAtTheStop asserts the crank-rocker's verdict and
// its turnaround against their closed form, and the flash against the
// turnaround.
// The stop's face lies along the follower's leading flank at θc = 144.5°, and
// the follower first reaches θc at the crank angle the four-bar puts the
// coupler pin at B = O4 + 70·(cos θc, sin θc):
//
//	θ2* = atan2(B) + acos((30² + |B|² − 80²)/(2·30·|B|)) ≈ 167.62°,  s* = θ2*/360° ≈ 0.4656
//
// At 1/256 the first collision is the first grid point past it, 120/256.
// The check proves every interval clear up to 118/256 and leaves
// [118/256, 119/256], where the follower's flank closes on the stop's face
// nearly parallel, undecided, so the clip turns at 118/256 and holds frames
// 192 to 224 there, with the follower and the stop in the hit colour.
// At the first collision the follower has turned δ = θ4 − θc past the face, and the flank's
// overlap with the stop, from u1 = 30 to u2 = 60 mm along the follower and
// 6 mm deep along Z, is the wedge between the two lines, both 8 mm off the
// pivot:
//
//	V = 6·((u2 − u1)·8·(1/cos δ − 1) + tan δ·(u2² − u1²)/2)
func TestLinkageLoopClipTurnsAtTheStop(t *testing.T) {
	t.Parallel()
	scene, err := crankRockerScene(t.Context())
	require.NoError(t, err)
	report, err := scene.verify(t.Context())
	require.NoError(t, err)

	thetaC := rockerStopDeg * math.Pi / 180
	sStar := rockerTheta2(thetaC) / (2 * math.Pi)
	require.InDelta(t, thetaC, rockerTheta4(2*math.Pi*sStar), 1e-12)
	require.Greater(t, sStar, 119.0/256)
	require.Less(t, sStar, 120.0/256)
	require.Less(t, rockerTheta4(2*math.Pi*119/256), thetaC)
	require.Greater(t, rockerTheta4(2*math.Pi*120/256), thetaC)

	require.Equal(t, decad.Interfering, report.Status)
	require.NotEmpty(t, report.Collisions)
	first := report.Collisions[0]
	follower, stop := scene.parts[2].body, scene.parts[3].body
	require.Same(t, follower, first.A)
	require.Same(t, stop, first.B)
	require.Equal(t, 120.0/256, first.At.Mag())
	delta := rockerTheta4(2*math.Pi*first.At.Mag()) - thetaC
	u1, u2 := rockerStopFrom, rockerStopTo
	volume := rockerThick * ((u2-u1)*rockerEnd*(1/math.Cos(delta)-1) + math.Tan(delta)*(u2*u2-u1*u1)/2)
	require.InDelta(t, volume, first.Volume.Value.Base(), first.Volume.Bound.Base())
	require.Less(t, first.Volume.Bound.Base(), first.Volume.Value.Base())

	// The four pins are measured in their bores at every pose the check
	// evaluates.
	requireJointGaps(t, scene, report, 4)

	// The turnaround is the end of the proven-clear stretch, short of s*, and
	// nothing between it and the first collision is proven clear.
	require.Equal(t, 118.0/256, provenClear(report))
	turn, err := turnaroundOf(report)
	require.NoError(t, err)
	require.Equal(t, provenClear(report), turn.s)
	require.LessOrEqual(t, turn.s, sStar)
	require.Less(t, sStar, turn.hit.At.Mag())
	require.Same(t, first.A, turn.hit.A)

	clip, style, _, err := scene.clip(turn, linkageClipLength, smokeWidth, smokeHeight)
	require.NoError(t, err)
	holdFirst, holdLast := requireFlash(t, scene, clip, style, turn)
	require.Equal(t, 192, holdFirst)
	require.Equal(t, 224, holdLast)
	requireFlashPixels(t, clip, style, holdFirst, holdLast)
}
