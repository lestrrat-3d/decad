package main

import (
	"math"
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"

	"github.com/lestrrat-3d/kinetograph/render"
)

// These tests run the crank-rocker, a closed loop, through the real
// producers and the real viewer: decad's VerifyLinkage and Schedule,
// kinetograph's schedule tracks (Scene.AddSchedule), and two rendered frames.
//
// Each leg was seen red by breaking what it guards: the schedule tracks read
// through a drive fraction perturbed to 1 + 1e-12 at the drive's end fail the
// bit-identical pose leg at frame 1; a hit colour of gold fails the pixel
// leg at frame 120; the stop's angle moved to 144.6° fails the volume leg;
// and the coupler pins set on their bores' centres fail the joint-contact
// leg.

// TestLinkageLoopClipMatchesSchedule asserts that every link's part in the
// crank-rocker's clip takes exactly the transform the scene's Schedule
// returns at the frame's drive fraction, frame i of the drive reading
// s = i/256, and that the follower — a dependent joint — turns as the
// four-bar's construction says: θ4(θ2) − θ4(0), with θ2 = 360°·s, within
// 1e-9 rad of its pose's own rotation.
func TestLinkageLoopClipMatchesSchedule(t *testing.T) {
	t.Parallel()
	scene, err := crankRockerScene(t.Context())
	require.NoError(t, err)
	clip, _, err := scene.clip(nil, linkageClipLength, smokeWidth, smokeHeight)
	require.NoError(t, err)

	links := scene.linkage.Links()
	byName := make(map[string]int)
	for _, part := range scene.parts {
		if part.link != nil {
			byName[part.name] = slices.Index(links, part.link)
		}
	}
	for _, c := range []struct {
		frame int
		s     float64
	}{{0, 0}, {1, 1.0 / 256}, {119, 119.0 / 256}, {120, 120.0 / 256}, {200, 200.0 / 256}, {256, 1}, {300, 1}} {
		frame, err := clip.Frame(t.Context(), c.frame)
		require.NoError(t, err)
		want, err := scene.schedule.PoseAt(t.Context(), units.Scalar(c.s))
		require.NoError(t, err)
		found := 0
		for _, pose := range frame.Poses {
			index, ok := byName[pose.Name]
			if !ok {
				continue
			}
			found++
			require.Equal(t, transformBits(want.Poses[index]), transformBits(pose.Transform),
				"part %s at frame %d", pose.Name, c.frame)
		}
		require.Equal(t, len(byName), found, "frame %d", c.frame)

		follower := want.Poses[byName["follower"]].Basis().EX
		turn := math.Atan2(follower.Y, follower.X)
		th2 := c.s * 2 * math.Pi
		require.InDelta(t, rockerTheta4(th2)-rockerTheta4(0), turn, 1e-9, "frame %d", c.frame)
	}
}

// TestLinkageLoopClipMarksFirstCollision asserts the crank-rocker's verdict
// against its closed form and the frame the clip marks against the verdict.
// The stop's face lies along the follower's leading flank at θc = 144.5°, and
// the follower first reaches θc at the crank angle the four-bar puts the
// coupler pin at B = O4 + 70·(cos θc, sin θc):
//
//	θ2* = atan2(B) + acos((30² + |B|² − 80²)/(2·30·|B|)) ≈ 167.62°,  s* = θ2*/360° ≈ 0.4656
//
// At 1/256 the first collision is the first grid point past it, 120/256,
// and the clip draws the follower in the hit colour from frame 120 on.
// There the follower has turned δ = θ4 − θc past the face, and the flank's
// overlap with the stop, from u1 = 30 to u2 = 60 mm along the follower and
// 6 mm deep along Z, is the wedge between the two lines, both 8 mm off the
// pivot:
//
//	V = 6·((u2 − u1)·8·(1/cos δ − 1) + tan δ·(u2² − u1²)/2)
func TestLinkageLoopClipMarksFirstCollision(t *testing.T) {
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

	clip, style, err := scene.clip(report, linkageClipLength, smokeWidth, smokeHeight)
	require.NoError(t, err)
	marked, err := firstFrameAt(clip, first.At.Mag())
	require.NoError(t, err)
	require.Equal(t, 120, marked)
	renderer, err := render.New(t.Context(), clip, style)
	require.NoError(t, err)
	for _, c := range []struct {
		frame int
		tint  bool
	}{{marked - 1, false}, {marked, true}} {
		img, err := renderer.Frame(t.Context(), c.frame)
		require.NoError(t, err)
		require.Equal(t, c.tint, hitTinted(img) > 0, "frame %d", c.frame)
	}
}
