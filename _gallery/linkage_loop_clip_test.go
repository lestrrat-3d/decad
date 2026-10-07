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
// bit-identical pose leg at frame 1; and a hit colour of gold fails the pixel
// leg at frame 36.

// TestLinkageLoopClipMatchesSchedule asserts that every link's part in the
// crank-rocker's clip takes exactly the transform the scene's Schedule
// returns at the frame's drive fraction, frame i of the drive reading
// s = i/256, and that the follower — a dependent joint — turns as the
// four-bar's construction says: θ4(θ2) − θ4(0) within 1e-9 rad of its pose's
// own rotation.
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
	}{{0, 0}, {1, 1.0 / 256}, {36, 36.0 / 256}, {110, 110.0 / 256}, {256, 1}, {300, 1}} {
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
		th2 := c.s * math.Pi / 2
		require.InDelta(t, rockerTheta4(th2)-rockerTheta4(0), turn, 1e-9, "frame %d", c.frame)
	}
}

// TestLinkageLoopClipMarksFirstCollision asserts the crank-rocker's verdict
// against its closed form and the frame the clip marks against the verdict.
// The follower's top corner B − 4·n4, n4 = (−sin θ4, cos θ4), sits at
// y = 70·sin θ4 − 4·cos θ4 and rises past the wall's face y = 68.5 at
// θ2 = 12.625006°, s₁ = 0.140278; at 1/256 the first collision is the first
// grid point past it, 36/256, and the clip draws the follower in the hit
// colour from frame 36 on.
func TestLinkageLoopClipMarksFirstCollision(t *testing.T) {
	t.Parallel()
	scene, err := crankRockerScene(t.Context())
	require.NoError(t, err)
	report, err := scene.verify(t.Context())
	require.NoError(t, err)

	top := func(s float64) float64 {
		th4 := rockerTheta4(s * math.Pi / 2)
		return rockerFollower*math.Sin(th4) - rockerHalf*math.Cos(th4)
	}
	require.Less(t, top(35.0/256), rockerWallFace)
	require.Greater(t, top(36.0/256), rockerWallFace)

	require.Equal(t, decad.Interfering, report.Status)
	require.NotEmpty(t, report.Collisions)
	first := report.Collisions[0]
	follower, wall := scene.parts[2].body, scene.parts[3].body
	require.Same(t, follower, first.A)
	require.Same(t, wall, first.B)
	require.Equal(t, 36.0/256, first.At.Mag())

	clip, style, err := scene.clip(report, linkageClipLength, smokeWidth, smokeHeight)
	require.NoError(t, err)
	marked, err := firstFrameAt(clip, first.At.Mag())
	require.NoError(t, err)
	require.Equal(t, 36, marked)
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
