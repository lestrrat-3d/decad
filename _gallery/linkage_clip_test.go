package main

import (
	"image"
	"math"
	"slices"
	"testing"
	"time"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"

	"github.com/lestrrat-3d/kinetograph/render"
)

// These tests run the folding arm through the real producers and the real
// viewer: decad's VerifyLinkage and Linkage.PoseAt, the linkageTrack bridge,
// kinetograph's driven nodes and one rendered frame.
//
// Each leg was seen red by breaking what it guards: driveFraction dividing by
// one nanosecond more than the duration, driveFraction without its upper
// clamp, and a track returning the first link's pose for every link each fail
// the bit-identical pose leg; firstFrameAt testing > instead of >= fails the
// marked-frame leg (frame 87); hitFades switching one frame late fails the
// fade leg at frame 86; and a hit colour of gold fails the pixel leg at
// frame 86.

// transformBits are a transform's twelve components as raw float64 bits, so
// two transforms compare equal only when they are bit-identical.
func transformBits(tr r3.Transform) [12]uint64 {
	b := tr.Basis()
	t := tr.Translation()
	var out [12]uint64
	for i, v := range []r3.Vec{b.EX, b.EY, b.EZ, t} {
		out[3*i] = math.Float64bits(v.X)
		out[3*i+1] = math.Float64bits(v.Y)
		out[3*i+2] = math.Float64bits(v.Z)
	}
	return out
}

// TestLinkageTrackMatchesPoseAt asserts that every link's driven node takes
// exactly the transform PoseAt returns at the frame's drive fraction: frame i
// of the drive at 64 fps reads s = i/256 exactly, and a time past the drive
// holds s = 1.
func TestLinkageTrackMatchesPoseAt(t *testing.T) {
	t.Parallel()
	scene, err := foldingArmScene(t.Context())
	require.NoError(t, err)

	frame := time.Second / linkageFPS
	for _, link := range scene.linkage.Links() {
		track, err := newLinkageTrack(scene.linkage, scene.drive, link, linkageDriveDuration)
		require.NoError(t, err)
		index := track.index
		require.Same(t, link, scene.linkage.Links()[index])
		for _, c := range []struct {
			at time.Duration
			s  float64
		}{
			{0, 0},
			{frame, 1.0 / 256},
			{85 * frame, 85.0 / 256},
			{86 * frame, 86.0 / 256},
			{171 * frame, 171.0 / 256},
			{256 * frame, 1},
			{300 * frame, 1}, // the hold after the drive
			{-frame, 0},
		} {
			got, err := track.At(c.at)
			require.NoError(t, err)
			want, err := scene.linkage.PoseAt(scene.drive, units.Scalar(c.s))
			require.NoError(t, err)
			require.Equal(t, transformBits(want.Poses[index]), transformBits(got), "link %d at %s", index, c.at)
		}
	}

	// The forearm keeps its orientation and rides the elbow's circle: at
	// s = 86/256 its corner (96, 14, 22) sits at (48·cos θ + 48,
	// 48·sin θ + 14, 22) with θ = 90°·86/256.
	elbow, err := newLinkageTrack(scene.linkage, scene.drive, scene.elbow, linkageDriveDuration)
	require.NoError(t, err)
	pose, err := elbow.At(86 * frame)
	require.NoError(t, err)
	theta := math.Pi / 2 * 86 / 256
	corner := pose.Apply(r3.NewVec(2*linkageElbow, linkageHalfWidth, 22))
	require.InDelta(t, linkageElbow*math.Cos(theta)+linkageElbow, corner.X, 1e-9)
	require.InDelta(t, linkageElbow*math.Sin(theta)+linkageHalfWidth, corner.Y, 1e-9)
	require.InDelta(t, 22, corner.Z, 1e-9)
}

// TestLinkageClipMarksFirstCollision asserts the scene's verdict against its
// closed form and the frame the clip marks against the verdict. The forearm's
// top face y = 48·sin θ + 14 reaches the wall face y = 38 at sin θ = 1/2,
// θ* = 30°, s* = 1/3. The check bisects a dyadic grid down to 1/256, so its
// first collision is the first grid point past s*, 86/256, where the overlap
// is the slab 48 × (48·sin θ − 24) × 10 mm³. The clip draws the forearm in its
// own colour through frame 85 and in the hit colour from frame 86 on.
func TestLinkageClipMarksFirstCollision(t *testing.T) {
	t.Parallel()
	scene, err := foldingArmScene(t.Context())
	require.NoError(t, err)
	report, err := scene.verify(t.Context())
	require.NoError(t, err)

	sStar := math.Asin((linkageWallFace-linkageHalfWidth)/linkageElbow) / (math.Pi / 2)
	require.InDelta(t, 1.0/3, sStar, 1e-15)
	grid := math.Ceil(sStar/linkageResolution) * linkageResolution
	require.Equal(t, 86.0/256, grid)

	require.Equal(t, decad.Interfering, report.Status)
	require.NotEmpty(t, report.Collisions)
	first := report.Collisions[0]
	forearm, wall := scene.parts[1].body, scene.parts[2].body
	require.Same(t, forearm, first.A)
	require.Same(t, wall, first.B)
	require.Equal(t, grid, first.At.Mag())
	require.Greater(t, first.At.Mag(), sStar)
	theta := math.Pi / 2 * first.At.Mag()
	slab := 48 * (linkageElbow*math.Sin(theta) + linkageHalfWidth - linkageWallFace) * 10
	require.InDelta(t, slab, first.Volume.Value.Base(), 1e-3)
	require.Less(t, first.Volume.Bound.Base(), first.Volume.Value.Base())

	clip, style, err := scene.clip(report, linkageClipLength, smokeWidth, smokeHeight)
	require.NoError(t, err)
	require.Equal(t, 320, clip.FrameCount())
	marked := firstFrameAt(clip, first.At.Mag())
	require.Equal(t, 86, marked)
	require.Equal(t, first.At.Mag(), driveFraction(clip.FrameTime(marked), linkageDriveDuration))

	// Every frame draws exactly one copy of the forearm: its own colour
	// before the marked frame, the hit colour from it on. The upper arm and
	// the wall carry no fade.
	own, hit := style.Parts["forearm"], style.Parts["forearm-hit"]
	require.NotNil(t, own.Fade)
	require.NotNil(t, hit.Fade)
	require.Nil(t, style.Parts["upper-arm"].Fade)
	require.Nil(t, style.Parts["wall"].Fade)
	for i := range clip.FrameCount() {
		ownAt, err := own.Fade.At(clip.FrameTime(i))
		require.NoError(t, err)
		hitAt, err := hit.Fade.At(clip.FrameTime(i))
		require.NoError(t, err)
		want := 0.0
		if i >= marked {
			want = 1
		}
		require.Equal(t, want, hitAt.Mag(), "frame %d", i)
		require.Equal(t, 1-want, ownAt.Mag(), "frame %d", i)
	}

	// The marked frame renders, with the forearm on the node its link drives.
	frame, err := clip.Frame(t.Context(), marked)
	require.NoError(t, err)
	want, err := scene.linkage.PoseAt(scene.drive, first.At)
	require.NoError(t, err)
	elbow := slices.Index(scene.linkage.Links(), scene.elbow)
	require.GreaterOrEqual(t, elbow, 0)
	found := 0
	for _, pose := range frame.Poses {
		if pose.Name != "forearm" && pose.Name != "forearm-hit" {
			continue
		}
		found++
		require.Equal(t, transformBits(want.Poses[elbow]), transformBits(pose.Transform), "part %s", pose.Name)
	}
	require.Equal(t, 2, found)

	// The frame before the mark draws no pixel in the hit colour, and the
	// marked frame draws some.
	renderer, err := render.New(t.Context(), clip, style)
	require.NoError(t, err)
	for _, c := range []struct {
		frame int
		tint  bool
	}{{marked - 1, false}, {marked, true}} {
		img, err := renderer.Frame(t.Context(), c.frame)
		require.NoError(t, err)
		require.Equal(t, smokeWidth, img.Bounds().Dx())
		require.Equal(t, smokeHeight, img.Bounds().Dy())
		require.Equal(t, c.tint, hitTinted(img) > 0, "frame %d", c.frame)
	}
}

// hitTinted counts img's pixels in the hit colour: red well above both green
// and blue, which coral is under every light of the style and no other part's
// colour is.
func hitTinted(img *image.RGBA) int {
	n := 0
	for y := range img.Bounds().Dy() {
		for x := range img.Bounds().Dx() {
			c := img.RGBAAt(x, y)
			if int(c.R)-int(c.G) > 90 && int(c.R)-int(c.B) > 90 {
				n++
			}
		}
	}
	return n
}
