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

	"github.com/lestrrat-3d/kinetograph"
	"github.com/lestrrat-3d/kinetograph/render"
)

// These tests run the folding arm through the real producers and the real
// viewer: decad's VerifyLinkage and Linkage.PoseAt, kinetograph's AddLinkage
// and its driven nodes, and four rendered frames.
//
// Each leg was seen red by breaking what it guards: the clip driven to one
// grid step past the turnaround fails the proven-clear leg; the nodes driven
// by a linear channel while the clip reports the eased one fails the
// bit-identical pose leg; flashFades switching one frame late fails the flash leg at frame
// 192; a hit colour of gold fails the pixel leg at frame 192; the stop's end
// moved 2 mm along X in the scene alone fails the volume leg; and the
// clearance kernel's windowed nested cell deleted fails the joint-contact
// leg, since the elbow pin and its bore then part by a few ulps at most poses
// and the declared pair publishes nothing there.

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

// provenClear is the end of the stretch [0, s] whose every interval report
// proves IntervalClear, read here from the report itself rather than through
// turnaroundOf.
func provenClear(report *decad.LinkageReport) float64 {
	end := 0.0
	for _, iv := range report.Intervals {
		if iv.Outcome != decad.IntervalClear {
			return end
		}
		end = iv.To.Mag()
	}
	return end
}

// requireClipPoses asserts that every frame of clip reads a drive fraction s
// with 0 ≤ s ≤ limit from fraction, the channel the clip's nodes read, that
// every part on a link — the hit copy
// included — takes exactly the transform poseAt returns at that s, bit for
// bit, and that the frames reading s = turn.s are exactly those inside the
// hold.
func requireClipPoses(t *testing.T, scene *linkageScene, clip *kinetograph.Clip, fraction *kinetograph.Channel,
	turn turnaround, limit float64, poseAt func(float64) (decad.LinkagePose, error)) {
	t.Helper()
	links := scene.linkage.Links()
	byName := make(map[string]int)
	for _, part := range scene.parts {
		if part.link == nil {
			continue
		}
		byName[part.name] = slices.Index(links, part.link)
		if part.body == turn.hit.A || part.body == turn.hit.B {
			byName[part.name+"-hit"] = slices.Index(links, part.link)
		}
	}
	require.Equal(t, int(linkageClipLength/(time.Second/linkageFPS)), clip.FrameCount())
	for i := range clip.FrameCount() {
		at, err := fraction.At(clip.FrameTime(i))
		require.NoError(t, err)
		s := at.Mag()
		require.GreaterOrEqual(t, s, 0.0, "frame %d", i)
		require.LessOrEqual(t, s, limit, "frame %d reads s = %v, past the proven-clear %v", i, s, limit)
		held := clip.FrameTime(i) >= linkageHoldStart && clip.FrameTime(i) <= linkageHoldEnd
		require.Equal(t, held, s == turn.s, "frame %d reads s = %v", i, s)
		want, err := poseAt(s)
		require.NoError(t, err)
		frame, err := clip.Frame(t.Context(), i)
		require.NoError(t, err)
		found := 0
		for _, pose := range frame.Poses {
			index, ok := byName[pose.Name]
			if !ok {
				continue
			}
			found++
			require.Equal(t, transformBits(want.Poses[index]), transformBits(pose.Transform),
				"part %s at frame %d", pose.Name, i)
		}
		require.Equal(t, len(byName), found, "frame %d", i)
	}
}

// requireFlash asserts that the first collision's two bodies, and no other
// part, switch colour, and that each draws its hit-coloured copy on exactly
// the frames of the hold at the turnaround and its own colour on every other
// frame. It returns the hold's first and last frame.
func requireFlash(t *testing.T, scene *linkageScene, clip *kinetograph.Clip, style render.Style,
	turn turnaround) (int, int) {
	t.Helper()
	first, last := -1, -1
	for _, part := range scene.parts {
		hitPart := part.body == turn.hit.A || part.body == turn.hit.B
		own, hit := style.Parts[part.name], style.Parts[part.name+"-hit"]
		if !hitPart {
			require.Nil(t, own.Fade, "part %s", part.name)
			continue
		}
		require.NotNil(t, own.Fade, "part %s", part.name)
		require.NotNil(t, hit.Fade, "part %s", part.name)
		for i := range clip.FrameCount() {
			ownAt, err := own.Fade.At(clip.FrameTime(i))
			require.NoError(t, err)
			hitAt, err := hit.Fade.At(clip.FrameTime(i))
			require.NoError(t, err)
			want := 0.0
			if clip.FrameTime(i) >= linkageHoldStart && clip.FrameTime(i) <= linkageHoldEnd {
				want = 1
				if first < 0 {
					first = i
				}
				last = i
			}
			require.Equal(t, want, hitAt.Mag(), "part %s at frame %d", part.name, i)
			require.Equal(t, 1-want, ownAt.Mag(), "part %s at frame %d", part.name, i)
		}
	}
	return first, last
}

// requireFlashPixels renders the frames on either side of each end of the
// hold, [first, last], and asserts the hit colour shows inside it only.
func requireFlashPixels(t *testing.T, clip *kinetograph.Clip, style render.Style, first, last int) {
	t.Helper()
	renderer, err := render.New(t.Context(), clip, style)
	require.NoError(t, err)
	for _, c := range []struct {
		frame int
		tint  bool
	}{{first - 1, false}, {first, true}, {last, true}, {last + 1, false}} {
		img, err := renderer.Frame(t.Context(), c.frame)
		require.NoError(t, err)
		require.Equal(t, c.tint, hitTinted(img) > 0, "frame %d", c.frame)
	}
}

// TestLinkageClipMatchesPoseAt asserts that the folding arm's clip shows only
// poses the check proves clear, each exactly as PoseAt builds it: every
// frame's drive fraction lies in [0, s], s the end of the stretch from 0 that
// VerifyLinkage proves clear, and every link's part takes PoseAt's transform
// at that fraction bit for bit.
func TestLinkageClipMatchesPoseAt(t *testing.T) {
	t.Parallel()
	scene, err := foldingArmScene(t.Context())
	require.NoError(t, err)
	report, err := scene.verify(t.Context())
	require.NoError(t, err)
	turn, err := turnaroundOf(report)
	require.NoError(t, err)
	clip, _, fraction, err := scene.clip(turn, linkageClipLength, smokeWidth, smokeHeight)
	require.NoError(t, err)
	requireClipPoses(t, scene, clip, fraction, turn, provenClear(report), func(s float64) (decad.LinkagePose, error) {
		return scene.linkage.PoseAt(scene.drive, units.Scalar(s))
	})

	// The forearm keeps its orientation and rides the elbow's circle: at the
	// turnaround the top of its tip end, (96, 12, 20), sits at
	// (48·cos θ + 48, 48·sin θ + 12, 20) with θ = 90°·s.
	frame, err := clip.Frame(t.Context(), int(linkageHoldStart/(time.Second/linkageFPS)))
	require.NoError(t, err)
	theta := math.Pi / 2 * turn.s
	checked := false
	for _, pose := range frame.Poses {
		if pose.Name != "forearm" {
			continue
		}
		checked = true
		top := linkageForearmZ + linkageBarThick
		corner := pose.Transform.Apply(r3.NewVec(2*linkageElbow, linkageEnd, top))
		require.InDelta(t, linkageElbow*math.Cos(theta)+linkageElbow, corner.X, 1e-9)
		require.InDelta(t, linkageElbow*math.Sin(theta)+linkageEnd, corner.Y, 1e-9)
		require.InDelta(t, top, corner.Z, 1e-9)
	}
	require.True(t, checked)
}

// TestLinkageClipTurnsAtTheStop asserts the scene's verdict and its
// turnaround against their closed form, and the flash against the turnaround. The
// forearm's upper flank y = 48·sin θ + 12 reaches the stop's face y = 37.5
// at sin θ = 25.5/48, θ* = asin(17/32) ≈ 32.09°, s* = θ*/90° ≈ 0.3566. The
// check bisects a dyadic grid down to 1/256, so its first collision is the
// first grid point past s*, 92/256.
// There the forearm reaches h = 48·sin θ − 25.5 past the face, and its overlap
// with the stop, 8 mm deep along Z, is the strip of the flat flank from the
// stop's end x = 64 to the tip's centre x = 48·cos θ + 48, plus the half of
// the tip's circular segment of height h beyond that centre:
//
//	V = 8·((48·cos θ − 16)·h + seg(h)/2),  seg(h) = R²·acos((R − h)/R) − (R − h)·√(2Rh − h²),  R = 12
//
// Every interval up to 91/256 is proven clear, so the clip turns there, the
// last grid point before s*, and holds frames 192 to 224 at it, where the
// forearm and the stop flash in the hit colour.
func TestLinkageClipTurnsAtTheStop(t *testing.T) {
	t.Parallel()
	scene, err := foldingArmScene(t.Context())
	require.NoError(t, err)
	report, err := scene.verify(t.Context())
	require.NoError(t, err)

	sStar := math.Asin((linkageStopFace-linkageEnd)/linkageElbow) / (math.Pi / 2)
	require.InDelta(t, math.Asin(17.0/32)/(math.Pi/2), sStar, 1e-15)
	grid := math.Ceil(sStar/linkageResolution) * linkageResolution
	require.Equal(t, 92.0/256, grid)

	require.Equal(t, decad.Interfering, report.Status)
	require.NotEmpty(t, report.Collisions)
	first := report.Collisions[0]
	forearm, stop := scene.parts[1].body, scene.parts[3].body
	require.Same(t, forearm, first.A)
	require.Same(t, stop, first.B)
	require.Equal(t, grid, first.At.Mag())
	require.Greater(t, first.At.Mag(), sStar)
	theta := math.Pi / 2 * first.At.Mag()
	h := linkageElbow*math.Sin(theta) + linkageEnd - linkageStopFace
	r := linkageEnd
	seg := r*r*math.Acos((r-h)/r) - (r-h)*math.Sqrt(2*r*h-h*h)
	volume := linkageBarThick * ((linkageElbow*math.Cos(theta)+linkageElbow-linkageStopX)*h + seg/2)
	require.InDelta(t, volume, first.Volume.Value.Base(), first.Volume.Bound.Base())
	require.Less(t, first.Volume.Bound.Base(), first.Volume.Value.Base())

	// Both pins are measured in their bores at 0.5 mm at every pose the check
	// evaluates.
	requireJointGaps(t, scene, report, 2)

	// The turnaround is the last grid point the check proves clear before
	// s*, and the first collision the next one.
	require.Equal(t, 91.0/256, provenClear(report))
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

// requireJointGaps asserts that report declares want joint contacts and that
// every pose it evaluates measures each of them at the 0.5 mm every pin
// clears its bore by, on the bore's centre.
func requireJointGaps(t *testing.T, scene *linkageScene, report *decad.LinkageReport, want int) {
	t.Helper()
	require.Len(t, report.JointContacts, want)
	require.NotEmpty(t, report.Poses)
	for _, pose := range report.Poses {
		measured := 0
		for _, c := range pose.Clearances {
			for _, jc := range report.JointContacts {
				if (jc.A != c.A || jc.B != c.B) && (jc.A != c.B || jc.B != c.A) {
					continue
				}
				measured++
				require.InDelta(t, 0.5, c.Gap.Value.Base(), 1e-9, "%s/%s at s = %v",
					scene.partName(c.A), scene.partName(c.B), pose.Pose.At.Mag())
				require.LessOrEqual(t, c.Gap.Bound.Base(), 1e-9, "%s/%s at s = %v",
					scene.partName(c.A), scene.partName(c.B), pose.Pose.At.Mag())
			}
		}
		require.Equal(t, want, measured, "joint contacts measured at s = %v", pose.Pose.At.Mag())
	}
}
