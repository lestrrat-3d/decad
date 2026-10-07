package main

import (
	"image"
	"math"
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"

	"github.com/lestrrat-3d/kinetograph/render"
)

// These tests run the folding arm through the real producers and the real
// viewer: decad's VerifyLinkage and Linkage.PoseAt, kinetograph's AddLinkage
// and its driven nodes, and one rendered frame.
//
// Each leg was seen red by breaking what it guards: a drive fraction channel
// ending one frame late (4 s plus 1/64 s) fails the bit-identical pose leg;
// firstFrameAt testing > instead of >= fails the
// marked-frame leg (frame 93); hitFades switching one frame late fails the
// fade leg at frame 92; a hit colour of gold fails the pixel leg at
// frame 92; the stop's end moved 2 mm along X in the scene alone fails the
// volume leg; and the elbow pin set on its bore's centre fails the
// joint-contact leg.

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

// TestLinkageClipMatchesPoseAt asserts that every link's part in the clip
// takes exactly the transform PoseAt returns at the frame's drive fraction:
// frame i of the drive at 64 fps reads s = i/256 exactly, and a frame past the
// drive holds s = 1.
func TestLinkageClipMatchesPoseAt(t *testing.T) {
	t.Parallel()
	scene, err := foldingArmScene(t.Context())
	require.NoError(t, err)
	clip, _, err := scene.clip(nil, linkageClipLength, smokeWidth, smokeHeight)
	require.NoError(t, err)

	byName := make(map[string]*decad.Link)
	for _, part := range scene.parts {
		if part.link != nil {
			byName[part.name] = part.link
		}
	}
	require.Len(t, byName, 5, "the upper arm, the forearm and the elbow pin's three bodies")
	index := func(link *decad.Link) int { return slices.Index(scene.linkage.Links(), link) }
	for _, c := range []struct {
		frame int
		s     float64
	}{
		{0, 0},
		{1, 1.0 / 256},
		{91, 91.0 / 256},
		{92, 92.0 / 256},
		{171, 171.0 / 256},
		{256, 1},
		{300, 1}, // the hold after the drive
	} {
		frame, err := clip.Frame(t.Context(), c.frame)
		require.NoError(t, err)
		want, err := scene.linkage.PoseAt(scene.drive, units.Scalar(c.s))
		require.NoError(t, err)
		found := 0
		for _, pose := range frame.Poses {
			link, ok := byName[pose.Name]
			if !ok {
				continue
			}
			found++
			require.Equal(t, transformBits(want.Poses[index(link)]), transformBits(pose.Transform),
				"part %s at frame %d", pose.Name, c.frame)
		}
		require.Equal(t, len(byName), found, "frame %d", c.frame)
	}

	// The forearm keeps its orientation and rides the elbow's circle: at
	// s = 92/256 the top of its tip end, (96, 12, 20), sits at
	// (48·cos θ + 48, 48·sin θ + 12, 20) with θ = 90°·92/256.
	frame, err := clip.Frame(t.Context(), 92)
	require.NoError(t, err)
	theta := math.Pi / 2 * 92 / 256
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

// TestLinkageClipMarksFirstCollision asserts the scene's verdict against its
// closed form and the frame the clip marks against the verdict. The
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
// The clip draws the forearm in its own colour through frame 91 and in the
// hit colour from frame 92 on.
func TestLinkageClipMarksFirstCollision(t *testing.T) {
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

	// Both pins are measured in their bores at every pose the check
	// evaluates: the shoulder post on its bore's centre at 0.5 mm, the elbow
	// pin pinOffset off centre at between 0.25 and 0.75 mm.
	requireJointGaps(t, scene, report, 2)

	clip, style, err := scene.clip(report, linkageClipLength, smokeWidth, smokeHeight)
	require.NoError(t, err)
	require.Equal(t, 320, clip.FrameCount())
	marked, err := firstFrameAt(clip, first.At.Mag())
	require.NoError(t, err)
	require.Equal(t, 92, marked)
	fraction, err := linkageFraction()
	require.NoError(t, err)
	at, err := fraction.At(clip.FrameTime(marked))
	require.NoError(t, err)
	require.Equal(t, first.At.Mag(), at.Mag())

	// Every frame draws exactly one copy of the forearm: its own colour
	// before the marked frame, the hit colour from it on. The upper arm and
	// the stop carry no fade.
	own, hit := style.Parts["forearm"], style.Parts["forearm-hit"]
	require.NotNil(t, own.Fade)
	require.NotNil(t, hit.Fade)
	require.Nil(t, style.Parts["upper-arm"].Fade)
	require.Nil(t, style.Parts["stop"].Fade)
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

// requireJointGaps asserts that report declares want joint contacts and that
// every pose it evaluates measures each of them: a gap of 0.5 mm for a pin
// on its bore's centre, and one between 0.5 − pinOffset and 0.5 + pinOffset
// for a pin set pinOffset off it.
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
				gap := c.Gap.Value.Base()
				require.GreaterOrEqual(t, gap, 0.5-pinOffset-1e-9, "%s/%s at s = %v",
					scene.partName(c.A), scene.partName(c.B), pose.Pose.At.Mag())
				require.LessOrEqual(t, gap, 0.5+pinOffset+1e-9, "%s/%s at s = %v",
					scene.partName(c.A), scene.partName(c.B), pose.Pose.At.Mag())
			}
		}
		require.Equal(t, want, measured, "joint contacts measured at s = %v", pose.Pose.At.Mag())
	}
}
