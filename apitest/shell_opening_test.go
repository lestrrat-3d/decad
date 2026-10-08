package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// polygonPrism extrudes a polygon on XY by 10 along +z, the sweep of every
// docs/shell-opening-design.md §9 fixture.
func polygonPrism(t *testing.T, pts [][2]float64) (*decad.Document, *decad.Body) {
	t.Helper()
	s, p := polygonSketch(t, pts)
	doc := decad.New()
	body, err := doc.Extrude(s, p, decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	require.NoError(t, err)
	return doc, body
}

// planeFaces lists a body's planar faces whose outward normal is n and whose
// plane lies at offset along n.
func planeFaces(t *testing.T, b *decad.Body, n r3.Vec, offset float64) []*decad.Face {
	t.Helper()
	var out []*decad.Face
	for _, f := range b.Faces() {
		plane, ok := f.Surface().(decad.Plane)
		if !ok || plane.Frame.N() != n {
			continue
		}
		if plane.Frame.Origin().Dot(n) == offset {
			out = append(out, f)
		}
	}
	return out
}

// sideFaceAt selects the receiver's one side face facing n at offset along
// it, by the role it was created with.
func sideFaceAt(t *testing.T, b *decad.Body, n r3.Vec, offset float64) *decad.FaceQuery {
	t.Helper()
	found := planeFaces(t, b, n, offset)
	require.Len(t, found, 1)
	return decad.Faces(decad.FaceCreatedBy(found[0].Origins()[0]))
}

// loopBox is the axis-aligned box a face loop's vertices span.
func loopBox(l *decad.Loop) (r3.Vec, r3.Vec) {
	lo := r3.NewVec(math.Inf(1), math.Inf(1), math.Inf(1))
	hi := r3.NewVec(math.Inf(-1), math.Inf(-1), math.Inf(-1))
	for _, c := range l.CoEdges() {
		p := c.Start().Position().Value
		lo = r3.NewVec(math.Min(lo.X, p.X), math.Min(lo.Y, p.Y), math.Min(lo.Z, p.Z))
		hi = r3.NewVec(math.Max(hi.X, p.X), math.Max(hi.Y, p.Y), math.Max(hi.Z, p.Z))
	}
	return lo, hi
}

// requireExactVolumeArea asserts an Exact volume and area equal to their
// closed forms.
func requireExactVolumeArea(t *testing.T, b *decad.Body, volume, area float64) {
	t.Helper()
	vol, err := b.Volume()
	require.NoError(t, err)
	require.Equal(t, decad.Exact, vol.Exactness)
	require.Equal(t, volume, volumeMM(t, vol))
	a, err := b.Area()
	require.NoError(t, err)
	require.Equal(t, decad.Exact, a.Exactness)
	require.Equal(t, area, a.Value.Base())
}

// requireSoundAndMeshed asserts the document verifies Sound and the body
// meshes watertight with its occupied-volume proof, the mesh enclosing the
// closed-form volume (every face planar, so the chords are the faces).
func requireSoundAndMeshed(t *testing.T, doc *decad.Document, b *decad.Body, volume float64) {
	t.Helper()
	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.Equal(t, decad.Sound, report.Status)
	mesh, err := b.Tessellate(t.Context(), units.Millimeters(0.05), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	requireWatertight(t, mesh)
	require.True(t, mesh.VolumeVerified())
	require.InDelta(t, volume, meshVolume(mesh), 1e-9*volume)
}

// requireBrepRoles asserts a BO2 result: every face a brep role, and no cap
// role minted.
func requireBrepRoles(t *testing.T, b *decad.Body) {
	t.Helper()
	for _, f := range b.Faces() {
		require.Regexp(t, `^face\(\d+\)$`, f.Origins()[0].Role)
	}
	_, err := decad.Faces(decad.FaceCreatedBy(decad.CapStart(b))).SelectFaces(b)
	require.ErrorIs(t, err, decad.ErrNoMatch)
}

// uChannelBox is the U-channel fixture's 40×20 section.
var uChannelBox = [][2]float64{{0, 0}, {40, 0}, {40, 20}, {0, 20}}

// TestShellSideOpeningUChannel is docs/shell-opening-design.md §9's U-channel:
// the box x∈[0,40], y∈[0,20], z∈[0,10] with its x = 0 face removed, t = 2.
// Inward with both caps kept the cavity is (0,38)×(2,18)×(2,8): 8000 − 3648,
// the area 2704 + 1768, 11 faces, the x = 0 plane one face whose hole is the
// opening. Shown to fail with brepgeom.StackedWallRegions' hole grouping
// replaced by the one-face-per-loop rule (the opening's clockwise loop then
// missed, SO5).
func TestShellSideOpeningUChannel(t *testing.T) {
	t.Parallel()
	minusX := r3.NewVec(-1, 0, 0)
	opening := func(t *testing.T, box *decad.Body) *decad.FaceQuery {
		t.Helper()
		return sideFaceAt(t, box, minusX, 0)
	}
	// requireHole asserts the x = 0 plane is one face with one hole spanning
	// lo to hi.
	requireHole := func(t *testing.T, b *decad.Body, lo, hi r3.Vec) {
		t.Helper()
		opened := planeFaces(t, b, minusX, 0)
		require.Len(t, opened, 1, "the x = 0 plane is one face")
		loops := opened[0].Loops()
		require.Len(t, loops, 2, "an outer loop and the opening")
		require.True(t, loops[0].IsOuter())
		require.False(t, loops[1].IsOuter())
		gotLo, gotHi := loopBox(loops[1])
		require.Equal(t, lo, gotLo)
		require.Equal(t, hi, gotHi)
	}

	t.Run("inward, both caps kept", func(t *testing.T) {
		t.Parallel()
		doc, box := polygonPrism(t, uChannelBox)
		body, err := box.Shell(t.Context(), opening(t, box), units.Millimeters(2))
		require.NoError(t, err)
		requireExactVolumeArea(t, body, 4352, 4472)
		c, err := body.Centroid()
		require.NoError(t, err)
		require.InDelta(t, 1417.0/68, c.Value.X, 1e-12)
		require.InDelta(t, 10, c.Value.Y, 1e-12)
		require.InDelta(t, 5, c.Value.Z, 1e-12)
		require.Len(t, body.Faces(), 11)
		require.Len(t, body.Lumps(), 1)
		require.Len(t, body.Shells(), 1)
		require.False(t, body.Shells()[0].IsVoid(), "the cavity reaches the outside through the opening")
		requireHole(t, body, r3.NewVec(0, 2, 2), r3.NewVec(0, 18, 8))
		requireBrepRoles(t, body)
		require.Equal(t, []*decad.Body{body}, doc.Bodies())
		requireSoundAndMeshed(t, doc, body, 4352)

		// Table DO's DO12: a placement re-lifts every face.
		rotation, err := r3.Rotation(r3.NewVec(1, 2, 3), units.Degrees(37))
		require.NoError(t, err)
		placed, err := body.Placed(t.Context(), rotation)
		require.NoError(t, err)
		require.Len(t, placed.Faces(), 11)
		requireVolumeNear(t, placed, 4352)
		mesh, err := placed.Tessellate(t.Context(), units.Millimeters(0.05))
		require.NoError(t, err)
		requireWatertight(t, mesh)
	})

	t.Run("inward, the end cap removed too", func(t *testing.T) {
		t.Parallel()
		doc, box := polygonPrism(t, uChannelBox)
		sel := opening(t, box).Or(decad.FaceCreatedBy(decad.CapEnd(box)))
		body, err := box.Shell(t.Context(), sel, units.Millimeters(2))
		require.NoError(t, err)
		// The cavity (0,38)×(2,18)×(2,10), 8 tall: 8000 − 608·8. The area is
		// the box's 2800 less the 16×8 opening and the 38×16 top, plus the
		// cavity's floor 608 and its three walls 8·(38 + 16 + 38).
		requireExactVolumeArea(t, body, 3136, 2800-128-608+608+8*92)
		require.Len(t, body.Faces(), 10)
		requireSoundAndMeshed(t, doc, body, 3136)
	})

	t.Run("both caps removed is a prism over the wall section", func(t *testing.T) {
		t.Parallel()
		doc, box := polygonPrism(t, uChannelBox)
		body, err := box.Shell(t.Context(), opening(t, box).Or(decad.NormalTo(r3.NewVec(0, 0, 1))), units.Millimeters(2))
		require.NoError(t, err)
		vol, err := body.Volume()
		require.NoError(t, err)
		require.Equal(t, decad.Exact, vol.Exactness)
		require.Equal(t, 1920.0, volumeMM(t, vol))
		for _, cap := range []decad.FeatureRef{decad.CapStart(body), decad.CapEnd(body)} {
			faces, err := decad.Faces(decad.FaceCreatedBy(cap)).SelectFaces(body)
			require.NoError(t, err)
			require.Len(t, faces, 1)
		}
		// The U section's eight walls, the two rims among them, and two caps.
		require.Len(t, body.Faces(), 10)
		requireSoundAndMeshed(t, doc, body, 1920)
	})

	t.Run("both caps removed at t = 5 keeps every walk", func(t *testing.T) {
		t.Parallel()
		_, box := polygonPrism(t, uChannelBox)
		body, err := box.Shell(t.Context(), opening(t, box).Or(decad.NormalTo(r3.NewVec(0, 0, 1))), units.Millimeters(5))
		require.NoError(t, err)
		// The far wall x = 40 is 20 long and loses 5 at each end, so the
		// cavity is (0,35)×(5,15): (800 − 350)·10.
		requireExactVolumeArea(t, body, 4500, 2*450+10*(40+20+40+5+35+10+35+5))
	})

	t.Run("outward, both caps removed is a prism with rounded corners", func(t *testing.T) {
		t.Parallel()
		doc, box := polygonPrism(t, uChannelBox)
		sel := opening(t, box).Or(decad.NormalTo(r3.NewVec(0, 0, 1)))
		body, err := box.Shell(t.Context(), sel, units.Millimeters(2), decad.WithShellSense(decad.Outward))
		require.NoError(t, err)
		// W is O = [0,42]×[−2,22] with its two corners beyond x = 40 rounded
		// to radius 2 (modify §7's arc join), less the box's section: area
		// 1008 − 2(4 − π) − 800 over the sweep of 10.
		volume := (208 - 2*(4-math.Pi)) * 10
		requireVolumeNear(t, body, volume)
		require.Len(t, body.Faces(), 12, "the U's ten walls with its two corner cylinders, and two caps")
		report, err := doc.Verify(t.Context())
		require.NoError(t, err)
		require.Equal(t, decad.Sound, report.Status)
	})

	t.Run("outward, both caps kept, an arc join is staged (SO5)", func(t *testing.T) {
		t.Parallel()
		// The two convex corners of K offset outward to arcs tangent to
		// their neighbours, and the stacked record build keys no tangent
		// junction of a line with a circle: an engine miss, SO5.
		doc, box := polygonPrism(t, uChannelBox)
		before := snapshotDocument(t, doc)
		_, err := box.Shell(t.Context(), opening(t, box), units.Millimeters(2), decad.WithShellSense(decad.Outward))
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.ErrorContains(t, err, "SO5")
		require.Equal(t, before.bodies, doc.Bodies())
	})

	t.Run("outward, both caps kept, one kept wall", func(t *testing.T) {
		t.Parallel()
		// Removing x = 0, y = 20 and x = 40 keeps the y = 0 wall alone: both
		// rims run backward along the removed carriers to (0,−2) and
		// (40,−2), so O = [0,40]×[−2,20] over [−2,12], less the box.
		doc, box := polygonPrism(t, uChannelBox)
		sel := opening(t, box).Or(decad.Facing(r3.NewVec(0, 1, 0))).Or(decad.Facing(r3.NewVec(1, 0, 0)))
		body, err := box.Shell(t.Context(), sel, units.Millimeters(2), decad.WithShellSense(decad.Outward))
		require.NoError(t, err)
		// The outer area: two 880 caps, the y = −2 wall 40·14 and the
		// three removed planes' strips 2·(22·14 − 200) + (40·14 − 400);
		// the cavity: the box's 2800 less its three removed faces.
		requireExactVolumeArea(t, body, 880*14-8000, 2*880+40*14+2*(22*14-200)+(40*14-400)+2800-400-200-200)
		// Two caps, the cavity's floor and ceiling, the walls y = −2 and
		// y = 0, a C-shaped face on each of x = 0 and x = 40, and the y = 20
		// plane's two strips, one per kept cap.
		require.Len(t, body.Faces(), 10)
		require.Len(t, planeFaces(t, body, r3.NewVec(0, 1, 0), 20), 2)
		requireSoundAndMeshed(t, doc, body, 880*14-8000)
	})
}

// TestShellSideOpeningRefusals covers Table SO's gates on the U-channel. Each
// refusal leaves the receiver live and the document unchanged.
func TestShellSideOpeningRefusals(t *testing.T) {
	t.Parallel()
	minusX := r3.NewVec(-1, 0, 0)
	caps := decad.NormalTo(r3.NewVec(0, 0, 1))
	for _, tc := range []struct {
		name string
		sel  func(t *testing.T, box *decad.Body) *decad.FaceQuery
		t    float64
		want error
		text string
	}{
		{"two kept caps eat the sweep (SO3)", func(t *testing.T, box *decad.Body) *decad.FaceQuery {
			return sideFaceAt(t, box, minusX, 0)
		}, 10, decad.ErrDegenerate, "SO3"},
		{"the far wall's offset is consumed (S11a)", func(t *testing.T, box *decad.Body) *decad.FaceQuery {
			return sideFaceAt(t, box, minusX, 0).Or(caps)
		}, 10, decad.ErrUnsupported, "drop"},
		{"two runs (SO6)", func(*testing.T, *decad.Body) *decad.FaceQuery {
			return decad.Faces(decad.NormalTo(r3.NewVec(1, 0, 0)))
		}, 2, decad.ErrUnsupported, "one connected run"},
		{"every side face (SO6)", func(*testing.T, *decad.Body) *decad.FaceQuery {
			return decad.Faces(decad.NormalTo(r3.NewVec(1, 0, 0))).Or(decad.NormalTo(r3.NewVec(0, 1, 0)))
		}, 2, decad.ErrUnsupported, "not all of them"},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc, box := polygonPrism(t, uChannelBox)
			before := snapshotDocument(t, doc)
			body, err := box.Shell(t.Context(), tc.sel(t, box), units.Millimeters(tc.t))
			require.Nil(t, body)
			require.ErrorIs(t, err, tc.want)
			require.ErrorContains(t, err, tc.text)
			require.Equal(t, before.bodies, doc.Bodies())
			_, err = box.Duplicate(t.Context())
			require.NoError(t, err, "the receiver stays live")
		})
	}

	t.Run("an oblique walk is staged (SO5)", func(t *testing.T) {
		t.Parallel()
		doc, tri := polygonPrism(t, [][2]float64{{0, 0}, {12, 0}, {0, 9}})
		before := snapshotDocument(t, doc)
		_, err := tri.Shell(t.Context(), sideFaceAt(t, tri, r3.NewVec(0, -1, 0), 0), units.Millimeters(1.5))
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.ErrorContains(t, err, "SO5")
		require.Equal(t, before.bodies, doc.Bodies())
	})
}

// lPrism is §9's L prism: (0,0) (30,0) (30,10) (10,10) (10,30) (0,30), 10
// tall.
var lPrism = [][2]float64{{0, 0}, {30, 0}, {30, 10}, {10, 10}, {10, 30}, {0, 30}}

// TestShellSideOpeningLPrism is §9's L prism opened on one leg: its x = 10
// face (10,10)→(10,30) removed, inward, t = 2, both caps kept. The reflex end
// at (10,10) cuts at (10,8) and the convex end at (10,30) at (10,28), so the
// cavity C = (10,28) (2,28) (2,2) (28,2) (28,8) (10,8) has area 316 and the
// body 5000 − 316·6 = 3104. The x = 10 plane holds two faces: a U facing +x
// and the rim (10,8)→(10,10) over the cavity's height facing −x. Shown to
// fail with sideOpeningRecord's Engine.Event marks deleted: the cavity's walk
// along x = 10 then held no vertex at the reflex end (10,10), its edges did
// not pair with the rim's and the U's, and the build refused (SO5).
func TestShellSideOpeningLPrism(t *testing.T) {
	t.Parallel()
	plusX := r3.NewVec(1, 0, 0)
	minusX := r3.NewVec(-1, 0, 0)
	shellL := func(t *testing.T) (*decad.Document, *decad.Body) {
		t.Helper()
		doc, l := polygonPrism(t, lPrism)
		body, err := l.Shell(t.Context(), sideFaceAt(t, l, plusX, 10), units.Millimeters(2))
		require.NoError(t, err)
		return doc, body
	}
	t.Run("record", func(t *testing.T) {
		t.Parallel()
		doc, body := shellL(t)
		requireExactVolumeArea(t, body, 3104, 2092+1148)
		require.Len(t, body.Faces(), 16)
		require.Len(t, body.Shells(), 1)
		u := planeFaces(t, body, plusX, 10)
		require.Len(t, u, 1, "the U facing +x")
		require.Len(t, u[0].Loops(), 1)
		rim := planeFaces(t, body, minusX, -10)
		require.Len(t, rim, 1, "the reflex rim facing −x")
		lo, hi := loopBox(rim[0].Loops()[0])
		require.Equal(t, r3.NewVec(10, 8, 2), lo)
		require.Equal(t, r3.NewVec(10, 10, 8), hi)
		requireBrepRoles(t, body)
		requireSoundAndMeshed(t, doc, body, 3104)
	})
	t.Run("route E chamfers an outer edge", func(t *testing.T) {
		t.Parallel()
		_, body := shellL(t)
		sel := decad.Edges(decad.ParallelTo(r3.NewVec(0, 0, 1)), decad.EndpointAt(r3.Vec{})).Exactly(1)
		chamfered, err := body.Chamfer(t.Context(), sel, units.Millimeters(1))
		require.NoError(t, err)
		vol, err := chamfered.Volume()
		require.NoError(t, err)
		require.Equal(t, decad.Exact, vol.Exactness)
		require.Equal(t, 3104.0-5, volumeMM(t, vol))
	})
	t.Run("a second shell reads as no prism (SB10)", func(t *testing.T) {
		t.Parallel()
		_, body := shellL(t)
		_, err := body.Shell(t.Context(), decad.Faces(decad.Facing(r3.NewVec(0, 0, 1))), units.Millimeters(1))
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.ErrorContains(t, err, "SB10")
	})
}
