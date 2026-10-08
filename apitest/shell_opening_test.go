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

// requireBrepRoles asserts a BO2 result: every face a brep role (face(k) for
// a planar face, wall(k) for a swept one), and no cap role minted.
func requireBrepRoles(t *testing.T, b *decad.Body) {
	t.Helper()
	for _, f := range b.Faces() {
		require.Regexp(t, `^(face|wall)\(\d+\)$`, f.Origins()[0].Role)
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

	t.Run("a circular walk is staged (SO5)", func(t *testing.T) {
		t.Parallel()
		s, p := semicircleSketch(t)
		doc := decad.New()
		half, err := doc.Extrude(s, p, decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
		require.NoError(t, err)
		before := snapshotDocument(t, doc)
		_, err = half.Shell(t.Context(), sideFaceAt(t, half, r3.NewVec(0, -1, 0), 0), units.Millimeters(1))
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.ErrorContains(t, err, "circular walk")
		require.Equal(t, before.bodies, doc.Bodies())
	})
}

// TestShellSideOpeningObliqueRefusals covers Table SO's gates where a removed
// face is oblique. Each refusal leaves the receiver live and the document
// unchanged.
func TestShellSideOpeningObliqueRefusals(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name string
		pts  [][2]float64
		sel  *decad.FaceQuery
		t    float64
		text string
	}{
		// The removed face (20,0)→(21,1) is √2 long and meets the kept y = 0
		// wall at 135°, so its cut lies t/sin 45° = 1.5√2 along it, past its
		// far end.
		{"a forward cut past the removed face's far end (SO2)", [][2]float64{{0, 0}, {20, 0}, {21, 1}, {0, 10}},
			decad.Faces(decad.Facing(r3.NewVec(1, -1, 0))), 1.5, "SO2"},
		// The hypotenuse recorded as two collinear segments through (6, 4.5)
		// is one walk whose pieces the record cannot state alike in the cap
		// and wall slabs.
		{"an oblique removed face of two segments (SO5)", [][2]float64{{0, 0}, {12, 0}, {6, 4.5}, {0, 9}},
			decad.Faces(decad.Facing(r3.NewVec(3, 4, 0))), 3, "2 collinear segments"},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc, body := polygonPrism(t, tc.pts)
			before := snapshotDocument(t, doc)
			got, err := body.Shell(t.Context(), tc.sel, units.Millimeters(tc.t))
			require.Nil(t, got)
			require.ErrorIs(t, err, decad.ErrUnsupported)
			require.ErrorContains(t, err, tc.text)
			require.Equal(t, before.bodies, doc.Bodies())
			_, err = body.Duplicate(t.Context())
			require.NoError(t, err, "the receiver stays live")
		})
	}
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

// requireVolumeAreaNear asserts the body's volume and area each lie within
// their own published bound of the closed form, widened by a few ulps of it
// for the closed form's own float evaluation.
func requireVolumeAreaNear(t *testing.T, b *decad.Body, volume, area float64) {
	t.Helper()
	requireVolumeNear(t, b, volume)
	a, err := b.Area()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(a.Value.Base()-area), a.Bound.Base()+1e-12*area,
		`area %v must lie within its bound %v of the closed form %v`, a.Value.Base(), a.Bound.Base(), area)
}

// requireOneHole asserts the plane facing n at offset holds one face with
// one hole, the hole's box within slack of lo to hi.
func requireOneHole(t *testing.T, b *decad.Body, n r3.Vec, offset float64, lo, hi r3.Vec, slack float64) {
	t.Helper()
	faces := planeFaces(t, b, n, offset)
	require.Len(t, faces, 1)
	loops := faces[0].Loops()
	require.Len(t, loops, 2, "an outer loop and the opening")
	require.True(t, loops[0].IsOuter())
	require.False(t, loops[1].IsOuter())
	gotLo, gotHi := loopBox(loops[1])
	require.InDelta(t, 0, gotLo.Sub(lo).Len(), slack)
	require.InDelta(t, 0, gotHi.Sub(hi).Len(), slack)
}

// facesOnPlane counts the body's planar faces lying on the plane through p
// with unit normal ±n, by the sign of their outward normal along n.
func facesOnPlane(t *testing.T, b *decad.Body, n, p r3.Vec) (int, int) {
	t.Helper()
	along, against := 0, 0
	for _, f := range b.Faces() {
		plane, ok := f.Surface().(decad.Plane)
		if !ok {
			continue
		}
		fn := plane.Frame.N()
		if fn.Cross(n).Len() > 1e-12 || math.Abs(plane.Frame.Origin().Sub(p).Dot(n)) > 1e-12 {
			continue
		}
		if fn.Dot(n) > 0 {
			along++
		} else {
			against++
		}
	}
	return along, against
}

// obliqueTriangle is §9's right triangle with legs 12 and 9.
var obliqueTriangle = [][2]float64{{0, 0}, {12, 0}, {0, 9}}

// TestShellSideOpeningOblique is docs/shell-opening-design.md §9's oblique
// fixtures, each 10 tall with both caps kept, inward unless named, every
// oblique cut a float solve whose displacement the faces carry:
//
//   - acute: the triangle without its y = 0 leg at t = 1.5 cuts at (9.5, 0)
//     against the hypotenuse and (1.5, 0) at the right angle; C = (1.5,0)
//     (9.5,0) (1.5,6), area 24, volume 540 − 24·7 = 372;
//   - obtuse: the trapezoid (0,0) (14,0) (11,4) (3,4) without its y = 4 side
//     at t = 1 cuts at (9.75, 4) and (4.25, 4); C has area 23.25, volume
//     440 − 23.25·8 = 254;
//   - the triangle without its hypotenuse at t = 3 cuts at (8, 3) and
//     (3, 6.75); C has area 9.375, volume 540 − 9.375·4 = 502.5, and the
//     hypotenuse plane holds four swept faces;
//   - a slanted reflex end: (0,0) (30,0) (30,10) (14,10) (10,30) (0,30)
//     without its oblique face (14,10)→(10,30) at t = 2 cuts backward at
//     (14.4, 8) and forward at (10.4, 28); C has area 364, volume
//     5400 − 364·6 = 3216.
//
// Outward, the hypotenuse kept alone cuts backward along both legs, and the
// slanted reflex section's y = 10 wall kept alone cuts forward along the
// removed oblique face, which splits the cavity region P there.
//
// Shown to fail: with P's split at a forward cut deleted, the hypotenuse,
// slanted reflex and outward forward-cut fixtures failed the area identity
// (SO5); with the oblique ends' vertex marks deleted the hypotenuse and
// slanted reflex meshes did not close; with solveOpeningCut's axis hold
// deleted the obtuse fixture failed the area identity (SO5); and with R' no
// longer split at a backward cut's corner the slanted reflex fixture failed
// the area identity (SO5).
func TestShellSideOpeningOblique(t *testing.T) {
	t.Parallel()
	shell := func(t *testing.T, pts [][2]float64, sel *decad.FaceQuery, thickness float64, opts ...decad.ShellOption) (*decad.Document, *decad.Body) {
		t.Helper()
		doc, receiver := polygonPrism(t, pts)
		body, err := receiver.Shell(t.Context(), sel, units.Millimeters(thickness), opts...)
		require.NoError(t, err)
		require.Len(t, body.Lumps(), 1)
		require.Len(t, body.Shells(), 1)
		require.False(t, body.Shells()[0].IsVoid(), "the cavity reaches the outside through the opening")
		return doc, body
	}
	minusY := r3.NewVec(0, -1, 0)

	t.Run("acute", func(t *testing.T) {
		t.Parallel()
		doc, body := shell(t, obliqueTriangle, decad.Faces(decad.Facing(minusY)), 1.5)
		// The outer 2·54 + 10·(15 + 9) + (120 − 8·7), the cavity 2·24 + 7·(10 + 6).
		requireVolumeAreaNear(t, body, 372, 108+240+64+48+112)
		require.Len(t, body.Faces(), 9)
		requireOneHole(t, body, minusY, 0, r3.NewVec(1.5, 0, 1.5), r3.NewVec(9.5, 0, 8.5), 1e-12)
		requireBrepRoles(t, body)
		requireSoundAndMeshed(t, doc, body, 372)
	})
	t.Run("obtuse", func(t *testing.T) {
		t.Parallel()
		plusY := r3.NewVec(0, 1, 0)
		doc, body := shell(t, [][2]float64{{0, 0}, {14, 0}, {11, 4}, {3, 4}}, decad.Faces(decad.Facing(plusY)), 1)
		// The outer 2·44 + 10·(14 + 5 + 5) + (80 − 5.5·8), the cavity
		// 2·23.25 + 8·(10 + 3.75 + 3.75).
		requireVolumeAreaNear(t, body, 254, 88+240+36+46.5+140)
		require.Len(t, body.Faces(), 11)
		requireOneHole(t, body, plusY, 4, r3.NewVec(4.25, 4, 1), r3.NewVec(9.75, 4, 9), 1e-12)
		requireSoundAndMeshed(t, doc, body, 254)
	})
	t.Run("oblique removed face", func(t *testing.T) {
		t.Parallel()
		n := r3.NewVec(0.6, 0.8, 0)
		doc, body := shell(t, obliqueTriangle, decad.Faces(decad.Facing(n)), 3)
		// The outer 2·54 + 10·(12 + 9) + (150 − 6.25·4), the cavity
		// 2·9.375 + 4·(5 + 3.75).
		requireVolumeAreaNear(t, body, 502.5, 108+210+125+18.75+35)
		require.Len(t, body.Faces(), 12)
		along, against := facesOnPlane(t, body, n, r3.NewVec(12, 0, 0))
		require.Equal(t, 4, along, "two columns and two strips, coplanar")
		require.Zero(t, against)
		requireBrepRoles(t, body)
		requireSoundAndMeshed(t, doc, body, 502.5)
	})
	t.Run("slanted reflex end", func(t *testing.T) {
		t.Parallel()
		n, ok := r3.NewVec(5, 1, 0).Normalize()
		require.True(t, ok)
		pts := [][2]float64{{0, 0}, {30, 0}, {30, 10}, {14, 10}, {10, 30}, {0, 30}}
		doc, body := shell(t, pts, decad.Faces(decad.Facing(n)), 2)
		// The outer 2·540 + 10·(30 + 10 + 16 + 10 + 30), the cavity
		// 2·364 + 6·(8.4 + 26 + 26 + 6 + 13.6); on the removed face's carrier
		// the two strips 2·2·4√26, the forward column 6·0.4√26 and the
		// reflex rim 6·0.4√26 facing the other way.
		requireVolumeAreaNear(t, body, 3216, 1080+960+728+480+20.8*math.Sqrt(26))
		require.Len(t, body.Faces(), 18)
		along, against := facesOnPlane(t, body, n, r3.NewVec(14, 10, 0))
		require.Equal(t, 3, along, "two strips and the forward column")
		require.Equal(t, 1, against, "the reflex rim faces back into the cavity")
		requireSoundAndMeshed(t, doc, body, 3216)
	})
	t.Run("outward, one kept oblique wall", func(t *testing.T) {
		t.Parallel()
		// The hypotenuse alone kept: its offset 3x + 4y = 41 cuts backward
		// along y = 0 at (41/3, 0) and along x = 0 at (0, 41/4), so O is the
		// triangle of area 1681/24 over [−1, 11] less the receiver.
		sel := decad.Faces(decad.Facing(minusY)).Or(decad.Facing(r3.NewVec(-1, 0, 0)))
		doc, body := shell(t, obliqueTriangle, sel, 1, decad.WithShellSense(decad.Outward))
		// Two O caps, the offset wall 205/12·12, the y = 0 and x = 0 planes'
		// C-shaped faces 44 and 33, the cavity's floor, ceiling and wall.
		requireVolumeAreaNear(t, body, 2*1681.0/24+10*(1681.0/24-54), 2*1681.0/24+205+44+33+108+150)
		require.Len(t, body.Faces(), 8)
		requireSoundAndMeshed(t, doc, body, 300.5)
	})
	t.Run("outward, a forward cut along an oblique removed face", func(t *testing.T) {
		t.Parallel()
		// The slanted reflex section with only its y = 10 wall (30,10)→(14,10)
		// kept: the offset y = 12 cuts forward along the removed oblique face
		// at (13.6, 12) and backward along x = 30 at (30, 12), so O is P with
		// the strip (14,10) (30,10) (30,12) (13.6,12) of area 32.4 added.
		pts := [][2]float64{{0, 0}, {30, 0}, {30, 10}, {14, 10}, {10, 30}, {0, 30}}
		n, ok := r3.NewVec(5, 1, 0).Normalize()
		require.True(t, ok)
		doc, receiver := polygonPrism(t, pts)
		sel := sideFaceAt(t, receiver, r3.NewVec(0, 1, 0), 30).Or(decad.Facing(minusY)).
			Or(decad.Facing(r3.NewVec(1, 0, 0))).Or(decad.Facing(r3.NewVec(-1, 0, 0))).Or(decad.Facing(n))
		body, err := receiver.Shell(t.Context(), sel, units.Millimeters(2), decad.WithShellSense(decad.Outward))
		require.NoError(t, err)
		// Two O caps, the receiver's section at both interfaces, the kept wall
		// 16·10 and its offset 16.4·14, the removed carriers' strips
		// 4·(82 + 3.6√26) and the two rims 10·0.4√26 and 10·2.
		requireVolumeAreaNear(t, body, 572.4*14-5400, 1144.8+1080+160+229.6+328+20+18.4*math.Sqrt(26))
		require.Len(t, body.Faces(), 16)
		along, against := facesOnPlane(t, body, n, r3.NewVec(14, 10, 0))
		require.Equal(t, 2, along, "the two strips beyond the cut")
		require.Equal(t, 1, against, "the rim ends the wall facing the opening")
		requireSoundAndMeshed(t, doc, body, 572.4*14-5400)
	})
	t.Run("both caps removed is a prism over the wall section", func(t *testing.T) {
		t.Parallel()
		sel := decad.Faces(decad.Facing(minusY)).Or(decad.NormalTo(r3.NewVec(0, 0, 1)))
		doc, body := shell(t, obliqueTriangle, sel, 1.5)
		// W: the triangle's 54 less C's 24, its six walls 15 + 9 + 1.5 + 6 +
		// 10 + 2.5 long.
		requireVolumeAreaNear(t, body, 300, 60+440)
		require.Len(t, body.Faces(), 8)
		for _, cap := range []decad.FeatureRef{decad.CapStart(body), decad.CapEnd(body)} {
			faces, err := decad.Faces(decad.FaceCreatedBy(cap)).SelectFaces(body)
			require.NoError(t, err)
			require.Len(t, faces, 1)
		}
		requireSoundAndMeshed(t, doc, body, 300)
	})
}
