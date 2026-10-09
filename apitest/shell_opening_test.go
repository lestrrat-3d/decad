package apitest_test

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
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

// requireHole asserts the x = 0 plane, facing −x, is one face with one hole
// spanning lo to hi.
func requireHole(t *testing.T, b *decad.Body, lo, hi r3.Vec) {
	t.Helper()
	opened := planeFaces(t, b, r3.NewVec(-1, 0, 0), 0)
	require.Len(t, opened, 1, "the x = 0 plane is one face")
	loops := opened[0].Loops()
	require.Len(t, loops, 2, "an outer loop and the opening")
	require.True(t, loops[0].IsOuter())
	require.False(t, loops[1].IsOuter())
	gotLo, gotHi := loopBox(loops[1])
	require.Equal(t, lo, gotLo)
	require.Equal(t, hi, gotHi)
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

	t.Run("outward, both caps kept, the arc joins meet their lines tangentially", func(t *testing.T) {
		t.Parallel()
		// The two convex corners of K offset outward to arcs of radius 2
		// tangent to their neighbours (§4.3's tangent line–circle junction).
		// The wall slab holds W, area 208 − 2(4 − π), over [0, 10]; each cap
		// slab holds O = [0,42]×[−2,22] with the two corners rounded, area
		// 1008 − 2(4 − π), over 2: 6000 + 28π in all. Shown to fail with
		// stackedbrep's tangentFoot deleted (the junction missed, SO5).
		doc, box := polygonPrism(t, uChannelBox)
		body, err := box.Shell(t.Context(), opening(t, box), units.Millimeters(2), decad.WithShellSense(decad.Outward))
		require.NoError(t, err)
		vol, err := body.Volume()
		require.NoError(t, err)
		requirePiLinearEnclosed(t, vol, 6000, 28)
		// Two caps, the cavity's floor and ceiling, the outer walls y = −2,
		// x = 42 and y = 22 with the two corner cylinders, the box's three
		// kept walls, and the x = 0 plane's one face, whose hole is the
		// removed face itself.
		require.Len(t, body.Faces(), 13)
		requireHole(t, body, r3.NewVec(0, 0, 0), r3.NewVec(0, 20, 10))
		cylinders := 0
		for _, f := range body.Faces() {
			if c, ok := f.Surface().(decad.Cylinder); ok {
				cylinders++
				require.Equal(t, 2.0, c.Radius.Base())
			}
		}
		require.Equal(t, 2, cylinders)
		requireBrepRoles(t, body)
		report, err := doc.Verify(t.Context())
		require.NoError(t, err)
		require.Equal(t, decad.Sound, report.Status)
		mesh, err := body.Tessellate(t.Context(), units.Millimeters(0.05), decad.WithVerification(decad.VerifyAll))
		require.NoError(t, err)
		requireWatertight(t, mesh)
		require.True(t, mesh.VolumeVerified())
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
// fail with the stack engine's Event marks deleted: the cavity's walk
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

// arcPrism extrudes a section of lines and arcs on XY by 10 along +z: build
// draws its entities on s, and the section is the one profile they bound.
func arcPrism(t *testing.T, build func(s *sketch.Sketch)) (*decad.Document, *decad.Body) {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	build(s)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Profiles(), 1)
	doc := decad.New()
	body, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	require.NoError(t, err)
	return doc, body
}

// dSection is §9's D section: the semicircle of radius 5 about the origin
// from (0,−5) through (5,0) to (0,5), closed by the chord x = 0.
func dSection(s *sketch.Sketch) {
	o := s.CreatePoint(0, 0)
	s.Fix(o)
	a := s.CreatePoint(0, -5)
	b := s.CreatePoint(0, 5)
	s.CreateArc(o, a, b)
	s.CreateLine(b, a)
}

// cylinderFaces lists a body's cylindrical faces of radius r whose axis
// passes through (u, v) on XY.
func cylinderFaces(b *decad.Body, u, v, r float64) []*decad.Face {
	var out []*decad.Face
	for _, f := range b.Faces() {
		c, ok := f.Surface().(decad.Cylinder)
		if ok && c.Radius.Base() == r && c.Origin.X == u && c.Origin.Y == v {
			out = append(out, f)
		}
	}
	return out
}

// requireRatEnclosed asserts the measurement's interval value ± bound holds
// the whole closed-form bracket [lo, hi], compared over big.Rat.
func requireRatEnclosed(t *testing.T, got decad.Measurement, lo, hi *big.Rat) {
	t.Helper()
	value := new(big.Rat).SetFloat64(got.Value.Base())
	bound := new(big.Rat).SetFloat64(got.Bound.Base())
	require.LessOrEqual(t, new(big.Rat).Sub(value, bound).Cmp(lo), 0, "the interval reaches down to the closed form")
	require.GreaterOrEqual(t, new(big.Rat).Add(value, bound).Cmp(hi), 0, "the interval reaches up to the closed form")
}

// dClosedForm brackets the D section fixtures' closed forms base + p·π +
// a·A + r·√21, where A is acos(3/5) when r is zero and acos(2/5) otherwise,
// each constant bracketed to 50 digits: the low end when high is false and
// the high end when it is true.
func dClosedForm(base, p, a, r int64, high bool) *big.Rat {
	bracket := func(lo, hi string, coeff int64) *big.Rat {
		pick := lo
		if (coeff > 0) == high {
			pick = hi
		}
		v, _ := new(big.Rat).SetString(pick)
		return v.Mul(v, big.NewRat(coeff, 1))
	}
	acosLo, acosHi := "0.92729521800161223242851246292242880405707410857224", "0.92729521800161223242851246292242880405707410857225"
	if r != 0 {
		acosLo, acosHi = "1.15927948072740859984658379402241583724288356456052", "1.15927948072740859984658379402241583724288356456053"
	}
	v := big.NewRat(base, 1)
	v.Add(v, bracket("3.14159265358979323846264338327950288419716939937510", "3.14159265358979323846264338327950288419716939937511", p))
	v.Add(v, bracket(acosLo, acosHi, a))
	return v.Add(v, bracket("4.58257569495584000658804719372800848898445657676797", "4.58257569495584000658804719372800848898445657676798", r))
}

// TestShellSideOpeningArcs is docs/shell-opening-design.md §9's D section, 10
// tall, both caps kept, inward. Removing the chord at t = 1 keeps the arc: its
// offset is the radius-4 arc, the cuts the exact feet (0, ±4), and the cavity
// the half-disc of radius 4 over [1, 9]: 125π − 8π·8 = 61π. Removing the arc
// at t = 3 keeps the chord: its offset x = 3 meets the arc's own circle at
// (3, ±4), the rims are arcs about the origin, and the cavity is the circular
// segment beyond x = 3, area 25·acos(3/5) − 12, over [3, 7]. Shown to fail:
// the arc removed inward and outward with the removed arc stated whole in P
// and R' (splitAtCuts false for an arc: the area identity failed, SO5); the
// arc removed outward's tessellation with every wall column holding both
// side lines' splits (the mesh did not close); the
// float cut under a kept cap with requireRemovedArcKey deleted (the engine
// missed instead, without naming the cut).
func TestShellSideOpeningArcs(t *testing.T) {
	t.Parallel()
	t.Run("chord removed", func(t *testing.T) {
		t.Parallel()
		doc, d := arcPrism(t, dSection)
		body, err := d.Shell(t.Context(), sideFaceAt(t, d, r3.NewVec(-1, 0, 0), 0), units.Millimeters(1))
		require.NoError(t, err)
		vol, err := body.Volume()
		require.NoError(t, err)
		requirePiLinearEnclosed(t, vol, 0, 61)
		requireHole(t, body, r3.NewVec(0, -4, 1), r3.NewVec(0, 4, 9))
		// Two caps, the cavity's floor and ceiling, the outer cylinder over
		// [0, 10], the cavity's radius-4 cylinder and the x = 0 plane.
		require.Len(t, body.Faces(), 7)
		require.Len(t, cylinderFaces(body, 0, 0, 5), 1)
		require.Len(t, cylinderFaces(body, 0, 0, 4), 1)
		requireBrepRoles(t, body)
		report, err := doc.Verify(t.Context())
		require.NoError(t, err)
		require.Equal(t, decad.Sound, report.Status)
		mesh, err := body.Tessellate(t.Context(), units.Millimeters(0.05), decad.WithVerification(decad.VerifyAll))
		require.NoError(t, err)
		requireWatertight(t, mesh)
		require.True(t, mesh.VolumeVerified())
	})
	t.Run("arc removed", func(t *testing.T) {
		t.Parallel()
		doc, d := arcPrism(t, dSection)
		removed := cylinderFaces(d, 0, 0, 5)
		require.Len(t, removed, 1)
		sel := decad.Faces(decad.FaceCreatedBy(removed[0].Origins()[0]))
		body, err := d.Shell(t.Context(), sel, units.Millimeters(3))
		require.NoError(t, err)
		// 125π − 4·(25·acos(3/5) − 12) = 48 + 125π − 100·acos(3/5).
		vol, err := body.Volume()
		require.NoError(t, err)
		requireRatEnclosed(t, vol, dClosedForm(48, 125, -100, 0, false), dClosedForm(48, 125, -100, 0, true))
		// The arc's circle holds the two rim columns, each one face over
		// [0, 10] with its side line split at the cavity's levels z = 3 and
		// z = 7, and the floor and ceiling strips between (3, −4) and (3, 4).
		onArc := cylinderFaces(body, 0, 0, 5)
		require.Len(t, onArc, 4)
		columns := 0
		for _, f := range onArc {
			lo, hi := loopBox(f.Loops()[0])
			if lo.Z == 0 && hi.Z == 10 {
				columns++
				// Eight corners: the column's four, and both side lines
				// split at z = 3 and z = 7 — the cut's where the cavity
				// begins, the corner's where it is marked (§4.3).
				require.Len(t, f.Loops()[0].CoEdges(), 8)
				continue
			}
			require.Equal(t, 3.0, hi.Z-lo.Z, "a strip spans one cap slab")
			require.Equal(t, r3.NewVec(3, -4, lo.Z), r3.NewVec(lo.X, lo.Y, lo.Z))
			require.Equal(t, 4.0, hi.Y)
		}
		require.Equal(t, 2, columns)
		// Two caps, the cavity's floor and ceiling, the chord plane x = 0, its
		// offset x = 3 and the four pieces on the arc's circle.
		require.Len(t, body.Faces(), 10)
		require.Len(t, planeFaces(t, body, r3.NewVec(1, 0, 0), 3), 1)
		requireBrepRoles(t, body)
		report, err := doc.Verify(t.Context())
		require.NoError(t, err)
		require.Equal(t, decad.Sound, report.Status)
		mesh, err := body.Tessellate(t.Context(), units.Millimeters(0.05), decad.WithVerification(decad.VerifyAll))
		require.NoError(t, err)
		requireWatertight(t, mesh)
		require.True(t, mesh.VolumeVerified())

		// With both caps removed W is BO1's prism, 10·(25π/2 − (25·acos(3/5)
		// − 12)) = 120 + 125π − 250·acos(3/5). Its rims end at the cut's
		// parameter on the arc's own record, a float whose point is not
		// (3, ±4) itself, and the prism charges that gap
		// (TestSideOpeningRegionsDSection pins the gap's enclosure).
		_, d = arcPrism(t, dSection)
		removed = cylinderFaces(d, 0, 0, 5)
		require.Len(t, removed, 1)
		sel = decad.Faces(decad.FaceCreatedBy(removed[0].Origins()[0])).Or(decad.NormalTo(r3.NewVec(0, 0, 1)))
		prism, err := d.Shell(t.Context(), sel, units.Millimeters(3))
		require.NoError(t, err)
		vol, err = prism.Volume()
		require.NoError(t, err)
		requireRatEnclosed(t, vol, dClosedForm(120, 125, -250, 0, false), dClosedForm(120, 125, -250, 0, true))
	})
	t.Run("arc removed, outward", func(t *testing.T) {
		t.Parallel()
		// The chord's offset x = −3 meets the arc's circle at (−3, ±4),
		// walking it backward from each corner, so each rim runs along the
		// circle beyond the corner and O is the disc less the segment left of
		// x = −3: 25π − (25·acos(3/5) − 12). The wall W = O − D over [0, 10]
		// and O over the two cap slabs of 3: 192 + 275π − 400·acos(3/5).
		doc, d := arcPrism(t, dSection)
		removed := cylinderFaces(d, 0, 0, 5)
		require.Len(t, removed, 1)
		sel := decad.Faces(decad.FaceCreatedBy(removed[0].Origins()[0]))
		body, err := d.Shell(t.Context(), sel, units.Millimeters(3), decad.WithShellSense(decad.Outward))
		require.NoError(t, err)
		vol, err := body.Volume()
		require.NoError(t, err)
		requireRatEnclosed(t, vol, dClosedForm(192, 275, -400, 0, false), dClosedForm(192, 275, -400, 0, true))
		// Two caps, the cavity's floor and ceiling, the chord plane x = 0 and
		// its offset x = −3, the two rim columns over [−3, 13] and the cap
		// slabs' two strips between the corners (0, ±5).
		require.Len(t, body.Faces(), 10)
		require.Len(t, cylinderFaces(body, 0, 0, 5), 4)
		requireBrepRoles(t, body)
		report, err := doc.Verify(t.Context())
		require.NoError(t, err)
		require.Equal(t, decad.Sound, report.Status)
		mesh, err := body.Tessellate(t.Context(), units.Millimeters(0.05), decad.WithVerification(decad.VerifyAll))
		require.NoError(t, err)
		requireWatertight(t, mesh)
		require.True(t, mesh.VolumeVerified())
	})
	t.Run("arc removed at a float cut", func(t *testing.T) {
		t.Parallel()
		// At t = 2 the offset chord x = 2 meets the arc's circle at
		// (2, ±√21), a float solve. With both caps removed the wall is a
		// prism over 125π/10 − (25·acos(2/5) − 2√21), volume 125π −
		// 250·acos(2/5) + 20√21 within its bound (the cut's own charge is
		// TestSideOpeningRegionsDSection's). Each kept cap adds a slab of the
		// cavity C = 25·acos(2/5) − 2√21, two units tall: one cap kept,
		// 125π − 200·acos(2/5) + 16√21; both, 125π − 150·acos(2/5) + 12√21.
		// The rims and the pieces of R and R' are parameter ranges of the
		// arc's own record, so they key its circle although the cut is a
		// float. Outward the offset x = −2 meets the circle at (−2, ±√21)
		// behind each corner, so the rims and R' run on the arc's complement:
		// O = 25π − (25·acos(2/5) − 2√21) over 14, less the D's 125π. Shown
		// to fail: every kept-cap body here refused SO5 before the pieces
		// were stated as ranges (the rims read their radius from the cut).
		arcFace := func(t *testing.T, d *decad.Body) *decad.FaceQuery {
			t.Helper()
			removed := cylinderFaces(d, 0, 0, 5)
			require.Len(t, removed, 1)
			return decad.Faces(decad.FaceCreatedBy(removed[0].Origins()[0]))
		}
		_, d := arcPrism(t, dSection)
		body, err := d.Shell(t.Context(), arcFace(t, d).Or(decad.NormalTo(r3.NewVec(0, 0, 1))), units.Millimeters(2))
		require.NoError(t, err)
		vol, err := body.Volume()
		require.NoError(t, err)
		requireRatEnclosed(t, vol, dClosedForm(0, 125, -250, 20, false), dClosedForm(0, 125, -250, 20, true))

		for _, tc := range []struct {
			name         string
			sel          func(*decad.FaceQuery) *decad.FaceQuery
			pi, acos, rt int64
			faces        int
			outward      bool
		}{
			// Two caps, the cavity's floor and ceiling, the chord plane, its
			// offset x = 2 and the four pieces on the arc's circle.
			{"both caps kept", func(q *decad.FaceQuery) *decad.FaceQuery { return q }, 125, -150, 12, 10, false},
			// The end cap removed: the kept cap, W's top at z = 10, the
			// floor, the chord plane, its offset and three pieces on the
			// circle — two rim columns and the floor strip.
			{"end cap removed", func(q *decad.FaceQuery) *decad.FaceQuery {
				return q.Or(decad.Facing(r3.NewVec(0, 0, 1)))
			}, 125, -200, 16, 8, false},
			// Outward, both caps kept: the faces of the t = 3 outward body.
			{"outward, both caps kept", func(q *decad.FaceQuery) *decad.FaceQuery { return q }, 225, -350, 28, 10, true},
		} {
			t.Run(tc.name, func(t *testing.T) {
				doc, d := arcPrism(t, dSection)
				var opts []decad.ShellOption
				if tc.outward {
					opts = append(opts, decad.WithShellSense(decad.Outward))
				}
				body, err := d.Shell(t.Context(), tc.sel(arcFace(t, d)), units.Millimeters(2), opts...)
				require.NoError(t, err)
				vol, err := body.Volume()
				require.NoError(t, err)
				require.Positive(t, vol.Bound.Base(), "the float cut is charged")
				requireRatEnclosed(t, vol, dClosedForm(0, tc.pi, tc.acos, tc.rt, false), dClosedForm(0, tc.pi, tc.acos, tc.rt, true))
				require.Len(t, body.Faces(), tc.faces)
				requireBrepRoles(t, body)
				report, err := doc.Verify(t.Context())
				require.NoError(t, err)
				require.Equal(t, decad.Sound, report.Status)
				mesh, err := body.Tessellate(t.Context(), units.Millimeters(0.05), decad.WithVerification(decad.VerifyAll))
				require.NoError(t, err)
				requireWatertight(t, mesh)
				require.True(t, mesh.VolumeVerified())
			})
		}
	})
	t.Run("a removed arc tangent to the kept chord (SO1)", func(t *testing.T) {
		t.Parallel()
		// The quarter arc about the origin from (0,−5) to (5,0) meets, at a
		// right corner, the concave arc about (5,5) running to (0,5); the
		// chord x = 0 closes the section. Removing the concave arc leaves
		// its end (0,5) a cusp against the kept chord, which the arc touches
		// there: no rim closes the wall, SO1.
		doc, d := arcPrism(t, func(s *sketch.Sketch) {
			o := s.CreatePoint(0, 0)
			s.Fix(o)
			c := s.CreatePoint(5, 5)
			s.Fix(c)
			a := s.CreatePoint(0, -5)
			b := s.CreatePoint(5, 0)
			e := s.CreatePoint(0, 5)
			s.CreateArc(o, a, b)
			s.CreateArc(c, e, b)
			s.CreateLine(e, a)
		})
		removed := cylinderFaces(d, 5, 5, 5)
		require.Len(t, removed, 1)
		before := snapshotDocument(t, doc)
		_, err := d.Shell(t.Context(), decad.Faces(decad.FaceCreatedBy(removed[0].Origins()[0])), units.Millimeters(1))
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.ErrorContains(t, err, "SO1")
		require.Equal(t, before.bodies, doc.Bodies())
	})
}

// closedTerm is one term coeff·c of a closed form whose constant c is
// bracketed to 50 digits by lo and hi.
type closedTerm struct {
	coeff  *big.Rat
	lo, hi string
}

// The constants of TestShellSideOpeningCircleJunctions' closed forms.
const (
	piLo     = "3.14159265358979323846264338327950288419716939937510"
	piHi     = "3.14159265358979323846264338327950288419716939937511"
	acos35Lo = "0.92729521800161223242851246292242880405707410857224"
	acos35Hi = "0.92729521800161223242851246292242880405707410857225"
	// atan(8/15) = 2·atan(1/4).
	atan815Lo = "0.48995732625372830834416496242255162182828819676236"
	atan815Hi = "0.48995732625372830834416496242255162182828819676237"
)

// closedFormBracket is base + Σ coeff·c over the terms, each constant taken
// at the end of its bracket that makes the sum lowest, and highest.
func closedFormBracket(t *testing.T, base *big.Rat, terms ...closedTerm) (*big.Rat, *big.Rat) {
	t.Helper()
	lo, hi := new(big.Rat).Set(base), new(big.Rat).Set(base)
	for _, term := range terms {
		cLo, ok := new(big.Rat).SetString(term.lo)
		require.True(t, ok)
		cHi, ok := new(big.Rat).SetString(term.hi)
		require.True(t, ok)
		if term.coeff.Sign() < 0 {
			cLo, cHi = cHi, cLo
		}
		lo.Add(lo, new(big.Rat).Mul(term.coeff, cLo))
		hi.Add(hi, new(big.Rat).Mul(term.coeff, cHi))
	}
	return lo, hi
}

// requireVolumeBracketed asserts the body's volume interval holds the whole
// closed-form bracket.
func requireVolumeBracketed(t *testing.T, b *decad.Body, base *big.Rat, terms ...closedTerm) {
	t.Helper()
	vol, err := b.Volume()
	require.NoError(t, err)
	lo, hi := closedFormBracket(t, base, terms...)
	requireRatEnclosed(t, vol, lo, hi)
}

// requireSoundMesh asserts the document verifies Sound and the body meshes
// watertight with its occupied-volume proof.
func requireSoundMesh(t *testing.T, doc *decad.Document, b *decad.Body) {
	t.Helper()
	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.Equal(t, decad.Sound, report.Status)
	mesh, err := b.Tessellate(t.Context(), units.Millimeters(0.05), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	requireWatertight(t, mesh)
	require.True(t, mesh.VolumeVerified())
}

// notchSection is a section whose kept arc meets a removed arc at an end
// corner: the concave arc of radius 13 about the origin from (0,13) to
// (13,0), with the material outside it; the arc of radius 17 about (−2,8)
// from (13,0) through (15,8) to (13,16); then x = 13 up to (13,20), y = 20
// back to (0,20) and x = 0 down to (0,13).
func notchSection(s *sketch.Sketch) {
	o := s.CreatePoint(0, 0)
	c := s.CreatePoint(-2, 8)
	a := s.CreatePoint(0, 13)
	b := s.CreatePoint(13, 0)
	e := s.CreatePoint(13, 16)
	f := s.CreatePoint(13, 20)
	g := s.CreatePoint(0, 20)
	for _, p := range []*sketch.Point{o, c, a, b, e, f, g} {
		s.Fix(p)
	}
	s.CreateArc(o, b, a)
	s.CreateArc(c, b, e)
	s.CreateLine(e, f)
	s.CreateLine(f, g)
	s.CreateLine(g, a)
}

// halfLensSection is the half of the lens of two radius-5 circles about
// (0,0) and (6,0) above y = 0: the arc about the origin from (5,0) to (3,4),
// the arc about (6,0) from (3,4) to (1,0), and the chord y = 0.
func halfLensSection(s *sketch.Sketch) {
	a := s.CreatePoint(0, 0)
	b := s.CreatePoint(6, 0)
	p := s.CreatePoint(5, 0)
	q := s.CreatePoint(3, 4)
	r := s.CreatePoint(1, 0)
	for _, pt := range []*sketch.Point{a, b, p, q, r} {
		s.Fix(pt)
	}
	s.CreateArc(a, p, q)
	s.CreateArc(b, q, r)
	s.CreateLine(r, p)
}

// TestShellSideOpeningCircleJunctions covers docs/shell-opening-design.md
// §4.3's junctions at one recorded point, each fixture 10 tall:
//
//   - two arcs at an end corner: notchSection without every walk but its
//     concave arc, inward at t = 4. The kept arc's offset, radius 17 about
//     the origin, meets the removed arc's circle at (15,8) and the removed
//     x = 0 at (0,17), both exact. With β = atan(8/15), P has area 140 −
//     169π/4 + 289β and C, the region beyond the offset arc, 156 − 72.25π +
//     289β, so the body is 10·P − 2·C = 1088 − 278π + 2312β; with both caps
//     removed W = P − C = 30π − 16 is a prism, 300π − 160;
//   - two arcs at an interior corner: halfLensSection without its chord at
//     t = 1.25. Both arcs offset to radius 3.75 and meet at the miter
//     (3, 2.25); the chord's cuts are (3.75, 0) and (2.25, 0). With A =
//     acos(3/5), P is 25A − 12 and C 14.0625(π/2 − A) − 6.75, so the body is
//     10·P − 7.5·C = 355.46875A − 52.734375π − 69.375, and with both caps
//     removed 10·(P − C) = 390.625A − 70.3125π − 52.5;
//   - an arc join on a slanted wall: the right triangle (0,0) (12,0) (0,9)
//     without its y = 0 leg, outward at t = 1, both caps kept. The corner
//     (0,9) rounds to an arc of radius 1 tangent to the slanted offset
//     3x + 4y = 41 at (0.6, 9.8) and to x = −1 at (−1, 9); the arc reads its
//     radius from the float foot, so neither junction is an exact tangency.
//     O is the triangle (−1,0) (41/3,0) (−1,11) less the corner's fillet,
//     242/3 − 2 + (π − A)/2, so the body is 12·O − 10·54 = 404 + 6π − 6A.
//
// Shown to fail: with the circle–circle junction missing again, both arc
// fixtures with a kept cap refused (SO5); with the arc–arc end corner refused
// again in sideOpeningRegions, both notch fixtures refused; with the
// line–circle junction at one recorded point deleted, the slanted wall
// refused (SO5).
func TestShellSideOpeningCircleJunctions(t *testing.T) {
	t.Parallel()
	notchRemoved := func(t *testing.T, d *decad.Body) *decad.FaceQuery {
		t.Helper()
		onArc := cylinderFaces(d, -2, 8, 17)
		require.Len(t, onArc, 1)
		return decad.Faces(decad.FaceCreatedBy(onArc[0].Origins()[0])).
			Or(decad.Facing(r3.NewVec(1, 0, 0))).
			Or(decad.Facing(r3.NewVec(0, 1, 0))).
			Or(decad.Facing(r3.NewVec(-1, 0, 0)))
	}
	r := func(num, den int64) *big.Rat { return big.NewRat(num, den) }

	t.Run("two arcs at an end corner", func(t *testing.T) {
		t.Parallel()
		doc, d := arcPrism(t, notchSection)
		body, err := d.Shell(t.Context(), notchRemoved(t, d), units.Millimeters(4))
		require.NoError(t, err)
		requireVolumeBracketed(t, body, r(1088, 1),
			closedTerm{r(-278, 1), piLo, piHi}, closedTerm{r(2312, 1), atan815Lo, atan815Hi})
		// Two caps, the cavity's floor and ceiling, the kept arc and its
		// offset, the removed arc's circle in three pieces (the rim column
		// from (13,0) to (15,8) over [0, 10] and a strip in each cap slab),
		// a strip in each cap slab on x = 13 and on y = 20, and the x = 0
		// plane's one face, the rim (0,13)→(0,17) among it.
		require.Len(t, body.Faces(), 14)
		require.Len(t, cylinderFaces(body, 0, 0, 13), 1)
		require.Len(t, cylinderFaces(body, 0, 0, 17), 1)
		require.Len(t, cylinderFaces(body, -2, 8, 17), 3)
		requireBrepRoles(t, body)
		requireSoundMesh(t, doc, body)
	})
	t.Run("two arcs at an end corner, both caps removed", func(t *testing.T) {
		t.Parallel()
		doc, d := arcPrism(t, notchSection)
		body, err := d.Shell(t.Context(), notchRemoved(t, d).Or(decad.NormalTo(r3.NewVec(0, 0, 1))), units.Millimeters(4))
		require.NoError(t, err)
		vol, err := body.Volume()
		require.NoError(t, err)
		requirePiLinearEnclosed(t, vol, -160, 300)
		requireSoundMesh(t, doc, body)
	})
	t.Run("two arcs at an end corner, a float cut under kept caps (SO5)", func(t *testing.T) {
		t.Parallel()
		// At t = 2 the cut y = 4(1 + √239)/17 is a float solve: the rim, a
		// range of the removed arc's record, ends where its parameter
		// evaluates, and the offset arc ends at the held cut, so the two
		// circles meet at two walked ends and the record build misses.
		doc, d := arcPrism(t, notchSection)
		before := snapshotDocument(t, doc)
		_, err := d.Shell(t.Context(), notchRemoved(t, d), units.Millimeters(2))
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.ErrorContains(t, err, "SO5")
		require.Equal(t, before.bodies, doc.Bodies())
	})
	t.Run("two arcs at an interior corner", func(t *testing.T) {
		t.Parallel()
		for _, tc := range []struct {
			name          string
			capsRemoved   bool
			base, a, pi   *big.Rat
			faces, planes int
		}{
			// Two caps, the cavity's floor and ceiling, the two arcs and
			// their offsets, and the chord plane's one face with the
			// opening as its hole.
			{"both caps kept", false, r(-555, 8), r(22750, 64), r(-3375, 64), 9, 5},
			// The wall section's two caps, the two arcs, their offsets and
			// the two rims on the chord's plane.
			{"both caps removed", true, r(-105, 2), r(3125, 8), r(-1125, 16), 8, 4},
		} {
			t.Run(tc.name, func(t *testing.T) {
				t.Parallel()
				doc, d := arcPrism(t, halfLensSection)
				sel := decad.Faces(decad.Facing(r3.NewVec(0, -1, 0)))
				if tc.capsRemoved {
					sel = sel.Or(decad.NormalTo(r3.NewVec(0, 0, 1)))
				}
				body, err := d.Shell(t.Context(), sel, units.Millimeters(1.25))
				require.NoError(t, err)
				requireVolumeBracketed(t, body, tc.base,
					closedTerm{tc.a, acos35Lo, acos35Hi}, closedTerm{tc.pi, piLo, piHi})
				require.Len(t, body.Faces(), tc.faces)
				require.Len(t, cylinderFaces(body, 0, 0, 3.75), 1)
				require.Len(t, cylinderFaces(body, 6, 0, 3.75), 1)
				planes := 0
				for _, f := range body.Faces() {
					if _, ok := f.Surface().(decad.Plane); ok {
						planes++
					}
				}
				require.Equal(t, tc.planes, planes)
				requireSoundMesh(t, doc, body)
			})
		}
	})
	t.Run("an arc join on a slanted wall under kept caps", func(t *testing.T) {
		t.Parallel()
		doc, tri := polygonPrism(t, obliqueTriangle)
		body, err := tri.Shell(t.Context(), decad.Faces(decad.Facing(r3.NewVec(0, -1, 0))), units.Millimeters(1),
			decad.WithShellSense(decad.Outward))
		require.NoError(t, err)
		requireVolumeBracketed(t, body, r(404, 1),
			closedTerm{r(6, 1), piLo, piHi}, closedTerm{r(-6, 1), acos35Lo, acos35Hi})
		// Two caps, the cavity's floor and ceiling, the two kept walls, the
		// slanted offset, the corner's cylinder, x = −1, and the y = 0 plane's
		// one face whose hole is the removed leg.
		require.Len(t, body.Faces(), 10)
		cylinders := 0
		for _, f := range body.Faces() {
			c, ok := f.Surface().(decad.Cylinder)
			if !ok {
				continue
			}
			cylinders++
			require.Equal(t, 0.0, c.Origin.X)
			require.Equal(t, 9.0, c.Origin.Y)
			require.InDelta(t, 1, c.Radius.Base(), 1e-12)
		}
		require.Equal(t, 1, cylinders)
		requireOneHole(t, body, r3.NewVec(0, -1, 0), 0, r3.NewVec(0, 0, 0), r3.NewVec(12, 0, 10), 1e-12)
		requireBrepRoles(t, body)
		requireSoundMesh(t, doc, body)
	})
}

// The constants of TestShellSideOpeningArcExtension's closed forms, each
// bracketed to 50 digits. The outward notch at t = 4 cuts the removed arc's
// circle at q = ((35 + 8√38)/17, (−140 + 2√38)/17); phiQ is q's angle about
// the origin and psiQ its angle about the removed arc's centre (−2, 8).
const (
	atan512Lo = "0.39479111969976151674009953038958058689517020757570"
	atan512Hi = "0.39479111969976151674009953038958058689517020757571"
	sqrt38Lo  = "6.16441400296897645025019238145424422523562402344457"
	sqrt38Hi  = "6.16441400296897645025019238145424422523562402344458"
	phiQLo    = "-0.98713781515123056742052203586530040837764883202801"
	phiQHi    = "-0.98713781515123056742052203586530040837764883202800"
	psiQLo    = "-1.14900488525731485852274066345089293337645014963932"
	psiQHi    = "-1.14900488525731485852274066345089293337645014963931"
)

// notchEndSection is notchSection's kept arc ended at (5, 12) against a
// vertical removed face: the concave arc of radius 13 about the origin from
// (0,13) to (5,12), with the material outside it, then x = 5 up to (5,20),
// y = 20 back to (0,20) and x = 0 down to (0,13).
func notchEndSection(s *sketch.Sketch) {
	o := s.CreatePoint(0, 0)
	a := s.CreatePoint(0, 13)
	b := s.CreatePoint(5, 12)
	e := s.CreatePoint(5, 20)
	g := s.CreatePoint(0, 20)
	for _, p := range []*sketch.Point{o, a, b, e, g} {
		s.Fix(p)
	}
	s.CreateArc(o, b, a)
	s.CreateLine(b, e)
	s.CreateLine(e, g)
	s.CreateLine(g, a)
}

// TestShellSideOpeningArcExtension covers a kept arc whose offset runs past
// its own end to the rim cut (docs/shell-opening-design.md §2.4's consumption
// row), each fixture 10 tall and shelled outward with every walk but the
// concave arc removed:
//
//   - notchEndSection at t = 2.375: the offset arc, radius 10.625 about the
//     origin, starts at the exact cut (0, 10.625) and runs past the arc's end
//     angle atan(12/5) to the cut (5, 9.375) on x = 5. With β = atan(8/15) and
//     A = atan(5/12), P is 70 − 84.5A, the outer region O = K' then R' is
//     76.5625 − 56.4453125β, and the wall W = O − P, so the body is
//     10·W + 2·2.375·O = 27475/64 − (426275/512)β + 845A with both caps
//     kept and 10·W = 525/8 − (36125/64)β + 845A with both removed;
//   - notchSection at t = 4: the offset arc of radius 9 runs past (13,0) to
//     the removed arc's circle at q, a float solve on the arc's complement.
//     With both caps removed the wall W, by Green's theorem over its four
//     pieces, is 22π − 52 + 2√38 + 40.5φ − 144.5(β + ψ), φ and ψ q's angles
//     about the two centres, and the body is 10·W; with a cap kept the rim's
//     range and the offset arc end at two walked points, so the record build
//     misses (SO5), as at the inward notch's float cut;
//   - notchEndSection's arc started instead by the slant (−7,18)→(0,13): the
//     slant's cut trims the offset arc to the angle 59.8°, past the
//     extended end at 61.9°, so the trimmed arc runs backward (S11a).
//
// Shown to fail: with offsetOpenChain reading every walk through WalkConsumed
// again, every building fixture here refused S11a.
func TestShellSideOpeningArcExtension(t *testing.T) {
	t.Parallel()
	r := func(num, den int64) *big.Rat { return big.NewRat(num, den) }
	outward := decad.WithShellSense(decad.Outward)
	// planes names the x = 5 (or x = 13), y = 20 and x = 0 faces; Or extends
	// the query it is called on, so each subtest takes its own.
	planes := func() *decad.FaceQuery {
		return decad.Faces(decad.Facing(r3.NewVec(1, 0, 0))).
			Or(decad.Facing(r3.NewVec(0, 1, 0))).
			Or(decad.Facing(r3.NewVec(-1, 0, 0)))
	}
	notchRemoved := func(t *testing.T, d *decad.Body) *decad.FaceQuery {
		t.Helper()
		onArc := cylinderFaces(d, -2, 8, 17)
		require.Len(t, onArc, 1)
		return planes().Or(decad.FaceCreatedBy(onArc[0].Origins()[0]))
	}

	t.Run("a vertical end face", func(t *testing.T) {
		t.Parallel()
		for _, tc := range []struct {
			name        string
			capsRemoved bool
			base, beta  *big.Rat
			faces       int
		}{
			// Two caps, the two faces exposing P at z = 0 and z = 10, the
			// kept arc and its offset, the x = 5 and x = 0 planes each one
			// face around the opening, and a strip of y = 20 in each cap
			// slab.
			{"both caps kept", false, r(27475, 64), r(-426275, 512), 10},
			// The wall section's two caps, the kept arc, its offset and the
			// two rims.
			{"both caps removed", true, r(525, 8), r(-36125, 64), 6},
		} {
			t.Run(tc.name, func(t *testing.T) {
				t.Parallel()
				doc, d := arcPrism(t, notchEndSection)
				sel := planes()
				if tc.capsRemoved {
					sel = sel.Or(decad.NormalTo(r3.NewVec(0, 0, 1)))
				}
				body, err := d.Shell(t.Context(), sel, units.Millimeters(2.375), outward)
				require.NoError(t, err)
				requireVolumeBracketed(t, body, tc.base,
					closedTerm{tc.beta, atan815Lo, atan815Hi}, closedTerm{r(845, 1), atan512Lo, atan512Hi})
				require.Len(t, cylinderFaces(body, 0, 0, 13), 1)
				require.Len(t, cylinderFaces(body, 0, 0, 10.625), 1)
				require.Len(t, body.Faces(), tc.faces)
				requireSoundMesh(t, doc, body)
			})
		}
	})
	t.Run("the notch, both caps removed", func(t *testing.T) {
		t.Parallel()
		doc, d := arcPrism(t, notchSection)
		sel := notchRemoved(t, d).Or(decad.NormalTo(r3.NewVec(0, 0, 1)))
		body, err := d.Shell(t.Context(), sel, units.Millimeters(4), outward)
		require.NoError(t, err)
		vol, err := body.Volume()
		require.NoError(t, err)
		require.Positive(t, vol.Bound.Base(), "the float cut is charged")
		requireVolumeBracketed(t, body, r(-520, 1),
			closedTerm{r(220, 1), piLo, piHi}, closedTerm{r(20, 1), sqrt38Lo, sqrt38Hi},
			closedTerm{r(405, 1), phiQLo, phiQHi}, closedTerm{r(-1445, 1), atan815Lo, atan815Hi},
			closedTerm{r(-1445, 1), psiQLo, psiQHi})
		requireSoundMesh(t, doc, body)
	})
	t.Run("the notch, a cap kept (SO5)", func(t *testing.T) {
		t.Parallel()
		doc, d := arcPrism(t, notchSection)
		before := snapshotDocument(t, doc)
		_, err := d.Shell(t.Context(), notchRemoved(t, d), units.Millimeters(4), outward)
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.ErrorContains(t, err, "SO5")
		require.Equal(t, before.bodies, doc.Bodies())
	})
	t.Run("a trim past the extended end (S11a)", func(t *testing.T) {
		t.Parallel()
		doc, d := arcPrism(t, func(s *sketch.Sketch) {
			o := s.CreatePoint(0, 0)
			a := s.CreatePoint(0, 13)
			b := s.CreatePoint(5, 12)
			e := s.CreatePoint(5, 20)
			f := s.CreatePoint(-7, 20)
			g := s.CreatePoint(-7, 18)
			for _, p := range []*sketch.Point{o, a, b, e, f, g} {
				s.Fix(p)
			}
			s.CreateArc(o, b, a)
			s.CreateLine(b, e)
			s.CreateLine(e, f)
			s.CreateLine(f, g)
			s.CreateLine(g, a)
		})
		before := snapshotDocument(t, doc)
		_, err := d.Shell(t.Context(), planes().Or(decad.Facing(r3.NewVec(-5, -7, 0))), units.Millimeters(2.375), outward)
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.ErrorContains(t, err, "drops a section feature")
		require.Equal(t, before.bodies, doc.Bodies())
	})
}
