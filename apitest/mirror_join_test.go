package apitest_test

import (
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// The join tests of docs/mirror-pattern-design.md §8: Mirrored with WithJoin
// rewrites a prism's or a stacked prism's own section into the union of the
// receiver and its image, and refuses every receiver §5.1 does not admit.

var (
	joinPlusX  = r3.NewVec(1, 0, 0)
	joinMinusX = r3.NewVec(-1, 0, 0)
	joinPlusZ  = r3.NewVec(0, 0, 1)
)

// joinWallAt selects the one planar face of b facing dir whose plane lies at
// coordinate at along dir, by that face's own provenance: FaceCreatedBy over
// its first origin.
func joinWallAt(t *testing.T, b *decad.Body, dir r3.Vec, at float64) *decad.FaceQuery {
	t.Helper()
	faces, err := decad.Faces(decad.Planar(), decad.Facing(dir)).SelectFaces(b)
	require.NoError(t, err)
	var picked []*decad.Face
	for _, f := range faces {
		pl, ok := f.Surface().(decad.Plane)
		require.True(t, ok)
		if math.Abs(pl.Frame.Origin().Dot(dir)-at) < 1e-9 {
			picked = append(picked, f)
		}
	}
	require.Len(t, picked, 1, "one wall faces %v at %v", dir, at)
	require.NotEmpty(t, picked[0].Origins())
	return decad.Faces(decad.FaceCreatedBy(picked[0].Origins()[0]))
}

// requireJoinRefused asserts err wraps want and names the admission row,
// and that the refused call left the document as it was.
func requireJoinRefused(t *testing.T, err, want error, row string, doc *decad.Document, before []*decad.Body) {
	t.Helper()
	require.ErrorIs(t, err, want)
	if row != "" {
		require.ErrorContains(t, err, row)
	}
	require.Equal(t, before, doc.Bodies(), "a refused join leaves the document unchanged")
}

// TestMirrorJoinOneWallMakesATee is §8's one-wall join: the L of §2 joined
// across its wall x = 0 is a T of twice the volume, Exact, one lump with ten
// faces, its centroid on the mirror plane, six convex and two concave
// vertical edges, and filletable.
func TestMirrorJoinOneWallMakesATee(t *testing.T) {
	t.Parallel()
	sc := newMirrorScene()
	l := sc.mirrorL(t)
	tee, err := l.Mirrored(t.Context(), decad.MirrorFace{
		Body: l, Face: decad.Faces(decad.Planar(), decad.Facing(joinMinusX))}, decad.WithJoin())
	require.NoError(t, err)
	require.Equal(t, []*decad.Body{tee}, sc.doc.Bodies())

	v, err := tee.Volume()
	require.NoError(t, err)
	require.Equal(t, decad.Exact, v.Exactness)
	require.Equal(t, 3500.0, volumeMM(t, v))
	require.Len(t, tee.Lumps(), 1)
	require.Len(t, tee.Faces(), 10)
	requireManifold(t, tee)
	requireMirrorBox(t, tee, r3.NewVec(-20, 0, 0), r3.NewVec(20, 20, 10))

	c, err := tee.Centroid()
	require.NoError(t, err)
	require.Zero(t, c.Value.X, "the T is symmetric about x = 0")

	convex, err := decad.Edges(decad.Convex(), decad.ParallelTo(joinPlusZ)).SelectEdges(tee)
	require.NoError(t, err)
	require.Len(t, convex, 6)
	concave, err := decad.Edges(decad.Concave(), decad.ParallelTo(joinPlusZ)).SelectEdges(tee)
	require.NoError(t, err)
	require.Len(t, concave, 2)
	report, err := sc.doc.Verify(t.Context())
	require.NoError(t, err)
	require.Equal(t, decad.Sound, report.Status, "diagnostics: %v", report.Diagnostics)
	_, err = tee.Fillet(t.Context(), decad.Edges(decad.Convex(), decad.ParallelTo(joinPlusZ)), units.Millimeters(1))
	require.NoError(t, err)
}

// joinUCorners is §8's U, 30×20 with its notch open at x = 30. The notch
// ends in a V, so its two tip walls are the only planar faces facing +x.
var joinUCorners = [][2]float64{
	{0, 0}, {30, 0}, {30, 2.5}, {20, 2.5}, {15, 10}, {20, 17.5}, {30, 17.5}, {30, 20}, {0, 20},
}

// TestMirrorJoinTwoWallsMakesARing is §8's two-wall join: the U joined across
// both tip walls closes its notch and the notch's image into one hole. The
// receiver's area is 600 − 187.5, so the ring is (1200 − 375) × 10 mm³.
func TestMirrorJoinTwoWallsMakesARing(t *testing.T) {
	t.Parallel()
	sc := newMirrorScene()
	u := sc.poly(t, sc.w.XY(), joinUCorners, mirrorAlong(10))
	tips, err := decad.Faces(decad.Planar(), decad.Facing(joinPlusX)).SelectFaces(u)
	require.NoError(t, err)
	require.Len(t, tips, 2, "the premise: the two tip walls are the only +x faces")

	ring, err := u.Mirrored(t.Context(), decad.MirrorFace{
		Body: u, Face: decad.Faces(decad.Planar(), decad.Facing(joinPlusX))}, decad.WithJoin())
	require.NoError(t, err)
	v, err := ring.Volume()
	require.NoError(t, err)
	require.Equal(t, decad.Exact, v.Exactness)
	require.Equal(t, 8250.0, volumeMM(t, v))
	require.Len(t, ring.Lumps(), 1)
	shells := ring.Shells()
	require.Len(t, shells, 1)
	require.False(t, shells[0].IsVoid())
	requireManifold(t, ring)
	requireMirrorBox(t, ring, r3.NewVec(0, 0, 0), r3.NewVec(60, 20, 10))
	caps, err := topCap(ring).SelectFaces(ring)
	require.NoError(t, err)
	require.Len(t, caps, 1)
	require.Len(t, caps[0].Loops(), 2, "one outer loop and one hole")
}

// TestMirrorJoinKeepsHolesAndTheirImages joins a plate with a round hole
// across its wall x = 0: the hole stays and its image joins it, so the cap
// holds three loops and the volume encloses 2·(200 − 4π)·4.
//
// Shown to fail with the circle image's CCW flag flipped (CircleSeg{m(C),
// Radius, !CCW}): the flag then contradicts the image's range and the join
// refuses its own record.
func TestMirrorJoinKeepsHolesAndTheirImages(t *testing.T) {
	t.Parallel()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 0, 20, 10)
	s.Fix(rect.A)
	c := s.CreatePoint(10, 5)
	s.Fix(c)
	s.CreateCircle(c, 2)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	var profile *sketch.Profile
	for _, p := range s.Profiles() {
		if len(p.Holes) == 1 {
			profile = p
		}
	}
	require.NotNil(t, profile)
	doc := decad.New()
	plate, err := doc.Extrude(s, profile, mirrorAlong(4))
	require.NoError(t, err)

	joined, err := plate.Mirrored(t.Context(), decad.MirrorFace{
		Body: plate, Face: decad.Faces(decad.Planar(), decad.Facing(joinMinusX))}, decad.WithJoin())
	require.NoError(t, err)
	v, err := joined.Volume()
	require.NoError(t, err)
	requirePiLinearEnclosed(t, v, 1600, -32)
	caps, err := topCap(joined).SelectFaces(joined)
	require.NoError(t, err)
	require.Len(t, caps, 1)
	require.Len(t, caps[0].Loops(), 3)
	requireMirrorBox(t, joined, r3.NewVec(-20, 0, 0), r3.NewVec(20, 10, 4))
}

// TestMirrorJoinStackedPlate is §8's stacked join: a 10 mm cube with a 4×4×4
// pocket from the top (936 mm³) joined across its wall x = 10 is a 20×10×10
// plate with two pockets, Exact 1872 mm³, whose two pocket floors are its own
// faces.
//
// Shown to fail with the interfaces left as the receiver's: the stacked
// audit then refuses the second pocket's floor record (ErrDegenerate).
func TestMirrorJoinStackedPlate(t *testing.T) {
	t.Parallel()
	sc := newMirrorScene()
	plate := sc.box(t, sc.w.XY(), 0, 0, 10, 10, mirrorAlong(10))
	top, err := sc.w.CreateOffsetPlane(sc.w.XY(), 6)
	require.NoError(t, err)
	pocket := sc.box(t, top, 3, 3, 7, 7, mirrorAlong(4))
	part, err := decad.Cut(t.Context(), plate, pocket)
	require.NoError(t, err)
	v, err := part.Volume()
	require.NoError(t, err)
	require.Equal(t, 936.0, volumeMM(t, v))

	joined, err := part.Mirrored(t.Context(), decad.MirrorFace{Body: part, Face: joinWallAt(t, part, joinPlusX, 10)}, decad.WithJoin())
	require.NoError(t, err)
	v, err = joined.Volume()
	require.NoError(t, err)
	require.Equal(t, decad.Exact, v.Exactness)
	require.Equal(t, 1872.0, volumeMM(t, v))
	require.Len(t, joined.Lumps(), 1)
	requireManifold(t, joined)
	requireMirrorBox(t, joined, r3.NewVec(0, 0, 0), r3.NewVec(20, 10, 10))
	up, err := decad.Faces(decad.Planar(), decad.Facing(joinPlusZ)).SelectFaces(joined)
	require.NoError(t, err)
	require.Len(t, up, 3, "the top cap and two pocket floors")
	caps, err := topCap(joined).SelectFaces(joined)
	require.NoError(t, err)
	require.Len(t, caps, 1)
	require.Len(t, caps[0].Loops(), 3, "the top cap carries both pocket openings")
	report, err := sc.doc.Verify(t.Context())
	require.NoError(t, err)
	require.Equal(t, decad.Sound, report.Status, "diagnostics: %v", report.Diagnostics)
}

// TestMirrorJoinRefusals is §8's refusal list plus §5.1's other rows. Each
// refusal names its admission row and leaves the document unchanged.
//
// Legs shown to fail (each removed in mirror_join.go, the fixture watched go
// red, then restored): J4's wholeness test (the fragment wall then reaches
// J5), J5's circle test (the bulging arc then joins), and J7's displacement
// test (the displaced plate then joins).
func TestMirrorJoinRefusals(t *testing.T) {
	t.Parallel()
	t.Run("a mirror frame", func(t *testing.T) {
		t.Parallel()
		sc := newMirrorScene()
		box := sc.box(t, sc.w.XY(), 0, 0, 20, 10, mirrorAlong(10))
		before := sc.doc.Bodies()
		_, err := box.Mirrored(t.Context(), mirrorAtX30(t), decad.WithJoin())
		requireJoinRefused(t, err, decad.ErrUnsupported, "MirrorFace", sc.doc, before)
	})
	t.Run("a cap", func(t *testing.T) {
		t.Parallel()
		sc := newMirrorScene()
		box := sc.box(t, sc.w.XY(), 0, 0, 20, 10, mirrorAlong(10))
		before := sc.doc.Bodies()
		_, err := box.Mirrored(t.Context(), decad.MirrorFace{Body: box, Face: topCap(box)}, decad.WithJoin())
		requireJoinRefused(t, err, decad.ErrDegenerate, "not a wall", sc.doc, before)
	})
	t.Run("a curved wall", func(t *testing.T) {
		t.Parallel()
		sc := newMirrorScene()
		pin := sc.cylinder(t, sc.w.XY(), 0, 0, 2, mirrorAlong(10))
		before := sc.doc.Bodies()
		_, err := pin.Mirrored(t.Context(), decad.MirrorFace{Body: pin, Face: decad.Faces(decad.Cylindrical())}, decad.WithJoin())
		requireJoinRefused(t, err, decad.ErrDegenerate, "curved wall", sc.doc, before)
	})
	t.Run("another body's face", func(t *testing.T) {
		t.Parallel()
		sc := newMirrorScene()
		box := sc.box(t, sc.w.XY(), 0, 0, 20, 10, mirrorAlong(10))
		wall := sc.box(t, sc.w.XY(), 30, 0, 35, 10, mirrorAlong(10))
		before := sc.doc.Bodies()
		_, err := box.Mirrored(t.Context(), decad.MirrorFace{
			Body: wall, Face: decad.Faces(decad.Planar(), decad.Facing(joinMinusX))}, decad.WithJoin())
		requireJoinRefused(t, err, decad.ErrUnsupported, "face of the receiver", sc.doc, before)
	})
	t.Run("no face", func(t *testing.T) {
		t.Parallel()
		sc := newMirrorScene()
		box := sc.box(t, sc.w.XY(), 0, 0, 20, 10, mirrorAlong(10))
		before := sc.doc.Bodies()
		_, err := box.Mirrored(t.Context(), decad.MirrorFace{Body: box, Face: decad.Faces(decad.Cylindrical())}, decad.WithJoin())
		requireJoinRefused(t, err, decad.ErrCardinality, "", sc.doc, before)
		var se *decad.SelectionError
		require.ErrorAs(t, err, &se)
		require.Equal(t, "at least 1", se.Expected)
	})
	t.Run("walls off one line (J3)", func(t *testing.T) {
		t.Parallel()
		// The rectangular notch's inner wall at x = 20 faces +x beside the
		// two tips at x = 30.
		sc := newMirrorScene()
		u := sc.poly(t, sc.w.XY(), [][2]float64{
			{0, 0}, {30, 0}, {30, 2.5}, {20, 2.5}, {20, 17.5}, {30, 17.5}, {30, 20}, {0, 20},
		}, mirrorAlong(10))
		before := sc.doc.Bodies()
		_, err := u.Mirrored(t.Context(), decad.MirrorFace{
			Body: u, Face: decad.Faces(decad.Planar(), decad.Facing(joinPlusX))}, decad.WithJoin())
		requireJoinRefused(t, err, decad.ErrDegenerate, "(J3)", sc.doc, before)
	})
	t.Run("a wall fragment (J4)", func(t *testing.T) {
		t.Parallel()
		// The union's +x wall at x = 10 is the part of the square's right
		// side below the triangle: a fragment of a longer recorded line.
		sc := newMirrorScene()
		square := sc.box(t, sc.w.XY(), 0, 0, 10, 10, mirrorAlong(10))
		tri := sc.poly(t, sc.w.XY(), [][2]float64{{5, 5}, {16, 5}, {5, 16}}, mirrorAlong(10))
		part, err := decad.Union(t.Context(), square, tri)
		require.NoError(t, err)
		for _, f := range part.Faces() {
			require.NotEqual(t, decad.KindFaceted, f.Surface().Kind(), "the premise: the union is analytic")
		}
		walls, err := decad.Faces(decad.Planar(), decad.Facing(joinPlusX)).SelectFaces(part)
		require.NoError(t, err)
		require.Len(t, walls, 1)
		before := sc.doc.Bodies()
		_, err = part.Mirrored(t.Context(), decad.MirrorFace{
			Body: part, Face: decad.Faces(decad.Planar(), decad.Facing(joinPlusX))}, decad.WithJoin())
		requireJoinRefused(t, err, decad.ErrUnsupported, "(J4)", sc.doc, before)
	})
	t.Run("an arc bulging past the line (J5)", func(t *testing.T) {
		t.Parallel()
		sc := newMirrorScene()
		body := joinBulgeBody(t, sc)
		arcs, err := decad.Edges(decad.Circular()).SelectEdges(body)
		require.NoError(t, err)
		require.NotEmpty(t, arcs)
		for _, e := range arcs {
			// The premise: both arc ends lie on the material side x > 0, so
			// the bulge alone reaches past the line.
			require.Greater(t, e.Start().Position().Value.X, 0.0)
			require.Greater(t, e.End().Position().Value.X, 0.0)
		}
		box, err := body.Bounds()
		require.NoError(t, err)
		require.Less(t, box.Min.X, 0.0, "the premise: the arc bulges past x = 0")
		before := sc.doc.Bodies()
		_, err = body.Mirrored(t.Context(), decad.MirrorFace{
			Body: body, Face: decad.Faces(decad.Planar(), decad.Facing(joinMinusX))}, decad.WithJoin())
		requireJoinRefused(t, err, decad.ErrUnsupported, "(J5)", sc.doc, before)
	})
	t.Run("a displaced section (J7)", func(t *testing.T) {
		t.Parallel()
		// A square tool placed by a translation of 0.1 mm re-expresses into
		// the plate's frame with rounding, so the cut's section carries a
		// displacement.
		sc := newMirrorScene()
		plate := sc.box(t, sc.w.XY(), 0, 0, 20, 10, mirrorAlong(5))
		tool := sc.box(t, sc.w.XY(), 4, 4, 6, 6, mirrorAlong(5))
		shift, err := r3.Translation(r3.NewVec(0.1, 0, 0))
		require.NoError(t, err)
		moved, err := tool.Placed(t.Context(), shift)
		require.NoError(t, err)
		part, err := decad.Cut(t.Context(), plate, moved)
		require.NoError(t, err)
		v, err := part.Volume()
		require.NoError(t, err)
		require.NotEqual(t, decad.Exact, v.Exactness, "the premise: the cut carries a displacement")
		before := sc.doc.Bodies()
		_, err = part.Mirrored(t.Context(), decad.MirrorFace{Body: part, Face: joinWallAt(t, part, joinMinusX, 0)}, decad.WithJoin())
		requireJoinRefused(t, err, decad.ErrUnsupported, "(J7)", sc.doc, before)
	})
	t.Run("a revolve (J1)", func(t *testing.T) {
		t.Parallel()
		sc := newMirrorScene()
		s, profile := meshPolygonSketch(t, sc.w, sc.w.XY(), [][2]float64{{0, 2}, {10, 2}, {10, 6}, {0, 6}})
		body, err := sc.doc.Revolve(s, profile,
			decad.SketchLine{Start: decad.Point2{U: 0, V: 0}, End: decad.Point2{U: 1, V: 0}},
			decad.AngleExtent{A: units.Degrees(90), Dir: decad.Along})
		require.NoError(t, err)
		before := sc.doc.Bodies()
		_, err = body.Mirrored(t.Context(), decad.MirrorFace{Body: body, Face: decad.Faces(decad.FaceCreatedBy(decad.CapStart(body)))}, decad.WithJoin())
		requireJoinRefused(t, err, decad.ErrUnsupported, "prism", sc.doc, before)
	})
	t.Run("a canceled context", func(t *testing.T) {
		t.Parallel()
		sc := newMirrorScene()
		l := sc.mirrorL(t)
		canceled, cancel := context.WithCancel(t.Context())
		cancel()
		before := sc.doc.Bodies()
		_, err := l.Mirrored(canceled, decad.MirrorFace{
			Body: l, Face: decad.Faces(decad.Planar(), decad.Facing(joinMinusX))}, decad.WithJoin())
		requireJoinRefused(t, err, context.Canceled, "", sc.doc, before)
	})
}

// joinBulgeBody is a 20 mm block whose left side is the wall x = 0 for
// y ∈ [15, 20], then steps in and runs down an arc about (2.5, 10) from
// (2, 13) to (2, 7) that bulges to x ≈ −0.54.
func joinBulgeBody(t *testing.T, sc *mirrorScene) *decad.Body {
	t.Helper()
	s, err := sc.w.CreateSketch(sc.w.XY())
	require.NoError(t, err)
	pt := func(u, v float64) *sketch.Point {
		p := s.CreatePoint(u, v)
		s.Fix(p)
		return p
	}
	a, b, c := pt(0, 20), pt(0, 15), pt(2, 13)
	center := pt(2.5, 10)
	d, e, f, g := pt(2, 7), pt(3, 0), pt(20, 0), pt(20, 20)
	s.CreateLine(a, b)
	s.CreateLine(b, c)
	s.CreateArc(center, c, d)
	s.CreateLine(d, e)
	s.CreateLine(e, f)
	s.CreateLine(f, g)
	s.CreateLine(g, a)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Profiles(), 1)
	body, err := sc.doc.Extrude(s, s.Profiles()[0], mirrorAlong(5))
	require.NoError(t, err)
	return body
}
