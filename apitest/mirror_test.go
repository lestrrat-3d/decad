package apitest_test

import (
	"context"
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// The fixtures of docs/mirror-pattern-design.md §2 and §8: every payload class
// mirrored across the plane x = 30, and the plane named by a face.

// mirrorScene is one fixture's world and document.
type mirrorScene struct {
	w   *sketch.World
	doc *decad.Document
}

func newMirrorScene() *mirrorScene {
	return &mirrorScene{w: sketch.NewWorld(), doc: decad.New()}
}

// poly extrudes the closed polygon pts on plane by e.
func (sc *mirrorScene) poly(t *testing.T, plane *sketch.Plane, pts [][2]float64, e decad.Extent) *decad.Body {
	t.Helper()
	s, profile := meshPolygonSketch(t, sc.w, plane, pts)
	body, err := sc.doc.Extrude(s, profile, e)
	require.NoError(t, err)
	return body
}

// box extrudes the rectangle [x0, x1]×[y0, y1] on plane by e.
func (sc *mirrorScene) box(t *testing.T, plane *sketch.Plane, x0, y0, x1, y1 float64, e decad.Extent) *decad.Body {
	t.Helper()
	return sc.poly(t, plane, [][2]float64{{x0, y0}, {x1, y0}, {x1, y1}, {x0, y1}}, e)
}

// cylinder extrudes the circle of radius r about (cx, cy) on plane by e.
func (sc *mirrorScene) cylinder(t *testing.T, plane *sketch.Plane, cx, cy, r float64, e decad.Extent) *decad.Body {
	t.Helper()
	s, err := sc.w.CreateSketch(plane)
	require.NoError(t, err)
	c := s.CreatePoint(cx, cy)
	s.Fix(c)
	s.CreateCircle(c, r)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Profiles(), 1)
	body, err := sc.doc.Extrude(s, s.Profiles()[0], e)
	require.NoError(t, err)
	return body
}

func mirrorAlong(h float64) decad.Extent {
	return decad.Distance{D: units.Millimeters(h), Dir: decad.Along}
}

// mirrorLCorners is §2's L: area 175 mm², its wall x = 0 spanning y ∈ [0, 20],
// and its two +x walls at x = 5 and x = 20.
var mirrorLCorners = [][2]float64{{0, 0}, {20, 0}, {20, 5}, {5, 5}, {5, 20}, {0, 20}}

// mirrorL is §2's L prism, 10 mm tall: 1750 mm³.
func (sc *mirrorScene) mirrorL(t *testing.T) *decad.Body {
	t.Helper()
	return sc.poly(t, sc.w.XY(), mirrorLCorners, mirrorAlong(10))
}

// mirrorAtX30 is §8's mirror plane x = 30, spanned by +y and +z.
func mirrorAtX30(t *testing.T) decad.MirrorFrame {
	t.Helper()
	f, err := r3.NewFrame(r3.NewVec(30, 0, 0), r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1))
	require.NoError(t, err)
	return decad.MirrorFrame{Frame: f}
}

// mirrorRat is v as an exact rational.
func mirrorRat(v float64) *big.Rat { return new(big.Rat).SetFloat64(v) }

// requireMirroredScalar asserts the image's reading and the source's agree
// within the sum of their bounds, compared exactly: both enclose the same true
// quantity, which a reflection preserves.
func requireMirroredScalar(t *testing.T, src, img decad.Measurement, unit units.Unit, what string) {
	t.Helper()
	sv, err := src.Value.In(unit)
	require.NoError(t, err)
	iv, err := img.Value.In(unit)
	require.NoError(t, err)
	sb, err := src.Bound.In(unit)
	require.NoError(t, err)
	ib, err := img.Bound.In(unit)
	require.NoError(t, err)
	gap := new(big.Rat).Sub(mirrorRat(iv), mirrorRat(sv))
	allow := new(big.Rat).Add(mirrorRat(sb), mirrorRat(ib))
	require.LessOrEqual(t, gap.Abs(gap).Cmp(allow), 0,
		"%s: image %v and source %v differ by more than the bounds %v + %v", what, iv, sv, ib, sb)
}

// requireMirroredCentroid asserts the image centroid lies within the sum of
// both bounds of the source centroid reflected across x = 30, (60 − x, y, z),
// compared exactly.
func requireMirroredCentroid(t *testing.T, src, img decad.VecMeasurement) {
	t.Helper()
	sb, err := src.Bound.In(units.Millimeter)
	require.NoError(t, err)
	ib, err := img.Bound.In(units.Millimeter)
	require.NoError(t, err)
	want := [3]*big.Rat{
		new(big.Rat).Sub(big.NewRat(60, 1), mirrorRat(src.Value.X)),
		mirrorRat(src.Value.Y),
		mirrorRat(src.Value.Z),
	}
	got := [3]float64{img.Value.X, img.Value.Y, img.Value.Z}
	squared := new(big.Rat)
	for k := range want {
		d := new(big.Rat).Sub(mirrorRat(got[k]), want[k])
		squared.Add(squared, d.Mul(d, d))
	}
	allow := new(big.Rat).Add(mirrorRat(sb), mirrorRat(ib))
	require.LessOrEqual(t, squared.Cmp(allow.Mul(allow, allow)), 0,
		"image centroid %v is not the source centroid %v reflected across x = 30 within %v + %v", img.Value, src.Value, ib, sb)
}

// TestMirroredCopyEveryPayload is docs/mirror-pattern-design.md §8's
// every-payload test over §3's consumer table: each payload class mirrored
// across the plane x = 30 keeps its volume and area within their bounds,
// lands its centroid at x = 60 − x_source, verifies Sound beside its Sound
// source, and tessellates outward (positive signed volume). Prism and cup
// volumes stay Exact.
//
// Legs shown to fail (each broken in mirror.go, the fixture watched go red,
// then restored):
//   - the reflection itself: with mirrorTransform returning r3.Identity, every
//     case's centroid misses x = 60 − x_source;
//   - the plane: with the frame's origin dropped (a plane through the world
//     origin), every centroid lands at −x_source instead.
func TestMirroredCopyEveryPayload(t *testing.T) {
	t.Parallel()
	testcases := []struct {
		Name  string
		Build func(*testing.T, *mirrorScene) *decad.Body
		Exact bool
	}{
		{Name: "prism", Exact: true, Build: func(t *testing.T, sc *mirrorScene) *decad.Body {
			return sc.mirrorL(t)
		}},
		{Name: "revolve", Build: func(t *testing.T, sc *mirrorScene) *decad.Body {
			// §2 M7: a stepped section revolved 270° about the sketch's u axis.
			s, profile := meshPolygonSketch(t, sc.w, sc.w.XY(),
				[][2]float64{{0, 2}, {10, 2}, {10, 6}, {4, 6}, {4, 4}, {0, 4}})
			body, err := sc.doc.Revolve(s, profile,
				decad.SketchLine{Start: decad.Point2{U: 0, V: 0}, End: decad.Point2{U: 1, V: 0}},
				decad.AngleExtent{A: units.Degrees(270), Dir: decad.Along})
			require.NoError(t, err)
			return body
		}},
		{Name: "loft", Build: func(t *testing.T, sc *mirrorScene) *decad.Body {
			return partsBinLoft(t, sc.doc)
		}},
		{Name: "straight sweep", Build: func(t *testing.T, sc *mirrorScene) *decad.Body {
			return partsBinSweep(t, sc.doc)
		}},
		{Name: "arc sweep", Build: func(t *testing.T, sc *mirrorScene) *decad.Body {
			// A quarter turn about the x axis of the rectangle [0, 10]×[0, 6].
			s, profile := meshPolygonSketch(t, sc.w, sc.w.XY(), [][2]float64{{0, 0}, {10, 0}, {10, 6}, {0, 6}})
			body, err := sc.doc.Sweep(t.Context(), s, profile, sweepArcPath(t))
			require.NoError(t, err)
			return body
		}},
		{Name: "stacked", Build: func(t *testing.T, sc *mirrorScene) *decad.Body {
			// A blind 10×10×4 pocket from the top of a 20×20×10 plate.
			plate := sc.box(t, sc.w.XY(), 0, 0, 20, 20, mirrorAlong(10))
			top, err := sc.w.CreateOffsetPlane(sc.w.XY(), 6)
			require.NoError(t, err)
			pocket := sc.box(t, top, 5, 5, 15, 15, mirrorAlong(4))
			body, err := decad.Cut(t.Context(), plate, pocket)
			require.NoError(t, err)
			return body
		}},
		{Name: "cup", Exact: true, Build: func(t *testing.T, sc *mirrorScene) *decad.Body {
			return partsBinCup(t, sc.doc)
		}},
		{Name: "cap blend", Build: func(t *testing.T, sc *mirrorScene) *decad.Body {
			box := sc.box(t, sc.w.XY(), 0, 0, 20, 10, mirrorAlong(10))
			body, err := box.Chamfer(t.Context(), capLoopEdges(box), units.Millimeters(2))
			require.NoError(t, err)
			return body
		}},
		{Name: "faceted", Build: func(t *testing.T, sc *mirrorScene) *decad.Body {
			// §2 M9: a 20 mm cube cross-drilled along y, which takes the mesh path.
			plate := sc.box(t, sc.w.XY(), 0, 0, 20, 20, mirrorAlong(20))
			drill := sc.cylinder(t, sc.w.XZ(), 10, 10, 3, decad.Symmetric{D: units.Millimeters(30)})
			body, err := decad.Cut(t.Context(), plate, drill)
			require.NoError(t, err)
			return body
		}},
	}
	for _, tc := range testcases {
		t.Run(tc.Name, func(t *testing.T) {
			t.Parallel()
			sc := newMirrorScene()
			src := tc.Build(t, sc)
			img, err := src.MirroredCopy(t.Context(), mirrorAtX30(t))
			require.NoError(t, err)
			require.ElementsMatch(t, []*decad.Body{src, img}, sc.doc.Bodies(), "a copy leaves its source live")

			sv, err := src.Volume()
			require.NoError(t, err)
			iv, err := img.Volume()
			require.NoError(t, err)
			requireMirroredScalar(t, sv, iv, units.CubicMillimeter, "volume")
			if tc.Exact {
				require.Equal(t, decad.Exact, sv.Exactness)
				require.Equal(t, decad.Exact, iv.Exactness)
			}
			sa, err := src.Area()
			require.NoError(t, err)
			ia, err := img.Area()
			require.NoError(t, err)
			requireMirroredScalar(t, sa, ia, units.SquareMillimeter, "area")

			sc0, err := src.Centroid()
			require.NoError(t, err)
			ic, err := img.Centroid()
			require.NoError(t, err)
			requireMirroredCentroid(t, sc0, ic)

			report, err := sc.doc.Verify(t.Context())
			require.NoError(t, err)
			srcReport, err := report.ForBody(src)
			require.NoError(t, err)
			imgReport, err := report.ForBody(img)
			require.NoError(t, err)
			// Every source here verifies Sound, so §8's "Sound wherever the
			// source is" asserts both.
			require.Equal(t, decad.Sound, srcReport.Status, "source diagnostics: %v", srcReport.Diagnostics)
			require.Equal(t, decad.Sound, imgReport.Status, "image diagnostics: %v", imgReport.Diagnostics)

			mesh, err := img.Tessellate(t.Context(), units.Millimeters(.05))
			require.NoError(t, err)
			require.Greater(t, meshVolume(mesh), 0.0, "the mirrored mesh winds outward")
		})
	}
}

// TestMirroredCopyFlipsTheLsFacing is §2 M1's reading of the mirrored L: the
// source's two +x walls face −x on the image and its one −x wall faces +x,
// at x = 60, and the convex-edge census is unchanged. Shown to fail with
// mirrorTransform returning r3.Identity: the image keeps two +x walls.
func TestMirroredCopyFlipsTheLsFacing(t *testing.T) {
	t.Parallel()
	sc := newMirrorScene()
	l := sc.mirrorL(t)
	img, err := l.MirroredCopy(t.Context(), mirrorAtX30(t))
	require.NoError(t, err)

	plusX := decad.Faces(decad.Planar(), decad.Facing(r3.NewVec(1, 0, 0)))
	minusX := decad.Faces(decad.Planar(), decad.Facing(r3.NewVec(-1, 0, 0)))
	src, err := plusX.SelectFaces(l)
	require.NoError(t, err)
	require.Len(t, src, 2)
	facing, err := plusX.SelectFaces(img)
	require.NoError(t, err)
	require.Len(t, facing, 1, "only the image of the wall x = 0 faces +x")
	backs, err := minusX.SelectFaces(img)
	require.NoError(t, err)
	require.Len(t, backs, 2)

	box, err := img.Bounds()
	require.NoError(t, err)
	bound, err := box.Bound.In(units.Millimeter)
	require.NoError(t, err)
	require.InDelta(t, 60.0, box.Max.X, bound, "the wall x = 0 lands on x = 60")
	require.InDelta(t, 40.0, box.Min.X, bound, "the wall x = 20 lands on x = 40")

	convex := func(b *decad.Body) int {
		n := 0
		for _, e := range b.Edges() {
			if e.IsConvex() {
				n++
			}
		}
		return n
	}
	require.Equal(t, convex(l), convex(img))
	require.Len(t, img.Edges(), len(l.Edges()))
}

// TestMirroredRetiresTheReceiver is §2 M6: Mirrored is Placed under the
// reflection, so the receiver is retired and the image is the one live body.
// Shown to fail with Mirrored delegating to PlacedCopy: the receiver stays
// live.
func TestMirroredRetiresTheReceiver(t *testing.T) {
	t.Parallel()
	sc := newMirrorScene()
	l := sc.mirrorL(t)
	img, err := l.Mirrored(t.Context(), mirrorAtX30(t))
	require.NoError(t, err)
	require.Equal(t, []*decad.Body{img}, sc.doc.Bodies())

	v, err := img.Volume()
	require.NoError(t, err)
	require.Equal(t, decad.Exact, v.Exactness)
	require.Equal(t, 1750.0, volumeMM(t, v))

	_, err = l.Mirrored(t.Context(), mirrorAtX30(t))
	require.ErrorIs(t, err, decad.ErrRetiredBody)
	_, err = l.MirroredCopy(t.Context(), mirrorAtX30(t))
	require.ErrorIs(t, err, decad.ErrRetiredBody)
}

// TestMirrorFaceResolvesAPlane is §8's MirrorFace bullet: the plane is the
// selected face's own Plane.Frame, through the implicit exactly-one rule.
//
// Legs shown to fail (each broken in mirror.go, the fixture watched go red,
// then restored):
//   - the flat-faceted arm: with it removed, a Faceted face reads
//     ErrDegenerate, not ErrUnsupported;
//   - the face body's liveness gate: with it removed, a retired face body
//     resolves instead of reading ErrRetiredBody.
func TestMirrorFaceResolvesAPlane(t *testing.T) {
	t.Parallel()
	t.Run("wall", func(t *testing.T) {
		t.Parallel()
		sc := newMirrorScene()
		box := sc.box(t, sc.w.XY(), 0, 0, 20, 10, mirrorAlong(10))
		img, err := box.MirroredCopy(t.Context(), decad.MirrorFace{
			Body: box, Face: decad.Faces(decad.Planar(), decad.Facing(r3.NewVec(1, 0, 0)))})
		require.NoError(t, err)
		requireMirrorBox(t, img, r3.NewVec(20, 0, 0), r3.NewVec(40, 10, 10))
		require.ElementsMatch(t, []*decad.Body{box, img}, sc.doc.Bodies())
	})
	t.Run("cap", func(t *testing.T) {
		t.Parallel()
		// Without a join, any analytic planar face names a mirror plane: the
		// top cap reflects the box onto z ∈ [10, 20].
		sc := newMirrorScene()
		box := sc.box(t, sc.w.XY(), 0, 0, 20, 10, mirrorAlong(10))
		img, err := box.Mirrored(t.Context(), decad.MirrorFace{Body: box, Face: topCap(box)})
		require.NoError(t, err)
		requireMirrorBox(t, img, r3.NewVec(0, 0, 10), r3.NewVec(20, 10, 20))
		require.Equal(t, []*decad.Body{img}, sc.doc.Bodies())
	})
	t.Run("another live body's face", func(t *testing.T) {
		t.Parallel()
		sc := newMirrorScene()
		box := sc.box(t, sc.w.XY(), 0, 0, 20, 10, mirrorAlong(10))
		wall := sc.box(t, sc.w.XY(), 30, 0, 35, 10, mirrorAlong(10))
		img, err := box.MirroredCopy(t.Context(), decad.MirrorFace{
			Body: wall, Face: decad.Faces(decad.Planar(), decad.Facing(r3.NewVec(-1, 0, 0)))})
		require.NoError(t, err)
		requireMirrorBox(t, img, r3.NewVec(40, 0, 0), r3.NewVec(60, 10, 10))
		require.ElementsMatch(t, []*decad.Body{box, wall, img}, sc.doc.Bodies(), "the face body stays live")
	})
	t.Run("two walls are ErrCardinality", func(t *testing.T) {
		t.Parallel()
		sc := newMirrorScene()
		l := sc.mirrorL(t)
		_, err := l.Mirrored(t.Context(), decad.MirrorFace{
			Body: l, Face: decad.Faces(decad.Planar(), decad.Facing(r3.NewVec(1, 0, 0)))})
		require.ErrorIs(t, err, decad.ErrCardinality)
		var se *decad.SelectionError
		require.ErrorAs(t, err, &se)
		require.Equal(t, "exactly 1", se.Expected)
		require.Equal(t, 2, se.Actual)
		require.Equal(t, []*decad.Body{l}, sc.doc.Bodies())
	})
	t.Run("no face is ErrCardinality", func(t *testing.T) {
		t.Parallel()
		sc := newMirrorScene()
		box := sc.box(t, sc.w.XY(), 0, 0, 20, 10, mirrorAlong(10))
		_, err := box.MirroredCopy(t.Context(), decad.MirrorFace{Body: box, Face: decad.Faces(decad.Cylindrical())})
		require.ErrorIs(t, err, decad.ErrCardinality)
		var se *decad.SelectionError
		require.ErrorAs(t, err, &se)
		require.Equal(t, "exactly 1", se.Expected)
		require.Equal(t, 0, se.Actual)
	})
	t.Run("curved face is ErrDegenerate", func(t *testing.T) {
		t.Parallel()
		sc := newMirrorScene()
		pin := sc.cylinder(t, sc.w.XY(), 0, 0, 3, mirrorAlong(10))
		_, err := pin.MirroredCopy(t.Context(), decad.MirrorFace{Body: pin, Face: decad.Faces(decad.Cylindrical())})
		require.ErrorIs(t, err, decad.ErrDegenerate)
		require.Equal(t, []*decad.Body{pin}, sc.doc.Bodies())
	})
	t.Run("flat faceted face is ErrUnsupported", func(t *testing.T) {
		t.Parallel()
		sc := newMirrorScene()
		plate := sc.box(t, sc.w.XY(), 0, 0, 20, 20, mirrorAlong(20))
		walls, err := decad.Faces(decad.Planar(), decad.Facing(r3.NewVec(1, 0, 0))).SelectFaces(plate)
		require.NoError(t, err)
		require.Len(t, walls, 1)
		require.Len(t, walls[0].Origins(), 1)
		// The drill's frame is tilted off the plate's axes, which keeps the
		// pair off the analytic class-B reach and on the mesh path.
		tilted, err := r3.NewFrame(r3.NewVec(10, 10, 10), r3.NewVec(0.8, 0, 0.6), r3.NewVec(-0.6, 0, 0.8))
		require.NoError(t, err)
		plane, err := sc.w.CreatePlaneFromFrame(tilted)
		require.NoError(t, err)
		drill := sc.cylinder(t, plane, 0, 0, 3, decad.Symmetric{D: units.Millimeters(30)})
		drilled, err := decad.Cut(t.Context(), plate, drill)
		require.NoError(t, err)
		// The mesh path keeps the plate's wall role on the flat faceted face
		// that descends from it.
		sel := decad.Faces(decad.FaceCreatedBy(walls[0].Origins()[0]))
		faces, err := sel.SelectFaces(drilled)
		require.NoError(t, err)
		require.Len(t, faces, 1)
		require.IsType(t, decad.Faceted{}, faces[0].Surface(), "the premise: the wall carries no analytic Plane")
		_, err = drilled.MirroredCopy(t.Context(), decad.MirrorFace{Body: drilled, Face: sel})
		require.ErrorIs(t, err, decad.ErrUnsupported)
	})
	t.Run("body gates", func(t *testing.T) {
		t.Parallel()
		sc := newMirrorScene()
		box := sc.box(t, sc.w.XY(), 0, 0, 20, 10, mirrorAlong(10))
		wall := decad.Faces(decad.Planar(), decad.Facing(r3.NewVec(1, 0, 0)))

		_, err := box.MirroredCopy(t.Context(), decad.MirrorFace{Face: wall})
		require.ErrorIs(t, err, decad.ErrDegenerate, "a nil face body")
		_, err = box.MirroredCopy(t.Context(), decad.MirrorFace{Body: box})
		require.ErrorIs(t, err, decad.ErrDegenerate, "a nil face selector")

		other := newMirrorScene()
		foreign := other.box(t, other.w.XY(), 0, 0, 20, 10, mirrorAlong(10))
		_, err = box.MirroredCopy(t.Context(), decad.MirrorFace{Body: foreign, Face: wall})
		require.ErrorIs(t, err, decad.ErrForeignBody)

		retired := sc.box(t, sc.w.XY(), 30, 0, 40, 10, mirrorAlong(10))
		require.NoError(t, sc.doc.Remove(retired))
		_, err = box.MirroredCopy(t.Context(), decad.MirrorFace{Body: retired, Face: wall})
		require.ErrorIs(t, err, decad.ErrRetiredBody)
		require.Equal(t, []*decad.Body{box}, sc.doc.Bodies())
	})
}

// requireMirrorBox asserts b's bounds are [lo, hi] within their bound.
func requireMirrorBox(t *testing.T, b *decad.Body, lo, hi r3.Vec) {
	t.Helper()
	box, err := b.Bounds()
	require.NoError(t, err)
	bound, err := box.Bound.In(units.Millimeter)
	require.NoError(t, err)
	for k, pair := range [][2]float64{
		{lo.X, box.Min.X}, {lo.Y, box.Min.Y}, {lo.Z, box.Min.Z},
		{hi.X, box.Max.X}, {hi.Y, box.Max.Y}, {hi.Z, box.Max.Z},
	} {
		require.InDelta(t, pair[0], pair[1], bound, "bounds component %d of %v..%v", k, box.Min, box.Max)
	}
}

// TestMirrorFrameGates covers the gates Mirrored and MirroredCopy share with
// Placed, plus the frame's own: every refusal leaves the document unchanged.
//
// Legs shown to fail (each broken in mirror.go, the fixture watched go red,
// then restored):
//   - the non-finite mapping: with r3.ErrNonFinite reported as ErrDegenerate,
//     the far plane misses ErrNotFinite;
//   - the nil-pointer variant arm: with it removed, a typed nil *MirrorFrame
//     panics instead of reading ErrDegenerate.
func TestMirrorFrameGates(t *testing.T) {
	t.Parallel()
	sc := newMirrorScene()
	box := sc.box(t, sc.w.XY(), 0, 0, 20, 10, mirrorAlong(10))
	plane := mirrorAtX30(t)
	canceled, cancel := context.WithCancel(t.Context())
	cancel()
	far, err := r3.NewFrame(r3.NewVec(math.MaxFloat64, 0, 0), r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1))
	require.NoError(t, err)

	testcases := []struct {
		Name  string
		Plane decad.MirrorPlane
		Ctx   context.Context //nolint:containedctx // each row's context is the input under test.
		Opts  []decad.MirrorOption
		Want  error
	}{
		{Name: "zero frame", Plane: decad.MirrorFrame{}, Ctx: t.Context(), Want: decad.ErrDegenerate},
		{Name: "nil plane", Plane: nil, Ctx: t.Context(), Want: decad.ErrDegenerate},
		{Name: "typed nil frame", Plane: (*decad.MirrorFrame)(nil), Ctx: t.Context(), Want: decad.ErrDegenerate},
		{Name: "typed nil face", Plane: (*decad.MirrorFace)(nil), Ctx: t.Context(), Want: decad.ErrDegenerate},
		{Name: "far plane", Plane: decad.MirrorFrame{Frame: far}, Ctx: t.Context(), Want: decad.ErrNotFinite},
		{Name: "nil context", Plane: plane, Ctx: nil, Want: decad.ErrDegenerate},
		{Name: "canceled context", Plane: plane, Ctx: canceled, Want: context.Canceled},
	}
	for _, tc := range testcases {
		t.Run(tc.Name, func(t *testing.T) {
			_, err := box.Mirrored(tc.Ctx, tc.Plane, tc.Opts...)
			require.ErrorIs(t, err, tc.Want, "Mirrored")
			_, err = box.MirroredCopy(tc.Ctx, tc.Plane)
			require.ErrorIs(t, err, tc.Want, "MirroredCopy")
			require.Equal(t, []*decad.Body{box}, sc.doc.Bodies())
		})
	}

	_, err = box.Mirrored(t.Context(), plane, nil)
	require.ErrorIs(t, err, decad.ErrDegenerate, "a nil option")
	var none *decad.Body
	_, err = none.MirroredCopy(t.Context(), plane)
	require.ErrorIs(t, err, decad.ErrDegenerate, "a nil receiver")
	require.Equal(t, []*decad.Body{box}, sc.doc.Bodies())

	// A pointer variant names the same plane its value does.
	img, err := box.MirroredCopy(t.Context(), &plane)
	require.NoError(t, err)
	requireMirrorBox(t, img, r3.NewVec(40, 0, 0), r3.NewVec(60, 10, 10))
}
