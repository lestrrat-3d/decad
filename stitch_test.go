package decad_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is docs/surface-design.md's T3/T4/T6-shaped public tests for
// Stitch, plus Table R's R12/R13/R14/R17 rows and the two out-of-scope
// refusals a stitched solid pins for this increment (tessellation and the
// pair relation). The internal fixtures for Table R's R7 rows (a
// non-orientable set and a non-manifold one) and R9 live in
// stitch_internal_test.go, since decad's public seam admits no way to
// author either shape directly.

// stitchBoxSheets builds T1's rectangle walls plus a patch at each end —
// docs/surface-design.md's worked box example — and returns the three
// live sheet bodies ready to hand to Stitch. topHeight lets a caller draw
// the top patch off the wall's own rim (T4's displaced-patch fixture).
func stitchBoxSheets(t *testing.T, doc *decad.Document, topHeight float64) (walls, bottom, top *decad.Body) {
	t.Helper()
	w := sketch.NewWorld()

	ws, wp := plateSketch(t)
	walls, err := doc.Extrude(ws, wp, decad.Distance{D: units.Millimeters(10), Dir: decad.Along}, decad.WithSurfaceResult())
	require.NoError(t, err)

	bs, bp := plateSketch(t)
	bottom, err = doc.Patch(bs, bp)
	require.NoError(t, err)

	topPlane, err := w.CreateOffsetPlane(w.XY(), topHeight)
	require.NoError(t, err)
	ts, err := w.CreateSketch(topPlane)
	require.NoError(t, err)
	rect := ts.CreateRectangle(0, 0, 100, 60)
	ts.Fix(rect.A)
	_, err = ts.Solve(t.Context())
	require.NoError(t, err)
	top, err = doc.Patch(ts, ts.Profiles()[0])
	require.NoError(t, err)

	return walls, bottom, top
}

// TestStitchClosesBoxFromThreeSheets is docs/surface-design.md's T3: T1's
// walls plus a patch at each end, stitched.
func TestStitchClosesBoxFromThreeSheets(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	walls, bottom, top := stitchBoxSheets(t, doc, 10)

	box, err := decad.Stitch(walls, bottom, top)
	require.NoError(t, err)

	require.Equal(t, decad.BodySolid, box.Kind())
	require.True(t, box.IsSolid())

	decadtest.MeasuresVolume(t, box, units.CubicMillimeters(60000), decadtest.Exactly())
	decadtest.MeasuresArea(t, box, units.SquareMillimeters(15200), decadtest.Exactly())
	decadtest.MeasuresCentroid(t, box, r3.NewVec(50, 30, 5), decadtest.Exactly())

	_, err = decad.Edges(decad.Free()).SelectEdges(box)
	require.ErrorIs(t, err, decad.ErrNoMatch)

	require.Len(t, box.Faces(), 6)
	require.Len(t, box.Edges(), 12)
	require.Len(t, box.Vertices(), 8)
	require.Len(t, box.Lumps(), 1)

	for _, e := range box.Edges() {
		require.Len(t, e.Faces(), 2)
		require.True(t, e.IsConvex(), "every edge of an axis-aligned box is convex")
	}

	// Every operand retires; the document holds only the stitched result.
	require.Len(t, doc.Bodies(), 1)
	require.Same(t, box, doc.Bodies()[0])

	decadtest.IsSound(t, doc)
}

// TestStitchDisplacedPatchStaysASheet is docs/surface-design.md's T4: T3's
// operands with one patch (the top) displaced 1e-9 mm off the wall's own
// rim level. Every one of the top patch's four corners then differs from
// the wall's top rim in its z coordinate's bit pattern, so none of the
// four pairs is bit-identical and none joins (Table J's J4).
func TestStitchDisplacedPatchStaysASheet(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	walls, bottom, top := stitchBoxSheets(t, doc, 10+1e-9)

	sheet, err := decad.Stitch(walls, bottom, top)
	require.NoError(t, err)

	require.Equal(t, decad.BodySheet, sheet.Kind())
	require.False(t, sheet.IsSolid())

	free, err := decad.Edges(decad.Free()).Exactly(8).SelectEdges(sheet)
	require.NoError(t, err)
	require.Len(t, free, 8)

	decadtest.IsSound(t, doc)
}

// TestStitchClosedCurvedSheetIsUnsupported is docs/surface-design.md's T6
// second half: a half-disc revolved a full turn about its diameter, as a
// surface, is a closed sheet with no free edge (proven first, so this test
// cannot pass through an earlier gate); stitching it alone is
// [decad.ErrUnsupported] (Table R row R8), since the boundary is already
// closed and every one of its faces is curved.
func TestStitchClosedCurvedSheetIsUnsupported(t *testing.T) {
	t.Parallel()
	s, p := semicircleSketch(t)
	doc := decad.New()
	sheet, err := doc.Revolve(s, p, uAxis, decad.FullRevolution{}, decad.WithSurfaceResult())
	require.NoError(t, err)

	// The operand really is closed before Stitch ever sees it.
	require.Equal(t, decad.BodySheet, sheet.Kind())
	_, err = decad.Edges(decad.Free()).SelectEdges(sheet)
	require.ErrorIs(t, err, decad.ErrNoMatch)

	_, err = decad.Stitch(sheet)
	require.ErrorIs(t, err, decad.ErrUnsupported)

	// The failed call leaves the document unchanged.
	require.Len(t, doc.Bodies(), 1)
	require.Same(t, sheet, doc.Bodies()[0])
}

// TestStitchPlacedBoxIsApproximateSolid is the placed-box case: T3's box,
// placed under a non-identity rigid motion, stays a solid whose volume and
// area are Approximate — a rigid motion rounds every coordinate, so the
// "recorded weld" decision replays the same admission rather than
// re-deriving it from the rounded, placed coordinates.
func TestStitchPlacedBoxIsApproximateSolid(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	walls, bottom, top := stitchBoxSheets(t, doc, 10)
	box, err := decad.Stitch(walls, bottom, top)
	require.NoError(t, err)

	motion, err := r3.Translation(r3.NewVec(5, 7, 11))
	require.NoError(t, err)
	rot, err := r3.RotationAround(r3.NewVec(5, 7, 11), r3.NewVec(0, 0, 1), units.Degrees(30))
	require.NoError(t, err)
	composed, err := motion.Then(rot)
	require.NoError(t, err)

	placed, err := box.Placed(composed)
	require.NoError(t, err)

	require.Equal(t, decad.BodySolid, placed.Kind())
	require.True(t, placed.IsSolid())

	decadtest.MeasuresVolume(t, placed, units.CubicMillimeters(60000), decadtest.WithinRel(units.Scalar(1e-6)))
	vol, err := placed.Volume()
	require.NoError(t, err)
	require.Equal(t, decad.Approximate, vol.Exactness)

	area, err := placed.Area()
	require.NoError(t, err)
	require.Equal(t, decad.Approximate, area.Exactness)

	_, err = decad.Edges(decad.Free()).SelectEdges(placed)
	require.ErrorIs(t, err, decad.ErrNoMatch)

	require.Len(t, doc.Bodies(), 1)
	require.Same(t, placed, doc.Bodies()[0])
}

// TestStitchTableR is docs/surface-design.md's T13-shaped coverage of every
// Table R row this increment raises through the public seam: R12 (no body,
// and a nil operand), R13 (a foreign body), R14 (a retired body), and R17
// (a solid operand).
func TestStitchTableR(t *testing.T) {
	t.Parallel()

	t.Run("R12 no body", func(t *testing.T) {
		t.Parallel()
		_, err := decad.Stitch()
		require.ErrorIs(t, err, decad.ErrDegenerate)
	})

	t.Run("R12 nil operand", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		_, _, top := stitchBoxSheets(t, doc, 10)
		_, err := decad.Stitch(top, nil)
		require.ErrorIs(t, err, decad.ErrDegenerate)
		require.Len(t, doc.Bodies(), 3)
	})

	t.Run("R13 foreign body", func(t *testing.T) {
		t.Parallel()
		doc1 := decad.New()
		_, _, top1 := stitchBoxSheets(t, doc1, 10)
		doc2 := decad.New()
		_, bottom2, _ := stitchBoxSheets(t, doc2, 10)
		_, err := decad.Stitch(top1, bottom2)
		require.ErrorIs(t, err, decad.ErrForeignBody)
	})

	t.Run("R14 retired body", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		walls, bottom, top := stitchBoxSheets(t, doc, 10)
		_, err := decad.Stitch(walls, bottom, top)
		require.NoError(t, err)
		_, err = decad.Stitch(walls)
		require.ErrorIs(t, err, decad.ErrRetiredBody)
	})

	t.Run("R17 solid operand", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		solid := decadtest.NewBlock(t, doc, 0, 0, 10, 10, units.Millimeters(10))
		_, err := decad.Stitch(solid)
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.Len(t, doc.Bodies(), 1)
		require.Same(t, solid, doc.Bodies()[0])
	})
}

// TestStitchSolidDoesNotYetTessellate pins one of the two out-of-scope
// refusals docs/surface-design.md §14's increment 5 names: a stitched
// solid's mesh is staged, so Tessellate/STL/OBJ refuse it exactly as they
// refuse any other payload this evaluator has not wired a chording arm for.
func TestStitchSolidDoesNotYetTessellate(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	walls, bottom, top := stitchBoxSheets(t, doc, 10)
	box, err := decad.Stitch(walls, bottom, top)
	require.NoError(t, err)

	_, err = box.Tessellate(units.Millimeters(0.1))
	require.ErrorIs(t, err, decad.ErrUnsupported)

	var sink discardWriter
	err = box.STL(sink)
	require.ErrorIs(t, err, decad.ErrUnsupported)
	err = box.OBJ(sink)
	require.ErrorIs(t, err, decad.ErrUnsupported)
}

type discardWriter struct{}

func (discardWriter) Write(p []byte) (int, error) { return len(p), nil }

// TestStitchSolidDoesNotYetReachAPairRelation pins the second out-of-scope
// refusal: a stitched solid's clearance-kernel carrier model is staged, so a
// pair holding one never reaches a proven interference or clearance row —
// the report reads undecided rather than falsely Sound.
func TestStitchSolidDoesNotYetReachAPairRelation(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	walls, bottom, top := stitchBoxSheets(t, doc, 10)
	_, err := decad.Stitch(walls, bottom, top)
	require.NoError(t, err)

	// A plain solid whose box overlaps the stitched box's, so the pair
	// cannot be dismissed by box separation alone.
	decadtest.NewBlock(t, doc, 5, 5, 20, 20, units.Millimeters(5))

	report := decadtest.Verify(t, doc, decad.WithClearances())
	require.NotEqual(t, decad.Sound, report.Status)
	require.Empty(t, report.Interferences)
	require.Empty(t, report.Clearances)
}
