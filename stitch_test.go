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

// TestStitchNonzeroBoundRimStaysFreeAgainstExactPatch is docs/surface-design.md
// §6.2's Table J row J5, watched directly rather than through J4
// (TestStitchDisplacedPatchStaysASheet already covers J4's bit-identical
// coordinate match with two zero-bound patches). A wall extruded 5 inches
// Along carries a top rim computed from that unit conversion
// (document.go's magnitudeInBounded): 5 inches does not convert to
// millimetres exactly, so the rim's z coordinate holds a nonzero bound even
// though the held float64 value prints as the round number 127. A patch
// built directly at z = 127 mm — a stated, recorded coordinate — carries a
// zero bound at the identical coordinate. J4 (bit-identical held
// coordinates, matched unordered) holds for all four corner pairs; only J5
// (every held value's bound is zero) tells the pairs apart, and J5 alone is
// why the rim and the patch stay unwelded — welding them would claim a
// meeting the wall's own unit-conversion rounding never proved. Removing
// J5's zero-bound condition from stitch_weld.go's classOf collapses the
// free-edge count below from 12 to 4, because the four rim/patch pairs then
// wrongly join.
func TestStitchNonzeroBoundRimStaysFreeAgainstExactPatch(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	ws, wp := plateSketch(t)
	wall, err := doc.Extrude(ws, wp, decad.Distance{D: units.Inches(5), Dir: decad.Along}, decad.WithSurfaceResult())
	require.NoError(t, err)

	w := sketch.NewWorld()
	topPlane, err := w.CreateOffsetPlane(w.XY(), 127)
	require.NoError(t, err)
	ts, err := w.CreateSketch(topPlane)
	require.NoError(t, err)
	rect := ts.CreateRectangle(0, 0, 100, 60)
	ts.Fix(rect.A)
	_, err = ts.Solve(t.Context())
	require.NoError(t, err)
	patch, err := doc.Patch(ts, ts.Profiles()[0])
	require.NoError(t, err)

	// The fixture's premise, proven rather than assumed: the wall's rim at
	// z=127 is Approximate with a nonzero bound, and the patch's boundary
	// at the same z=127 is Exact with a zero bound.
	var rimApproxAt127, patchExactAt127 bool
	for _, v := range wall.Vertices() {
		pos := v.Position()
		if pos.Value.Z == 127 && pos.Exactness == decad.Approximate && pos.Bound.Mag() > 0 {
			rimApproxAt127 = true
		}
	}
	for _, v := range patch.Vertices() {
		pos := v.Position()
		if pos.Value.Z == 127 && pos.Exactness == decad.Exact && pos.Bound.Mag() == 0 {
			patchExactAt127 = true
		}
	}
	require.True(t, rimApproxAt127, "the wall's top rim must be Approximate with a nonzero bound at z=127")
	require.True(t, patchExactAt127, "the patch's boundary must be Exact with a zero bound at z=127")

	sheet, err := decad.Stitch(wall, patch)
	require.NoError(t, err)

	require.Equal(t, decad.BodySheet, sheet.Kind())
	require.False(t, sheet.IsSolid())

	// Nothing welds: the wall's own 8 rim edges (bottom and top) plus the
	// patch's 4 boundary edges all stay free.
	free, err := decad.Edges(decad.Free()).Exactly(12).SelectEdges(sheet)
	require.NoError(t, err)
	require.Len(t, free, 12)

	// The stitched result absorbs both live operands; the document holds
	// only it.
	require.Len(t, doc.Bodies(), 1)
	require.Same(t, sheet, doc.Bodies()[0])
}

// offAxisPlateSketch is plateSketch's off-axis, non-origin-centred twin: a
// sketch plane whose axis is (1, 2, 3) normalized, nowhere near a coordinate
// axis, and whose origin sits away from the world origin. A test that needs
// a fixture no axis-aligned or origin-centred coincidence can silently zero
// a term for (the flagship §26 rotated-plane regression already states why)
// uses this instead of plateSketch.
func offAxisPlateSketch(t *testing.T) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	axis, ok := r3.NewVec(1, 2, 3).Normalize()
	require.True(t, ok)
	ref := r3.NewVec(1, 0, 0)
	u, ok := ref.Sub(axis.Scale(ref.Dot(axis))).Normalize()
	require.True(t, ok)
	frame, err := r3.NewFrame(r3.NewVec(7, -3, 11), u, axis.Cross(u))
	require.NoError(t, err)

	w := sketch.NewWorld()
	plane, err := w.CreatePlaneFromFrame(frame)
	require.NoError(t, err)
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 0, 100, 60)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	return s, s.Profiles()[0]
}

// TestStitchRefusesIdenticalBoundedRimsWithNoSharedDenotation is
// docs/surface-design.md §15's T41 — the proof the CURVE half of the
// shared-denotation certificate is an identity check, never a tolerance.
// Two INDEPENDENT Extrude calls building the identical profile at the
// identical Symmetric extent hold bit-identical rim coordinates and
// identical nonzero bounds at BOTH ends (proven first, exactly as T14
// proves its own premise) — Symmetric rather than Along, since Along's
// zero-bound bottom rim would legitimately weld through the EXISTING
// zero-bound route regardless of any certificate, proving nothing about it.
// Stitch still refuses every one of the 16 rim/rim corner pairs: each
// call's own evalPrismContext mints a FRESH curveID, so no pair ever shares
// one, whatever their held data says. This is the test that would go green
// if the certificate were ever swapped for a bound-magnitude comparison.
func TestStitchRefusesIdenticalBoundedRimsWithNoSharedDenotation(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	ws, wp := plateSketch(t)
	wallA, err := doc.Extrude(ws, wp, decad.Symmetric{D: units.Inches(2.5)}, decad.WithSurfaceResult())
	require.NoError(t, err)
	wallB, err := doc.Extrude(ws, wp, decad.Symmetric{D: units.Inches(2.5)}, decad.WithSurfaceResult())
	require.NoError(t, err)

	// The fixture's whole point, proven rather than assumed: a rim vertex
	// of each independent wall holds a BIT-IDENTICAL coordinate and an
	// IDENTICAL non-zero bound.
	rimOf := func(b *decad.Body) decad.VecMeasurement {
		rim, err := decad.Edges(decad.Free()).Exactly(8).SelectEdges(b)
		require.NoError(t, err)
		return rim[0].Start().Position()
	}
	rimA, rimB := rimOf(wallA), rimOf(wallB)
	require.Equal(t, decad.Approximate, rimA.Exactness)
	require.Greater(t, rimA.Bound.Mag(), 0.0)
	require.Equal(t, rimA.Value, rimB.Value, "the two independent builds hold the bit-identical rim coordinate")
	require.Equal(t, rimA.Bound, rimB.Bound, "the two independent builds hold the identical nonzero bound")

	sheet, err := decad.Stitch(wallA, wallB)
	require.NoError(t, err)

	require.Equal(t, decad.BodySheet, sheet.Kind())
	require.False(t, sheet.IsSolid())
	// Nothing welds: both walls' own 16 rim edges (8 apiece) stay free.
	free, err := decad.Edges(decad.Free()).Exactly(16).SelectEdges(sheet)
	require.NoError(t, err)
	require.Len(t, free, 16)
}

// TestStitchClosesABoundedPatchedWallWithChargedVolumeBound is
// docs/surface-design.md §15's T42 — the CURVE certificate's own
// mass-accumulator obligation. A Symmetric surface-extruded wall on an
// off-axis, non-origin sketch plane (offAxisPlateSketch — every OTHER
// fixture in this file sits on the axis-aligned XY plane, where a rounding
// term this test means to exercise could silently read zero) has BOTH rims
// bounded; Body.Patch caps both in one call (the LEVEL certificate's own
// flagship), closing every edge with no weld at all. The single-operand
// Stitch that follows re-audits that already-closed boundary into a
// BodySolid — and every welded vertex CLASS here is bounded, not zero, so
// the published Volume must charge that bound (docs/surface-design.md
// §6.4's amendment) rather than publish Exact over a triangle set whose
// true vertices are not the held ones.
func TestStitchClosesABoundedPatchedWallWithChargedVolumeBound(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	s, p := offAxisPlateSketch(t)
	wall, err := doc.Extrude(s, p, decad.Symmetric{D: units.Inches(2.5)}, decad.WithSurfaceResult())
	require.NoError(t, err)

	rim, err := decad.Edges(decad.Free()).Exactly(8).SelectEdges(wall)
	require.NoError(t, err)
	require.Equal(t, decad.Approximate, rim[0].Start().Position().Exactness)
	require.Greater(t, rim[0].Start().Position().Bound.Mag(), 0.0)

	capped, err := wall.Patch(decad.Edges(decad.Free()).Exactly(8))
	require.NoError(t, err)
	require.Equal(t, decad.BodySheet, capped.Kind())
	require.Len(t, capped.Faces(), 6)
	_, err = decad.Edges(decad.Free()).SelectEdges(capped)
	require.ErrorIs(t, err, decad.ErrNoMatch)

	solid, err := decad.Stitch(capped)
	require.NoError(t, err)
	require.Equal(t, decad.BodySolid, solid.Kind())
	require.True(t, solid.IsSolid())

	vol, err := solid.Volume()
	require.NoError(t, err)
	require.Equal(t, decad.Approximate, vol.Exactness)
	require.Greater(t, vol.Bound.Base(), 0.0)
	// 100 x 60 x 5 inches (127 mm): the enclosure assertion that matters —
	// the analytic volume must lie inside the PUBLISHED bound, which is
	// only true once the welded classes' own bound is charged into it.
	analytic := 100.0 * 60.0 * 127.0
	require.GreaterOrEqual(t, analytic, vol.Value.Base()-vol.Bound.Base())
	require.LessOrEqual(t, analytic, vol.Value.Base()+vol.Bound.Base())
}

// TestStitchRefusesAPlacedSheetAgainstItsUnplacedSiblings is
// docs/surface-design.md §15's T42's own placement leg: a placement breaks
// the CURVE certificate. Unstitching a bounded, closed box (stitch_test.go's
// TestStitchClosesABoundedPatchedWallWithChargedVolumeBound) frees every
// edge; placing ONE of the six resulting sheets by a rigid motion neither
// axis-aligned nor centred on the origin, then re-stitching all six, must
// NOT re-weld the moved sheet's own 4 edges against its former neighbours —
// it no longer occupies the place any curve token comparison is stated
// under (denotation.go's curveToken.xform) — while the 5 UNPLACED siblings
// still weld normally among themselves, exactly as they would if the moved
// sheet had never been unstitched at all.
func TestStitchRefusesAPlacedSheetAgainstItsUnplacedSiblings(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	s, p := offAxisPlateSketch(t)
	wall, err := doc.Extrude(s, p, decad.Symmetric{D: units.Inches(2.5)}, decad.WithSurfaceResult())
	require.NoError(t, err)
	capped, err := wall.Patch(decad.Edges(decad.Free()).Exactly(8))
	require.NoError(t, err)
	solid, err := decad.Stitch(capped)
	require.NoError(t, err)
	require.Equal(t, decad.BodySolid, solid.Kind())

	sheets, err := solid.Unstitch()
	require.NoError(t, err)
	require.Len(t, sheets, 6)

	axis, ok := r3.NewVec(3, -1, 2).Normalize()
	require.True(t, ok)
	xf, err := r3.RotationAround(r3.NewVec(41, -17, 9), axis, units.Degrees(37))
	require.NoError(t, err)
	placed, err := sheets[0].Placed(xf)
	require.NoError(t, err)

	operands := append([]*decad.Body{placed}, sheets[1:]...)
	restitched, err := decad.Stitch(operands...)
	require.NoError(t, err)

	// The moved sheet's own boundary and its former neighbours' matching
	// copies stay free (4 + 4 = 8); the remaining internal welds among the
	// 5 unplaced siblings still close, exactly as an unmoved re-stitch would.
	require.Equal(t, decad.BodySheet, restitched.Kind())
	require.False(t, restitched.IsSolid())
	free, err := decad.Edges(decad.Free()).Exactly(8).SelectEdges(restitched)
	require.NoError(t, err)
	require.Len(t, free, 8)
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
