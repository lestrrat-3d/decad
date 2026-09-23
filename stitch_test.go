package decad_test

import (
	"strings"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is docs/surface-design.md's T3/T4-shaped public tests for
// Stitch, plus Table R's R12/R13/R14/R17 rows and the two out-of-scope
// refusals a stitched solid pins for this increment (tessellation and the
// pair relation). The internal fixtures for Table R's R7 rows (a
// non-orientable set and a non-manifold one) and R9 live in
// stitch_internal_test.go, since decad's public seam admits no way to
// author either shape directly. T6's own stitched-solid half (T50) moved to
// stitch_flux_test.go once the Sphere arm retired its "ErrUnsupported"
// outcome — TestStitchClosedCurvedSheetIsUnsupported is gone, replaced by
// TestStitchSphereRevolveSheetClosesToABall.

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

// stitchBoxSheetsAtOffset is stitchBoxSheets generalised to an arbitrary
// axis-aligned span [x0,x1]×[y0,y1]×[z0,z1], for T76/T77/T78's two-box
// pinch fixtures: stitchBoxSheets itself is pinned to the fixed 100×60
// rectangle plateSketch draws at the origin, with no way to offset it in X
// or Y.
func stitchBoxSheetsAtOffset(t *testing.T, doc *decad.Document, x0, y0, x1, y1, z0, z1 float64) (walls, bottom, top *decad.Body) {
	t.Helper()
	w := sketch.NewWorld()

	basePlane, err := w.CreateOffsetPlane(w.XY(), z0)
	require.NoError(t, err)
	ws, err := w.CreateSketch(basePlane)
	require.NoError(t, err)
	rect := ws.CreateRectangle(x0, y0, x1, y1)
	ws.Fix(rect.A)
	_, err = ws.Solve(t.Context())
	require.NoError(t, err)
	walls, err = doc.Extrude(ws, ws.Profiles()[0], decad.Distance{D: units.Millimeters(z1 - z0), Dir: decad.Along}, decad.WithSurfaceResult())
	require.NoError(t, err)

	bs, err := w.CreateSketch(basePlane)
	require.NoError(t, err)
	brect := bs.CreateRectangle(x0, y0, x1, y1)
	bs.Fix(brect.A)
	_, err = bs.Solve(t.Context())
	require.NoError(t, err)
	bottom, err = doc.Patch(bs, bs.Profiles()[0])
	require.NoError(t, err)

	topPlane, err := w.CreateOffsetPlane(w.XY(), z1)
	require.NoError(t, err)
	ts, err := w.CreateSketch(topPlane)
	require.NoError(t, err)
	trect := ts.CreateRectangle(x0, y0, x1, y1)
	ts.Fix(trect.A)
	_, err = ts.Solve(t.Context())
	require.NoError(t, err)
	top, err = doc.Patch(ts, ts.Profiles()[0])
	require.NoError(t, err)

	return walls, bottom, top
}

// TestStitchRefusesTwoBoxesPinchedAtOneVertex is docs/surface-design.md's
// T76: T1's box (x∈[0,100], y∈[0,60], z∈[0,10]) and a second, otherwise
// disjoint box (x∈[100,200], y∈[60,120], z∈[10,20]) sharing exactly ONE
// corner — (100,60,10) — with no edge at all joining the two boxes there.
// The shared vertex table (stitch_weld.go) merges the two bit-identical
// corners into one entry whether or not any edge connects them, so both
// boxes weld cleanly under Table J and checkStitchClosure's own
// directed-edge parity leg sees nothing wrong either: every edge still
// bounds exactly one or two faces with correct parity. Only the vertex-link
// audit sees that the six faces meeting at the shared corner (three from
// each box) form two disconnected fans rather than one connected fan, which
// is exactly the pinch this test proves Stitch now refuses on every
// build arm, not the curved-closed arm alone.
func TestStitchRefusesTwoBoxesPinchedAtOneVertex(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	wallsA, bottomA, topA := stitchBoxSheets(t, doc, 10)
	wallsB, bottomB, topB := stitchBoxSheetsAtOffset(t, doc, 100, 60, 200, 120, 10, 20)

	_, err := decad.Stitch(wallsA, bottomA, topA, wallsB, bottomB, topB)
	require.ErrorIs(t, err, decad.ErrDegenerate)

	// A failed Stitch leaves every operand live and unretired.
	require.Len(t, doc.Bodies(), 6)
}

// TestStitchTwoDisjointBoxesFormATwoLumpSolid is docs/surface-design.md's
// T77, T76's own narrowing companion: the identical two boxes translated
// apart so NO vertex is shared at all. Without this test, a change that
// refused every multi-body Stitch (rather than only a pinched one) would
// pass T76 just as well — this is what proves the vertex-link audit only
// narrows what Stitch admits, rather than blocking the ordinary case.
func TestStitchTwoDisjointBoxesFormATwoLumpSolid(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	wallsA, bottomA, topA := stitchBoxSheets(t, doc, 10)
	wallsB, bottomB, topB := stitchBoxSheetsAtOffset(t, doc, 200, 0, 300, 60, 0, 10)

	box, err := decad.Stitch(wallsA, bottomA, topA, wallsB, bottomB, topB)
	require.NoError(t, err)

	require.Equal(t, decad.BodySolid, box.Kind())
	require.True(t, box.IsSolid())
	require.Len(t, box.Vertices(), 16)
	require.Len(t, box.Lumps(), 2)
	decadtest.MeasuresVolume(t, box, units.CubicMillimeters(120000), decadtest.Exactly())

	decadtest.IsSound(t, doc)
}

// TestStitchRefusesOpenAssemblyPinchedAtOneVertex is docs/surface-design.md's
// T78: T76's own two boxes with BOTH top patches left off, so the assembly
// stays open (each box's own top rim is a residual free edge) while still
// sharing the identical pinched corner vertex. Decision 1 runs the
// vertex-link audit on the open, all-planar arm exactly as it does the
// closed one, so this proves the pinch refuses even though §6.3's "a
// residual free edge is never an error" rule would otherwise let an open
// assembly like this one back as a BodySheet.
func TestStitchRefusesOpenAssemblyPinchedAtOneVertex(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	wallsA, bottomA, _ := stitchBoxSheets(t, doc, 10)
	wallsB, bottomB, _ := stitchBoxSheetsAtOffset(t, doc, 100, 60, 200, 120, 10, 20)

	_, err := decad.Stitch(wallsA, bottomA, wallsB, bottomB)
	require.ErrorIs(t, err, decad.ErrDegenerate)

	// A failed Stitch retires nothing: the 4 operands plus the 2 unused top
	// patches (never passed to this call) all stay live.
	require.Len(t, doc.Bodies(), 6)
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

// TestStitchCurvedSolidDoesNotYetTessellate is docs/surface-design.md's T62:
// a stitched solid holding a face that is not a Plane bounded entirely by
// Line3 edges has no recorded triangle set to restate (§14 Table D row 5),
// so Tessellate/STL/OBJ refuse it exactly as they refuse any other payload
// this evaluator has not wired a chording arm for, naming the offending
// face's own kind rather than the stitchPayload class — and the refusal
// touches nothing the body already proved: its analytic Volume/Centroid
// read unchanged afterward. This replaces
// TestStitchSolidDoesNotYetTessellate, whose all-planar box case now
// succeeds (TestStitchSolidTessellatesItsOwnTriangleSet,
// tessellate_stitch_test.go).
func TestStitchCurvedSolidDoesNotYetTessellate(t *testing.T) {
	t.Parallel()

	// Each fixture's first non-planar face may be either wall the revolve
	// built, so the message is checked against the SET of curved kinds that
	// fixture can name, never a single hardcoded one.
	fixtures := map[string]struct {
		build func(t *testing.T) *decad.Body
		names []string
	}{
		"T46 frustum": {
			build: func(t *testing.T) *decad.Body {
				const uLen, vLo0, vHi0, vLo1, vHi1 = 10.0, 5.0, 15.0, 8.0, 12.0
				_, sheet := frustumSheet(t, uLen, vLo0, vHi0, vLo1, vHi1)
				solid, err := decad.Stitch(sheet)
				require.NoError(t, err)
				return solid
			},
			names: []string{"Cone"},
		},
		"T50 ball": {
			build: func(t *testing.T) *decad.Body {
				_, sheet := sphereRevolveSheet(t, 0, 10)
				solid, err := decad.Stitch(sheet)
				require.NoError(t, err)
				return solid
			},
			names: []string{"Sphere"},
		},
		"T53 torus body": {
			build: func(t *testing.T) *decad.Body {
				sheet := halfTorusRevolveSheet(t, 5, 10, 5)
				solid, err := decad.Stitch(sheet)
				require.NoError(t, err)
				return solid
			},
			names: []string{"Cylinder", "Torus"},
		},
	}

	for name, fx := range fixtures {
		t.Run(name, func(t *testing.T) {
			t.Parallel()
			solid := fx.build(t)
			require.Equal(t, decad.BodySolid, solid.Kind())

			wantVol, err := solid.Volume()
			require.NoError(t, err)
			wantCen, err := solid.Centroid()
			require.NoError(t, err)

			_, err = solid.Tessellate(units.Millimeters(0.1))
			require.ErrorIs(t, err, decad.ErrUnsupported)
			named := false
			for _, kind := range fx.names {
				named = named || strings.Contains(err.Error(), kind)
			}
			require.True(t, named, "the message %q names the face kind, not the payload class", err.Error())
			require.NotContains(t, err.Error(), "stitchPayload")

			var sink discardWriter
			err = solid.STL(sink)
			require.ErrorIs(t, err, decad.ErrUnsupported)
			err = solid.OBJ(sink)
			require.ErrorIs(t, err, decad.ErrUnsupported)

			gotVol, err := solid.Volume()
			require.NoError(t, err)
			gotCen, err := solid.Centroid()
			require.NoError(t, err)
			require.Equal(t, wantVol, gotVol, "a refused mesh never touches the analytic measurement")
			require.Equal(t, wantCen, gotCen)
		})
	}
}

type discardWriter struct{}

func (discardWriter) Write(p []byte) (int, error) { return len(p), nil }

// stitchBoxSheetsAtZ is stitchBoxSheets with its base plane offset to z0
// rather than z=0, so the whole assembly can float clear of another body's
// own z=0 base face with no Placed transform in play — a Placed motion would
// widen every vertex bound (rigidRoundAllow) and refuse the exact-carrier
// gate T67/T68/T69 exist to exercise.
func stitchBoxSheetsAtZ(t *testing.T, doc *decad.Document, z0 float64) (walls, bottom, top *decad.Body) {
	t.Helper()
	w := sketch.NewWorld()

	basePlane, err := w.CreateOffsetPlane(w.XY(), z0)
	require.NoError(t, err)
	ws, err := w.CreateSketch(basePlane)
	require.NoError(t, err)
	rect := ws.CreateRectangle(0, 0, 100, 60)
	ws.Fix(rect.A)
	_, err = ws.Solve(t.Context())
	require.NoError(t, err)
	walls, err = doc.Extrude(ws, ws.Profiles()[0], decad.Distance{D: units.Millimeters(10), Dir: decad.Along}, decad.WithSurfaceResult())
	require.NoError(t, err)

	bs, err := w.CreateSketch(basePlane)
	require.NoError(t, err)
	brect := bs.CreateRectangle(0, 0, 100, 60)
	bs.Fix(brect.A)
	_, err = bs.Solve(t.Context())
	require.NoError(t, err)
	bottom, err = doc.Patch(bs, bs.Profiles()[0])
	require.NoError(t, err)

	topPlane, err := w.CreateOffsetPlane(w.XY(), z0+10)
	require.NoError(t, err)
	ts, err := w.CreateSketch(topPlane)
	require.NoError(t, err)
	trect := ts.CreateRectangle(0, 0, 100, 60)
	ts.Fix(trect.A)
	_, err = ts.Solve(t.Context())
	require.NoError(t, err)
	top, err = doc.Patch(ts, ts.Profiles()[0])
	require.NoError(t, err)

	return walls, bottom, top
}

// TestStitchBoxReachesAProvenClearanceGap is docs/surface-design.md's T65:
// the stitched box's own clearance-kernel carrier model
// (docs/clearance-design.md §2/§6.4's zero-bound gate) lets a pair holding
// it reach a proven, Sound answer instead of reading undecided. Shown-to-
// fail: pulling the stitchPayload arm out of newBodyGeomBudget
// (clearance_geom.go) turns this same Sound report into Suspect with
// DiagUndecidedClearance and zero Clearance rows (verified by hand while
// writing this test).
func TestStitchBoxReachesAProvenClearanceGap(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	walls, bottom, top := stitchBoxSheets(t, doc, 10)
	box, err := decad.Stitch(walls, bottom, top)
	require.NoError(t, err)

	// A plain block 3 mm beyond the box's own +X wall (the box spans
	// x ∈ [0, 100]).
	block := decadtest.NewBlock(t, doc, 103, 0, 113, 60, units.Millimeters(10))

	report := decadtest.Verify(t, doc, decad.WithClearances())
	require.Equal(t, decad.Sound, report.Status)
	requireExactGap(t, report, 3)
	decadtest.MeasuresClearance(t, report, box, block, units.Millimeters(3), decadtest.Exactly())
}

// TestStitchOverlappingSolidReportsRealInterference is
// docs/surface-design.md's T72. This replaces
// TestStitchOverlappingSolidStaysUndecided (T66), whose whole subject — a
// stitched solid's overlap ever reaching a real Interference row — this row
// retires: the clearance kernel already proved this shape of pair
// pairOverlapping before this increment
// (TestClearancePairProvesStitchedSolidOverlapDespiteTheBooleanRefusal,
// clearance_internal_test.go), but requireVolumeProvingPayload
// (boolean.go) refused the stitched operand before measuredInterference
// ever consulted that verdict. A CLOSED, all-planar, zero-vertex-bound
// stitched solid no longer hits that refusal, so the pair now resolves
// through the ordinary mesh boolean (measuredInterference's own
// evaluateBoolean(opIntersect) fallback) to a real, bounded volume.
//
// The overlap here is a clean crossing rather than T66's own fixture: T66's
// block shared the box's own y ∈ [0, 60] extent and its z = 0 base plane,
// which the mesh boolean's own (pre-existing, unrelated) coplanar-contact
// gate still refuses as undecided — a genuine limit of the general boolean,
// not of this increment's volume proof. This fixture's block sits strictly
// inside the box's y-range and off both its z-caps, so no operand face lands
// on the other operand's own face plane.
func TestStitchOverlappingSolidReportsRealInterference(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	walls, bottom, top := stitchBoxSheets(t, doc, 10)
	box, err := decad.Stitch(walls, bottom, top)
	require.NoError(t, err)

	// The box spans x ∈ [0,100], y ∈ [0,60], z ∈ [0,10]. The block spans
	// x ∈ [50,150], y ∈ [10,50] (strictly inside the box's own y-range), and
	// z ∈ [3,13] (crossing both z-caps without landing on either) — a clean
	// transversal crossing whose exact analytic overlap is
	// 50 × 40 × 7 = 14000 mm³.
	block := boxBodyAtZ(t, doc, 50, 10, 150, 50, 3, 10)

	report := decadtest.Verify(t, doc, decad.WithClearances())
	require.Equal(t, decad.Interfering, report.Status)
	require.Empty(t, report.Clearances)
	require.Len(t, report.Interferences, 1)
	// Not Exactly(): the mesh boolean's own crossing computes new rim
	// vertices from a float intersection, so the result carries the final
	// weld's own tiny rounding bound even though both operands are
	// themselves zero-bound — the bound this test proves is nonzero, not
	// zero, unlike the disjoint-union case (TestStitchSolidUnionComposesTheCorrectVolume).
	decadtest.MeasuresInterference(t, report, box, block, units.CubicMillimeters(14000))
}

// TestStitchSmallStitchedBoxContainedInABlock is docs/surface-design.md's
// T67: a small stitched box wholly inside a large plain block reaches the
// solid-solid path's own strict-containment certificate —
// TestVerifyStrictContainmentReusesInnerVolume (interference_test.go) pins
// the identical mechanism for two plain solids — without ever tessellating
// either operand: the witness cast this needs comes straight off the
// clearance kernel's own carrier model, never a mesh.
func TestStitchSmallStitchedBoxContainedInABlock(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	walls, bottom, top := stitchBoxSheetsAtZ(t, doc, 45)
	box, err := decad.Stitch(walls, bottom, top)
	require.NoError(t, err)
	wantVol, err := box.Volume()
	require.NoError(t, err)

	block := decadtest.NewBlock(t, doc, -50, -50, 200, 150, units.Millimeters(200))

	report := decadtest.Verify(t, doc, decad.WithClearances())
	require.Equal(t, decad.Interfering, report.Status)
	require.Len(t, report.Interferences, 1)
	require.Empty(t, report.Clearances)
	decadtest.MeasuresInterference(t, report, box, block, wantVol.Value, decadtest.Exactly())
}

// TestStitchBoundedStitchedSolidStaysUndecided is docs/surface-design.md's
// T68: T42's own certificate-welded stitched solid — every vertex bound
// nonzero even at identity, no placement in play — never reaches the
// clearance kernel's own carrier model, so a pair holding it reads undecided
// rather than a falsely precise gap. Shown-to-fail: deleting the zero-bound
// gate (clearance_geom.go's stitchPayload arm) lets this exact pair read
// Sound with an Exact Clearance row that does not account for the
// certificate's own residual bound — a falsely decided answer, watched red
// by hand while writing this test.
func TestStitchBoundedStitchedSolidStaysUndecided(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	s, p := offAxisPlateSketch(t)
	wall, err := doc.Extrude(s, p, decad.Symmetric{D: units.Inches(2.5)}, decad.WithSurfaceResult())
	require.NoError(t, err)
	capped, err := wall.Patch(decad.Edges(decad.Free()).Exactly(8))
	require.NoError(t, err)
	solid, err := decad.Stitch(capped)
	require.NoError(t, err)

	for _, v := range solid.Vertices() {
		require.Greater(t, v.Position().Bound.Mag(), 0.0,
			"every vertex of the CURVE-welded solid carries a nonzero bound, at identity")
	}

	bb, err := solid.Bounds()
	require.NoError(t, err)
	far := bb.Max.Add(r3.NewVec(50, 50, 50))
	decadtest.NewBlock(t, doc, far.X, far.Y, far.X+10, far.Y+10, units.Millimeters(10))

	report := decadtest.Verify(t, doc, decad.WithClearances())
	require.Equal(t, decad.Suspect, report.Status)
	require.Empty(t, report.Clearances)
	require.Empty(t, report.Interferences)
	diags := decadtest.FindDiagnostics(t, report, decad.DiagUndecidedClearance)
	require.Len(t, diags, 1)
}

// TestStitchPlacedStitchedSolidStaysUndecided is docs/surface-design.md's
// T69: T58's own zero-bound box, Placed under a non-identity rigid motion,
// reaches the identical gate T68 does by the other route — the placement's
// own rigidRoundAllow widens every vertex bound rather than a certificate
// weld's own class bound (stitch.go's "recorded weld" comment). Neither
// route alone would be trusted; together they prove the gate reads the
// vertex bound itself, not one particular cause of it.
func TestStitchPlacedStitchedSolidStaysUndecided(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	walls, bottom, top := stitchBoxSheets(t, doc, 10)
	box, err := decad.Stitch(walls, bottom, top)
	require.NoError(t, err)

	motion, err := r3.Translation(r3.NewVec(500, 500, 500))
	require.NoError(t, err)
	placed, err := box.Placed(motion)
	require.NoError(t, err)

	for _, v := range placed.Vertices() {
		require.Greater(t, v.Position().Bound.Mag(), 0.0, "a placement widens every vertex bound")
	}

	bb, err := placed.Bounds()
	require.NoError(t, err)
	decadtest.NewBlock(t, doc, bb.Min.X-20, bb.Min.Y, bb.Min.X-10, bb.Max.Y, units.Millimeters(10))

	report := decadtest.Verify(t, doc, decad.WithClearances())
	require.Equal(t, decad.Suspect, report.Status)
	require.Empty(t, report.Clearances)
	require.Empty(t, report.Interferences)
	diags := decadtest.FindDiagnostics(t, report, decad.DiagUndecidedClearance)
	require.Len(t, diags, 1)
}
