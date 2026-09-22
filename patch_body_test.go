package decad_test

import (
	"context"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is docs/surface-design.md's public-surface tests for Body.Patch
// (§5.2), amended by this PR from a one-chain rule to "one or more chains,
// each proven independently": a planar fill of one or more closed chains of
// a body's own free edges.

// circleSheet builds a solved circle of radius r, centered at the origin, on
// the XY plane, and patches it directly — a Document.Patch sheet with
// exactly one free edge: a whole Circle3, its own start and end vertex the
// same (record.go's "a whole edge is a LoopRecord on its own").
func circleSheet(t *testing.T, doc *decad.Document, r float64) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	c := s.CreatePoint(0, 0)
	s.CreateCircle(c, r)
	s.Fix(c)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	sheet, err := doc.Patch(s, s.Profiles()[0])
	require.NoError(t, err)
	return sheet
}

// surfaceTube builds a 100x60 mm rectangle surface-extruded 10 mm Along, with
// WithSurfaceResult: a sheet with no caps and 8 free edges, two congruent
// four-edge rims — docs/surface-design.md's own motivating case for the
// one-or-more-chains amendment, since no edge predicate tells the two rims
// apart.
func surfaceTube(t *testing.T, doc *decad.Document) *decad.Body {
	t.Helper()
	s, p := plateSketch(t)
	tube, err := doc.Extrude(s, p, decad.Distance{D: units.Millimeters(10), Dir: decad.Along}, decad.WithSurfaceResult())
	require.NoError(t, err)
	return tube
}

// TestBodyPatchDoublesAFlatSheetWithANegatedNormal fills a Document.Patch
// sheet's own sole (4-edge, free) boundary — the "hole" a flat sheet's own
// free rim names, since nothing lies on its far side yet. Every edge is
// traversed by the SAME one face's outer loop, reversal flips a CCW outer
// walk into a CW one, and the new face's frame is chosen (patchChainOrientedNormal)
// to make that reversed walk its OWN outer (CCW) loop — the composed effect
// is the exact negation of the original face's normal.
func TestBodyPatchDoublesAFlatSheetWithANegatedNormal(t *testing.T) {
	t.Parallel()
	s, p := plateSketch(t)
	doc := decad.New()
	sheet, err := doc.Patch(s, p)
	require.NoError(t, err)

	faces, err := decad.Faces(decad.Planar()).Exactly(1).SelectFaces(sheet)
	require.NoError(t, err)
	origNormal, err := faces[0].NormalAt(r3.NewVec(50, 30, 0))
	require.NoError(t, err)

	doubled, err := sheet.Patch(decad.Edges(decad.Free()).Exactly(4))
	require.NoError(t, err)

	require.Equal(t, decad.BodySheet, doubled.Kind())
	require.False(t, doubled.IsSolid())
	decadtest.MeasuresArea(t, doubled, units.SquareMillimeters(12000), decadtest.Exactly())

	_, err = decad.Edges(decad.Free()).SelectEdges(doubled)
	require.ErrorIs(t, err, decad.ErrNoMatch)

	newFaces, err := decad.Faces(decad.Planar()).Exactly(2).SelectFaces(doubled)
	require.NoError(t, err)
	var sawOriginal, sawNegated bool
	for _, f := range newFaces {
		n, err := f.NormalAt(r3.NewVec(50, 30, 0))
		require.NoError(t, err)
		switch {
		case r3.NewVec(n.Value.X, n.Value.Y, n.Value.Z).Sub(origNormal.Value).Len() < 1e-9:
			sawOriginal = true
		case r3.NewVec(n.Value.X, n.Value.Y, n.Value.Z).Sub(origNormal.Value.Scale(-1)).Len() < 1e-9:
			sawNegated = true
		}
	}
	require.True(t, sawOriginal, "the copied original face keeps its own normal")
	require.True(t, sawNegated, "the new fill face's normal is the exact negation of the original face's")
}

// TestBodyPatchFillsASingleClosedCircularRim is the §5.2 correction this PR
// carries: a single CLOSED edge is a complete chain on its own, which
// §5.2's original "shared with exactly two other selected edges" wording
// would have wrongly refused (a whole circle shares its vertex with no
// OTHER selected edge at all).
func TestBodyPatchFillsASingleClosedCircularRim(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	disc := circleSheet(t, doc, 10)

	free, err := decad.Edges(decad.Free()).Exactly(1).SelectEdges(disc)
	require.NoError(t, err)
	_, isCircle := free[0].Curve().(decad.Circle3)
	require.True(t, isCircle, "a circle profile's own free edge is a whole Circle3")

	doubled, err := disc.Patch(decad.Edges(decad.Free()).Exactly(1))
	require.NoError(t, err)

	require.Equal(t, decad.BodySheet, doubled.Kind())
	decadtest.MeasuresArea(t, doubled, units.SquareMillimeters(2*100*3.141592653589793), decadtest.Within(units.SquareMillimeters(1e-6)))
	_, err = decad.Edges(decad.Free()).SelectEdges(doubled)
	require.ErrorIs(t, err, decad.ErrNoMatch)
	require.Len(t, doubled.Faces(), 2)
}

// TestBodyPatchFillsAHoleWithTheSameNormal is §5.2's own hole-filling case,
// the mirror image of TestBodyPatchDoublesAFlatSheetWithANegatedNormal: the
// holed face traverses the hole's rim CLOCKWISE (moments.go's own "holes
// clockwise" convention), and the patch must traverse the SAME edge
// counter-clockwise (the opposite sense), which the right-hand rule sends
// back to the SAME side rather than the opposite one — filling a hole keeps
// the normal; filling a face's own outer boundary negates it. No other test
// in this file exercises this branch: every other fixture fills an outer
// (or whole-circle) boundary.
func TestBodyPatchFillsAHoleWithTheSameNormal(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	s, p := rectWithHoleSketch(t)
	sheet, err := doc.Patch(s, p)
	require.NoError(t, err)

	faces, err := decad.Faces(decad.Planar()).Exactly(1).SelectFaces(sheet)
	require.NoError(t, err)
	holeNormal, err := faces[0].NormalAt(r3.NewVec(10, 10, 0))
	require.NoError(t, err)

	_, err = decad.Edges(decad.Free()).Exactly(5).SelectEdges(sheet)
	require.NoError(t, err)

	filled, err := sheet.Patch(decad.Edges(decad.Free(), decad.Concave()).Exactly(1))
	require.NoError(t, err)

	require.Equal(t, decad.BodySheet, filled.Kind())
	require.Len(t, filled.Faces(), 2)

	_, err = decad.Edges(decad.Free()).Exactly(4).SelectEdges(filled)
	require.NoError(t, err)

	circ, err := decad.Edges(decad.Circular()).Exactly(1).SelectEdges(filled)
	require.NoError(t, err)
	require.Len(t, circ[0].Faces(), 2, "the hole's own edge now bounds both faces")

	for _, f := range filled.Faces() {
		n, err := f.NormalAt(r3.NewVec(10, 10, 0))
		require.NoError(t, err)
		got := r3.NewVec(n.Value.X, n.Value.Y, n.Value.Z)
		require.InDeltaf(t, 0, got.Sub(holeNormal.Value).Len(), 1e-9,
			"filling a hole keeps every face's normal on the holed face's own side")
	}

	// The value cancels back to the rectangle's full 6000 mm² exactly (the
	// hole's own subtraction and the fill's own addition are the same
	// magnitude), but the bound composed from two circular integrations is
	// nonzero, so the reading is Approximate rather than Exact.
	decadtest.MeasuresArea(t, filled, units.SquareMillimeters(6000), decadtest.WithinRel(units.Scalar(1e-9)))
}

// TestBodyPatchCapsBothRimsOfATubeInOneCall is the contract change's own
// flagship case: a surface-extruded tube's 8 free edges resolve as ONE
// selection (Edges(Free()).Exactly(8)) — no predicate separates its two
// congruent rims — and the amended contract fills BOTH of the two chains
// that selection partitions into, in the one call, rather than refusing it
// as more than one chain.
func TestBodyPatchCapsBothRimsOfATubeInOneCall(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	tube := surfaceTube(t, doc)

	_, err := decad.Edges(decad.Free()).Exactly(8).SelectEdges(tube)
	require.NoError(t, err)

	capped, err := tube.Patch(decad.Edges(decad.Free()).Exactly(8))
	require.NoError(t, err)

	require.Equal(t, decad.BodySheet, capped.Kind())
	decadtest.MeasuresArea(t, capped, units.SquareMillimeters(3200+2*6000), decadtest.Exactly())

	// The tube's four walls (100x10 or 60x10 mm) and the two new 100x60 mm
	// caps are all planar; the caps are the two faces at exactly 6000 mm².
	faces, err := decad.Faces(decad.Planar()).Exactly(6).SelectFaces(capped)
	require.NoError(t, err)
	var caps int
	for _, f := range faces {
		area, err := f.Area()
		require.NoError(t, err)
		if area.Value.Base() == 6000 {
			caps++
			require.Equal(t, decad.Exact, area.Exactness)
		}
	}
	require.Equal(t, 2, caps, "exactly the two new faces have the cap's own 6000 mm^2 area")

	_, err = decad.Edges(decad.Free()).SelectEdges(capped)
	require.ErrorIs(t, err, decad.ErrNoMatch)
	require.Len(t, capped.Faces(), 6)
}

// TestBodyPatchClosesASheetsLastFreeEdgeStaysASheet is
// docs/surface-design.md's own decision: a patch that closes a sheet's last
// free edge leaves a CLOSED sheet reporting BodySheet with IsSolid() false —
// Stitch alone makes the material claim.
func TestBodyPatchClosesASheetsLastFreeEdgeStaysASheet(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	tube := surfaceTube(t, doc)

	bs, bp := plateSketch(t)
	bottom, err := doc.Patch(bs, bp)
	require.NoError(t, err)

	open, err := decad.Stitch(tube, bottom)
	require.NoError(t, err)
	require.Equal(t, decad.BodySheet, open.Kind())
	free, err := decad.Edges(decad.Free()).Exactly(4).SelectEdges(open)
	require.NoError(t, err)
	require.NotEmpty(t, free)

	closed, err := open.Patch(decad.Edges(decad.Free()).Exactly(4))
	require.NoError(t, err)

	require.Equal(t, decad.BodySheet, closed.Kind())
	require.False(t, closed.IsSolid())
	_, err = closed.Volume()
	require.ErrorIs(t, err, decad.ErrNotSolid)
	_, err = decad.Edges(decad.Free()).SelectEdges(closed)
	require.ErrorIs(t, err, decad.ErrNoMatch)
}

// TestBodyPatchReproducesThroughPlacement is §5.2's Placed/Duplicate/
// PlacedCopy claim: each re-evaluates the payload and reproduces the patch
// with no further code, on the same terms Document.Patch's own
// TestPatchReproducesThroughPlacement (patch_test.go) proves for a single
// patch face.
func TestBodyPatchReproducesThroughPlacement(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	tube := surfaceTube(t, doc)
	capped, err := tube.Patch(decad.Edges(decad.Free()).Exactly(8))
	require.NoError(t, err)
	decadtest.MeasuresArea(t, capped, units.SquareMillimeters(3200+2*6000), decadtest.Exactly())

	// A placement's own rounding (rigidRoundAllow) is charged into the
	// rebuilt geometry even for a clean translation, so a placed patch reads
	// Approximate exactly as a placed Stitch does
	// (TestStitchPlacedBoxIsApproximateSolid, stitch_test.go) — proven at or
	// below a generous relative tolerance rather than bit-exact.
	motion, err := r3.Translation(r3.NewVec(50, 0, 0))
	require.NoError(t, err)
	placed, err := capped.Placed(motion)
	require.NoError(t, err)
	require.Equal(t, decad.BodySheet, placed.Kind())
	decadtest.MeasuresArea(t, placed, units.SquareMillimeters(3200+2*6000), decadtest.WithinRel(units.Scalar(1e-6)))
	decadtest.MeasuresBounds(t, placed, r3.NewVec(50, 0, 0), r3.NewVec(150, 60, 10), decadtest.WithinRel(units.Scalar(1e-6)))
	_, err = decad.Edges(decad.Free()).SelectEdges(placed)
	require.ErrorIs(t, err, decad.ErrNoMatch)

	dup, err := placed.Duplicate()
	require.NoError(t, err)
	decadtest.MeasuresArea(t, dup, units.SquareMillimeters(3200+2*6000), decadtest.WithinRel(units.Scalar(1e-6)))

	copyMotion, err := r3.Translation(r3.NewVec(0, 50, 0))
	require.NoError(t, err)
	copied, err := dup.PlacedCopy(copyMotion)
	require.NoError(t, err)
	decadtest.MeasuresArea(t, copied, units.SquareMillimeters(3200+2*6000), decadtest.WithinRel(units.Scalar(1e-6)))
	decadtest.MeasuresBounds(t, copied, r3.NewVec(50, 50, 0), r3.NewVec(150, 110, 10), decadtest.WithinRel(units.Scalar(1e-6)))
}

// TestBodyPatchAdmitsASingleBoundedRimAlongsideAnExactOne is
// docs/surface-design.md §15's T30: the LEVEL certificate's single-bounded-
// end shape. This fixture reuses T14's own shape (a 5-inch extrude's rim
// carries the unit conversion's rounding), selecting BOTH rims of the tube
// in one call: the bottom rim's vertices are recorded directly and carry a
// zero bound, so its chain passes gate 3's first (exact) arm unchanged; the
// top rim's are computed through the inch conversion and do not, but every
// one of them shares the SAME level token prism_build.go stamped for that
// end, so its chain now passes the second (LEVEL) arm in the very same
// call — a shape this evaluator refused before this certificate existed.
func TestBodyPatchAdmitsASingleBoundedRimAlongsideAnExactOne(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	s, p := plateSketch(t)
	tube, err := doc.Extrude(s, p, decad.Distance{D: units.Inches(5), Dir: decad.Along}, decad.WithSurfaceResult())
	require.NoError(t, err)

	// No predicate tells the two congruent rims apart (docs/surface-design.md
	// §5.2), so the premise is proved over the whole free-edge set: one rim's
	// vertices read Exact (recorded directly), the other's read Approximate
	// (computed through the inch conversion).
	rim, err := decad.Edges(decad.Free()).Exactly(8).SelectEdges(tube)
	require.NoError(t, err)
	var sawExact, sawApproximate bool
	for _, e := range rim {
		switch e.Start().Position().Exactness {
		case decad.Exact:
			sawExact = true
		case decad.Approximate:
			sawApproximate = true
		}
	}
	require.True(t, sawExact, "the bottom rim is recorded directly, at zero bound")
	require.True(t, sawApproximate, "the top rim is computed through the inch conversion")

	capped, err := tube.Patch(decad.Edges(decad.Free()).Exactly(8))
	require.NoError(t, err)

	require.Equal(t, decad.BodySheet, capped.Kind())
	require.Len(t, capped.Faces(), 6)
	_, err = decad.Edges(decad.Free()).SelectEdges(capped)
	require.ErrorIs(t, err, decad.ErrNoMatch)
}

// TestBodyPatchCapsBothBoundedRimsOfASymmetricSurfaceExtrudedWall is
// docs/surface-design.md §15's T26 — the LEVEL certificate's own flagship:
// BOTH rims of a Symmetric extrude are computed through the same inch
// conversion (docs/evaluator-design.md's own resolveLinearExtent), so both
// carry a nonzero bound and neither reaches gate 3's exact arm at all. Each
// rim still shares its own one level token across its four vertices and
// edges, so both chains close through the second (LEVEL) arm in one call.
func TestBodyPatchCapsBothBoundedRimsOfASymmetricSurfaceExtrudedWall(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	s, p := plateSketch(t)
	wall, err := doc.Extrude(s, p, decad.Symmetric{D: units.Inches(2.5)}, decad.WithSurfaceResult())
	require.NoError(t, err)

	rim, err := decad.Edges(decad.Free()).Exactly(8).SelectEdges(wall)
	require.NoError(t, err)
	rv := rim[0].Start().Position()
	require.Equal(t, decad.Approximate, rv.Exactness, "both rims are computed through the inch conversion")
	require.Greater(t, rv.Bound.Mag(), 0.0)

	capped, err := wall.Patch(decad.Edges(decad.Free()).Exactly(8))
	require.NoError(t, err)

	require.Equal(t, decad.BodySheet, capped.Kind())
	require.False(t, capped.IsSolid())
	require.Len(t, capped.Faces(), 6)
	_, err = decad.Edges(decad.Free()).SelectEdges(capped)
	require.ErrorIs(t, err, decad.ErrNoMatch)

	area, err := capped.Area()
	require.NoError(t, err)
	require.Equal(t, decad.Approximate, area.Exactness)
	require.Greater(t, area.Bound.Base(), 0.0)
	// The wall's perimeter is 320 mm (T1's own 100x60 rectangle) and its
	// height is 5 inches (127 mm) total; the two new caps are 6000 mm² apiece.
	decadtest.MeasuresArea(t, capped, units.SquareMillimeters(320*127+2*6000), decadtest.WithinRel(units.Scalar(1e-6)))
}

// TestBodyPatchCapsBothBoundedRimsOnARotatedSketchPlane is T26's own
// rotated-frame regression: held vertex coordinates at one recorded level
// are only APPROXIMATELY coplanar in float64 once the sketch plane is not
// axis-aligned (frame.ToWorldUV(u, v) rounds differently per (u, v) —
// denotation.go's own doc comment), so a normal FITTED to them would be
// tilted by that rounding, with no term in axialDelta to cover it. Every
// other fixture in this file sits on the XY plane, where that rounding is
// always exactly zero (u*U+v*V never touches a nonzero cross term) and so
// never exercises this at all. This one does not: the plane's axis is
// (1, 2, 3) normalized, nowhere near a coordinate axis.
func TestBodyPatchCapsBothBoundedRimsOnARotatedSketchPlane(t *testing.T) {
	t.Parallel()
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

	doc := decad.New()
	wall, err := doc.Extrude(s, s.Profiles()[0], decad.Symmetric{D: units.Inches(2.5)}, decad.WithSurfaceResult())
	require.NoError(t, err)

	rim, err := decad.Edges(decad.Free()).Exactly(8).SelectEdges(wall)
	require.NoError(t, err)
	require.Equal(t, decad.Approximate, rim[0].Start().Position().Exactness)

	capped, err := wall.Patch(decad.Edges(decad.Free()).Exactly(8))
	require.NoError(t, err)

	require.Equal(t, decad.BodySheet, capped.Kind())
	require.Len(t, capped.Faces(), 6)
	_, err = decad.Edges(decad.Free()).SelectEdges(capped)
	require.ErrorIs(t, err, decad.ErrNoMatch)
	decadtest.MeasuresArea(t, capped, units.SquareMillimeters(320*127+2*6000), decadtest.WithinRel(units.Scalar(1e-6)))

	// The published normal must be the recorded axis (either sign, since
	// which cap this one is stays unasserted here) to well within any float64
	// rounding a fit to held vertices could introduce — never off by
	// anywhere near the scale that fitting error would actually reach for
	// this deliberately non-axis-aligned frame.
	faces, err := decad.Faces(decad.Planar()).SelectFaces(capped)
	require.NoError(t, err)
	var sawCap bool
	for _, f := range faces {
		origins := f.Origins()
		if len(origins) != 1 || origins[0].Role != "patch" {
			continue
		}
		sawCap = true
		// Plane's own NormalAt reads only the frame, never p.
		n, err := f.NormalAt(r3.Vec{})
		require.NoError(t, err)
		got := r3.NewVec(n.Value.X, n.Value.Y, n.Value.Z)
		dPos := got.Sub(axis).Len()
		dNeg := got.Sub(axis.Scale(-1)).Len()
		require.Less(t, min(dPos, dNeg), 1e-9, "the cap's published normal must track the sketch plane's own axis")
	}
	require.True(t, sawCap, "at least one new cap face was found")
}

// TestBodyPatchTableR is docs/surface-design.md's T13-shaped test for
// Body.Patch: every reachable Table R row through the public seam, with
// errors.Is and the document left unchanged. R6 (non-planar/bounded chain)
// and R18 (orientation disagreement) reach no shape decad's public seam can
// author directly (docs/surface-design.md §2.3: every builder leaves one
// shell consistently oriented) and are pinned in patch_body_internal_test.go
// instead, alongside R19's own no-payload receiver.
func TestBodyPatchTableR(t *testing.T) {
	t.Parallel()

	t.Run("R4 selection holds a shared edge", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		s, p := plateSketch(t)
		solid, err := doc.Extrude(s, p, decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
		require.NoError(t, err)

		before := doc.Bodies()
		_, err = solid.Patch(decad.Edges().Exactly(12))
		require.ErrorIs(t, err, decad.ErrDegenerate)
		require.Equal(t, before, doc.Bodies())
	})

	t.Run("R5 selection is not closed chains", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		s, p := plateSketch(t)
		sheet, err := doc.Patch(s, p)
		require.NoError(t, err)

		// The rectangle's two 100 mm edges are opposite, non-adjacent sides:
		// selecting only them leaves each of their four vertices touched by
		// exactly one selected edge, degree 1, never 2.
		before := doc.Bodies()
		_, err = sheet.Patch(decad.Edges(decad.Free(), decad.LongerThan(units.Millimeters(80))).Exactly(2))
		require.ErrorIs(t, err, decad.ErrDegenerate)
		require.Equal(t, before, doc.Bodies())
	})

	t.Run("R16 selector resolves nothing", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		s, p := plateSketch(t)
		solid, err := doc.Extrude(s, p, decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
		require.NoError(t, err)

		before := doc.Bodies()
		_, err = solid.Patch(decad.Edges(decad.Free()))
		require.ErrorIs(t, err, decad.ErrNoMatch)
		require.Equal(t, before, doc.Bodies())
	})
}

// TestBodyPatchContextCancellationLeavesDocumentUnchanged mirrors
// TestPatchContextCancellationLeavesDocumentUnchanged (patch_test.go).
func TestBodyPatchContextCancellationLeavesDocumentUnchanged(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	sheet := circleSheet(t, doc, 10)
	ctx, cancel := context.WithCancel(t.Context())
	cancel()

	_, err := sheet.PatchContext(ctx, decad.Edges(decad.Free()).Exactly(1))
	require.ErrorIs(t, err, context.Canceled)
	require.Len(t, doc.Bodies(), 1)

	var nilContext context.Context
	_, err = sheet.PatchContext(nilContext, decad.Edges(decad.Free()).Exactly(1))
	require.ErrorIs(t, err, decad.ErrDegenerate)
	require.Len(t, doc.Bodies(), 1)
}
