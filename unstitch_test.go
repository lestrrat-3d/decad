package decad_test

import (
	"context"
	"errors"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is docs/surface-design.md's T3/T5-shaped public tests for
// Unstitch, the reach-limit round trip §6.5 names, Table R's R11/R12/R14
// rows, and Unstitch's own multi-produce atomicity. stitchBoxSheets
// (stitch_test.go), annularSketch/uAxis (revolve_test.go),
// quarterTurn (surface_revolve_test.go), and boxBody/diskBody/translated
// (clearance_test.go/boolean_test.go) are this test package's own shared
// fixtures, reused rather than duplicated. R13 (ErrForeignBody) has no
// Unstitch-reachable fixture: unlike Stitch, which compares several operands
// against each other's document, Unstitch takes one receiver and no second
// document to be foreign to, so it is not tested here.

// unstitchBox builds docs/surface-design.md's T3 box (stitchBoxSheets'
// worked example) and stitches it, returning the live BodySolid ready to
// hand to Unstitch.
func unstitchBox(t *testing.T, doc *decad.Document) *decad.Body {
	t.Helper()
	walls, bottom, top := stitchBoxSheets(t, doc, 10)
	box, err := decad.Stitch(walls, bottom, top)
	require.NoError(t, err)
	return box
}

// TestUnstitchBoxYieldsSixFreeSheets is docs/surface-design.md's T3-shaped
// coverage of Unstitch: the design's box (100x60x10, 15200 mm^2, 60000
// mm^3), unstitched into one single-face sheet per face, in Faces() order.
func TestUnstitchBoxYieldsSixFreeSheets(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	box := unstitchBox(t, doc)
	wantFaces := box.Faces()
	require.Len(t, wantFaces, 6)

	results, err := box.Unstitch()
	require.NoError(t, err)
	require.Len(t, results, 6)

	var areaSum float64
	for i, r := range results {
		require.Equal(t, decad.BodySheet, r.Kind())
		require.False(t, r.IsSolid())
		require.Len(t, r.Faces(), 1)

		rf := r.Faces()[0]
		require.NotSame(t, wantFaces[i], rf,
			"Unstitch never reuses the receiver's own Face pointer")
		wantArea, err := wantFaces[i].Area()
		require.NoError(t, err)
		gotArea, err := rf.Area()
		require.NoError(t, err)
		require.Equal(t, wantArea, gotArea, "result i carries Faces()[i]'s own reading")

		edges := r.Edges()
		require.NotEmpty(t, edges)
		for _, e := range edges {
			require.True(t, e.IsFree(), "every edge of an unstitched single-face sheet is free")
			require.Len(t, e.Faces(), 1)
		}
		free, err := decad.Edges(decad.Free()).SelectEdges(r)
		require.NoError(t, err)
		require.Len(t, free, len(edges))

		_, err = r.Volume()
		require.ErrorIs(t, err, decad.ErrNotSolid)

		areaSum += gotArea.Value.Base()
	}
	require.Equal(t, 15200.0, areaSum, "the six areas sum to exactly 15200 mm^2")

	// The receiver retires; the document holds exactly the six results, in
	// Faces() order.
	bodies := doc.Bodies()
	require.Len(t, bodies, 6)
	for i, r := range results {
		require.Same(t, r, bodies[i])
	}
	require.NotContains(t, bodies, box)
}

// TestUnstitchRestitchRoundTripMatchesOriginal is docs/surface-design.md's
// T5: the T3 box, unstitched then re-stitched, reproduces it bit for bit —
// every edge Unstitch freed was welded from a proven-coincident pair, so
// Table J re-admits every one of them.
func TestUnstitchRestitchRoundTripMatchesOriginal(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	box := unstitchBox(t, doc)
	wantVol, err := box.Volume()
	require.NoError(t, err)
	wantArea, err := box.Area()
	require.NoError(t, err)

	sheets, err := box.Unstitch()
	require.NoError(t, err)
	require.Len(t, sheets, 6)

	restitched, err := decad.Stitch(sheets...)
	require.NoError(t, err)

	require.Equal(t, decad.BodySolid, restitched.Kind())
	require.True(t, restitched.IsSolid())

	gotVol, err := restitched.Volume()
	require.NoError(t, err)
	require.Equal(t, wantVol, gotVol, "volume matches the original bit for bit")
	require.Equal(t, decad.Exact, gotVol.Exactness)
	require.Zero(t, gotVol.Bound.Base())

	gotArea, err := restitched.Area()
	require.NoError(t, err)
	require.Equal(t, wantArea, gotArea)
	require.Equal(t, decad.Exact, gotArea.Exactness)

	_, err = decad.Edges(decad.Free()).SelectEdges(restitched)
	require.ErrorIs(t, err, decad.ErrNoMatch)

	require.Len(t, doc.Bodies(), 1)
	require.Same(t, restitched, doc.Bodies()[0])
}

// TestUnstitchRestitchRoundTripClosesABoundedBox is docs/surface-design.md
// §15's T42's own copy-path leg: the CURVE half of the shared-denotation
// certificate must survive Unstitch's own copy (copyFaceUnderContext) for a
// BOUNDED body, not only an exact one — TestUnstitchRestitchRoundTripMatchesOriginal
// already covers the exact case. A Symmetric surface-extruded wall on an
// off-axis, non-origin plane (offAxisPlateSketch) has both rims bounded;
// Body.Patch caps both, and a single-operand Stitch closes the result to a
// BodySolid (docs/surface-design.md §15's T42, stitch_test.go). Unstitching
// that solid frees every edge, INCLUDING the bounded rim edges Table J's J5
// could never admit on bit-identity alone — the round trip closes again only
// because each unstitched copy still carries the ORIGINAL curveID Body.Patch's
// own rim edges minted, so the second Stitch's certificate route re-admits
// exactly the same pairs the first Body.Patch call already proved coincident.
func TestUnstitchRestitchRoundTripClosesABoundedBox(t *testing.T) {
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
	wantVol, err := solid.Volume()
	require.NoError(t, err)

	sheets, err := solid.Unstitch()
	require.NoError(t, err)
	require.Len(t, sheets, 6)

	restitched, err := decad.Stitch(sheets...)
	require.NoError(t, err)

	require.Equal(t, decad.BodySolid, restitched.Kind(), "the round trip closes again rather than falling back to a sheet")
	require.True(t, restitched.IsSolid())

	gotVol, err := restitched.Volume()
	require.NoError(t, err)
	require.Equal(t, wantVol, gotVol, "volume matches the pre-unstitch solid's own reading bit for bit")

	_, err = decad.Edges(decad.Free()).SelectEdges(restitched)
	require.ErrorIs(t, err, decad.ErrNoMatch)
}

// TestUnstitchRevolveRoundTripStaysASheet is the reach-limit round trip
// docs/surface-design.md §6.5 names: unstitching a surface-result Revolve
// body and re-stitching it stays a SHEET, because the profile's own seam —
// its boundary copy at phi0 and phi1, a partial revolve's own free rim —
// never had a partner to weld against, before or after the round trip. The
// CURVE half of the shared-denotation certificate (denotation.go) now
// closes the revolve's own internal junction welds it could not before
// (a Circle3/Arc3 carries no bound field of its own, so Table J's J5 could
// never decide them on bit-identity alone), reproducing the never-unstitched
// sheet's own free-edge shape exactly rather than leaving every edge free.
func TestUnstitchRevolveRoundTripStaysASheet(t *testing.T) {
	t.Parallel()
	s, p := annularSketch(t)
	doc := decad.New()
	sheet, err := doc.Revolve(s, p, uAxis, quarterTurn, decad.WithSurfaceResult())
	require.NoError(t, err)
	require.Equal(t, decad.BodySheet, sheet.Kind())

	// Measure the original's own free-edge shape BEFORE unstitching it (which
	// retires sheet), so the round trip below is checked against what the
	// original actually publishes rather than a number typed into the test.
	// The 8 is still asserted directly here, so a silent change to the
	// original's own shape still surfaces — it is the round-trip comparison
	// itself that must read this measurement back, not a literal.
	wantFree, err := decad.Edges(decad.Free()).SelectEdges(sheet)
	require.NoError(t, err)
	require.Len(t, wantFree, 8, "the profile's own seam at phi0 and phi1: a partial revolve's own free rim")
	wantEdges := len(sheet.Edges())

	pieces, err := sheet.Unstitch()
	require.NoError(t, err)
	require.Len(t, pieces, 4)
	for _, piece := range pieces {
		require.Equal(t, decad.BodySheet, piece.Kind())
	}

	restitched, err := decad.Stitch(pieces...)
	require.NoError(t, err)

	require.Equal(t, decad.BodySheet, restitched.Kind())
	require.False(t, restitched.IsSolid())

	// The certificate now closes the 4 internal junction welds (each shared,
	// by construction, between two adjacent side faces before Unstitch ever
	// split them), leaving free exactly the seam edges — the profile's own
	// boundary at phi0 and phi1, which never had a partner to weld against,
	// before or after this round trip. That is the SAME free-edge shape the
	// original, never-unstitched sheet itself carried (wantFree/wantEdges,
	// measured above): the certificate loses no information the original
	// build already published, it only lets the round trip REPRODUCE it.
	free, err := decad.Edges(decad.Free()).Exactly(len(wantFree)).SelectEdges(restitched)
	require.NoError(t, err)
	require.Len(t, free, len(wantFree))
	require.Len(t, restitched.Edges(), wantEdges)
}

// TestUnstitchTableR covers Unstitch's own Table R rows: R11 (a Faceted
// body), R12 (a nil receiver), and R14 (a retired receiver).
func TestUnstitchTableR(t *testing.T) {
	t.Parallel()

	t.Run("R11 Faceted body", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		plate := boxBody(t, doc, 0, 0, 20, 20, 8)
		tool := translated(t, diskBody(t, doc, 14, 6, 2), 0, 0, -6)
		cut, err := decad.Cut(plate, tool)
		require.NoError(t, err)
		require.Equal(t, decad.KindFaceted, cut.Faces()[0].Surface().Kind())

		before := doc.Bodies()
		_, err = cut.Unstitch()
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.Equal(t, before, doc.Bodies())
	})

	t.Run("R12 nil receiver", func(t *testing.T) {
		t.Parallel()
		var nilBody *decad.Body
		_, err := nilBody.Unstitch()
		require.ErrorIs(t, err, decad.ErrDegenerate)
	})

	t.Run("R14 retired receiver", func(t *testing.T) {
		t.Parallel()
		doc := decad.New()
		box := unstitchBox(t, doc)
		_, err := box.Unstitch()
		require.NoError(t, err)

		_, err = box.Unstitch()
		require.ErrorIs(t, err, decad.ErrRetiredBody)
	})
}

// countingCtx counts every ctx.Err() poll and, once failAt is positive and
// the count reaches it, reports err instead of nil. It is
// TestUnstitchAtomicCommitLeavesDocumentUnchanged's own tool for forcing
// UnstitchContext to fail strictly AFTER real per-face work has already run
// — the shape a partial-commit bug needs to be observable in — without
// depending on any particular internal call count.
type countingCtx struct {
	context.Context //nolint:containedctx // deterministic cancellation wrapper used only within one test call.
	calls           *int
	failAt          int
	err             error
}

func (c countingCtx) Err() error {
	*c.calls++
	if c.failAt > 0 && *c.calls >= c.failAt {
		return c.err
	}
	return nil
}

// TestUnstitchAtomicCommitLeavesDocumentUnchanged proves
// Document.commitMany's atomicity: a context canceled partway through
// UnstitchContext's per-face loop — after at least one face's own build has
// already run to completion — leaves the document exactly as it was before
// the call, never a state with the receiver retired and only some of its
// replacements registered.
func TestUnstitchAtomicCommitLeavesDocumentUnchanged(t *testing.T) {
	t.Parallel()

	// First, measure how many times a full, successful Unstitch polls
	// ctx.Err() over the box fixture, on a disposable document.
	measureDoc := decad.New()
	measureBox := unstitchBox(t, measureDoc)
	var total int
	_, err := measureBox.UnstitchContext(countingCtx{Context: t.Context(), calls: &total})
	require.NoError(t, err)
	require.Positive(t, total)

	// Now force failure roughly halfway through that same call shape, on a
	// fresh document — late enough that real work already happened, early
	// enough that later faces never get the chance to build or commit.
	doc := decad.New()
	box := unstitchBox(t, doc)
	forceErr := errors.New("forced mid-unstitch failure")
	var calls int
	_, err = box.UnstitchContext(countingCtx{Context: t.Context(), calls: &calls, failAt: total / 2, err: forceErr})
	require.ErrorIs(t, err, forceErr)

	require.Len(t, doc.Bodies(), 1)
	require.Same(t, box, doc.Bodies()[0])
	require.True(t, doc.Bodies()[0].IsSolid())
}

// TestUnstitchBoxFacesReportOwnTightBounds is docs/surface-design.md's T3 box
// again, this time asserting each of the six straight-edged, planar result
// sheets reports its OWN tight box rather than the whole receiver's: a wall
// is a flat slab across the two axes its rectangle spans and zero-thick
// across the third, the bottom sheet is z=0's rectangle, the top is z=10's —
// in [Body.Faces] order, matching stitchBoxSheets' own wall/bottom/top
// construction (four wall faces, then the bottom patch, then the top patch).
// Every box is Exact with a zero bound: an unplaced, all-straight-edged
// planar face's held vertices are exact, and the extreme along any axis is
// always attained at one of them.
func TestUnstitchBoxFacesReportOwnTightBounds(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	box := unstitchBox(t, doc)

	results, err := box.Unstitch()
	require.NoError(t, err)
	require.Len(t, results, 6)

	wantBoxes := []struct {
		min, max r3.Vec
	}{
		{r3.NewVec(0, 0, 0), r3.NewVec(100, 0, 10)},    // wall y=0
		{r3.NewVec(100, 0, 0), r3.NewVec(100, 60, 10)}, // wall x=100
		{r3.NewVec(0, 60, 0), r3.NewVec(100, 60, 10)},  // wall y=60
		{r3.NewVec(0, 0, 0), r3.NewVec(0, 60, 10)},     // wall x=0
		{r3.NewVec(0, 0, 0), r3.NewVec(100, 60, 0)},    // bottom, z=0
		{r3.NewVec(0, 0, 10), r3.NewVec(100, 60, 10)},  // top, z=10
	}
	require.Len(t, wantBoxes, len(results))

	for i, r := range results {
		got, err := r.Bounds()
		require.NoError(t, err)
		require.Equal(t, wantBoxes[i].min, got.Min, "result %d min", i)
		require.Equal(t, wantBoxes[i].max, got.Max, "result %d max", i)
		require.Equal(t, decad.Exact, got.Exactness, "result %d exactness", i)
		require.Zero(t, got.Bound.Base(), "result %d bound", i)
	}
}

// TestUnstitchBoxFacesBoundsUnionSpansOriginal proves the six per-face tight
// boxes TestUnstitchBoxFacesReportOwnTightBounds asserts individually lose
// nothing together: their componentwise union reproduces the original box's
// own Bounds, read before Unstitch retires the receiver.
func TestUnstitchBoxFacesBoundsUnionSpansOriginal(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	box := unstitchBox(t, doc)
	want, err := box.Bounds()
	require.NoError(t, err)

	results, err := box.Unstitch()
	require.NoError(t, err)
	require.Len(t, results, 6)

	have := false
	var lo, hi r3.Vec
	for _, r := range results {
		b, err := r.Bounds()
		require.NoError(t, err)
		if !have {
			lo, hi = b.Min, b.Max
			have = true
			continue
		}
		lo = r3.NewVec(math.Min(lo.X, b.Min.X), math.Min(lo.Y, b.Min.Y), math.Min(lo.Z, b.Min.Z))
		hi = r3.NewVec(math.Max(hi.X, b.Max.X), math.Max(hi.Y, b.Max.Y), math.Max(hi.Z, b.Max.Z))
	}
	require.Equal(t, want.Min, lo, "the union of the six mins reproduces the original min")
	require.Equal(t, want.Max, hi, "the union of the six maxes reproduces the original max")
}

// TestUnstitchCylinderWallBoundsAreSoundButWholeReceiver covers a CURVED
// face: unstitching a disc (a cylindrical wall between two circular planar
// caps) must NOT take the wall's own two seam vertices as its box — a
// cylinder bulges past them — and must not take a circular cap's own single
// rim vertex as ITS box either, since a planar surface with a curved edge
// bulges past its vertices exactly as a curved surface does. Every one of
// the three results instead reports the sound whole-receiver box: it
// contains both the seam vertices AND the wall's true radius extent, and it
// is Exact with a zero bound because the unplaced receiver's own box already
// was — not because any of these three boxes is proven the tightest one its
// own face could in principle earn (docs/surface-design.md §6.5).
func TestUnstitchCylinderWallBoundsAreSoundButWholeReceiver(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	disk := diskBody(t, doc, 0, 0, 10)
	want, err := disk.Bounds()
	require.NoError(t, err)
	require.Equal(t, r3.NewVec(-10, -10, 0), want.Min)
	require.Equal(t, r3.NewVec(10, 10, 20), want.Max)
	require.Equal(t, decad.Exact, want.Exactness)
	require.Zero(t, want.Bound.Base())

	results, err := disk.Unstitch()
	require.NoError(t, err)
	require.Len(t, results, 3)

	for i, r := range results {
		got, err := r.Bounds()
		require.NoError(t, err)
		require.Equal(t, want, got, "result %d reuses the receiver's whole sound box", i)

		// Sound: every held vertex of this face lies within the box.
		for _, v := range r.Vertices() {
			p := v.Position().Value
			require.GreaterOrEqual(t, p.X, got.Min.X)
			require.GreaterOrEqual(t, p.Y, got.Min.Y)
			require.GreaterOrEqual(t, p.Z, got.Min.Z)
			require.LessOrEqual(t, p.X, got.Max.X)
			require.LessOrEqual(t, p.Y, got.Max.Y)
			require.LessOrEqual(t, p.Z, got.Max.Z)
		}

		// Sound over the wall's true radius extent too: the box reaches
		// all the way to the cylinder's radius on every side, which no
		// vertex-only reading of this face could ever prove (the wall
		// holds only its two seam vertices at x=10,y=0; a cap holds only
		// its one rim vertex).
		require.Equal(t, -10.0, got.Min.X, "result %d", i)
		require.Equal(t, -10.0, got.Min.Y, "result %d", i)
		require.Equal(t, 10.0, got.Max.X, "result %d", i)
		require.Equal(t, 10.0, got.Max.Y, "result %d", i)
	}
}

// TestUnstitchedCapToFaceStopMatchesUnUnstitchedCap is
// docs/surface-design.md's T81: a ToFace stop resolved against an
// unstitched cap must publish the identical Exactness and Bound as the same
// stop resolved against the un-unstitched plate — Unstitch changes
// representation, never the proof a body-relative stop reads off the cap it
// selects. The plate's own extrude depth is stated in inches, a genuine
// unit-conversion rounding (document.go), so the cap's own axialDelta is
// nonzero and the stop reads Approximate on both sides unless the copy
// drops it.
func TestUnstitchedCapToFaceStopMatchesUnUnstitchedCap(t *testing.T) {
	t.Parallel()
	s, plateProf, pinProf := plateAndPin(t)

	doc1 := decad.New()
	plate, err := doc1.Extrude(s, plateProf, decad.Distance{D: units.Inches(10), Dir: decad.Along})
	require.NoError(t, err)
	pin, err := doc1.Extrude(s, pinProf, decad.ToFace{Body: plate, Face: capEndFace(plate)})
	require.NoError(t, err)
	want, err := pin.Bounds()
	require.NoError(t, err)
	require.Equal(t, decad.Approximate, want.Exactness,
		"the inch-stated depth's own unit-conversion rounding must actually reach the stop")

	doc2 := decad.New()
	plate2, err := doc2.Extrude(s, plateProf, decad.Distance{D: units.Inches(10), Dir: decad.Along})
	require.NoError(t, err)
	capSelector := capEndFace(plate2)
	sheets, err := plate2.Unstitch()
	require.NoError(t, err)
	var capSheet *decad.Body
	for _, sh := range sheets {
		if faces, err := capSelector.SelectFaces(sh); err == nil && len(faces) == 1 {
			capSheet = sh
			break
		}
	}
	require.NotNil(t, capSheet, "one of the six unstitched sheets must be the cap")

	pin2, err := doc2.Extrude(s, pinProf, decad.ToFace{Body: capSheet, Face: decad.Faces()})
	require.NoError(t, err)
	got, err := pin2.Bounds()
	require.NoError(t, err)

	require.Equal(t, want.Exactness, got.Exactness,
		"an unstitched cap's own stop must read exactly as approximate as the un-unstitched plate's")
	require.Equal(t, want.Bound, got.Bound,
		"an unstitched cap's own stop must carry exactly the un-unstitched plate's own axial bound")
}
