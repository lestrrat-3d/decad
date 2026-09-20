package decad_test

import (
	"context"
	"errors"
	"testing"

	"github.com/lestrrat-3d/decad"
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

// TestUnstitchRevolveRoundTripStaysASheet is the reach-limit round trip
// docs/surface-design.md §6.5 names explicitly: unstitching a
// surface-result Revolve body and re-stitching it stays a SHEET, because
// the revolve's own rim and junction curves (Circle3/Arc3) never satisfy
// Table J's J5 — the same reach limit R8 already states for Stitch, not a
// separate one.
func TestUnstitchRevolveRoundTripStaysASheet(t *testing.T) {
	t.Parallel()
	s, p := annularSketch(t)
	doc := decad.New()
	sheet, err := doc.Revolve(s, p, uAxis, quarterTurn, decad.WithSurfaceResult())
	require.NoError(t, err)
	require.Equal(t, decad.BodySheet, sheet.Kind())

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

	free, err := decad.Edges(decad.Free()).SelectEdges(restitched)
	require.NoError(t, err)
	require.NotEmpty(t, free)
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
