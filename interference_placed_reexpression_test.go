package decad_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file pins docs/prism-boolean-design.md §3.4's split-boundary reroute
// as it reaches Verify's interference reading (§4.5, shared with the
// crossing sub-case's own selection, prism_boolean_crossing.go's
// resolvePrismCrossingCells): a coplanar pair whose 2D outlines genuinely
// cross is measured only while none of the reroute's three independent
// causes fires — a section displacement on either operand, a walk charge on
// either operand, or a nonidentity re-expression between the two frames.
// Each cause is pinned on its own here, because each alone is enough to
// reroute the pair to the mesh path, whose own coplanar contact refusal
// (docs/interference-design.md §5.2) then leaves it undecided.
//
// crossingBoxPair builds box A [0,10]x[0,10] against box B [5,15]x[5,15],
// both 5 mm tall: a single 5x5 mm crossing overlap over the shared 5 mm
// height, 125 mm^3. seated draws B directly at its final coordinates; placed
// builds B at the origin footprint and moves it into place with Body.Placed,
// reaching the identical footprint through a nonidentity re-expression;
// placedBackToIdentity moves B twice, by a motion and its inverse, so the
// ACCUMULATED placement the re-expression actually reads is the identity
// again even though the last Placed call received a nonidentity transform.
//
// splitCellBody is the walk-charge half: the SAME [1,5]x[0,10] footprint a
// plain rectangle draws, but cut out of the larger rectangle [1,11]x[0,10] by
// a line through (5,-2)-(5,14), so its bottom and top walls are Partial
// fragments whose narrowed range makes buildPrismScene compute their shared
// corner instead of reading it off the record (§7's walk charge). Nothing is
// placed and nothing is displaced: the pair is fully seated and reroutes on
// the walk charge alone.

func crossingBoxPairSeated(t *testing.T, doc *decad.Document) (a, b *decad.Body) {
	t.Helper()
	a = boxBody(t, doc, 0, 0, 10, 10, 5)
	b = boxBody(t, doc, 5, 5, 15, 15, 5)
	return a, b
}

func crossingBoxPairPlaced(t *testing.T, doc *decad.Document) (a, b *decad.Body) {
	t.Helper()
	a = boxBody(t, doc, 0, 0, 10, 10, 5)
	origin := boxBody(t, doc, 0, 0, 10, 10, 5)
	b = translated(t, origin, 5, 5, 0)
	return a, b
}

func crossingBoxPairPlacedBackToIdentity(t *testing.T, doc *decad.Document) (a, b *decad.Body) {
	t.Helper()
	a = boxBody(t, doc, 0, 0, 10, 10, 5)
	seated := boxBody(t, doc, 5, 5, 15, 15, 5)
	b = translated(t, translated(t, seated, 3, 7, 2), -3, -7, -2)
	return a, b
}

// splitCellBody extrudes the LEFT cell of the rectangle [1,11]x[0,10] split by
// the fixed line (5,-2)-(5,14), h mm along its own sketch normal. The cell's
// footprint is exactly [1,5]x[0,10], the same rectangle boxBody draws
// directly, so a pair built from this body differs from the plain-rectangle
// pair in provenance alone.
func splitCellBody(t *testing.T, doc *decad.Document, h float64) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	r := s.CreateRectangle(1, 0, 11, 10)
	s.Fix(r.A)
	lo := s.CreatePoint(5, -2)
	hi := s.CreatePoint(5, 14)
	s.Fix(lo)
	s.Fix(hi)
	s.CreateLine(lo, hi)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)

	var left *sketch.Profile
	for _, p := range s.Profiles() {
		minU, maxU := math.Inf(1), math.Inf(-1)
		for _, e := range p.Outer {
			for _, pt := range e.Polyline {
				minU = math.Min(minU, pt[0])
				maxU = math.Max(maxU, pt[0])
			}
		}
		if minU == 1 && maxU <= 5.0000001 {
			left = p
		}
	}
	require.NotNil(t, left, "the split rectangle's left cell must exist")

	body, err := doc.Extrude(s, left, decad.Distance{D: units.Millimeters(h), Dir: decad.Along})
	require.NoError(t, err)
	return body
}

func TestVerifyCrossingPairSeatedResolvesPlacedStaysUndecided(t *testing.T) {
	t.Run("seated section resolves through the analytic reading", func(t *testing.T) {
		doc := decad.New()
		a, b := crossingBoxPairSeated(t, doc)
		before := snapshotDocument(t, doc)

		report, err := doc.Verify(t.Context())
		require.NoError(t, err)
		require.Equal(t, decad.Interfering, report.Status)
		require.Len(t, report.Interferences, 1)

		row := report.Interferences[0]
		require.Same(t, a, row.A)
		require.Same(t, b, row.B)
		require.InDelta(t, 125.0, row.Volume.Value.Base(), 1e-9)
		require.Greater(t, row.Volume.Value.Base()-row.Volume.Bound.Base(), 0.0,
			"§6's positive-volume gate must hold")

		_, contact := findDiagnostic(report.Diagnostics, decad.DiagUnsupportedPairContact)
		require.False(t, contact, "a seated pair must never fall back to the mesh path's coplanar refusal")
		requireDocumentUnchanged(t, doc, before)
	})

	t.Run("placed section stays undecided", func(t *testing.T) {
		doc := decad.New()
		crossingBoxPairPlaced(t, doc)
		before := snapshotDocument(t, doc)

		report, err := doc.Verify(t.Context())
		require.NoError(t, err)
		require.Empty(t, report.Interferences,
			"a nonidentity re-expression reroutes the pair before either analytic path can measure it")
		require.Equal(t, decad.Suspect, report.Status)

		d, ok := findDiagnostic(report.Diagnostics, decad.DiagUnsupportedPairContact)
		require.True(t, ok, "the rerouted pair reaches the mesh path's own staged coplanar contact")
		require.Equal(t, decad.Suspect, d.Status)
		require.NotNil(t, d.Pair)

		_, broad := findDiagnostic(report.Diagnostics, decad.DiagUnsupportedPair)
		require.True(t, broad, "the placed pair keeps the broad compatibility code the seated pair clears")
		requireDocumentUnchanged(t, doc, before)
	})

	// §3.4 reads the operands' ACCUMULATED placement, never the motion one
	// Placed call received: newPrismReexpression compares pa.frame/pa.xform
	// against pb.frame/pb.xform, and PlacedContext composes onto the transform
	// the receiver already carries. So a body moved by a motion and then by its
	// inverse is back at the partner's own placement and the pair is measured
	// again, even though the last Placed call took a nonidentity transform.
	t.Run("a placement composed back to the identity is measured again", func(t *testing.T) {
		doc := decad.New()
		a, b := crossingBoxPairPlacedBackToIdentity(t, doc)
		before := snapshotDocument(t, doc)

		report, err := doc.Verify(t.Context())
		require.NoError(t, err)
		require.Equal(t, decad.Interfering, report.Status)
		require.Len(t, report.Interferences, 1,
			"a nonidentity motion composing back to the partner's placement leaves the analytic reading available")

		row := report.Interferences[0]
		require.Same(t, a, row.A)
		require.Same(t, b, row.B)
		require.InDelta(t, 125.0, row.Volume.Value.Base(), 1e-9)

		_, contact := findDiagnostic(report.Diagnostics, decad.DiagUnsupportedPairContact)
		require.False(t, contact, "the pair must never reach the mesh path's coplanar refusal")
		requireDocumentUnchanged(t, doc, before)
	})
}

// TestVerifySeatedWalkChargedPairStaysUndecided pins §3.4's walk-charge cause
// on its own: both operands are drawn seated in their own sketch, so the
// re-expression is the identity and no placement is ever applied, yet the pair
// still reroutes because operand A's section was trimmed out of a larger sketch
// arrangement and its consumed walls carry a walk charge (§7's delta_walk).
// The control arm draws the IDENTICAL footprint as a plain rectangle — no
// trim, no walk charge — and the same overlap is measured, so the difference
// between the two arms is the walk charge and nothing else. That this exact
// fixture yields a positive delta_walk under an identity re-expression is
// pinned white-box by TestPrismUnionTrimmedSourceSplitBoundaryFallsBack
// (prism_boolean_internal_test.go), over the same split-cell section.
func TestVerifySeatedWalkChargedPairStaysUndecided(t *testing.T) {
	const h = 5.0
	// [1,5]x[0,10] against [4,6]x[3,7]: one crossing region [4,5]x[3,7],
	// 1 x 4 mm^2 over the shared 5 mm height.
	const wantVolume = 20.0

	t.Run("an untrimmed seated pair of the same footprint is measured", func(t *testing.T) {
		doc := decad.New()
		a := boxBody(t, doc, 1, 0, 5, 10, h)
		b := boxBody(t, doc, 4, 3, 6, 7, h)
		before := snapshotDocument(t, doc)

		report, err := doc.Verify(t.Context())
		require.NoError(t, err)
		require.Equal(t, decad.Interfering, report.Status)
		require.Len(t, report.Interferences, 1)

		row := report.Interferences[0]
		require.Same(t, a, row.A)
		require.Same(t, b, row.B)
		require.InDelta(t, wantVolume, row.Volume.Value.Base(), 1e-9)
		requireDocumentUnchanged(t, doc, before)
	})

	t.Run("the walk-charged seated pair stays undecided", func(t *testing.T) {
		doc := decad.New()
		splitCellBody(t, doc, h)
		boxBody(t, doc, 4, 3, 6, 7, h)
		before := snapshotDocument(t, doc)

		report, err := doc.Verify(t.Context())
		require.NoError(t, err)
		require.Empty(t, report.Interferences,
			"a seated pair still reroutes on either operand's own walk charge")
		require.Equal(t, decad.Suspect, report.Status)

		d, ok := findDiagnostic(report.Diagnostics, decad.DiagUnsupportedPairContact)
		require.True(t, ok, "the rerouted pair reaches the mesh path's own staged coplanar contact")
		require.Equal(t, decad.Suspect, d.Status)
		requireDocumentUnchanged(t, doc, before)
	})
}
