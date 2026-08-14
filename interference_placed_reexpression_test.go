package decad_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/stretchr/testify/require"
)

// This file pins docs/prism-boolean-design.md §3.4's split-boundary reroute
// as it reaches Verify's interference reading (§4.5, shared with the
// crossing sub-case's own selection, prism_boolean_crossing.go's
// resolvePrismCrossingCells): a coplanar pair whose 2D outlines genuinely
// cross is measured only while operand B's re-expression into operand A's
// frame stays the identity. Body.Placed's rotation and/or translation makes
// that map nonidentity the moment one operand moved and the other did not,
// so the pair silently reroutes to the mesh path, whose own coplanar
// contact refusal (docs/interference-design.md §5.2) leaves it undecided —
// even though the same footprint overlap, drawn already seated in the
// sketch instead of built elsewhere and placed, resolves analytically.
//
// crossingBoxPair builds box A [0,10]x[0,10] against box B [5,15]x[5,15],
// both 5 mm tall: a single 5x5 mm crossing overlap over the shared 5 mm
// height, 125 mm^3. seated draws B directly at its final coordinates; placed
// builds B at the origin footprint and moves it into place with Body.Placed,
// reaching the identical footprint through a nonidentity re-expression.

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
}
