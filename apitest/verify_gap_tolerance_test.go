package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// pocketDocument holds a 10×10×10 block and a 6×6×5 block inside it whose top
// face is flush with the block's. Their true overlap is the inner block, 180 mm³.
func pocketDocument(t *testing.T, opts ...decad.DocumentOption) *decad.Document {
	t.Helper()
	doc := decad.New(opts...)
	boxBody(t, doc, 0, 0, 10, 10, 10)
	translated(t, boxBody(t, doc, 0, 0, 6, 6, 5), 2, 2, 5)
	return doc
}

// TestVerifyGapToleranceDecidesAFlushPair pins Verify reading a pair the exact
// predicates cannot classify. With a gap tolerance the pair reads the overlap it
// has, with a bound that covers the shift, and under ExactOnly it stays staged.
func TestVerifyGapToleranceDecidesAFlushPair(t *testing.T) {
	t.Parallel()

	t.Run("a gap tolerance reads the overlap", func(t *testing.T) {
		t.Parallel()
		report, err := pocketDocument(t).Verify(t.Context())
		require.NoError(t, err)
		require.Equal(t, decad.Interfering, report.Status)
		require.Len(t, report.Interferences, 1)
		row := report.Interferences[0]
		require.Equal(t, decad.Approximate, row.Volume.Exactness)
		require.LessOrEqual(t, math.Abs(row.Volume.Value.Base()-180), row.Volume.Bound.Base(),
			"the stated bound covers the distance to the true overlap")
		_, staged := findDiagnostic(report.Diagnostics, decad.DiagUnsupportedPairContact)
		require.False(t, staged)
	})

	t.Run("ExactOnly leaves the pair staged", func(t *testing.T) {
		t.Parallel()
		report, err := pocketDocument(t, decad.ExactOnly()).Verify(t.Context())
		require.NoError(t, err)
		require.Equal(t, decad.Suspect, report.Status)
		require.Empty(t, report.Interferences)
		_, staged := findDiagnostic(report.Diagnostics, decad.DiagUnsupportedPairContact)
		require.True(t, staged)
	})

	t.Run("an invalid tolerance is the Verify error", func(t *testing.T) {
		t.Parallel()
		doc := decad.New(decad.WithGapTolerance(units.Millimeters(-1)))
		boxBody(t, doc, 0, 0, 10, 10, 10)
		_, err := doc.Verify(t.Context())
		require.ErrorIs(t, err, decad.ErrNegativeMagnitude)
	})
}

// TestVerifyDoesNotChangeOperationWarnings proves that Verify uses its own
// request tolerances without changing warnings recorded by an earlier call.
func TestVerifyDoesNotChangeOperationWarnings(t *testing.T) {
	t.Parallel()

	doc := decad.New()
	a := boxBody(t, doc, 0, 0, 10, 10, 10)
	require.Empty(t, a.OperationDiagnostics(), "exact construction adds no adjustment warning")
	b := translated(t, boxBody(t, doc, 0, 0, 10, 10, 10), 0.5, 0, 10)
	joined, err := decad.Union(t.Context(), a, b)
	require.NoError(t, err)
	before := joined.OperationDiagnostics()
	require.Len(t, before, 1)
	require.Equal(t, decad.OpDiagGapShiftApplied, before[0].Code)

	loose, err := doc.Verify(t.Context(), decad.WithTolerance(units.Scalar(1)))
	require.NoError(t, err)
	require.Equal(t, decad.Sound, loose.Status)
	require.Equal(t, before, joined.OperationDiagnostics())

	strict, err := doc.Verify(t.Context(), decad.WithTolerance(units.Scalar(0.000000001)))
	require.NoError(t, err)
	require.Equal(t, decad.Suspect, strict.Status)
	require.NotEmpty(t, strict.Diagnostics)
	require.Equal(t, decad.DiagMeasurementBeyondTolerance, strict.Diagnostics[0].Code)
	require.Equal(t, before, joined.OperationDiagnostics())
}
