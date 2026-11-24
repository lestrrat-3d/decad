package apitest_test

import (
	"context"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/stretchr/testify/require"
)

// TestVerifySkippedPairReportsNothingForThePair holds WithSkippedPair to
// verification §1.3 over three overlapping boxes: the skipped pair, named in
// reverse order and twice, has no Interference row and no pair diagnostic,
// Report.SkippedPairs names it once with A the earlier body, the other two
// pairs keep their rows, and every body keeps its own report. Skipping every
// overlapping pair leaves a Sound report.
func TestVerifySkippedPairReportsNothingForThePair(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	a := boxBody(t, doc, 0, 0, 10, 10, 10)
	b := translated(t, boxBody(t, doc, 0, 0, 10, 10, 10), 5, 5, 5)
	c := translated(t, boxBody(t, doc, 0, 0, 10, 10, 10), -3, -3, 2)

	full, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.Len(t, full.Interferences, 3, "premise: every pair overlaps")
	require.Nil(t, full.SkippedPairs, "a call that skips nothing lists no skipped pairs")

	report, err := doc.Verify(t.Context(), decad.WithSkippedPair(b, a), decad.WithSkippedPair(a, b))
	require.NoError(t, err)
	require.Equal(t, []decad.DiagnosticPair{{A: a, B: b}}, report.SkippedPairs)
	require.Len(t, report.Interferences, 2)
	for _, row := range report.Interferences {
		require.False(t, row.A == a && row.B == b, "the skipped pair has no Interference row")
	}
	for _, d := range report.Diagnostics {
		if d.Pair != nil {
			require.False(t, d.Pair.A == a && d.Pair.B == b, "the skipped pair has no diagnostic: %s", d.Code)
		}
	}
	require.Len(t, report.Bodies, 3, "each body keeps its own checks")
	require.Equal(t, decad.Interfering, report.Status)

	clean, err := doc.Verify(t.Context(),
		decad.WithSkippedPair(a, b), decad.WithSkippedPair(a, c), decad.WithSkippedPair(c, b))
	require.NoError(t, err)
	require.Equal(t, decad.Sound, clean.Status)
	require.Empty(t, clean.Interferences)
	require.Equal(t, []decad.DiagnosticPair{{A: a, B: b}, {A: a, B: c}, {A: b, B: c}}, clean.SkippedPairs)
}

// TestVerifySkippedPairOmitsItsClearance holds a skipped box-apart pair under
// WithClearances to the same rule: no Clearance row and no undecided-clearance
// diagnostic, while the pair that is not skipped keeps its measured row.
func TestVerifySkippedPairOmitsItsClearance(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	a := boxBody(t, doc, 0, 0, 10, 10, 10)
	b := boxBody(t, doc, 20, 0, 30, 10, 10)
	c := boxBody(t, doc, 0, 30, 10, 40, 10)

	full, err := doc.Verify(t.Context(), decad.WithClearances())
	require.NoError(t, err)
	require.Len(t, full.Clearances, 3, "premise: every pair has a measured gap")

	report, err := doc.Verify(t.Context(), decad.WithClearances(),
		decad.WithSkippedPair(a, b), decad.WithSkippedPair(b, c))
	require.NoError(t, err)
	require.Len(t, report.Clearances, 1)
	require.Same(t, a, report.Clearances[0].A)
	require.Same(t, c, report.Clearances[0].B)
	require.Equal(t, full.Clearances[1].Gap, report.Clearances[0].Gap)
	require.Empty(t, report.Diagnostics)
	require.Equal(t, decad.Sound, report.Status)
}

// TestVerifySkippedPairRejectsInvalidPairs holds the option gate: a nil body,
// one body named twice, a body of another document and a retired body are
// each refused before any work, even under a canceled context.
func TestVerifySkippedPairRejectsInvalidPairs(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	a := boxBody(t, doc, 0, 0, 10, 10, 10)
	retired := boxBody(t, doc, 20, 0, 30, 10, 10)
	translated(t, retired, 0, 0, 1)
	foreign := boxBody(t, decad.New(), 0, 0, 10, 10, 10)

	canceled, cancel := context.WithCancel(t.Context())
	cancel()
	for _, tc := range []struct {
		name string
		a, b *decad.Body
		want error
	}{
		{name: "nil", a: a, b: nil, want: decad.ErrDegenerate},
		{name: "same body", a: a, b: a, want: decad.ErrDegenerate},
		{name: "foreign", a: foreign, b: a, want: decad.ErrForeignBody},
		{name: "retired", a: a, b: retired, want: decad.ErrRetiredBody},
	} {
		report, err := doc.Verify(canceled, decad.WithSkippedPair(tc.a, tc.b))
		require.ErrorIs(t, err, tc.want, tc.name)
		require.Nil(t, report, tc.name)
	}
}
