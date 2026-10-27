package apitest_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// This file is agent-docs/orthogonal-boolean-design.md's public-surface test for Verify's overlap volume of two faceted
// solids every face of which squares to a world axis, including a pair that shares a face plane.

func TestOrthogonalInterferenceReadsTheOverlapVolumeExactly(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	_ = blockIn(t, doc, r3.NewVec(0, 0, 0), r3.NewVec(10, 10, 10))
	_ = blockIn(t, doc, r3.NewVec(8, 0, 0), r3.NewVec(18, 10, 10)) // shares the y and z face planes
	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.Len(t, report.Interferences, 1)
	require.InDelta(t, 200, report.Interferences[0].Volume.Value.Base(), 1e-9, "a 2 x 10 x 10 slab")
	require.Less(t, report.Interferences[0].Volume.Bound.Base(), 1e-9)
	_, ok := wallDiagnostic(report, decad.DiagInterference)
	require.True(t, ok, "%+v", report.Diagnostics)
	_, refused := wallDiagnostic(report, decad.DiagUnsupportedPairContact)
	require.False(t, refused, "a shared face plane is no longer refused")

	// A pair that overlaps in an inner corner only, across no shared plane.
	other := decad.New()
	_ = blockIn(t, other, r3.NewVec(0, 0, 0), r3.NewVec(10, 10, 10))
	_ = blockIn(t, other, r3.NewVec(6, 7, 3), r3.NewVec(16, 17, 13))
	report, err = other.Verify(t.Context())
	require.NoError(t, err)
	require.Len(t, report.Interferences, 1)
	require.InDelta(t, 4*3*7, report.Interferences[0].Volume.Value.Base(), 1e-9)
}
