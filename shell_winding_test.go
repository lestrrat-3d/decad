package decad

import (
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// TestShellWindingRefusesAnInsideOutCavity is agent-docs/tessellation-design.md
// §4's per-shell winding audit on the real producer: the closed shell of a
// drilled bar, re-evaluated by Transform from its own record with the
// ClosedShell mark cleared. Without the mark the hole lining's outer wall
// anchors the cavity, so the re-evaluated body reads the ring cavity as a
// second, inside-out lump. Tessellate refuses it at every verification level,
// and Verify reads it as an invalid solid. The marked record re-evaluates to
// one lump around one void and passes both.
func TestShellWindingRefusesAnInsideOutCavity(t *testing.T) {
	t.Parallel()
	doc := New()
	box := internalBoxBodyAtZ(t, doc, -10, -10, 10, 10, 0, 10)
	rod := internalCircleBody(t, doc, 0, 3, -1, Distance{D: units.Millimeters(12), Dir: Along})
	drilled, err := Cut(t.Context(), box, rod)
	require.NoError(t, err)
	closed, err := Shell(t.Context(), drilled, nil, units.Millimeters(1), WithNoOpenings())
	require.NoError(t, err)
	sp, ok := closed.n.Payload.(stackedPrismPayload)
	require.True(t, ok)
	require.True(t, sp.ClosedShell)

	marked, err := Transform(t.Context(), closed, mustTranslation(r3.NewVec(0, 0, 50)))
	require.NoError(t, err)
	require.Len(t, marked.Lumps(), 1)
	_, err = marked.Tessellate(t.Context(), units.Millimeters(0.1), WithVerification(VerifyNone))
	require.NoError(t, err)

	// The cleared mark is the only change: the re-evaluation is the stacked
	// record's own Placed build, and its lump reading is what the audit reads.
	sp.ClosedShell = false
	marked.n.Payload = sp
	unmarked, err := Transform(t.Context(), marked, mustTranslation(r3.NewVec(0, 0, 50)))
	require.NoError(t, err)
	require.Len(t, unmarked.Lumps(), 2, `the cavity reads as a second lump`)
	for _, level := range []Verification{VerifyNone, VerifyAll} {
		_, err = unmarked.Tessellate(t.Context(), units.Millimeters(0.1), WithVerification(level))
		require.ErrorIs(t, err, ErrUnsupported)
		require.ErrorContains(t, err, "do not wind outward")
	}

	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.Equal(t, Unsound, report.Status)
	var invalid []string
	for _, d := range report.Diagnostics {
		if d.Code == DiagInvalidBody && d.Body == unmarked {
			invalid = append(invalid, d.Message)
		}
	}
	require.Len(t, invalid, 1)
	require.Contains(t, invalid[0], "do not wind outward")
}
