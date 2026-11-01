package apitest_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/stretchr/testify/require"
)

func TestVerifyCutAgainstCopiedTool(t *testing.T) {
	for _, changed := range []bool{false, true} {
		t.Run(map[bool]string{false: "same tool", true: "changed tool"}[changed], func(t *testing.T) {
			sc := newRetentionScene()
			target, original := sc.roundedBox(t), sc.tool(t, 20)
			tool, err := decad.Copy(t.Context(), original)
			require.NoError(t, err)
			cut, err := decad.Cut(t.Context(), target, tool)
			require.NoError(t, err)
			require.True(t, tookMeshPath(cut))
			if changed {
				original = translated(t, original, 1, 0, 0)
			}
			report, err := sc.doc.Verify(t.Context())
			require.NoError(t, err)
			if changed {
				pairFound := false
				for _, row := range report.Interferences {
					pairFound = pairFound || (row.A == cut && row.B == original) || (row.A == original && row.B == cut)
				}
				for _, d := range report.Diagnostics {
					if d.Pair != nil {
						pairFound = pairFound || (d.Pair.A == cut && d.Pair.B == original) ||
							(d.Pair.A == original && d.Pair.B == cut)
					}
				}
				require.True(t, pairFound, "a changed tool does not use the history proof")
				return
			}
			require.Empty(t, report.Interferences)
			for _, d := range report.Diagnostics {
				require.Nil(t, d.Pair)
			}
			later, err := decad.Copy(t.Context(), original)
			require.NoError(t, err)
			require.NoError(t, decad.Remove(original))
			require.Equal(t, []*decad.Body{cut, later}, sc.doc.Bodies())
			report, err = sc.doc.Verify(t.Context())
			require.NoError(t, err)
			require.Empty(t, report.Interferences)
			for _, d := range report.Diagnostics {
				require.Nil(t, d.Pair)
			}
			report, err = sc.doc.Verify(t.Context(), decad.WithClearances())
			require.NoError(t, err)
			require.Empty(t, report.Clearances)
			require.Equal(t, decad.Suspect, report.Status)
			d, found := findDiagnostic(report.Diagnostics, decad.DiagUndecidedClearance)
			require.True(t, found)
			require.NotNil(t, d.Pair)
			require.True(t, (d.Pair.A == cut && d.Pair.B == later) || (d.Pair.A == later && d.Pair.B == cut))
		})
	}
}
