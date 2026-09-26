package decad_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestLoftChainVerifyValidityAfterAudit(t *testing.T) {
	for _, mode := range []string{"built", "placed", "placed copy"} {
		t.Run(mode, func(t *testing.T) {
			pts := [][2]float64{{0, 0}, {10, 0}, {10, 8}, {22, 8}}
			s0, ch0, s1, ch1 := chainLoftPair(t, 5, pts, pts)
			doc := decad.New()
			body, err := doc.LoftChain(t.Context(), s0, ch0, s1, ch1)
			require.NoError(t, err)
			require.Len(t, body.Faces(), 6)

			if mode != "built" {
				motion, err := r3.Translation(r3.NewVec(100, 7, 3))
				require.NoError(t, err)
				require.NotEqual(t, r3.Identity(), motion)
				if mode == "placed" {
					body, err = body.Placed(t.Context(), motion)
				} else {
					body, err = body.PlacedCopy(t.Context(), motion)
				}
				require.NoError(t, err)
			}

			report, err := doc.Verify(t.Context())
			require.NoError(t, err)
			br, err := report.ForBody(body)
			require.NoError(t, err)
			require.Equal(t, decad.ValidityValid, br.Validity.Outcome)
			require.Empty(t, br.Validity.Diagnostics)
			for _, d := range report.Diagnostics {
				require.NotEqual(t, decad.DiagUndecidedValidity, d.Code)
			}
		})
	}
}
