package massmoment

import (
	"errors"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

func TestReplayPrismGramThreshold(t *testing.T) {
	for _, tc := range []struct {
		name      string
		numerator int64
		admitted  bool
	}{{"below", 499999, true}, {"equal", 500000, false}, {"above", 500001, false}} {
		t.Run(tc.name, func(t *testing.T) {
			var m [3][3]*big.Rat
			for i := range 3 {
				for j := range 3 {
					m[i][j] = new(big.Rat)
					if i == j {
						m[i][j].SetInt64(1)
					}
				}
			}
			m[0][1].SetFrac64(tc.numerator, 1000000)
			m[1][0].Neg(m[0][1])
			_, err := replayMatrixCharge(t.Context(), m, proofbound.NewCircleReplayBudget())
			if tc.admitted && err != nil || !tc.admitted && !errors.Is(err, decaderr.ErrUnsupported) {
				t.Fatalf("threshold %s: %v", tc.name, err)
			}
		})
	}
}
