package proof_test

import (
	"context"
	"errors"
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/stretchr/testify/require"
)

func TestReplayRationalFloatErrorBoundsExactDifference(t *testing.T) {
	tiny := new(big.Rat).SetFrac(big.NewInt(1), new(big.Int).Lsh(big.NewInt(1), 1075))
	for _, tc := range []struct {
		name  string
		exact *big.Rat
		held  float64
	}{
		{"equal", big.NewRat(3, 2), 1.5}, {"third", big.NewRat(1, 3), 1.0 / 3}, {"negative", big.NewRat(-7, 13), -.5},
		{"underflow", tiny, 0}, {"large offset", big.NewRat(1, 3), 1e100},
	} {
		t.Run(tc.name, func(t *testing.T) {
			got, err := proof.ReplayRationalFloatError(t.Context(), tc.exact, tc.held, proofbound.NewReplayTrialBudget())
			require.NoError(t, err)
			want := proof.RationalFloatError(tc.exact, tc.held)
			require.GreaterOrEqual(t, got, want)
			if want == 0 {
				require.Zero(t, got)
			}
		})
	}
}

func TestReplayRationalFloatErrorLimits(t *testing.T) {
	exact := big.NewRat(1, 3)
	held := .3
	base := proofbound.NewReplayTrialBudget()
	_, err := proof.ReplayRationalFloatError(t.Context(), exact, held, base)
	require.NoError(t, err)
	w, r := base.Usage()
	n := base.Bytes()
	for _, mode := range []string{replayBudgetTestExact, replayBudgetTestWork, replayBudgetTestRecords, replayBudgetTestBytes} {
		t.Run(mode, func(t *testing.T) {
			b := proofbound.NewReplayTrialBudget()
			ww, rr, nn := proofbound.ReplayTrialWork-w, proofbound.ReplayTrialRecords-r, proofbound.ReplayTrialBytes-n
			switch mode {
			case replayBudgetTestWork:
				ww++
			case replayBudgetTestRecords:
				rr++
			case replayBudgetTestBytes:
				nn++
			}
			require.NoError(t, b.SpendWork(t.Context(), ww))
			require.NoError(t, b.ReserveRecords(t.Context(), rr))
			require.NoError(t, b.ReserveBytes(t.Context(), nn))
			_, err := proof.ReplayRationalFloatError(t.Context(), exact, held, b)
			if mode == replayBudgetTestExact {
				require.NoError(t, err)
			} else {
				require.ErrorIs(t, err, decaderr.ErrUnsupported)
			}
		})
	}
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	_, err = proof.ReplayRationalFloatError(ctx, exact, held, proofbound.NewReplayTrialBudget())
	require.True(t, errors.Is(err, context.Canceled))
	_, err = proof.ReplayRationalFloatError(t.Context(), nil, held, proofbound.NewReplayTrialBudget())
	require.Error(t, err)
	_, err = proof.ReplayRationalFloatError(t.Context(), exact, math.Inf(1), proofbound.NewReplayTrialBudget())
	require.Error(t, err)
}
