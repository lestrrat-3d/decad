package proof_test

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/stretchr/testify/require"
)

func TestReplayFractionFloat64ChargesDenominatorScan(t *testing.T) {
	den := new(big.Int).Lsh(big.NewInt(1), 1024)
	words := uint64(len(den.Bits()))
	budget, err := proofbound.NewReplayBudgetLimits(words, 1, 1)
	require.NoError(t, err)
	got, err := proof.ReplayFractionFloat64(t.Context(), big.NewInt(1), den, budget)
	require.Error(t, err)
	require.Zero(t, got)
	work, records := budget.Usage()
	require.Equal(t, uint64(1), work)
	require.Zero(t, records)
	require.Zero(t, budget.Bytes())

	budget, err = proofbound.NewReplayBudgetLimits(words+1, 1, 1)
	require.NoError(t, err)
	_, err = proof.ReplayFractionFloat64(t.Context(), big.NewInt(1), den, budget)
	require.Error(t, err)
	work, _ = budget.Usage()
	require.Equal(t, words+1, work)
}
