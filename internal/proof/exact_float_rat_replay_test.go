package proof_test

import (
	"context"
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/stretchr/testify/require"
)

func TestReplayFloatRatMatchesFiniteBinary64(t *testing.T) {
	for _, bits := range []uint64{
		0, 1 << 63, 1, 1<<63 | 1, 1<<52 - 1,
		1 << 52, 0x3ff8000000000000, 0xbff8000000000000,
		0x7fefffffffffffff, 0xffefffffffffffff,
	} {
		value := math.Float64frombits(bits)
		budget := proofbound.NewReplayTrialBudget()
		got, err := proof.ReplayFloatRat(t.Context(), value, budget)
		require.NoError(t, err)
		require.Zero(t, got.Cmp(new(big.Rat).SetFloat64(value)), "binary64 bits %x", bits)
	}
}

func TestReplayFloatRatRejectsInvalidInputsBeforeArithmetic(t *testing.T) {
	for _, value := range []float64{math.NaN(), math.Inf(1), math.Inf(-1)} {
		budget := proofbound.NewReplayTrialBudget()
		got, err := proof.ReplayFloatRat(t.Context(), value, budget)
		require.ErrorIs(t, err, decaderr.ErrUnsupported)
		require.Nil(t, got)
		work, records := budget.Usage()
		require.Zero(t, work)
		require.Zero(t, records)
		require.Zero(t, budget.Bytes())
	}
	got, err := proof.ReplayFloatRat(t.Context(), 1, nil)
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
	require.Nil(t, got)

	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	got, err = proof.ReplayFloatRat(ctx, math.NaN(), nil)
	require.ErrorIs(t, err, context.Canceled)
	require.Nil(t, got)
}

func TestReplayFloatRatExactAndOneShortBudgets(t *testing.T) {
	const work, records, bytes = 128, 3, 2048
	for _, tc := range []struct {
		name                 string
		work, records, bytes uint64
		wantOK               bool
	}{
		{"exact", work, records, bytes, true},
		{"short work", work - 1, records, bytes, false},
		{"short records", work, records - 1, bytes, false},
		{"short bytes", work, records, bytes - 1, false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			budget, err := proofbound.NewReplayBudgetLimits(tc.work, tc.records, tc.bytes)
			require.NoError(t, err)
			got, err := proof.ReplayFloatRat(t.Context(), math.MaxFloat64, budget)
			if !tc.wantOK {
				require.Error(t, err)
				require.Nil(t, got)
				return
			}
			require.NoError(t, err)
			require.Zero(t, got.Cmp(new(big.Rat).SetFloat64(math.MaxFloat64)))
		})
	}
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	budget := proofbound.NewReplayTrialBudget()
	got, err := proof.ReplayFloatRat(ctx, math.MaxFloat64, budget)
	require.ErrorIs(t, err, context.Canceled)
	require.Nil(t, got)
	w, r := budget.Usage()
	require.Zero(t, w)
	require.Zero(t, r)
	require.Zero(t, budget.Bytes())
}
