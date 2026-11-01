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

func TestReplayRationalFloatErrorMatchesOrdinary(t *testing.T) {
	for _, tc := range []struct {
		name  string
		exact float64
		held  float64
	}{
		{"equal", 1.5, 1.5},
		{"positive", 1.25, 1.2},
		{"negative", -1.25, -1.2},
		{"tiny", math.SmallestNonzeroFloat64, 0},
	} {
		t.Run(tc.name, func(t *testing.T) {
			r := proof.FloatRat(tc.exact)
			got, err := proof.ReplayRationalFloatError(t.Context(), r, tc.held,
				proofbound.NewReplayTrialBudget())
			require.NoError(t, err)
			require.Equal(t, math.Float64bits(proof.RationalFloatError(r, tc.held)), math.Float64bits(got))
		})
	}
	for _, pi := range []*big.Rat{proofbound.PiLower, proofbound.PiUpper} {
		r := new(big.Rat).Quo(pi, big.NewRat(4, 1))
		held := 0.7853981633974483
		got, err := proof.ReplayRationalFloatError(t.Context(), r, held,
			proofbound.NewReplayTrialBudget())
		require.NoError(t, err)
		require.Equal(t, math.Float64bits(proof.RationalFloatError(r, held)), math.Float64bits(got))
	}
}

func TestReplayRationalFloatErrorNonfiniteAndNil(t *testing.T) {
	for _, tc := range []struct {
		exact *big.Rat
		held  float64
	}{
		{nil, 1},
		{big.NewRat(1, 1), math.NaN()},
		{big.NewRat(1, 1), math.Inf(1)},
		{big.NewRat(1, 1), math.Inf(-1)},
	} {
		budget := proofbound.NewReplayTrialBudget()
		got, err := proof.ReplayRationalFloatError(t.Context(), tc.exact, tc.held, budget)
		require.NoError(t, err)
		require.True(t, math.IsInf(got, 1))
		work, records := budget.Usage()
		require.Zero(t, work)
		require.Zero(t, records)
		require.Zero(t, budget.Bytes())
	}
	got, err := proof.ReplayRationalFloatError(t.Context(), big.NewRat(1, 1), 0, nil)
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
	require.Zero(t, got)
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	got, err = proof.ReplayRationalFloatError(ctx, nil, math.NaN(), nil)
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
	require.Zero(t, got)
	got, err = proof.ReplayRationalFloatError(ctx, nil, math.NaN(), proofbound.NewReplayTrialBudget())
	require.ErrorIs(t, err, context.Canceled)
	require.Zero(t, got)
}

func TestReplayRationalFloatErrorExtremeRounding(t *testing.T) {
	for _, tc := range []struct {
		name  string
		exact *big.Rat
	}{
		{"overflow", new(big.Rat).SetInt(new(big.Int).Lsh(big.NewInt(1), 1100))},
		{"underflow", new(big.Rat).SetFrac(big.NewInt(1), new(big.Int).Lsh(big.NewInt(1), 1200))},
	} {
		t.Run(tc.name, func(t *testing.T) {
			want := proof.RationalFloatError(tc.exact, 0)
			trial := proofbound.NewReplayTrialBudget()
			got, err := proof.ReplayRationalFloatError(t.Context(), tc.exact, 0, trial)
			require.NoError(t, err)
			require.Equal(t, math.Float64bits(want), math.Float64bits(got))
			work, records := trial.Usage()
			bytes := trial.Bytes()
			for _, limit := range []struct {
				name                 string
				work, records, bytes uint64
				wantOK               bool
			}{
				{"exact", work, records, bytes, true},
				{"short work", work - 1, records, bytes, false},
				{"short records", work, records - 1, bytes, false},
				{"short bytes", work, records, bytes - 1, false},
			} {
				t.Run(limit.name, func(t *testing.T) {
					budget, err := proofbound.NewReplayBudgetLimits(limit.work, limit.records, limit.bytes)
					require.NoError(t, err)
					got, err := proof.ReplayRationalFloatError(t.Context(), tc.exact, 0, budget)
					if !limit.wantOK {
						require.Error(t, err)
						require.Zero(t, got)
						return
					}
					require.NoError(t, err)
					require.Equal(t, math.Float64bits(want), math.Float64bits(got))
				})
			}
		})
	}
}

func TestReplayRationalFloatErrorBudgetEdges(t *testing.T) {
	exact := new(big.Rat).Quo(proofbound.PiUpper, big.NewRat(4, 1))
	held := 0.7853981633974483
	want := proof.RationalFloatError(exact, held)
	trial := proofbound.NewReplayTrialBudget()
	got, err := proof.ReplayRationalFloatError(t.Context(), exact, held, trial)
	require.NoError(t, err)
	require.Equal(t, math.Float64bits(want), math.Float64bits(got))
	work, records := trial.Usage()
	bytes := trial.Bytes()
	require.Positive(t, work)
	require.Positive(t, records)
	require.Positive(t, bytes)
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
			got, err := proof.ReplayRationalFloatError(t.Context(), exact, held, budget)
			if !tc.wantOK {
				require.Error(t, err)
				require.Zero(t, got)
				return
			}
			require.NoError(t, err)
			require.Equal(t, math.Float64bits(want), math.Float64bits(got))
		})
	}
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	budget := proofbound.NewReplayTrialBudget()
	got, err = proof.ReplayRationalFloatError(ctx, exact, held, budget)
	require.ErrorIs(t, err, context.Canceled)
	require.Zero(t, got)
	w, r := budget.Usage()
	require.Zero(t, w)
	require.Zero(t, r)
	require.Zero(t, budget.Bytes())
}
