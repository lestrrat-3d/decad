package proofbound_test

import (
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/stretchr/testify/require"
)

func TestBoundedArithmeticReplayExactLimits(t *testing.T) {
	a := proofbound.MeasuredScalar(1.1, 0.01)
	b := proofbound.MeasuredScalar(3.7, 0.02)
	tests := []struct {
		name string
		fn   func(context.Context, proofbound.BoundedScalar, proofbound.BoundedScalar,
			*proofbound.ReplayBudget) (proofbound.BoundedScalar, error)
		ordinary func(proofbound.BoundedScalar, proofbound.BoundedScalar) proofbound.BoundedScalar
	}{
		{"add", proofbound.BoundedAddReplay, proofbound.BoundedAdd},
		{"sub", proofbound.BoundedSubReplay, proofbound.BoundedSub},
		{"mul", proofbound.BoundedMulReplay, proofbound.BoundedMul},
		{"div", proofbound.BoundedDivReplay, proofbound.BoundedDiv},
		{"quotient", func(ctx context.Context, a, b proofbound.BoundedScalar,
			budget *proofbound.ReplayBudget) (proofbound.BoundedScalar, error) {
			return proofbound.BoundedQuotientReplay(ctx, a.Value, a.Bound, b.Value, b.Bound, budget)
		}, func(a, b proofbound.BoundedScalar) proofbound.BoundedScalar {
			return proofbound.BoundedQuotient(a.Value, a.Bound, b.Value, b.Bound)
		}},
	}
	for _, tc := range tests {
		t.Run(tc.name, func(t *testing.T) {
			meter := proofbound.NewReplayTrialBudget()
			got, err := tc.fn(t.Context(), a, b, meter)
			require.NoError(t, err)
			want := tc.ordinary(a, b)
			require.Equal(t, math.Float64bits(want.Value), math.Float64bits(got.Value))
			require.GreaterOrEqual(t, got.Bound, want.Bound)
			work, records := meter.Usage()
			require.Positive(t, work)
			require.Positive(t, records)
			require.Positive(t, meter.Bytes())
			exact, err := proofbound.NewReplayBudgetLimits(work, records, meter.Bytes())
			require.NoError(t, err)
			again, err := tc.fn(t.Context(), a, b, exact)
			require.NoError(t, err)
			require.Equal(t, math.Float64bits(got.Bound), math.Float64bits(again.Bound))
			short, err := proofbound.NewReplayBudgetLimits(work-1, records, meter.Bytes())
			require.NoError(t, err)
			_, err = tc.fn(t.Context(), a, b, short)
			require.Error(t, err)
			short, err = proofbound.NewReplayBudgetLimits(work, records-1, meter.Bytes())
			require.NoError(t, err)
			_, err = tc.fn(t.Context(), a, b, short)
			require.Error(t, err)
			short, err = proofbound.NewReplayBudgetLimits(work, records, meter.Bytes()-1)
			require.NoError(t, err)
			_, err = tc.fn(t.Context(), a, b, short)
			require.Error(t, err)
			ctx, cancel := context.WithCancel(t.Context())
			cancel()
			cancelled := proofbound.NewReplayTrialBudget()
			_, err = tc.fn(ctx, a, b, cancelled)
			require.ErrorIs(t, err, context.Canceled)
			work, records = cancelled.Usage()
			require.Zero(t, work)
			require.Zero(t, records)
			require.Zero(t, cancelled.Bytes())
		})
	}
	// The exact reference checks that the rounded product's reported bound
	// encloses its mathematical value, independently of the ordinary helper.
	product, err := proofbound.BoundedMulReplay(t.Context(), proofbound.ExactScalar(1.1),
		proofbound.ExactScalar(3.7), proofbound.NewReplayTrialBudget())
	require.NoError(t, err)
	exact := proof.FloatRat(1.1)
	exact.Mul(exact, proof.FloatRat(3.7))
	require.GreaterOrEqual(t, product.Bound, proof.RationalFloatError(exact, product.Value))
}
