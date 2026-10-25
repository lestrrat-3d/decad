package proofbound_test

import (
	"context"
	"fmt"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/stretchr/testify/require"
)

func TestWallWorkLimitCountsAndErrorTranslation(t *testing.T) {
	budget := proofbound.NewWallWorkBudget(2)
	done, err := budget.StepN(3)
	require.Equal(t, uint64(2), done)
	require.ErrorIs(t, err, proofbound.ErrWallWorkBudget)
	require.Equal(t, proofbound.ErrWallWorkBudget.Error(), err.Error())
	var detail *proofbound.WorkLimitError
	require.ErrorAs(t, err, &detail)
	require.Equal(t, "proof_steps", detail.Resource)
	require.Equal(t, uint64(3), detail.Observed)
	require.Equal(t, uint64(2), detail.Maximum)

	// A phase-boundary poll observes exactly the completed limit, whereas
	// another Step requests the first work item beyond it.
	require.ErrorAs(t, budget.Err(), &detail)
	require.Equal(t, uint64(2), detail.Observed)
	translated := fmt.Errorf("%w: the existing modeling refusal", decaderr.ErrUnsupported)
	carried := proofbound.PreserveWorkLimit(translated, err)
	require.ErrorIs(t, carried, decaderr.ErrUnsupported)
	require.NotErrorIs(t, carried, proofbound.ErrWallWorkBudget)
	require.Equal(t, translated.Error(), carried.Error())
	require.ErrorAs(t, carried, &detail)
	require.Equal(t, uint64(3), detail.Observed)
	require.Equal(t, uint64(2), detail.Maximum)
	require.Same(t, translated, proofbound.PreserveWorkLimit(translated, context.Canceled))
}

func TestWallWorkLimitPreservesOperationCancellation(t *testing.T) {
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	budget := proofbound.NewWallWorkBudgetWithOperation(0, proofbound.NewWorkBudget(ctx))
	require.Same(t, ctx.Err(), budget.Err())
	var detail *proofbound.WorkLimitError
	require.NotErrorAs(t, budget.Err(), &detail)
	// Step's operation poll retains its existing 256-step cadence.
	operation := &proofbound.WorkBudget{
		StepFn: ctx.Err,
		ErrFn:  ctx.Err,
	}
	require.Same(t, ctx.Err(), proofbound.NewWallWorkBudgetWithOperation(0, operation).Step())
}
