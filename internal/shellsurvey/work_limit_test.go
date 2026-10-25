package shellsurvey_test

import (
	"context"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/shellsurvey"
	"github.com/stretchr/testify/require"
)

func TestShellCandidateWorkLimitDetails(t *testing.T) {
	budget := proofbound.NewWorkBudget(t.Context())
	err := shellsurvey.RequireWallSurveyWork(budget, 100, 100)
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
	var detail *proofbound.WorkLimitError
	require.ErrorAs(t, err, &detail)
	count, ok := proofbound.WallCandidateWork(100, 100, false)
	require.True(t, ok)
	require.Equal(t, "shell_candidate_visits", detail.Resource)
	require.Equal(t, count, detail.Observed)
	require.Equal(t, uint64(1<<20), detail.Maximum)
	err = shellsurvey.RequireWallSurveyWork(budget, int(^uint(0)>>1), int(^uint(0)>>1))
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
	require.NotErrorAs(t, err, &detail)
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	err = shellsurvey.RequireWallSurveyWork(proofbound.NewWorkBudget(ctx), 100, 100)
	require.Same(t, ctx.Err(), err)
}
