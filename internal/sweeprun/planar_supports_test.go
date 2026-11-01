package sweeprun

import (
	"errors"
	"testing"

	"github.com/lestrrat-3d/decad/internal/planarsweep"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/stretchr/testify/require"
)

func TestPlanarSupportCacheReplaysSearchWork(t *testing.T) {
	var cache planarSupportCache
	var searches, firstSteps, replaySteps int
	first := &proofbound.WorkBudget{StepFn: func() error { firstSteps++; return nil }}
	search := func(poll func() error) ([]planarsweep.Support, error) {
		searches++
		for range 3 {
			if err := poll(); err != nil {
				return nil, err
			}
		}
		return []planarsweep.Support{}, nil
	}
	got, err := cache.get(first, search)
	require.NoError(t, err)
	require.Empty(t, got)
	require.Equal(t, 3, firstSteps)

	second := &proofbound.WorkBudget{StepFn: func() error { replaySteps++; return nil }}
	got, err = cache.get(second, search)
	require.NoError(t, err)
	require.Empty(t, got)
	require.Equal(t, 3, replaySteps)
	require.Equal(t, 1, searches)
}

func TestPlanarSupportCacheDoesNotKeepFailedSearch(t *testing.T) {
	var cache planarSupportCache
	wantErr := errors.New("support search stopped")
	budget := &proofbound.WorkBudget{StepFn: func() error { return nil }}
	_, err := cache.get(budget, func(poll func() error) ([]planarsweep.Support, error) {
		if pollErr := poll(); pollErr != nil {
			return nil, pollErr
		}
		return nil, wantErr
	})
	require.ErrorIs(t, err, wantErr)
	require.False(t, cache.ready)
}

func TestPlanarSupportCacheReturnsReplayBudgetError(t *testing.T) {
	cache := planarSupportCache{work: 2, ready: true}
	wantErr := errors.New("cancelled")
	steps := 0
	budget := &proofbound.WorkBudget{StepFn: func() error {
		steps++
		if steps == 2 {
			return wantErr
		}
		return nil
	}}
	_, err := cache.get(budget, func(func() error) ([]planarsweep.Support, error) {
		t.Fatal("cached search must not run")
		return nil, nil
	})
	require.ErrorIs(t, err, wantErr)
	require.Equal(t, 2, steps)
}
