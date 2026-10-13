package stitchweld_test

import (
	"errors"
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/stitchweld"
	"github.com/stretchr/testify/require"
)

func TestZeroBound(t *testing.T) {
	t.Parallel()
	tests := []struct {
		name string
		mags []float64
		want bool
	}{
		{"empty", nil, true},
		{"all zero", []float64{0, 0, 0}, true},
		{"last nonzero", []float64{0, 0, 1e-12}, false},
		{"first nonzero", []float64{2, 0, 0}, false},
	}
	for _, tc := range tests {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			got, err := stitchweld.ZeroBound(proofbound.NewWallWorkBudget(100), slices.Values(tc.mags))
			require.NoError(t, err)
			require.Equal(t, tc.want, got)
		})
	}
}

func TestMaxBound(t *testing.T) {
	t.Parallel()
	got, err := stitchweld.MaxBound(proofbound.NewWallWorkBudget(100), slices.Values([]float64{0.5, 3.25, 1, 3.25, 0}))
	require.NoError(t, err)
	require.Equal(t, 3.25, got)

	got, err = stitchweld.MaxBound(proofbound.NewWallWorkBudget(100), slices.Values([]float64{0, 0}))
	require.NoError(t, err)
	require.Zero(t, got)

	got, err = stitchweld.MaxBound(proofbound.NewWallWorkBudget(100), slices.Values([]float64(nil)))
	require.NoError(t, err)
	require.Zero(t, got)
}

func TestBoundBudgetChargesPerMagnitude(t *testing.T) {
	t.Parallel()
	mags := []float64{0, 0, 0, 0}

	// Four steps are affordable, so the walk completes.
	ok, err := stitchweld.ZeroBound(proofbound.NewWallWorkBudget(4), slices.Values(mags))
	require.NoError(t, err)
	require.True(t, ok)

	// Three steps run out on the fourth magnitude and the error propagates.
	ok, err = stitchweld.ZeroBound(proofbound.NewWallWorkBudget(3), slices.Values(mags))
	require.ErrorIs(t, err, proofbound.ErrWallWorkBudget)
	require.False(t, ok)

	best, err := stitchweld.MaxBound(proofbound.NewWallWorkBudget(3), slices.Values([]float64{1, 2, 3, 4}))
	require.ErrorIs(t, err, proofbound.ErrWallWorkBudget)
	require.Zero(t, best)

	// A nonzero magnitude stops the walk at its own step.
	steps := 0
	budget := &proofbound.WorkBudget{StepFn: func() error { steps++; return nil }, ErrFn: func() error { return nil }}
	ok, err = stitchweld.ZeroBound(budget, slices.Values([]float64{0, 1, 0, 0}))
	require.NoError(t, err)
	require.False(t, ok)
	require.Equal(t, 2, steps)

	// A custom error is returned unchanged.
	boom := errors.New("boom")
	failing := &proofbound.WorkBudget{StepFn: func() error { return boom }, ErrFn: func() error { return boom }}
	_, err = stitchweld.MaxBound(failing, slices.Values(mags))
	require.Same(t, boom, err)
}
