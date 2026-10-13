package proofbound_test

import (
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/stretchr/testify/require"
)

func TestPerimeterUpper(t *testing.T) {
	budget := proofbound.NewWorkBudget(t.Context())
	got, err := proofbound.PerimeterUpper(budget, []float64{1, 2, 4}, []float64{0.5, 0, 0.25})
	require.NoError(t, err)
	require.GreaterOrEqual(t, got, 7.75)
	require.Less(t, got, math.Nextafter(7.75, 8)+1e-12)

	got, err = proofbound.PerimeterUpper(budget, nil, nil)
	require.NoError(t, err)
	require.Zero(t, got)

	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	cancelled := proofbound.NewWorkBudget(ctx)
	lengths := make([]float64, proofbound.WorkPollInterval)
	bounds := make([]float64, proofbound.WorkPollInterval)
	_, err = proofbound.PerimeterUpper(cancelled, lengths, bounds)
	require.ErrorIs(t, err, context.Canceled)
}
