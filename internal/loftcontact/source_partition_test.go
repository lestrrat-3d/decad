package loftcontact_test

import (
	"context"
	"testing"

	"github.com/lestrrat-3d/decad/internal/loftcontact"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/sketch"
	"github.com/stretchr/testify/require"
)

func TestLoftBoundedProfilesGroundingCancelsInFlight(t *testing.T) {
	world := sketch.NewWorld()
	scene, err := world.CreateSketch(world.XY())
	require.NoError(t, err)
	for i := range 8 {
		scene.CreatePoint(float64(i), 0)
	}
	ctx, cancel := context.WithCancel(t.Context())
	defer cancel()
	steps := 0
	budget := &proofbound.WorkBudget{
		StepFn: func() error {
			steps++
			if steps == 3 {
				cancel()
			}
			return ctx.Err()
		},
		ErrFn: ctx.Err,
	}
	err = loftcontact.GroundPartitionScene(budget, scene)
	require.ErrorIs(t, err, context.Canceled)
	require.Equal(t, 3, steps)
	for i, point := range scene.Points() {
		require.Equal(t, i < 2, point.IsFixed(), "point %d", i)
	}
}
