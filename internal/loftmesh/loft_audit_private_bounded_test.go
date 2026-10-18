package loftmesh_test

import (
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestLoftCrossingAuditPrivateBoundedMatchesReference(t *testing.T) {
	verts := []r3.Vec{{}, {X: 2}, {X: 2, Y: 2}, {Y: 2}}
	tris := [][3]int{{0, 1, 2}, {0, 2, 3}}
	legacy := proofbound.NewWorkBudget(t.Context())
	require.NoError(t, loftmesh.LoftCrossingAuditBounded(legacy, verts, tris, 512, 1_000_000))
	private := proofbound.NewWallWorkBudgetWithOperation(65_536,
		proofbound.NewWorkBudget(t.Context()))
	require.NoError(t, loftmesh.LoftCrossingAuditPrivateBounded(private, verts, tris))
}

func TestLoftCrossingAuditPrivateBoundedRefusesBeforeExactLift(t *testing.T) {
	verts := []r3.Vec{{X: math.Ldexp(1, -34)}, {X: 2}, {Y: 2}}
	tris := [][3]int{{0, 1, 2}}
	steps := 0
	budget := &proofbound.WorkBudget{
		StepFn: func() error { steps++; return nil },
		ErrFn:  func() error { return nil },
	}
	err := loftmesh.LoftCrossingAuditPrivateBounded(budget, verts, tris)
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
	require.Equal(t, 1, steps)
}

func TestLoftCrossingAuditPrivateBoundedCancellation(t *testing.T) {
	verts := []r3.Vec{{}, {X: 2}, {Y: 2}}
	tris := [][3]int{{0, 1, 2}}
	steps := 0
	budget := &proofbound.WorkBudget{
		StepFn: func() error {
			steps++
			if steps == 2 {
				return context.Canceled
			}
			return nil
		},
		ErrFn: func() error { return nil },
	}
	require.ErrorIs(t, loftmesh.LoftCrossingAuditPrivateBounded(budget, verts, tris), context.Canceled)
	require.Equal(t, 2, steps)
}
