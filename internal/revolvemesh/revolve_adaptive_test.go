package revolvemesh

// CI: isolated-tests

import (
	"context"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestAdaptiveContactCandidatesMatchBothSearches(t *testing.T) {
	for name, m := range sweepMeshes(t) {
		for _, delta := range sweepDeltas {
			margin := proofbound.ProductUpper(2, delta)
			got, err := adaptiveContactCandidates(proofbound.NewWorkBudget(t.Context()), m.data, m.tris, margin)
			require.NoError(t, err, name)
			want, err := revolveContactCandidates(proofbound.NewWorkBudget(t.Context()), m.data, m.tris, margin)
			require.NoError(t, err, name)
			require.Equal(t, want, got, "%s delta %g", name, delta)
		}
	}
}

func TestContactSweepStrategyAndCancellation(t *testing.T) {
	sparse := make([]RevolveAuditTri, 128)
	for i := range sparse {
		x := float64(i)
		sparse[i].Box = [2]r3.Vec{{X: x}, {X: x + .1, Y: 1, Z: 1}}
	}
	useSweep, err := preferContactSweep(proofbound.NewWorkBudget(t.Context()), sparse, 0)
	require.NoError(t, err)
	require.True(t, useSweep)

	dense := make([]RevolveAuditTri, 3000)
	for i := range dense {
		dense[i].Box = [2]r3.Vec{{}, {X: 1, Y: 1, Z: 1}}
	}
	useSweep, err = preferContactSweep(proofbound.NewWorkBudget(t.Context()), dense, 0)
	require.NoError(t, err)
	require.False(t, useSweep)

	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	_, err = preferContactSweep(proofbound.NewWorkBudget(ctx), sparse, 0)
	require.ErrorIs(t, err, context.Canceled)
}
