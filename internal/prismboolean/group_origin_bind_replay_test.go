package prismboolean_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/prismboolean"
	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/stretchr/testify/require"
)

func TestReplayAdvanceGroupOrigins(t *testing.T) {
	previous := []int{0, 1}
	rows := [][]prismcells.Origin{{{IsB: true, Hole: -1}}, {{Region: 1, Hole: -1}}, {{Region: 0, Hole: -1}}}
	got, ok, err := prismboolean.ReplayAdvanceGroupOrigins(t.Context(), previous, rows, 2, proofbound.NewReplayTrialBudget())
	require.NoError(t, err)
	require.True(t, ok)
	require.Equal(t, []int{2, 1, 0}, got)
	for _, bad := range [][][]prismcells.Origin{
		{{{IsB: true, Region: 1, Hole: -1}}, {{Region: 1, Hole: -1}}, {{Region: 0, Hole: -1}}},
		{{{IsB: true, Hole: -1}}, {{Region: 1, Hole: -1}}, {{Region: 1, Hole: -1}}},
		{{{IsB: true, Hole: -1}}, {{Region: 1, Hole: 0}}, {{Region: 0, Hole: -1}}},
		{{{IsB: true, Hole: -1}}, {{Region: 1, Hole: -1, Segment: 1}}, {{Region: 0, Hole: -1}}},
	} {
		got, ok, err = prismboolean.ReplayAdvanceGroupOrigins(t.Context(), previous, bad, 2, proofbound.NewReplayTrialBudget())
		require.NoError(t, err)
		require.False(t, ok)
		require.Nil(t, got)
	}
}
