package boolshift_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/boolshift"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/stretchr/testify/require"
)

func TestPiecesCopyLabels(t *testing.T) {
	vertices := []int{0, 1}
	facets := []int{1}
	p := boolshift.NewPieces(vertices, facets, 2)
	vertices[0], facets[0] = 9, 9
	require.Equal(t, 0, p.Vertex(0))
	require.Equal(t, 1, p.Facet(0))
	snapshot, err := p.Snapshot(t.Context(), proofbound.NewBrakeReplayBudget(), nil)
	require.NoError(t, err)
	require.True(t, p.EqualSnapshot(snapshot))
	require.Equal(t, 2, snapshot.Count())
}
