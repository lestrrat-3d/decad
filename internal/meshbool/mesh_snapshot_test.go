package meshbool_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestEqualMeshSnapshotsScalarBitsAndLimits(t *testing.T) {
	t.Parallel()
	base := &topo.Mesh{Vertices: []r3.Vec{{}}, BoundaryOK: true, SymDiffOK: true}
	equal, err := meshbool.EqualMeshSnapshots(t.Context(), base, base, proofbound.NewReplayTrialBudget())
	require.NoError(t, err)
	require.True(t, equal)
	for _, tc := range []struct {
		name   string
		mutate func(*topo.Mesh)
	}{
		{"bound", func(m *topo.Mesh) { m.Bound = 1 }},
		{"coordinate bound", func(m *topo.Mesh) { m.CoordBound = 1 }},
		{"area slack", func(m *topo.Mesh) { m.AreaSlack = 1 }},
		{"symmetric difference", func(m *topo.Mesh) { m.VolSymDiff = 1 }},
		{"occupied volume proof", func(m *topo.Mesh) { m.SymDiffOK = false }},
		{"boundary proof", func(m *topo.Mesh) { m.BoundaryOK = false }},
		{"signed zero vertex", func(m *topo.Mesh) { m.Vertices[0].X = math.Copysign(0, -1) }},
		{"vertex y", func(m *topo.Mesh) { m.Vertices[0].Y = 1 }},
		{"vertex z", func(m *topo.Mesh) { m.Vertices[0].Z = 1 }},
	} {
		t.Run(tc.name, func(t *testing.T) {
			changed := *base
			changed.Vertices = []r3.Vec{{}}
			tc.mutate(&changed)
			equal, err := meshbool.EqualMeshSnapshots(t.Context(), base, &changed, proofbound.NewReplayTrialBudget())
			require.NoError(t, err)
			require.False(t, equal)
		})
	}
	oversized := &topo.Mesh{Triangles: make([][3]int, 4097)}
	budget := proofbound.NewReplayTrialBudget()
	equal, err = meshbool.EqualMeshSnapshots(t.Context(), oversized, oversized, budget)
	require.NoError(t, err)
	require.False(t, equal)
	work, records := budget.Usage()
	require.Equal(t, uint64(64), work)
	require.Zero(t, records)
}
