package loftmesh

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestFailedProjectedPatchKeepsIntraFamilyContactInAudit(t *testing.T) {
	verts := []r3.Vec{
		{X: 0, Y: 0}, {X: 2, Y: 0}, {X: 0, Y: 2},
		{X: 0.25, Y: 0.25}, {X: 1, Y: 0.25}, {X: 0.25, Y: 1},
	}
	tris := [][3]int{{0, 1, 2}, {3, 4, 5}}
	patch := ProjectedPatch{Triangles: []int{0, 1}, Axes: [2]int{0, 1}}
	require.NotEqual(t, ProjectedPatchHolds,
		ProjectedPatchProof(proofbound.NewWorkBudget(t.Context()), verts, tris, patch))

	work, err := loftCrossingAuditLimited(proofbound.NewWorkBudget(t.Context()), verts,
		tris, nil, loftGenericShortcuts, 10, len(tris), false, patch)
	require.Error(t, err)
	require.Equal(t, 1, work.Candidates)
	require.Equal(t, 1, work.Classifications)
}
