package loftmesh_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestProjectedPatchAuditSkipsOnlyCertifiedInternalPairs(t *testing.T) {
	verts := []r3.Vec{{X: 0, Y: 0}, {X: 2, Y: 0},
		{X: 2, Y: 2}, {X: 0, Y: 2}}
	tris := [][3]int{{0, 1, 2}, {0, 2, 3}}
	patch := []loftmesh.ProjectedPatch{{Triangles: []int{0, 1}, Axes: [2]int{0, 1}}}
	require.Error(t, loftmesh.LoftCrossingAuditBounded(
		proofbound.NewWorkBudget(t.Context()), verts, tris, len(tris), 0))
	require.NoError(t, loftmesh.LoftCrossingAuditBoundedProjectedPatches(
		proofbound.NewWorkBudget(t.Context()), verts, tris, patch, len(tris), 0))

	verts = append(verts, r3.Vec{X: 0.5, Y: 0.5, Z: -1},
		r3.Vec{X: 1.5, Y: 0.5, Z: 1}, r3.Vec{X: 1, Y: 1.5, Z: -1})
	tris = append(tris, [3]int{4, 5, 6})
	require.Error(t, loftmesh.LoftCrossingAuditBoundedProjectedPatches(
		proofbound.NewWorkBudget(t.Context()), verts, tris, patch, len(tris), 100))
}
