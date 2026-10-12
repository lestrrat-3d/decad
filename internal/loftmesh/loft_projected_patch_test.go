package loftmesh_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestProjectedPatchProofAcceptsLiftedGraph(t *testing.T) {
	verts := []r3.Vec{{X: 0, Y: 0, Z: 0}, {X: 2, Y: 0, Z: 1},
		{X: 2, Y: 2, Z: 0}, {X: 0, Y: 2, Z: -1}}
	tris := [][3]int{{0, 1, 2}, {0, 2, 3}}
	patch := loftmesh.ProjectedPatch{Triangles: []int{0, 1}, Axes: [2]int{0, 1}}
	require.Equal(t, loftmesh.ProjectedPatchHolds,
		loftmesh.ProjectedPatchProof(proofbound.NewWorkBudget(t.Context()), verts, tris, patch))
}

func TestProjectedPatchProofRefusesFoldedProjection(t *testing.T) {
	verts := []r3.Vec{{X: 0, Y: 0}, {X: 2, Y: 0},
		{X: 2, Y: 2}, {X: 1, Y: -1}}
	tris := [][3]int{{0, 1, 2}, {0, 2, 3}}
	patch := loftmesh.ProjectedPatch{Triangles: []int{0, 1}, Axes: [2]int{0, 1}}
	require.Equal(t, loftmesh.ProjectedPatchOrientation,
		loftmesh.ProjectedPatchProof(proofbound.NewWorkBudget(t.Context()), verts, tris, patch))
}

func TestProjectedPatchProofRefusesCrossedBoundary(t *testing.T) {
	verts := []r3.Vec{{}}
	tris := make([][3]int, 5)
	for i := range tris {
		angle := 2 * math.Pi * float64((2*i)%5) / 5
		verts = append(verts, r3.Vec{X: math.Cos(angle), Y: math.Sin(angle)})
		tris[i] = [3]int{0, i + 1, (i+1)%5 + 1}
	}
	patch := loftmesh.ProjectedPatch{Triangles: []int{0, 1, 2, 3, 4}, Axes: [2]int{0, 1}}
	require.Equal(t, loftmesh.ProjectedPatchSelfContact,
		loftmesh.ProjectedPatchProof(proofbound.NewWorkBudget(t.Context()), verts, tris, patch))
}

func TestProjectedPatchProofRefusesRepeatedCell(t *testing.T) {
	verts := []r3.Vec{{}, {X: 1}, {Y: 1}}
	tris := [][3]int{{0, 1, 2}, {0, 1, 2}}
	patch := loftmesh.ProjectedPatch{Triangles: []int{0, 1}, Axes: [2]int{0, 1}}
	require.Equal(t, loftmesh.ProjectedPatchStructure,
		loftmesh.ProjectedPatchProof(proofbound.NewWorkBudget(t.Context()), verts, tris, patch))
}
