package loftmesh_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestProjectedPatchProofEdgeDirections(t *testing.T) {
	tests := []struct {
		name  string
		verts []r3.Vec
		tris  [][3]int
		want  loftmesh.ProjectedPatchCondition
	}{
		{
			name:  "one edge canonical forward",
			verts: []r3.Vec{{}, {X: 2}, {Y: 2}},
			tris:  [][3]int{{0, 1, 2}},
			want:  loftmesh.ProjectedPatchHolds,
		},
		{
			name:  "one edge canonical reverse",
			verts: []r3.Vec{{}, {X: 2}, {Y: 2}},
			tris:  [][3]int{{0, 2, 1}},
			want:  loftmesh.ProjectedPatchHolds,
		},
		{
			name:  "shared edge opposite directions",
			verts: []r3.Vec{{}, {X: 2}, {X: 2, Y: 2}, {Y: 2}},
			tris:  [][3]int{{0, 1, 2}, {0, 2, 3}},
			want:  loftmesh.ProjectedPatchHolds,
		},
		{
			name:  "shared edge same direction",
			verts: []r3.Vec{{}, {X: 2}, {X: 2, Y: 2}, {X: 1, Y: 1}},
			tris:  [][3]int{{0, 1, 2}, {0, 1, 3}},
			want:  loftmesh.ProjectedPatchStructure,
		},
	}
	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			indices := make([]int, len(tt.tris))
			for i := range indices {
				indices[i] = i
			}
			patch := loftmesh.ProjectedPatch{Triangles: indices, Axes: [2]int{0, 1}}
			got := loftmesh.ProjectedPatchProof(proofbound.NewWorkBudget(t.Context()), tt.verts, tt.tris, patch)
			require.Equal(t, tt.want, got)
		})
	}
}

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

func BenchmarkProjectedPatchProof(b *testing.B) {
	const side = 150
	verts := make([]r3.Vec, 0, (side+1)*(side+1))
	for y := range side + 1 {
		for x := range side + 1 {
			verts = append(verts, r3.Vec{X: float64(x), Y: float64(y)})
		}
	}
	tris := make([][3]int, 0, 2*side*side)
	for y := range side {
		for x := range side {
			v00 := y*(side+1) + x
			v10 := v00 + 1
			v01 := v00 + side + 1
			v11 := v01 + 1
			tris = append(tris, [3]int{v00, v10, v11}, [3]int{v00, v11, v01})
		}
	}
	indices := make([]int, len(tris))
	for i := range indices {
		indices[i] = i
	}
	patch := loftmesh.ProjectedPatch{Triangles: indices, Axes: [2]int{0, 1}}
	b.ReportAllocs()
	b.ResetTimer()
	for range b.N {
		got := loftmesh.ProjectedPatchProof(proofbound.NewWorkBudget(b.Context()), verts, tris, patch)
		if got != loftmesh.ProjectedPatchHolds {
			b.Fatalf("projected patch proof returned %v", got)
		}
	}
}
