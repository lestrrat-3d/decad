package decad

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/stretchr/testify/require"
)

func TestCrossBoreCapPatchesHold(t *testing.T) {
	for _, tc := range []struct {
		name                 string
		radius, bore, height float64
		centerZ, setback     float64
	}{
		{"small", 5, 1, 10, 5, 0.5},
		{"large", 8, 1.5, 16, 8, 0.1875},
	} {
		t.Run(tc.name, func(t *testing.T) {
			rec := crossBoreExactRecord(t, tc.radius, tc.bore, tc.height, tc.centerZ)
			mesh, err := buildCrossBoreChamferMesh(t.Context(), rec, tc.setback)
			require.NoError(t, err)

			patches := crossBoreCapPatches(mesh.src)
			require.Len(t, patches, 2)
			minZ, maxZ := mesh.verts[0].Z, mesh.verts[0].Z
			for _, v := range mesh.verts {
				minZ, maxZ = min(minZ, v.Z), max(maxZ, v.Z)
			}
			var want [2][]int
			for i, s := range mesh.src {
				if s == 0 || s == 1 {
					want[s] = append(want[s], i)
				}
			}
			stations := len(patches[0].Triangles)
			require.Positive(t, stations)
			for s, patch := range patches {
				require.Equal(t, [2]int{0, 1}, patch.Axes)
				require.Len(t, patch.Triangles, stations)
				require.Equal(t, want[s], patch.Triangles)
				wantZ := minZ
				if s == 1 {
					wantZ = maxZ
				}
				for _, ti := range patch.Triangles {
					require.Equal(t, s, mesh.src[ti])
					for _, vi := range mesh.tris[ti] {
						require.Equal(t, wantZ, mesh.verts[vi].Z)
					}
				}
				require.Equal(t, loftmesh.ProjectedPatchHolds,
					loftmesh.ProjectedPatchProof(proofbound.NewPollWorkBudget(t.Context()),
						mesh.verts, mesh.tris, patch), "cap %d", s)
			}
		})
	}
}
