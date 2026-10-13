package decad

import (
	"slices"

	"github.com/lestrrat-3d/decad/internal/loftmesh"
)

// revolveSectorBorePatches proposes projected graphs from the completed
// result. The crossing audit independently proves each proposal and audits
// every pair outside a proven patch, including all cross-patch pairs.
func revolveSectorBorePatches(mesh *joinedMesh, oldCount, stations int,
	band bool, caps revolveSectorBoreCaps, bandGroup, wallGroup int) []loftmesh.ProjectedPatch {
	groups := make(map[int][]int)
	for i := range oldCount {
		group := mesh.src[i]
		if group == caps.lowGroup || group == caps.highGroup ||
			group == bandGroup || group == wallGroup {
			continue
		}
		groups[group] = append(groups[group], i)
	}
	keys := make([]int, 0, len(groups))
	for group := range groups {
		keys = append(keys, group)
	}
	slices.Sort(keys)
	patches := make([]loftmesh.ProjectedPatch, 0, len(keys)+4)
	for _, group := range keys {
		if len(groups[group]) < 2 {
			continue
		}
		patches = append(patches, loftmesh.ProjectedPatch{
			Triangles: groups[group], Axes: [2]int{1, 2},
		})
	}
	// A complete annulus has two boundary loops. Each angular half is a
	// single-boundary patch that the existing projected proof can check.
	for cap := range 2 {
		if band && cap == 0 {
			continue
		}
		for half := range 2 {
			start, end := half*stations/2, (half+1)*stations/2
			indices := make([]int, 0, 2*(end-start))
			for cell := start; cell < end; cell++ {
				base := oldCount + 6*cell + 2*cap
				indices = append(indices, base, base+1)
			}
			patches = append(patches, loftmesh.ProjectedPatch{
				Triangles: indices, Axes: [2]int{1, 2},
			})
		}
	}
	return patches
}
