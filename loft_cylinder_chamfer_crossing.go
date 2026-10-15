package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/loftcylinder"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// A planar cap tiles one simple outer loop and, for a bore, one nested hole.
// Every other triangle meets its plane only at a loop vertex or full loop
// edge. Thus all cap pairs have exactly the indexed contact of that tiling;
// the exact pair audit only needs the non-cap triangles. This avoids the
// quadratic box scan among the many coplanar cap triangles.
func loftCylinderChamferCrossingAudit(ctx context.Context, mesh *joinedMesh,
	source loftCylinderChamferSource, outer [2][]int, maxTriangles int) error {
	if len(mesh.Tris) > maxTriangles || len(mesh.Src) != len(mesh.Tris) ||
		source.z0 >= source.z1 || source.capGroups[0] == source.capGroups[1] {
		return fmt.Errorf("%w: cap chamfer has no bounded shell partition", ErrUnsupported)
	}
	var vertices [2]map[int]struct{}
	var edges [2]map[[2]int]struct{}
	for cap := range 2 {
		z := source.z0
		if cap == 1 {
			z = source.z1
		}
		if !loftcylinder.BoreRingSimple(ctx, mesh.Verts, outer[cap]) {
			return fmt.Errorf("%w: cap %d has no simple outer loop", ErrUnsupported, cap)
		}
		if len(source.inner[cap]) > 0 {
			if !loftcylinder.BoreRingSimple(ctx, mesh.Verts, source.inner[cap]) {
				return fmt.Errorf("%w: cap %d has no simple bore loop", ErrUnsupported, cap)
			}
			nested, err := loftcylinder.ChamferRingNested(ctx, mesh.Verts, outer[cap], source.inner[cap])
			if err != nil {
				return err
			}
			if !nested || !loftCylinderAuditAnnulus(ctx, mesh, outer[cap], source.inner[cap],
				source.capGroups[cap], z, cap == 0) {
				return fmt.Errorf("%w: cap %d does not tile a nested annulus", ErrUnsupported, cap)
			}
		} else if !loftCylinderAuditCap(ctx, mesh, outer[cap], source.capGroups[cap], z, cap == 0) {
			return fmt.Errorf("%w: cap %d does not tile its loop", ErrUnsupported, cap)
		}
		vertices[cap] = make(map[int]struct{}, len(outer[cap])+len(source.inner[cap]))
		edges[cap] = make(map[[2]int]struct{}, len(outer[cap])+len(source.inner[cap]))
		for _, ring := range [][]int{outer[cap], source.inner[cap]} {
			for i, a := range ring {
				b := ring[(i+1)%len(ring)]
				vertices[cap][a] = struct{}{}
				edges[cap][loftcylinder.ChamferEdge(a, b)] = struct{}{}
			}
		}
	}
	wall := make([][3]int, 0, len(mesh.Tris))
	for i, tri := range mesh.Tris {
		if i%256 == 0 && ctx.Err() != nil {
			return ctx.Err()
		}
		if mesh.Src[i] == source.capGroups[0] || mesh.Src[i] == source.capGroups[1] {
			continue
		}
		wall = append(wall, tri)
		for cap := range 2 {
			z := source.z0
			if cap == 1 {
				z = source.z1
			}
			var on []int
			for _, index := range tri {
				if index < 0 || index >= len(mesh.Verts) ||
					mesh.Verts[index].Z < source.z0 || mesh.Verts[index].Z > source.z1 {
					return fmt.Errorf("%w: cap chamfer wall leaves its axial slab", ErrUnsupported)
				}
				if mesh.Verts[index].Z == z {
					if _, ok := vertices[cap][index]; !ok {
						return fmt.Errorf("%w: cap chamfer wall meets a cap away from its loop", ErrUnsupported)
					}
					on = append(on, index)
				}
			}
			if len(on) == 3 {
				return fmt.Errorf("%w: cap chamfer wall lies in a cap plane", ErrUnsupported)
			}
			if len(on) == 2 {
				if _, ok := edges[cap][loftcylinder.ChamferEdge(on[0], on[1])]; !ok {
					return fmt.Errorf("%w: cap chamfer wall has no shared cap edge", ErrUnsupported)
				}
			}
		}
	}
	maxScans := uint64(min(64_000_000, 2_048*len(wall)))
	if err := loftmesh.LoftCrossingAuditBounded(proofbound.NewPollWorkBudget(ctx), mesh.Verts, wall,
		maxTriangles, maxScans); err != nil {
		return fmt.Errorf("cap chamfer wall crossing audit (%d facets, %d pair scans): %w",
			len(wall), maxScans, err)
	}
	return nil
}
