package loftcylinder

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// ChamferCrossingAudit checks that the rebuilt shell's caps tile their
// loops and that no two wall facets cross. A planar cap tiles one simple
// outer loop and, for a bore, one nested hole.
// Every other triangle meets its plane only at a loop vertex or full loop
// edge. Thus all cap pairs have exactly the indexed contact of that tiling;
// the exact pair audit only needs the non-cap triangles. This avoids the
// quadratic box scan among the many coplanar cap triangles.
func ChamferCrossingAudit(ctx context.Context, mesh *facetedrecord.JoinedMesh,
	source ChamferSource, outer [2][]int, maxTriangles int) error {
	if len(mesh.Tris) > maxTriangles || len(mesh.Src) != len(mesh.Tris) ||
		source.Z0 >= source.Z1 || source.CapGroups[0] == source.CapGroups[1] {
		return fmt.Errorf("%w: cap chamfer has no bounded shell partition", decaderr.ErrUnsupported)
	}
	var vertices [2]map[int]struct{}
	var edges [2]map[[2]int]struct{}
	for cap := range 2 {
		z := source.Z0
		if cap == 1 {
			z = source.Z1
		}
		if !BoreRingSimple(ctx, mesh.Verts, outer[cap]) {
			return fmt.Errorf("%w: cap %d has no simple outer loop", decaderr.ErrUnsupported, cap)
		}
		if len(source.Inner[cap]) > 0 {
			if !BoreRingSimple(ctx, mesh.Verts, source.Inner[cap]) {
				return fmt.Errorf("%w: cap %d has no simple bore loop", decaderr.ErrUnsupported, cap)
			}
			nested, err := ChamferRingNested(ctx, mesh.Verts, outer[cap], source.Inner[cap])
			if err != nil {
				return err
			}
			if !nested || !AuditAnnulus(ctx, mesh.Verts, mesh.Tris, mesh.Src, outer[cap],
				source.Inner[cap], source.CapGroups[cap], z, cap == 0) {
				return fmt.Errorf("%w: cap %d does not tile a nested annulus", decaderr.ErrUnsupported, cap)
			}
		} else if !AuditCap(ctx, mesh.Verts, mesh.Tris, mesh.Src, outer[cap],
			source.CapGroups[cap], z, cap == 0) {
			return fmt.Errorf("%w: cap %d does not tile its loop", decaderr.ErrUnsupported, cap)
		}
		vertices[cap] = make(map[int]struct{}, len(outer[cap])+len(source.Inner[cap]))
		edges[cap] = make(map[[2]int]struct{}, len(outer[cap])+len(source.Inner[cap]))
		for _, ring := range [][]int{outer[cap], source.Inner[cap]} {
			for i, a := range ring {
				b := ring[(i+1)%len(ring)]
				vertices[cap][a] = struct{}{}
				edges[cap][ChamferEdge(a, b)] = struct{}{}
			}
		}
	}
	wall := make([][3]int, 0, len(mesh.Tris))
	for i, tri := range mesh.Tris {
		if i%256 == 0 && ctx.Err() != nil {
			return ctx.Err()
		}
		if mesh.Src[i] == source.CapGroups[0] || mesh.Src[i] == source.CapGroups[1] {
			continue
		}
		wall = append(wall, tri)
		for cap := range 2 {
			z := source.Z0
			if cap == 1 {
				z = source.Z1
			}
			var on []int
			for _, index := range tri {
				if index < 0 || index >= len(mesh.Verts) ||
					mesh.Verts[index].Z < source.Z0 || mesh.Verts[index].Z > source.Z1 {
					return fmt.Errorf("%w: cap chamfer wall leaves its axial slab", decaderr.ErrUnsupported)
				}
				if mesh.Verts[index].Z == z {
					if _, ok := vertices[cap][index]; !ok {
						return fmt.Errorf("%w: cap chamfer wall meets a cap away from its loop", decaderr.ErrUnsupported)
					}
					on = append(on, index)
				}
			}
			if len(on) == 3 {
				return fmt.Errorf("%w: cap chamfer wall lies in a cap plane", decaderr.ErrUnsupported)
			}
			if len(on) == 2 {
				if _, ok := edges[cap][ChamferEdge(on[0], on[1])]; !ok {
					return fmt.Errorf("%w: cap chamfer wall has no shared cap edge", decaderr.ErrUnsupported)
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
