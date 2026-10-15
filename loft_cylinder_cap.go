package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/loftcylinder"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/r3"
)

// loftCylinderTagCaps gives the two already audited cap groups the result's
// own cap roles and the source blank's exact planar surface. The joined cap
// audit proves each disk tiles its outline; the bore path audits each annulus
// before calling this helper. The mesh's remaining groups keep their
// faceted surface identity.
func loftCylinderTagCaps(pp *facetedPayload, blank *Body, groups [2]int,
	levels [2]float64, ref producerID) error {
	if blank == nil || len(pp.Tris) != len(pp.Src) || groups[0] == groups[1] {
		return fmt.Errorf("%w: loft/cylinder caps have no separate source groups", ErrUnsupported)
	}
	roles := [2]string{provenance.RoleCapStart, provenance.RoleCapEnd}
	for cap, group := range groups {
		if group < 0 || group >= len(pp.Groups) {
			return fmt.Errorf("%w: loft/cylinder cap names no source group", ErrUnsupported)
		}
		var source *Face
		for _, face := range blank.Faces() {
			for _, origin := range face.originRefs() {
				if origin != (provenance.Ref{Producer: blank.n.Origin.Producer, Role: roles[cap]}) {
					continue
				}
				if source != nil {
					return fmt.Errorf("%w: loft/cylinder blank has duplicate cap roles", ErrUnsupported)
				}
				source = face
			}
		}
		if source == nil {
			return fmt.Errorf("%w: loft/cylinder blank has no cap source", ErrUnsupported)
		}
		plane, ok := source.n.Surface.(surfacegeom.Plane)
		if !ok || plane.Frame.Origin().Z != levels[cap] {
			return fmt.Errorf("%w: loft/cylinder blank cap has no matching plane", ErrUnsupported)
		}
		want := float64(1)
		if cap == 0 {
			want = -1
		}
		if plane.Frame.N() != (r3.Vec{Z: want}) {
			return fmt.Errorf("%w: loft/cylinder blank cap has no axial normal", ErrUnsupported)
		}
		count := 0
		for i, tri := range pp.Tris {
			if pp.Src[i] != group {
				continue
			}
			count++
			for _, index := range tri {
				if index < 0 || index >= len(pp.Verts) || pp.Verts[index].Z != levels[cap] {
					return fmt.Errorf("%w: loft/cylinder cap facet leaves its source plane", ErrUnsupported)
				}
			}
		}
		if count == 0 {
			return fmt.Errorf("%w: loft/cylinder cap group has no facets", ErrUnsupported)
		}
		g := &pp.Groups[group]
		g.Origins = append(append([]provenance.Ref(nil), g.Origins...),
			provenance.Ref{Producer: ref, Role: roles[cap]})
		g.Surface = plane
		g.Planar = true
		g.NormalBound = source.n.NormalBound
		g.Reversed = source.n.Reversed
	}
	return nil
}

// loftCylinderTagBridgedTop replaces the blank's exact top Plane tag when
// an original tooth ends below the held cap. The cap role remains intact;
// buildFacetedBody supplies the source-to-held displacement on its surface.
func loftCylinderTagBridgedTop(pp *facetedPayload, topGroup int, axialBridge []float64) {
	for _, shift := range axialBridge {
		if shift > 0 {
			top := &pp.Groups[topGroup]
			top.Surface = nil
			top.Planar = false
			return
		}
	}
}

// loftCylinderAuditAnnulus checks the exact planar area and directed edge
// balance of one rebuilt cap. The old disk audit supplies the outer loop; the
// bore ring and side-clearance gates supply a disjoint inner loop. The whole
// shell crossing audit excludes overlapping or folded cap facets.
func loftCylinderAuditAnnulus(ctx context.Context, mesh *joinedMesh,
	outer, inner []int, group int, z float64, bottom bool) bool {
	return loftcylinder.AuditAnnulus(ctx, mesh.Verts, mesh.Tris, mesh.Src, outer, inner, group, z, bottom)
}

// The cap group must tile exactly one simple CCW outline, with the origin
// strictly inside. A later centered bore may use that outline as its source.
func loftCylinderAuditCap(ctx context.Context, mesh *joinedMesh,
	outline []int, group int, z float64, bottom bool) bool {
	return loftcylinder.AuditCap(ctx, mesh.Verts, mesh.Tris, mesh.Src, outline, group, z, bottom)
}
