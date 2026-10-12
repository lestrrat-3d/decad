package decad

import (
	"fmt"

	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// Cut's authenticated bore wall is kept verbatim. Its same-height radial
// source correspondence remains valid because the new outer cap bands stay
// clear of the entire source bore shell and both cap annuli keep their inner
// loops. The caller audits the rebuilt annuli before publishing the result.
func loftCylinderChamferBoreInheritance(fp facetedPayload, mesh *joinedMesh,
	source loftCylinderChamferSource) error {
	if source.boreGroup < 0 {
		if fp.loftCylinderBore != nil || len(source.inner[0]) != 0 || len(source.inner[1]) != 0 {
			return fmt.Errorf("%w: chamfer has inconsistent no-bore cap records", ErrUnsupported)
		}
		return nil
	}
	record := fp.loftCylinderBore
	if record == nil || source.boreGroup != record.wallGroup ||
		source.boreRadius != record.boreRadius || source.boreBound != record.boreBound ||
		source.boreBound < 0 || proofbound.IsNonFinite(source.boreBound) ||
		source.boreBound > fp.meshBound ||
		proofbound.AbsSumUpper(source.boreRadius, source.boreBound, source.innerInset) >= source.blankRadius {
		return fmt.Errorf("%w: chamfer bore lost its source radial bound", ErrUnsupported)
	}
	if len(mesh.verts) < len(fp.verts) || len(fp.tris) != len(fp.src) ||
		len(mesh.tris) != len(mesh.src) {
		return fmt.Errorf("%w: chamfer bore has incomplete held triangles", ErrUnsupported)
	}
	for cap, ring := range source.inner {
		if len(ring) < 3 || len(ring) != len(record.inner[cap]) {
			return fmt.Errorf("%w: chamfer bore lost its inner cap loop", ErrUnsupported)
		}
		for i, index := range ring {
			if index != record.inner[cap][i] || index < 0 || index >= len(fp.verts) ||
				mesh.verts[index] != fp.verts[index] {
				return fmt.Errorf("%w: chamfer bore changed an inner cap station", ErrUnsupported)
			}
		}
	}
	old := make([][3]int, 0)
	for i, group := range fp.src {
		if group == source.boreGroup {
			old = append(old, fp.tris[i])
		}
	}
	if len(old) == 0 {
		return fmt.Errorf("%w: chamfer bore has no retained wall", ErrUnsupported)
	}
	kept := 0
	for i, group := range mesh.src {
		if group != source.boreGroup {
			continue
		}
		if kept >= len(old) || mesh.tris[i] != old[kept] {
			return fmt.Errorf("%w: chamfer changed a held bore-wall facet", ErrUnsupported)
		}
		for _, index := range mesh.tris[i] {
			if index < 0 || index >= len(fp.verts) || mesh.verts[index] != fp.verts[index] {
				return fmt.Errorf("%w: chamfer changed a held bore-wall vertex", ErrUnsupported)
			}
		}
		kept++
	}
	if kept != len(old) {
		return fmt.Errorf("%w: chamfer dropped a held bore-wall facet", ErrUnsupported)
	}
	return nil
}
