package facetedrecord

import "github.com/lestrrat-3d/r3"

// SlabFacetBound returns the largest facet bound over every facet of the held
// mesh that meets the slab of half-width halfWidth about the plane through
// origin with unit normal normal, and whether the mesh carries the per-vertex
// record that states one. A facet's bound is the largest record entry of its
// three corners (agent-docs/faceted-vertex-bounds-design.md §2). A facet meets
// the slab when its corners do not all lie beyond the slab on one side, which
// holds for every facet the plane crosses. A slab that holds no facet gives
// zero.
//
// verts, tris and beta are the mesh's vertices, facets and per-vertex record. A
// record that does not have one entry per vertex is no record.
func SlabFacetBound(verts []r3.Vec, tris [][3]int, beta []float64, origin, normal r3.Vec, halfWidth float64) (float64, bool) {
	if beta == nil || len(beta) != len(verts) {
		return 0, false
	}
	dist := make([]float64, len(verts))
	for i, v := range verts {
		dist[i] = v.Sub(origin).Dot(normal)
	}
	local := 0.0
	for _, tri := range tris {
		d0, d1, d2 := dist[tri[0]], dist[tri[1]], dist[tri[2]]
		if min(d0, d1, d2) > halfWidth || max(d0, d1, d2) < -halfWidth {
			continue
		}
		local = max(local, beta[tri[0]], beta[tri[1]], beta[tri[2]])
	}
	return local, true
}
