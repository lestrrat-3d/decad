package retainedblend

import "github.com/lestrrat-3d/decad/internal/facetedrecord"

// facetsOfFace lists the facets that faceOf maps to face index fi.
func facetsOfFace(faceOf []int, fi int) []int { return facetedrecord.FacetsOfFace(faceOf, fi) }

// boundaryLoops chains the directed boundary edges of the facet set into closed loops of vertex indices
// (facetedrecord.BoundaryLoops).
func boundaryLoops(tris [][3]int, facets []int) ([][]int, error) {
	return facetedrecord.BoundaryLoops(tris, facets)
}
