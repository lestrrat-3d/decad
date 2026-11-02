package orthoshell

import "github.com/lestrrat-3d/decad/internal/facetedrecord"

// FacetedPart admits an unplaced faceted solid whose vertices have no held
// displacement bound. Such a mesh can be read directly on its exact grid.
func FacetedPart(payload facetedrecord.Payload) (Part, bool) {
	if len(payload.Tris) == 0 {
		return Part{}, false
	}
	for _, bound := range payload.VertexBound {
		if bound != 0 {
			return Part{}, false
		}
	}
	return Part{Verts: payload.Verts, Tris: payload.Tris}, true
}
