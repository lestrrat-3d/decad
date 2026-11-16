package brepmodify

import (
	"slices"

	"github.com/lestrrat-3d/r3"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// heldFace reads the held mesh boundary of one face of a faceted body, for the loop edges that carry no exact curve (a
// FacetedCurve): the polygon a probe tests against must follow the held chain of such an edge, not its chord.
type heldFace struct {
	fp    facetedrecord.Payload
	index int
	// loops are the boundary loops of the face's facets, as indices of fp.Verts.
	loops [][]int
}

// newHeldFace reads the boundary loops of face index of fp. It reports false when they cannot be chained.
func newHeldFace(fp facetedrecord.Payload, index int) (*heldFace, bool) {
	if len(fp.FaceOf) != len(fp.Tris) {
		return nil, false
	}
	loops, err := facetedrecord.BoundaryLoops(fp.Tris, facetedrecord.FacetsOfFace(fp.FaceOf, index))
	if err != nil {
		return nil, false
	}
	return &heldFace{fp: fp, index: index, loops: loops}, true
}

// chain is the held boundary of the face from the position from to the position to, which are two corners of loop. The
// face's boundary leaves each of them along two arcs of its mesh loop; the edge's chain is the one that holds no other
// corner of loop. It reports false when the corners are not on one mesh loop, or when both arcs or neither are clear of
// the other corners, so the edge cannot be told from its neighbours.
func (h *heldFace) chain(loop *topo.Loop, from, to r3.Vec) ([]r3.Vec, bool) {
	var corners []r3.Vec
	for _, ce := range loop.CoEdges {
		corners = append(corners, ce.Edge.Start.Position, ce.Edge.End.Position)
	}
	for _, ml := range h.loops {
		a := slices.IndexFunc(ml, func(v int) bool { return h.fp.Verts[v] == from })
		b := slices.IndexFunc(ml, func(v int) bool { return h.fp.Verts[v] == to })
		if a < 0 || b < 0 || a == b {
			continue
		}
		n := len(ml)
		walk := func(step int) ([]r3.Vec, bool) {
			var out []r3.Vec
			for i := a; ; i = (i + step + n) % n {
				p := h.fp.Verts[ml[i]]
				if i != a && i != b && slices.Contains(corners, p) {
					return nil, false
				}
				out = append(out, p)
				if i == b {
					return out, true
				}
			}
		}
		fwd, okF := walk(1)
		bwd, okB := walk(-1)
		if okF == okB {
			return nil, false
		}
		if okF {
			return fwd, true
		}
		return bwd, true
	}
	return nil, false
}
