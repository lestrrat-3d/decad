package booleaneval

import (
	"github.com/lestrrat-3d/r3"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/surfaceplace"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
)

// spanningPieces marks a face in facePieces whose facets lie in more than one
// connected piece of the mesh.
const spanningPieces = -1

// facePieces maps each source face of m to the connected piece of pc its facets
// lie in, or to spanningPieces when they lie in several. A face with no facet in
// m has no entry.
func facePieces(m *topo.Mesh, pc pieces) map[*topo.Face]int {
	out := map[*topo.Face]int{}
	for f := range m.Triangles {
		if f >= len(m.Source) || m.Source[f] == nil {
			continue
		}
		src, c := m.Source[f], pc.ofFacet[f]
		prev, seen := out[src]
		switch {
		case !seen:
			out[src] = c
		case prev != c:
			out[src] = spanningPieces
		}
	}
	return out
}

// tagReach bounds the distance of any true boundary point of m from a held
// vertex: the largest vertex bound or mesh bound, plus the mesh bound, as a
// placement reads it (facetedrecord.PlacedTagDeparture).
func tagReach(m *topo.Mesh) (float64, error) {
	beta, err := topobuild.VertexBounds(m)
	if err != nil {
		return 0, err
	}
	reach := m.Bound
	for _, b := range beta {
		reach = max(reach, b)
	}
	return proofbound.AbsSumUpper(reach, m.Bound), nil
}

// movedGroup returns the group of a face of the second operand on a shifted
// retry (agent-docs/gap-tolerance-design.md §5.2), g being the group SourceTag
// read from the unmoved face. piece is the face's entry in facePieces, and ok
// whether it has one. verts and reach are the unmoved operand mesh's vertices
// and tagReach.
//
// The true solid of the result is A∘(B+s), so a face of a piece translated by s
// lies on its tag translated by s, never on the unmoved tag. A face of a piece
// no shift names keeps g. A face of a translated piece carries the translated
// tag, charged as a placement by the same translation
// (facetedrecord.PlacedTagDeparture): the charge is zero when every position
// parameter of the tag plus s is exactly representable, and otherwise the
// rounding of the moved origin. Every other face keeps its provenance and no
// tag: a grown piece (not a translation), a face spanning pieces, a tag that is
// not retained, a denoted patch, a Cone and a placement charge that is not
// stated.
func movedGroup(g facetedrecord.Group, piece int, ok bool, shifts map[int]pieceShift, verts []r3.Vec,
	reach float64) facetedrecord.Group {
	sh, moved := shifts[piece]
	return movedGroupWithShift(g, piece, ok, sh, moved, verts, reach)
}
func movedGroupWithShift(g facetedrecord.Group, piece int, ok bool, sh pieceShift, moved bool, verts []r3.Vec, reach float64) facetedrecord.Group {
	bare := facetedrecord.Group{Origins: g.Origins, Upstream: g.Upstream, Source: g.Source}
	if !ok || piece == spanningPieces {
		return bare
	}
	if !moved {
		return g
	}
	if sh.Grow != 0 || g.Surface == nil || !g.Retained || g.Denoted != nil {
		return bare
	}
	delta, err := r3.Translation(sh.Move)
	if err != nil {
		return bare
	}
	tag, err := surfaceplace.TransformSurface(g.Surface, delta)
	if err != nil {
		return bare
	}
	add, stated := facetedrecord.PlacedTagDeparture(g.Surface, tag, delta, verts, reach)
	if !stated {
		return bare
	}
	g.Surface = tag
	g.NormalBound = min(2, proofbound.AbsSumUpper(g.NormalBound, add))
	g.CapRecord = g.CapRecord.Under(delta)
	if g.CapRecord == nil {
		g.AxialDelta, g.HasAxialDelta = 0, false
	}
	return g
}
