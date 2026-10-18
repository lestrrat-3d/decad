package stitchrecord

import (
	"fmt"

	"github.com/lestrrat-3d/decad/internal/revolveplan"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
)

// SiblingRevolveSource proves that every stitched source face is an
// identity Unstitch copy of one original revolve sheet and that the weld plan
// only rejoins original edges and vertices. The original mesher's shared
// stations then still describe every welded seam.
func SiblingRevolveSource(b *topo.Body, sp Payload) (*topo.Body, map[*topo.Face]*topo.Face, string) {
	if b.Kind != topo.BodySheet {
		return nil, nil, "a newly welded curved stitch has no open-sheet proof"
	}
	var originalBody *topo.Body
	paired := make(map[*topo.Face]*topo.Face, len(sp.Faces))
	originalEdgeOf := map[*topo.Edge]*topo.Edge{}
	originalVertexOf := map[*topo.Vertex]*topo.Vertex{}
	copiesByOriginalEdge := map[*topo.Edge]map[*topo.Edge]struct{}{}
	for i, copied := range sp.Faces {
		piece := copied.Body
		if piece == nil || len(topobuild.Faces(piece)) != 1 || topobuild.Faces(piece)[0] != copied {
			return nil, nil, "a source is not a single-face unstitched sheet"
		}
		up, ok := piece.Payload.(UnstitchPayload)
		if !ok || up.Xform != r3.Identity() || up.Delta != 0 || up.Face == nil {
			return nil, nil, "an unstitched source has no identity copy proof"
		}
		original := up.Face
		if originalBody == nil {
			originalBody = original.Body
		}
		if originalBody == nil || original.Body != originalBody || sp.LiveFaces[i] == nil {
			return nil, nil, "the unstitched faces have no common original"
		}
		if _, exists := paired[original]; exists {
			return nil, nil, "an original face occurs more than once"
		}
		paired[original] = sp.LiveFaces[i]
		if len(copied.Loops) != len(original.Loops) {
			return nil, nil, "an unstitched face changed its loop layout"
		}
		for li, loop := range copied.Loops {
			oldLoop := original.Loops[li]
			if len(loop.CoEdges) != len(oldLoop.CoEdges) || loop.Outer != oldLoop.Outer {
				return nil, nil, "an unstitched face changed its coedge layout"
			}
			for ci, ce := range loop.CoEdges {
				old := oldLoop.CoEdges[ci]
				if ce.Forward != old.Forward {
					return nil, nil, "an unstitched coedge changed direction"
				}
				if prev, exists := originalEdgeOf[ce.Edge]; exists && prev != old.Edge {
					return nil, nil, "a copied edge has two original identities"
				}
				originalEdgeOf[ce.Edge] = old.Edge
				if copiesByOriginalEdge[old.Edge] == nil {
					copiesByOriginalEdge[old.Edge] = map[*topo.Edge]struct{}{}
				}
				copiesByOriginalEdge[old.Edge][ce.Edge] = struct{}{}
				for _, pair := range [][2]*topo.Vertex{{topobuild.CoEdgeStart(ce), topobuild.CoEdgeStart(old)}, {topobuild.CoEdgeEnd(ce), topobuild.CoEdgeEnd(old)}} {
					if prev, exists := originalVertexOf[pair[0]]; exists && prev != pair[1] {
						return nil, nil, "a copied vertex has two original identities"
					}
					originalVertexOf[pair[0]] = pair[1]
				}
			}
		}
	}
	if originalBody.Kind != topo.BodySheet {
		return nil, nil, "the original is not a revolve sheet"
	}
	if _, ok := originalBody.Payload.(revolveplan.Payload); !ok {
		return nil, nil, "the original has no revolve chording"
	}
	originalFaces := topobuild.Faces(originalBody)
	if len(originalFaces) != len(paired) {
		return nil, nil, "the unstitched set omits an original face"
	}
	for _, f := range originalFaces {
		if _, ok := paired[f]; !ok {
			return nil, nil, "the unstitched set omits an original face"
		}
	}
	classOriginal := map[int]*topo.Vertex{}
	for v, class := range sp.Plan.Table.Classes {
		original, ok := originalVertexOf[v]
		if !ok {
			return nil, nil, "a welded vertex has no original identity"
		}
		if prev, exists := classOriginal[class]; exists && prev != original {
			return nil, nil, "a vertex class merges different original vertices"
		}
		classOriginal[class] = original
	}
	if len(originalVertexOf) != len(sp.Plan.Table.Classes) {
		return nil, nil, "an unstitched vertex is absent from the weld plan"
	}
	groupOriginal := map[int]*topo.Edge{}
	groupCount := map[int]int{}
	for copied, gid := range sp.Plan.Group {
		original, ok := originalEdgeOf[copied]
		if !ok {
			return nil, nil, "a welded edge has no original identity"
		}
		if prev, exists := groupOriginal[gid]; exists && prev != original {
			return nil, nil, "a weld joins different original edges"
		}
		groupOriginal[gid] = original
		groupCount[gid]++
	}
	for gid, count := range groupCount {
		if count != 2 {
			return nil, nil, fmt.Sprintf("weld group %d has %d source edges", gid, count)
		}
	}
	for original, copies := range copiesByOriginalEdge {
		if len(original.Faces) == 2 {
			if len(copies) != 2 {
				return nil, nil, "an original two-face edge lacks two copies"
			}
			gid := -1
			for copied := range copies {
				got, ok := sp.Plan.Group[copied]
				if !ok || (gid >= 0 && got != gid) {
					return nil, nil, "an original two-face edge was not re-welded"
				}
				gid = got
			}
			continue
		}
		if len(copies) != 1 {
			return nil, nil, "an original free edge has multiple copies"
		}
		for copied := range copies {
			if _, welded := sp.Plan.Group[copied]; welded {
				return nil, nil, "an original free edge was newly welded"
			}
		}
	}
	return originalBody, paired, ""
}
