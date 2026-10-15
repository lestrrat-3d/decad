package topobuild

import "github.com/lestrrat-3d/decad/internal/topo"

// The New helpers allocate a node through alloc and fill it from v, keeping
// the Owner the allocator set: a builder writes a node literal the way root
// writes a handle literal, and the node stays the one its handle holds. Body
// has no such helper: its node holds locks, so a builder sets its fields.

// NewFace allocates a face node holding v.
func NewFace(alloc topo.Alloc, v topo.Face) *topo.Face {
	n := alloc.NewFace()
	v.Owner = n.Owner
	*n = v
	return n
}

// NewLoop allocates a loop node holding v.
func NewLoop(alloc topo.Alloc, v topo.Loop) *topo.Loop {
	n := alloc.NewLoop()
	v.Owner = n.Owner
	*n = v
	return n
}

// NewEdge allocates an edge node holding v.
func NewEdge(alloc topo.Alloc, v topo.Edge) *topo.Edge {
	n := alloc.NewEdge()
	v.Owner = n.Owner
	*n = v
	return n
}

// NewVertex allocates a vertex node holding v.
func NewVertex(alloc topo.Alloc, v topo.Vertex) *topo.Vertex {
	n := alloc.NewVertex()
	v.Owner = n.Owner
	*n = v
	return n
}

// Faces returns the nodes of b's faces across all shells, in the order root
// Body.Faces returns their handles, or nil for a body with none.
func Faces(b *topo.Body) []*topo.Face {
	var out []*topo.Face
	for _, l := range b.Lumps {
		for _, s := range l.Shells {
			out = append(out, s.Faces...)
		}
	}
	return out
}

// Edges returns the nodes of n's edges, each once, in the order root
// Body.Edges returns their handles: first seen walking lumps, shells, faces,
// loops and their uses.
func Edges(n *topo.Body) []*topo.Edge {
	seen := map[*topo.Edge]struct{}{}
	var out []*topo.Edge
	for _, l := range n.Lumps {
		for _, s := range l.Shells {
			for _, f := range s.Faces {
				for _, lp := range f.Loops {
					for _, ce := range lp.CoEdges {
						if _, ok := seen[ce.Edge]; ok {
							continue
						}
						seen[ce.Edge] = struct{}{}
						out = append(out, ce.Edge)
					}
				}
			}
		}
	}
	return out
}

// CoEdgeStart returns the node of the vertex where the use n starts, or nil
// for a use with no edge: root CoEdge.Start's answer, as a node.
func CoEdgeStart(n topo.CoEdge) *topo.Vertex {
	if n.Edge == nil {
		return nil
	}
	if n.Forward {
		return n.Edge.Start
	}
	return n.Edge.End
}

// CoEdgeEnd returns the node of the vertex where the use n ends, or nil for a
// use with no edge: root CoEdge.End's answer, as a node.
func CoEdgeEnd(n topo.CoEdge) *topo.Vertex {
	if n.Edge == nil {
		return nil
	}
	if n.Forward {
		return n.Edge.End
	}
	return n.Edge.Start
}
