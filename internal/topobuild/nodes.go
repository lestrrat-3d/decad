package topobuild

import (
	"github.com/lestrrat-3d/decad/internal/selectorquery"
	"github.com/lestrrat-3d/decad/internal/topo"
)

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

// OperandBodies returns the distinct bodies faces were read from, in
// first-seen order, skipping a face that holds no body.
func OperandBodies(faces []*topo.Face) []*topo.Body {
	seen := map[*topo.Body]struct{}{}
	var bodies []*topo.Body
	for _, f := range faces {
		if f.Body == nil {
			continue
		}
		if _, ok := seen[f.Body]; ok {
			continue
		}
		seen[f.Body] = struct{}{}
		bodies = append(bodies, f.Body)
	}
	return bodies
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

// Vertices returns the nodes of n's vertices, each once, in the order root
// Body.Vertices returns their handles: each edge of Edges in turn, its start
// before its end, skipping a nil endpoint.
func Vertices(n *topo.Body) []*topo.Vertex {
	seen := map[*topo.Vertex]struct{}{}
	var out []*topo.Vertex
	for _, e := range Edges(n) {
		for _, v := range [2]*topo.Vertex{e.Start, e.End} {
			if v == nil {
				continue
			}
			if _, ok := seen[v]; ok {
				continue
			}
			seen[v] = struct{}{}
			out = append(out, v)
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

// FacesByRole selects the last face carrying each role from b's own step,
// walking the faces in Faces order.
func FacesByRole(b *topo.Body) map[string]*topo.Face {
	faces := Faces(b)
	faceRoles := make([][]string, len(faces))
	for i, f := range faces {
		for _, o := range f.Origins {
			if o.Producer == b.Origin.Producer {
				faceRoles[i] = append(faceRoles[i], o.Role)
			}
		}
	}
	roles := make(map[string]*topo.Face)
	for role, i := range selectorquery.RoleIndex(faceRoles) {
		roles[role] = faces[i]
	}
	return roles
}

// LoopEdges is the edge of every coedge of l, in order.
func LoopEdges(l *topo.Loop) []*topo.Edge {
	out := make([]*topo.Edge, len(l.CoEdges))
	for i, ce := range l.CoEdges {
		out[i] = ce.Edge
	}
	return out
}
