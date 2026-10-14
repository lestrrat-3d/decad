package decad

import "github.com/lestrrat-3d/decad/internal/topo"

// Each topology handle holds its internal/topo node by value, and a node that
// another node links to carries its handle in Owner. The helpers here are the
// whole mapping between the two: bornX sets Owner when a handle is born, xHandle
// maps a linked node back to its handle, and xNode gives a handle's node
// (docs/evaluator-design.md §3). Data reads go node to node; a read maps back
// only where a handle leaves the code.

// noCopy makes go vet's copylocks check report a value copy of a handle that
// holds one: the copy's node would keep an Owner naming the original handle.
// Its methods are never called.
type noCopy struct{}

// Lock is the method copylocks looks for.
func (*noCopy) Lock() {}

// Unlock is the method copylocks looks for.
func (*noCopy) Unlock() {}

// bornVertex sets v's node Owner to v and returns v. Every root Vertex is
// born through it, so vertexHandle maps any vertex node an edge links back to the
// one handle that holds it.
func bornVertex(v *Vertex) *Vertex {
	v.n.Owner = v
	return v
}

// vertexHandle returns the handle that holds n, or nil for a nil n. A node whose
// handle was not born through bornVertex is an evaluator invariant failure,
// and the assertion panics.
func vertexHandle(n *topo.Vertex) *Vertex {
	if n == nil {
		return nil
	}
	return n.Owner.(*Vertex) //nolint:forcetypeassert // every vertex node is born in its handle; a bare one must fail loudly
}

// vertexNode returns v's node, or nil for a nil v.
func vertexNode(v *Vertex) *topo.Vertex {
	if v == nil {
		return nil
	}
	return &v.n
}

// bornBody sets b's node Owner to b and returns b. Every root Body is born
// through it, so bodyHandle maps the body node a face links back to the one
// handle that holds it.
func bornBody(b *Body) *Body {
	b.n.Owner = b
	return b
}

// bodyHandle returns the handle that holds n, or nil for a nil n. A node whose
// handle was not born through bornBody is an evaluator invariant failure, and
// the assertion panics.
func bodyHandle(n *topo.Body) *Body {
	if n == nil {
		return nil
	}
	return n.Owner.(*Body) //nolint:forcetypeassert // every body node is born in its handle; a bare one must fail loudly
}

// bodyNode returns b's node, or nil for a nil b.
func bodyNode(b *Body) *topo.Body {
	if b == nil {
		return nil
	}
	return &b.n
}

// bornFace sets f's node Owner to f and returns f. Every root Face is born
// through it, so faceHandle maps any face node an edge links back to the one
// handle that holds it.
func bornFace(f *Face) *Face {
	f.n.Owner = f
	return f
}

// faceHandle returns the handle that holds n, or nil for a nil n. A node whose
// handle was not born through bornFace is an evaluator invariant failure, and
// the assertion panics.
func faceHandle(n *topo.Face) *Face {
	if n == nil {
		return nil
	}
	return n.Owner.(*Face) //nolint:forcetypeassert // every face node is born in its handle; a bare one must fail loudly
}

// faceNode returns f's node, or nil for a nil f.
func faceNode(f *Face) *topo.Face {
	if f == nil {
		return nil
	}
	return &f.n
}

// faceHandles returns the handles of ns in order, or nil for an empty ns,
// as append([]*Face(nil), ...) of the same faces would.
func faceHandles(ns []*topo.Face) []*Face {
	if len(ns) == 0 {
		return nil
	}
	out := make([]*Face, len(ns))
	for i, n := range ns {
		out[i] = faceHandle(n)
	}
	return out
}

// faceNodes returns the nodes of fs in order, or nil for an empty fs.
func faceNodes(fs []*Face) []*topo.Face {
	if len(fs) == 0 {
		return nil
	}
	out := make([]*topo.Face, len(fs))
	for i, f := range fs {
		out[i] = faceNode(f)
	}
	return out
}
