package decad

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

// Each topology handle holds its internal/topo node by value, and a node that
// another node links to carries its handle in Owner. The helpers here are the
// whole mapping between the two: bornX sets Owner when a handle is born, xHandle
// maps a linked node back to its handle, and xNode gives a handle's node
// (agent-docs/evaluator-design.md §3). Data reads go node to node; a read maps back
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

// edgeNodes returns the nodes of es in order, or nil for an empty es.
func edgeNodes(es []*Edge) []*topo.Edge {
	if len(es) == 0 {
		return nil
	}
	out := make([]*topo.Edge, len(es))
	for i, e := range es {
		out[i] = edgeNode(e)
	}
	return out
}

// bornEdge sets e's node Owner to e and returns e. Every root Edge is born
// through it, so edgeHandle maps any edge node a loop's coedge links back to
// the one handle that holds it.
func bornEdge(e *Edge) *Edge {
	e.n.Owner = e
	return e
}

// edgeHandle returns the handle that holds n, or nil for a nil n. A node whose
// handle was not born through bornEdge is an evaluator invariant failure, and
// the assertion panics.
func edgeHandle(n *topo.Edge) *Edge {
	if n == nil {
		return nil
	}
	return n.Owner.(*Edge) //nolint:forcetypeassert // every edge node is born in its handle; a bare one must fail loudly
}

// edgeNode returns e's node, or nil for a nil e.
func edgeNode(e *Edge) *topo.Edge {
	if e == nil {
		return nil
	}
	return &e.n
}

// coedgeHandle returns the public CoEdge value that holds n. A CoEdge is a
// value with no Owner: nothing links to one, so the mapping is a copy.
func coedgeHandle(n topo.CoEdge) CoEdge { return CoEdge{n: n} }

// coedgeHandles returns the public CoEdge values that hold ns in order, or nil
// for an empty ns, as append([]CoEdge(nil), ...) of the same uses would.
func coedgeHandles(ns []topo.CoEdge) []CoEdge {
	if len(ns) == 0 {
		return nil
	}
	out := make([]CoEdge, len(ns))
	for i, n := range ns {
		out[i] = coedgeHandle(n)
	}
	return out
}

// bornLoop sets l's node Owner to l and returns l. Every root Loop is born
// through it, so loopHandle maps any loop node a face links back to the one
// handle that holds it.
func bornLoop(l *Loop) *Loop {
	l.n.Owner = l
	return l
}

// loopHandle returns the handle that holds n, or nil for a nil n. A node whose
// handle was not born through bornLoop is an evaluator invariant failure, and
// the assertion panics.
func loopHandle(n *topo.Loop) *Loop {
	if n == nil {
		return nil
	}
	return n.Owner.(*Loop) //nolint:forcetypeassert // every loop node is born in its handle; a bare one must fail loudly
}

// loopHandles returns the handles of ns in order, or nil for an empty ns,
// as append([]*Loop(nil), ...) of the same loops would.
func loopHandles(ns []*topo.Loop) []*Loop {
	if len(ns) == 0 {
		return nil
	}
	out := make([]*Loop, len(ns))
	for i, n := range ns {
		out[i] = loopHandle(n)
	}
	return out
}

// loopEdges returns the handles of the edges n's uses traverse, in walk order:
// Loop.Edges's answer for a loop node.
func loopEdges(n *topo.Loop) []*Edge {
	out := make([]*Edge, len(n.CoEdges))
	for i, ce := range n.CoEdges {
		out[i] = edgeHandle(ce.Edge)
	}
	return out
}

// bornFaceShell sets s's node Owner to s and returns s. Every root FaceShell is
// born through it, so faceShellHandle maps any shell node a lump links back to
// the one handle that holds it.
func bornFaceShell(s *FaceShell) *FaceShell {
	s.n.Owner = s
	return s
}

// faceShellHandle returns the handle that holds n, or nil for a nil n. A node
// whose handle was not born through bornFaceShell is an evaluator invariant
// failure, and the assertion panics.
func faceShellHandle(n *topo.FaceShell) *FaceShell {
	if n == nil {
		return nil
	}
	return n.Owner.(*FaceShell) //nolint:forcetypeassert // every shell node is born in its handle; a bare one must fail loudly
}

// faceShellHandles returns the handles of ns in order, or nil for an empty ns,
// as append([]*FaceShell(nil), ...) of the same shells would.
func faceShellHandles(ns []*topo.FaceShell) []*FaceShell {
	if len(ns) == 0 {
		return nil
	}
	out := make([]*FaceShell, len(ns))
	for i, n := range ns {
		out[i] = faceShellHandle(n)
	}
	return out
}

// bornLump sets l's node Owner to l and returns l. Every root Lump is born
// through it, so lumpHandle maps any lump node a body links back to the one
// handle that holds it.
func bornLump(l *Lump) *Lump {
	l.n.Owner = l
	return l
}

// lumpHandle returns the handle that holds n, or nil for a nil n. A node whose
// handle was not born through bornLump is an evaluator invariant failure, and
// the assertion panics.
func lumpHandle(n *topo.Lump) *Lump {
	if n == nil {
		return nil
	}
	return n.Owner.(*Lump) //nolint:forcetypeassert // every lump node is born in its handle; a bare one must fail loudly
}

// lumpHandles returns the handles of ns in order, or nil for an empty ns,
// as append([]*Lump(nil), ...) of the same lumps would.
func lumpHandles(ns []*topo.Lump) []*Lump {
	if len(ns) == 0 {
		return nil
	}
	out := make([]*Lump, len(ns))
	for i, n := range ns {
		out[i] = lumpHandle(n)
	}
	return out
}

// bodyHandles returns the handles of ns in order, or nil for an empty ns,
// as append([]*Body(nil), ...) of the same bodies would.
func bodyHandles(ns []*topo.Body) []*Body {
	if len(ns) == 0 {
		return nil
	}
	out := make([]*Body, len(ns))
	for i, n := range ns {
		out[i] = bodyHandle(n)
	}
	return out
}

// bodyNodes returns the nodes of bs in order, or nil for an empty bs.
func bodyNodes(bs []*Body) []*topo.Body {
	if len(bs) == 0 {
		return nil
	}
	out := make([]*topo.Body, len(bs))
	for i, b := range bs {
		out[i] = bodyNode(b)
	}
	return out
}

// bornDocument sets d's node Owner to d and returns d. Every root Document is
// born through it, so documentHandle maps the document node a body links
// back to the one handle that holds it.
func bornDocument(d *Document) *Document {
	d.n.Owner = d
	return d
}

// documentHandle returns the handle that holds n, or nil for a nil n. A node
// whose handle was not born through bornDocument is an evaluator invariant
// failure, and the assertion panics.
func documentHandle(n *topo.Document) *Document {
	if n == nil {
		return nil
	}
	return n.Owner.(*Document) //nolint:forcetypeassert // every document node is born in its handle; a bare one must fail loudly
}

// documentNode returns d's node, or nil for a nil d.
func documentNode(d *Document) *topo.Document {
	if d == nil {
		return nil
	}
	return &d.n
}

// placePayload re-evaluates p under composed as producer ref of d and returns
// the produced body's handle. It is the one root path into
// topo.Payload.Placed: it builds the environment from d and maps the node
// back. It never commits.
func placePayload(ctx context.Context, p topo.Payload, d *Document, ref producerID, composed r3.Transform) (*Body, error) {
	switch pl := p.(type) {
	case stitchPayload:
		documentHandle(&d.n)
		assertPlacedStitchOwners(pl.Faces, composed)
	case unstitchPayload:
		documentHandle(&d.n)
		assertUnstitchFaceOwners(pl.Face, composed)
	case facetedPayload:
		documentHandle(&d.n)
	}
	return bodyResult(p.Placed(ctx, envOf(&d.n, ref), composed))
}

// handleAlloc is the node allocator root hands internal builders (topo.Alloc):
// every node is born inside a fresh root handle, so its Owner is set before
// any builder stores a pointer to it. It has no state, so the interface
// conversion allocates nothing.
type handleAlloc struct{}

// NewBody returns the node of a fresh born Body handle.
func (handleAlloc) NewBody() *topo.Body { return &bornBody(new(Body)).n }

// NewLump returns the node of a fresh born Lump handle.
func (handleAlloc) NewLump() *topo.Lump { return &bornLump(new(Lump)).n }

// NewFaceShell returns the node of a fresh born FaceShell handle.
func (handleAlloc) NewFaceShell() *topo.FaceShell { return &bornFaceShell(new(FaceShell)).n }

// NewFace returns the node of a fresh born Face handle.
func (handleAlloc) NewFace() *topo.Face { return &bornFace(new(Face)).n }

// NewLoop returns the node of a fresh born Loop handle.
func (handleAlloc) NewLoop() *topo.Loop { return &bornLoop(new(Loop)).n }

// NewEdge returns the node of a fresh born Edge handle.
func (handleAlloc) NewEdge() *topo.Edge { return &bornEdge(new(Edge)).n }

// NewVertex returns the node of a fresh born Vertex handle.
func (handleAlloc) NewVertex() *topo.Vertex { return &bornVertex(new(Vertex)).n }

// envOf is the builder environment root hands an internal builder: the
// document node d (nil for a body built outside any document), the producer
// ref, and handleAlloc.
func envOf(d *topo.Document, ref producerID) topo.Env {
	return topo.Env{Doc: d, Ref: ref, Alloc: handleAlloc{}}
}

// bodyResult is an internal builder's result as a handle: the handle that
// holds n, or the error unchanged.
func bodyResult(n *topo.Body, err error) (*Body, error) {
	if err != nil {
		return nil, err
	}
	return bodyHandle(n), nil
}

// bornMesh sets m's node Owner to m and returns m. Every root Mesh is born
// through it, so meshHandle maps the mesh node a body's tessellation cache
// holds back to the one handle that holds it.
func bornMesh(m *Mesh) *Mesh {
	m.n.Owner = m
	return m
}

// meshHandle returns the handle that holds n, or nil for a nil n. A node whose
// handle was not born through bornMesh is an evaluator invariant failure, and
// the assertion panics.
func meshHandle(n *topo.Mesh) *Mesh {
	if n == nil {
		return nil
	}
	return n.Owner.(*Mesh) //nolint:forcetypeassert // every mesh node is born in its handle; a bare one must fail loudly
}

// meshNode returns m's node, or nil for a nil m.
func meshNode(m *Mesh) *topo.Mesh {
	if m == nil {
		return nil
	}
	return &m.n
}
