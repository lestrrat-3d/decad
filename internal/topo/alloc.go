package topo

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/r3"
)

// Alloc allocates topology nodes. Root implements it: each node is born inside
// its root handle, which sets the node's Owner, so a node pointer a builder
// stores stays the handle's own node. Internal unit tests use
// topotest.Alloc, whose nodes have no Owner.
type Alloc interface {
	NewBody() *Body
	NewLump() *Lump
	NewShell() *Shell
	NewFace() *Face
	NewLoop() *Loop
	NewEdge() *Edge
	NewVertex() *Vertex
}

// Env is what a builder needs from its caller: the document node it mints
// identities through and that owns the result, the producer the result's roles
// are minted under, and the node allocator.
type Env struct {
	Doc   *Document
	Ref   provenance.Producer
	Alloc Alloc
}

// Payload is the evaluator's own record of how a body was built: what
// Transform and Copy re-evaluate under a composed motion
// (docs/evaluator-design.md §8).
type Payload interface {
	// Transform is the accumulated rigid placement.
	Transform() r3.Transform
	// Placed re-evaluates the same record under the composed motion. It never
	// commits: root maps the result to its handle and commits it.
	Placed(ctx context.Context, env Env, composed r3.Transform) (*Body, error)
}
