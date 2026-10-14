package topo

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/r3"
)

// Env is what a builder needs from its caller: the document node it mints
// level and curve identities through and that owns the result, the producer
// the result's roles are minted under, and the allocator every node of the
// result comes from. Ref is not always Doc.NextProducer: a motion check
// builds under a transient producer, and a pattern builds a span of producers
// before it commits (docs/evaluator-design.md §8).
type Env struct {
	Doc   *Document
	Ref   provenance.Producer
	Alloc Alloc
}

// Payload is the evaluator's own record of how a body was built: what
// Transform and Copy re-evaluate under a composed motion. Each payload carries
// its accumulated rigid placement and rebuilds the same record under a new one
// (docs/evaluator-design.md §8).
type Payload interface {
	// Transform is the accumulated rigid placement.
	Transform() r3.Transform
	// Placed re-evaluates the same record under the composed motion, checking
	// ctx during any long rebuild. It never commits: the caller maps the
	// result to its handle and commits it.
	Placed(ctx context.Context, env Env, composed r3.Transform) (*Body, error)
}
