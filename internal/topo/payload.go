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

// SimpleProver is implemented by a payload whose recorded body can already
// prove its own boundary does not self-intersect — Table V's leg 4
// (docs/surface-design.md). A payload that does not implement it, and a nil
// payload, prove nothing. The two callers are the sheet-boundary audit, which
// asks it of a body's own payload, and Rule S and Rule P, which ask it of an
// operand's SOURCE payload before folding its faces into a curved-solid
// reading; asking through ProvesSimple keeps them from ever answering it two
// different ways. Payloads are stored by value in Body.Payload; a pointer
// payload is outside the contract, as for Transform and Placed. Admission is sound only in the proving direction: a method
// returns true only on an argument by construction (sketch's simple closed
// region, a build-time crossing audit, a full-turn axis argument), never on a
// residual, and every case it cannot decide answers false.
type SimpleProver interface {
	// ProvesSimple reports whether the payload's body proves leg 4, checking
	// ctx during any long reading.
	ProvesSimple(ctx context.Context) bool
}

// ProvesSimple reports whether p proves its own boundary simple: false for a
// nil payload and for one that does not implement SimpleProver.
func ProvesSimple(ctx context.Context, p Payload) bool {
	sp, ok := p.(SimpleProver)
	return ok && sp.ProvesSimple(ctx)
}
