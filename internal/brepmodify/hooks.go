package brepmodify

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/booleaneval"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/verifiedmesh"
)

// Hooks are the root-owned parts of a modify op: the Boolean dispatch the
// cutter routes run, the operand mesher and typed-error mapping of the mesh
// pipeline, and the public names refusals print.
type Hooks struct {
	// Mesher meshes each Boolean operand at the pair's chord tolerance.
	Mesher booleaneval.OperandMesher
	// Verified gives builders a body's mesh at VerifyAll.
	Verified verifiedmesh.Mesher
	// AsBooleanError maps an evaluator error to the public typed BooleanError.
	AsBooleanError func(meshbool.OperationKind, error) error
	// Boolean runs the full Boolean dispatch over a and b under env's
	// producer identity, without touching the document.
	Boolean func(ctx context.Context, op meshbool.OperationKind, env topo.Env, a, b *topo.Body) (*topo.Body, error)
	// CurveName names a curve in the public API's vocabulary.
	CurveName func(surfacegeom.Curve) string
}

// Result is a modify op's body and the number of consecutive producer
// identities it was evaluated under, from the environment's on. The caller
// commits the body and advances the document past the whole span; a span of
// zero is one identity.
type Result struct {
	Body *topo.Body
	Span int
}
