package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/brepmodify"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// This file is the root's side of agent-docs/brep-modify-design.md's receiver
// dispatch: the commit of a modify op's result and the hooks the node-level
// routes of internal/brepmodify call back into root through.

// publicCurveName names a curve as the public API does, for the refusal that
// prints it.
func publicCurveName(c surfacegeom.Curve) string { return fmt.Sprintf("%T", publicCurve(c)) }

// commitModifyResult commits a modify op's result in place of its receiver,
// after the receiver's liveness and the context are read once more at the
// commit edge (modify §13's atomic commit).
func commitModifyResult(ctx context.Context, b, body *Body) (*Body, error) {
	if err := documentHandle(b.n.Doc).requireLive(b); err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	documentHandle(b.n.Doc).commit(body, b)
	return body, nil
}

// commitResult commits a modify op's result in place of its receiver. A result
// evaluated under several producer identities advances the document past all
// of them; the node routes that build one have already read the context.
func commitResult(ctx context.Context, b *Body, r brepmodify.Result) (*Body, error) {
	body := bodyHandle(r.Body)
	if r.Span > 1 {
		documentHandle(b.n.Doc).commitSpan(body, producerID(r.Span), b)
		return body, nil
	}
	return commitModifyResult(ctx, b, body)
}

// modifyHooks are the root-owned parts of Fillet and Chamfer: the Boolean
// dispatch and mesh pipeline their cutter routes run, and the public names
// their refusals print.
func modifyHooks() brepmodify.Hooks {
	return brepmodify.Hooks{
		Mesher:         pairMeshes{},
		Verified:       verifyAllMesher{},
		AsBooleanError: asBooleanError,
		Boolean: func(ctx context.Context, op meshbool.OperationKind, env topo.Env, a, b *topo.Body) (*topo.Body, error) {
			out, err := booleanBody(ctx, op, bodyHandle(a), bodyHandle(b), env.Ref)
			return bodyNode(out), err
		},
		CurveName: publicCurveName,
	}
}
