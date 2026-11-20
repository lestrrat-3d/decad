package booleandispatch

import (
	"context"
	"errors"

	"github.com/lestrrat-3d/decad/internal/boolchain"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/rimsplit"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// CoarseHeldOperand returns the operand, 0 or 1, whose held facets the local
// chain-depth gate refused in cause (meshbool.CoarseHeldContactError,
// agent-docs/faceted-vertex-bounds-design.md §5). ok is false for any other
// refusal and for cancellation. It classifies the trigger of
// agent-docs/prism-boolean-design.md Part R's rim-split rebuild; it certifies
// no geometry.
func CoarseHeldOperand(cause error) (int, bool) {
	if cause == nil || errors.Is(cause, context.Canceled) || errors.Is(cause, context.DeadlineExceeded) {
		return 0, false
	}
	held, ok := errors.AsType[*meshbool.CoarseHeldContactError](cause)
	if !ok || held.Operand < 0 || held.Operand > 1 {
		return 0, false
	}
	return held.Operand, true
}

// RebuildUnionSplit evaluates Union(a, b) by rebuilding the recorded Union
// history beneath both operands with every mesh Boolean splitting the pieces
// beside its rims (boolchain.RebuildUnion; agent-docs/prism-boolean-design.md
// Part R, "Rim-split rebuild"). Step k builds under producer env.Ref+k through
// Body. A step whose result carries a warning that moves geometry is a miss,
// so the result denotes Union(a, b). It returns the result, the number of
// producer identities its steps used, and ok. A non-nil error is cancellation,
// or a step error for which surface reports true. It never writes the
// document. a, b, src, hooks and surface must not be nil.
func RebuildUnionSplit(ctx context.Context, a, b *topo.Body, src boolchain.Source, env topo.Env, hooks Hooks,
	surface func(error) bool) (*topo.Body, provenance.Producer, bool, error) {
	ctx = rimsplit.With(ctx, true)
	var used provenance.Producer
	full := func(ctx context.Context, op meshbool.OperationKind, x, y *topo.Body) (*topo.Body, bool, error) {
		e := env
		e.Ref += used
		body, err := Body(ctx, op, e, x, y, hooks)
		if cerr := ctx.Err(); cerr != nil {
			return nil, false, cerr
		}
		if err != nil {
			if surface(err) {
				return nil, false, err
			}
			return nil, false, nil
		}
		if !boolchain.WarningsMoveNoGeometry(body) {
			return nil, false, nil
		}
		used++
		return body, true, nil
	}
	body, ok, err := boolchain.RebuildUnion(ctx, a, b, src, full)
	if cerr := ctx.Err(); cerr != nil {
		return nil, 0, false, cerr
	}
	if err != nil || !ok {
		return nil, 0, false, err
	}
	return body, used, true, nil
}
