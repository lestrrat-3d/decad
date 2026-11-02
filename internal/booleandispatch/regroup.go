package booleandispatch

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/boolchain"
	"github.com/lestrrat-3d/decad/internal/booloperand"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// Regroup evaluates chain's plan (boolchain.Run). Step k builds under producer env.Ref+k.
// It returns the result, the number of producer identities its steps used, and ok.
// A non-nil error is cancellation, or a step error for which surface reports true.
// Regroup never writes the document (agent-docs/boolean-chain-regroup-design.md,
// Execution in booleandispatch). hooks and surface must not be nil.
func Regroup(ctx context.Context, chain boolchain.Chain, env topo.Env, hooks Hooks,
	surface func(error) bool) (*topo.Body, provenance.Producer, bool, error) {
	return regroup(ctx, env, hooks, surface, func(steps boolchain.Steps) (*topo.Body, bool, error) {
		return boolchain.Run(ctx, chain, steps)
	})
}

// RegroupUnionThree tries A union (B union C) through two ordinary dispatches.
// Each dispatch retains its own limits; no whole-call meter is shared. It
// rejects every warning, including geometry adjustments. It writes no document.
// bodies, hooks and surface must not contain nil values.
func RegroupUnionThree(ctx context.Context, bodies [3]*topo.Body, env topo.Env, hooks Hooks,
	surface func(error) bool) (*topo.Body, provenance.Producer, bool, error) {
	return regroup(ctx, env, hooks, surface, func(steps boolchain.Steps) (*topo.Body, bool, error) {
		return boolchain.UnionThree(ctx, bodies, steps.Full)
	})
}

// regroup supplies private producer identities and maps dispatch outcomes for
// plans whose steps must not change their operands' denoted sets.
func regroup(ctx context.Context, env topo.Env, hooks Hooks, surface func(error) bool,
	plan func(boolchain.Steps) (*topo.Body, bool, error)) (*topo.Body, provenance.Producer, bool, error) {
	var used provenance.Producer
	envAt := func() topo.Env {
		e := env
		e.Ref += used
		return e
	}
	// outcome maps one step's result to the boolchain contract: a canceled context or
	// a surfaced error is returned, any other failure or a rejected result is a miss.
	outcome := func(ctx context.Context, body *topo.Body, ok bool, err error, exact bool) (*topo.Body, bool, error) {
		if cerr := ctx.Err(); cerr != nil {
			return nil, false, cerr
		}
		if err != nil {
			if surface(err) {
				return nil, false, err
			}
			return nil, false, nil
		}
		if !ok || len(body.OperationWarnings) > 0 {
			return nil, false, nil
		}
		if _, held := booloperand.HeldFloor(body); exact && held {
			return nil, false, nil
		}
		used++
		return body, true, nil
	}
	steps := boolchain.Steps{
		Exact: func(ctx context.Context, op meshbool.OperationKind, x, y *topo.Body) (*topo.Body, bool, error) {
			body, ok, err := Analytic(ctx, op, envAt(), x, y, hooks)
			return outcome(ctx, body, ok, err, true)
		},
		Full: func(ctx context.Context, op meshbool.OperationKind, x, y *topo.Body) (*topo.Body, bool, error) {
			body, err := Body(ctx, op, envAt(), x, y, hooks)
			return outcome(ctx, body, err == nil, err, false)
		},
	}
	body, ok, err := plan(steps)
	if cerr := ctx.Err(); cerr != nil {
		return nil, 0, false, cerr
	}
	if err != nil || !ok {
		return nil, 0, false, err
	}
	return body, used, true, nil
}
