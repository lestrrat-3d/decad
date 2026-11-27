package booleandispatch

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/boolchain"
	"github.com/lestrrat-3d/decad/internal/booleaneval"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// ContinueLoftCut evaluates stock minus (loft union prism), using only private
// operations. The caller authenticates that its target is stock minus loft.
// A declined proof or moving operation leaves the original route in charge.
func ContinueLoftCut(ctx context.Context, env topo.Env, stock, loft, prism *topo.Body, hooks Hooks,
	surface func(error) bool) (*topo.Body, bool, error) {
	matched, err := booleaneval.SeparatedLoftPrismSections(ctx, loft, prism, proofbound.NewStrictReplayBudget())
	if err != nil {
		if ctx.Err() != nil {
			return nil, false, ctx.Err()
		}
		if surface(err) {
			return nil, false, err
		}
		return nil, false, nil
	}
	if !matched {
		return nil, false, nil
	}
	joined, err := Body(ctx, meshbool.OpUnion, env, loft, prism, hooks)
	if err != nil {
		if ctx.Err() != nil {
			return nil, false, ctx.Err()
		}
		if surface(err) {
			return nil, false, err
		}
		return nil, false, nil
	}
	if !boolchain.WarningsMoveNoGeometry(joined) {
		return nil, false, nil
	}
	env.Ref++
	ctx = booleaneval.WithLoftUnionSource(ctx, joined, loft, prism)
	result, err := Body(ctx, meshbool.OpCut, env, stock, joined, hooks)
	if err != nil {
		if ctx.Err() != nil {
			return nil, false, ctx.Err()
		}
		if surface(err) {
			return nil, false, err
		}
		return nil, false, nil
	}
	if !boolchain.WarningsMoveNoGeometry(result) {
		return nil, false, nil
	}
	return result, true, nil
}
