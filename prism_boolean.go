package decad

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// tryPrismBoolean keeps the root-only holed Intersect route ahead of the
// analytic entry, then passes the existing payloads to prismcells.
func tryPrismBoolean(ctx context.Context, op meshbool.OperationKind, a, b *Body) (prismPayload, bool, error) {
	if op == meshbool.OpIntersect {
		if result, ok, err := tryPrismHoledIntersect(ctx, a, b); ok || err != nil {
			return result, ok, err
		}
	}
	return prismcells.TryAnalytic(ctx, op, a.n.Payload, b.n.Payload)
}

// admitPrismPairBudget keeps the root body callers on their original budget.
func admitPrismPairBudget(budget *proofbound.WorkBudget, op meshbool.OperationKind, a, b *Body) (prismPayload, prismPayload, bool, error) {
	return prismcells.AdmitPairBudget(budget, op, a.n.Payload, b.n.Payload)
}
