package tessellation

import (
	"context"
	"math"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// ReplayStoreMax folds finite nonnegative per-vertex displacement bounds under
// the shared replay meter. It does not establish the individual bounds.
func ReplayStoreMax(ctx context.Context, store []float64, budget *proofbound.ReplayBudget) (float64, error) {
	if budget == nil {
		return 0, decaderr.ErrUnsupported
	}
	if err := budget.SpendWork(ctx, 1); err != nil {
		return 0, err
	}
	worst := 0.0
	for i := range store {
		if err := budget.SpendWork(ctx, 8); err != nil {
			return 0, err
		}
		d := store[i]
		if proofbound.IsNonFinite(d) || d < 0 {
			return 0, decaderr.ErrUnsupported
		}
		worst = math.Max(worst, d)
	}
	if err := ctx.Err(); err != nil {
		return 0, err
	}
	return worst, nil
}
