package tessellation

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// ReplaySymDiff sums already proven occupied-volume terms under the shared
// meter. It does not establish that the supplied terms cover the output.
func ReplaySymDiff(ctx context.Context, terms []float64, budget *proofbound.ReplayBudget) (float64, error) {
	if budget == nil {
		return 0, decaderr.ErrUnsupported
	}
	if err := budget.SpendWork(ctx, 1); err != nil {
		return 0, err
	}
	total := 0.0
	for i := range terms {
		if err := budget.SpendWork(ctx, 16); err != nil {
			return 0, err
		}
		total = proofbound.UpRound(total + proof.Abs(terms[i]))
	}
	if err := ctx.Err(); err != nil {
		return 0, err
	}
	if proofbound.IsNonFinite(total) {
		return 0, decaderr.ErrUnsupported
	}
	return total, nil
}
