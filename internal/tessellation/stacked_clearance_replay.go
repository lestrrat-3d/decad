package tessellation

import (
	"context"
	"math"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// ReplayStackedSlabClearance reserves the complete section-floor scan,
// loop-pair scan, and temporary storage before running the ordinary scalar
// clearance check. The input cap makes the quadratic reservation finite.
func ReplayStackedSlabClearance[F any](ctx context.Context, rings []StackedRing[F],
	columns []int, budget *proofbound.ReplayBudget) error {
	if budget == nil || len(columns) == 0 || len(columns) > 4096 {
		return decaderr.ErrUnsupported
	}
	var points, pairs uint64
	for i, column := range columns {
		if err := budget.SpendWork(ctx, 1); err != nil {
			return err
		}
		if column < 0 || column >= len(rings) {
			return decaderr.ErrUnsupported
		}
		ring := rings[column]
		if len(ring.Samples) < 3 || len(ring.Samples) > 4096 ||
			math.IsNaN(ring.Sag) || math.IsInf(ring.Sag, 0) || ring.Sag < 0 {
			return decaderr.ErrUnsupported
		}
		for _, earlier := range columns[:i] {
			if err := budget.SpendWork(ctx, 1); err != nil {
				return err
			}
			if earlier == column {
				return decaderr.ErrUnsupported
			}
			pairs += uint64(len(rings[earlier].Samples)) * uint64(len(ring.Samples))
		}
		points += uint64(len(ring.Samples))
		if points > 4096 || pairs > 4096*4096 {
			return decaderr.ErrUnsupported
		}
	}
	// Each pair runs at most four point-to-segment projections and hypot
	// calls. The temporary arrays include dynamic append slack and headers.
	work := 64*points + 256*pairs + 64*uint64(len(columns))
	records := 2*points + 4*uint64(len(columns))
	bytes := 256*points + 256*uint64(len(columns)) + 1024
	if err := budget.ReserveBundle(ctx, work, records, bytes); err != nil {
		return err
	}
	return StackedSlabClearance(ctx, rings, columns, RequireLoopClearance)
}
