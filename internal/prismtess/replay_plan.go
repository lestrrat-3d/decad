package prismtess

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
)

func rectanglePlanReplay(ctx context.Context, pp prismrecord.Payload, verifyAll bool,
	countFloor func(int, int) int, freeformTarget float64, budget *proofbound.ReplayBudget) error {
	if err := ctx.Err(); err != nil {
		return err
	}
	if pp.SurfaceResult || !verifyAll || countFloor != nil || freeformTarget != 0 ||
		len(pp.Profile.Holes) != 0 || len(pp.Profile.Outer.Segments) != 4 || pp.Walks == nil ||
		len(pp.Walks.Profile.Holes) != 0 || len(pp.Walks.Profile.Outer.Segments) != 4 ||
		len(pp.Walks.Outer) != 4 || len(pp.Walks.Holes) != 0 || !pp.Walks.Metered ||
		pp.Walks.Spent != 0 || pp.Walks.ReconstructionSpent != 0 {
		return fmt.Errorf("%w: bounded prism requires a recorded rectangular solid", decaderr.ErrUnsupported)
	}
	for i, segment := range pp.Profile.Outer.Segments {
		if err := budget.SpendWork(ctx, 1); err != nil {
			return err
		}
		if _, ok := segment.(sectionrecord.LineSeg); !ok {
			return fmt.Errorf("%w: bounded prism requires straight edges", decaderr.ErrUnsupported)
		}
		if _, ok := pp.Walks.Profile.Outer.Segments[i].(sectionrecord.LineSeg); !ok {
			return fmt.Errorf("%w: bounded prism requires straight recorded edges", decaderr.ErrUnsupported)
		}
	}
	// Build's free-form piece scan sees one loop and four line records. Its
	// reusable-walk check compares only those two profiles, each containing
	// four lines; 128 pays for their reflection nodes and scalar comparisons.
	if err := budget.SpendWork(ctx, 1+4+128); err != nil {
		return err
	}
	// Build allocates the free-form counter, one-loop slices, chorded record,
	// and bounded role-formatting state before the separately metered stages.
	return budget.ReserveRecords(ctx, 16)
}
