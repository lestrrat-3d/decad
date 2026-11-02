package boundarywalk

import (
	"context"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// CoalesceWalksNoMergeReplay returns the ordinary coalescer's output only
// when its exact merge predicate cannot be reached for any neighbor pair.
func CoalesceWalksNoMergeReplay(ctx context.Context, walks []survey2d.SideWalk,
	budget *proofbound.ReplayBudget) ([]survey2d.SideWalk, error) {
	if budget == nil {
		return nil, decaderr.ErrUnsupported
	}
	for i := range walks {
		if err := budget.SpendWork(ctx, 32); err != nil {
			return nil, err
		}
		if i > 0 && possibleCollinear(walks[i-1], walks[i]) {
			return nil, decaderr.ErrUnsupported
		}
	}
	if len(walks) > 1 {
		if err := budget.SpendWork(ctx, 32); err != nil {
			return nil, err
		}
		if possibleCollinear(walks[len(walks)-1], walks[0]) {
			return nil, decaderr.ErrUnsupported
		}
	}
	if err := budget.ReserveRecords(ctx, uint64(len(walks))); err != nil {
		return nil, err
	}
	if err := budget.ReserveArray(ctx, uint64(len(walks)), uint64(unsafe.Sizeof(survey2d.SideWalk{}))); err != nil {
		return nil, err
	}
	out := make([]survey2d.SideWalk, len(walks))
	copy(out, walks)
	return out, nil
}
