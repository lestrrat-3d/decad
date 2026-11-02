package stackedrecord

import (
	"context"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
)

// replayBlindSourceWalks binds the original target columns to the analytic
// walks retained by their producer. New cut holes have no source walk here.
func replayBlindSourceWalks(ctx context.Context, columns []WallColumn, target prismrecord.Payload,
	budget *proofbound.ReplayBudget) ([]*momentinput.ProfileWalks, bool, error) {
	if budget == nil || target.Walks == nil || !target.Walks.Metered ||
		target.Walks.Spent != 0 || target.Walks.ReconstructionSpent != 0 {
		return nil, false, nil
	}
	same, err := replayEqualBlindProfile(ctx, target.Walks.Profile, target.Profile, budget)
	if err != nil || !same {
		return nil, false, err
	}
	if len(target.Walks.Outer) != len(target.Profile.Outer.Segments) ||
		len(target.Walks.Holes) != len(target.Profile.Holes) {
		return nil, false, nil
	}
	for i := range target.Profile.Holes {
		if err := budget.SpendWork(ctx, 1); err != nil {
			return nil, false, err
		}
		if len(target.Walks.Holes[i]) != len(target.Profile.Holes[i].Segments) {
			return nil, false, nil
		}
	}
	if err := budget.ReserveRecords(ctx, uint64(len(columns))); err != nil {
		return nil, false, err
	}
	if err := budget.ReserveArray(ctx, uint64(len(columns)), uint64(unsafe.Sizeof((*momentinput.ProfileWalks)(nil)))); err != nil {
		return nil, false, err
	}
	walks := make([]*momentinput.ProfileWalks, len(columns))
	for i, col := range columns {
		if err := budget.SpendWork(ctx, 1); err != nil {
			return nil, false, err
		}
		if col.LoopIndex > len(target.Profile.Holes) {
			continue
		}
		var source sectionrecord.LoopRecord
		if col.LoopIndex == 0 {
			source = target.Profile.Outer
		} else {
			source = target.Profile.Holes[col.LoopIndex-1]
		}
		same, err := replayEqualBlindLoop(ctx, col.Loop, source, budget)
		if err != nil || !same {
			return nil, false, err
		}
		walks[i] = target.Walks
	}
	return walks, true, nil
}
