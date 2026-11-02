package prismcells

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/sketch"
)

// replayGroupLoopMatches compares a whole loop to one source tag set without
// allocating a temporary map. The caller still must prove region contact.
func replayGroupLoopMatches(ctx context.Context, budget *proofbound.ReplayBudget, loop []sketch.BoundaryEdge,
	tags map[sketch.Entity]Origin, isB bool, region, hole int) (bool, error) {
	count := 0
	if uint64(len(tags)) > ^uint64(0)/32 {
		return false, nil
	}
	// Charge the map read, cancellation poll and tag-field comparisons before
	// traversing the map. The loop performs no allocation.
	if err := budget.SpendWork(ctx, 32*uint64(len(tags))); err != nil {
		return false, err
	}
	for _, origin := range tags {
		if err := ctx.Err(); err != nil {
			return false, err
		}
		if origin.IsB == isB && origin.Region == region && origin.Hole == hole {
			count++
		}
	}
	if err := ctx.Err(); err != nil {
		return false, err
	}
	if count == 0 || count != len(loop) {
		return false, nil
	}
	for i := range loop {
		if err := budget.SpendWork(ctx, 32); err != nil {
			return false, err
		}
		edge := loop[i]
		origin, ok := tags[edge.Entity]
		if !ok || edge.Partial || origin.IsB != isB || origin.Region != region || origin.Hole != hole {
			return false, nil
		}
		for j := range i {
			if err := budget.SpendWork(ctx, 8); err != nil {
				return false, err
			}
			if loop[j].Entity == edge.Entity {
				return false, nil
			}
		}
	}
	return true, nil
}

// replayGroupCutMatchLoops proves only the whole-loop selection. Contact and
// geometry checks remain separate and must pass before this result is used.
func replayGroupCutMatchLoops(ctx context.Context, budget *proofbound.ReplayBudget, profiles []*sketch.Profile,
	tags map[sketch.Entity]Origin, targetHoles, toolRegions int) (*sketch.Profile, bool, error) {
	if budget == nil || targetHoles < 0 || toolRegions < 0 || targetHoles > int(^uint(0)>>1)-toolRegions {
		return nil, false, nil
	}
	n := targetHoles + toolRegions
	var found *sketch.Profile
	for i := range profiles {
		if err := budget.SpendWork(ctx, 1); err != nil {
			return nil, false, err
		}
		profile := profiles[i]
		if profile == nil || len(profile.Holes) != n {
			continue
		}
		ok, err := replayGroupLoopMatches(ctx, budget, profile.Outer, tags, false, 0, -1)
		if err != nil {
			return nil, false, err
		}
		if !ok {
			continue
		}
		all := true
		for h := range profile.Holes {
			if err := budget.SpendWork(ctx, 1); err != nil {
				return nil, false, err
			}
			loop := profile.Holes[h]
			matched := -1
			for k := range n {
				side, region, hole := false, 0, k
				if k >= targetHoles {
					side, region, hole = true, k-targetHoles, -1
				}
				ok, err = replayGroupLoopMatches(ctx, budget, loop, tags, side, region, hole)
				if err != nil {
					return nil, false, err
				}
				if ok {
					matched = k
					break
				}
			}
			if matched < 0 {
				all = false
				break
			}
			// A hole must not claim the same source as any prior hole.
			for j := range h {
				side, region, hole := false, 0, matched
				if matched >= targetHoles {
					side, region, hole = true, matched-targetHoles, -1
				}
				same, err := replayGroupLoopMatches(ctx, budget, profile.Holes[j], tags, side, region, hole)
				if err != nil {
					return nil, false, err
				}
				if same {
					all = false
					break
				}
			}
			if !all {
				break
			}
		}
		if !all {
			continue
		}
		if found != nil {
			return nil, false, nil
		}
		found = profile
	}
	return found, found != nil, nil
}
