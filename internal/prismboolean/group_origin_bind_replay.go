package prismboolean

import (
	"context"
	"math"

	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/prismplacement"
	"github.com/lestrrat-3d/decad/internal/stackedrecord"
)

// ReplayBindGroupOrigins binds replayed per-region source indices to the
// recorded group without relying on region order. Callers authenticate the
// replay lineage and source bodies separately.
func ReplayBindGroupOrigins(ctx context.Context, recorded, replayed stackedrecord.Payload, origins []int,
	budget ReplayCapBudget) ([]int, bool, error) {
	if budget == nil {
		return nil, false, nil
	}
	if err := budget.SpendWork(ctx, 1); err != nil {
		return nil, false, err
	}
	if !recorded.IsGroup() || !replayed.IsGroup() || len(recorded.Interfaces) != 0 || len(replayed.Interfaces) != 0 ||
		recorded.Frame != replayed.Frame || recorded.Xform != replayed.Xform || recorded.SectionDelta != replayed.SectionDelta ||
		math.IsNaN(recorded.SectionDelta) || math.IsInf(recorded.SectionDelta, 0) || recorded.SectionDelta < 0 {
		return nil, false, nil
	}
	ra, rb := recorded.Slabs[0], replayed.Slabs[0]
	if ra.Z0 != rb.Z0 || ra.Z1 != rb.Z1 || ra.Z0Delta != rb.Z0Delta || ra.Z1Delta != rb.Z1Delta {
		return nil, false, nil
	}
	n := len(ra.Regions)
	if n > 4096 || len(rb.Regions) != n || len(origins) != n {
		return nil, false, nil
	}
	if err := budget.ReserveRecords(ctx, uint64(2*n)); err != nil {
		return nil, false, err
	}
	if err := budget.ReserveArray(ctx, uint64(n), 8); err != nil {
		return nil, false, err
	}
	if err := budget.ReserveArray(ctx, uint64(n), 1); err != nil {
		return nil, false, err
	}
	result := make([]int, n)
	used := make([]bool, n)
	a := prismplacement.Operand{Frame: recorded.Frame, Xform: recorded.Xform, Z0: ra.Z0, Z1: ra.Z1, Z0Delta: ra.Z0Delta, Z1Delta: ra.Z1Delta, Z0Denoted: ra.Z0Denoted, Z1Denoted: ra.Z1Denoted}
	b := prismplacement.Operand{Frame: replayed.Frame, Xform: replayed.Xform, Z0: rb.Z0, Z1: rb.Z1, Z0Delta: rb.Z0Delta, Z1Delta: rb.Z1Delta, Z0Denoted: rb.Z0Denoted, Z1Denoted: rb.Z1Denoted}
	for i, region := range ra.Regions {
		match := -1
		for j, candidate := range rb.Regions {
			ok, err := ReplayWholeCircleMatch(ctx, region, candidate, a, b, budget)
			if err != nil {
				return nil, false, err
			}
			if !ok {
				continue
			}
			if match != -1 || used[j] || origins[j] < 0 || origins[j] >= n {
				return nil, false, nil
			}
			match = j
		}
		if match < 0 {
			return nil, false, nil
		}
		used[match] = true
		result[i] = origins[match]
	}
	for i, owner := range result {
		for j := range i {
			if err := budget.SpendWork(ctx, 1); err != nil {
				return nil, false, err
			}
			if result[j] == owner {
				return nil, false, nil
			}
		}
	}
	return result, true, nil
}

// ReplayAdvanceGroupOrigins carries one whole-circle Union step's scene tags
// back to the ordered original source indices. A changed, duplicate or missing
// source is refused; output region order may differ from input order.
func ReplayAdvanceGroupOrigins(ctx context.Context, previous []int, rows [][]prismcells.Origin,
	nextSource int, budget ReplayCapBudget) ([]int, bool, error) {
	if budget == nil {
		return nil, false, nil
	}
	if err := budget.SpendWork(ctx, 1); err != nil {
		return nil, false, err
	}
	if nextSource != len(previous) || len(rows) != len(previous)+1 || len(rows) > 4096 {
		return nil, false, nil
	}
	if err := budget.ReserveRecords(ctx, uint64(len(rows))); err != nil {
		return nil, false, err
	}
	if err := budget.ReserveArray(ctx, uint64(len(rows)), 8); err != nil {
		return nil, false, err
	}
	result := make([]int, len(rows))
	for i, row := range rows {
		if err := budget.SpendWork(ctx, 1); err != nil {
			return nil, false, err
		}
		if len(row) != 1 || row[0].Hole != -1 || row[0].Segment != 0 {
			return nil, false, nil
		}
		origin := row[0]
		owner := nextSource
		if origin.IsB {
			if origin.Region != 0 {
				return nil, false, nil
			}
		} else {
			if origin.Region < 0 || origin.Region >= len(previous) {
				return nil, false, nil
			}
			owner = previous[origin.Region]
			if owner < 0 || owner >= nextSource {
				return nil, false, nil
			}
		}
		for j := range i {
			if err := budget.SpendWork(ctx, 1); err != nil {
				return nil, false, err
			}
			if result[j] == owner {
				return nil, false, nil
			}
		}
		result[i] = owner
	}
	return result, true, nil
}
