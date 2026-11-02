package stackedrecord

import (
	"context"
	"strconv"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
)

// replayBlindPatches identifies the floor or ceiling columns of a previously
// authenticated two-slab blind-cut partition. It allocates no topology.
func replayBlindPatches(ctx context.Context, sp Payload, target prismrecord.Payload, section prismcells.ReplayBlindSection,
	inner float64, top bool, columns []WallColumn, rows [][]SlabLoop, budget *proofbound.ReplayBudget) ([]Patch, bool, error) {
	if budget == nil {
		return nil, false, nil
	}
	valid, err := replayValidateBlindGroup(ctx, sp, target, section, inner, top, budget)
	if err != nil || !valid {
		return nil, false, err
	}
	if len(rows) != 2 {
		return nil, false, nil
	}
	count := len(section.HoleRegions)
	if uint64(count) > ^uint64(0)/5 {
		return nil, false, nil
	}
	if err := budget.ReserveRecords(ctx, uint64(count)*5); err != nil {
		return nil, false, err
	}
	if err := budget.ReserveArray(ctx, uint64(count), uint64(unsafe.Sizeof(Patch{}))); err != nil {
		return nil, false, err
	}
	for range count {
		if err := budget.ReserveArray(ctx, 1, uint64(unsafe.Sizeof(PatchLoop{}))); err != nil {
			return nil, false, err
		}
		if err := budget.ReserveArray(ctx, 1, uint64(unsafe.Sizeof(sectionrecord.CurveSegment(nil)))); err != nil {
			return nil, false, err
		}
		// Both the decimal conversion and resulting role string are bounded by an
		// integer's maximum decimal width and are charged before formatting.
		if err := budget.ReserveBytes(ctx, 256); err != nil {
			return nil, false, err
		}
	}
	out := make([]Patch, 0, count)
	slab := 1
	exposed := sp.Interfaces[0].LowerExposed
	role := "floor(0,"
	if !top {
		slab, exposed, role = 0, sp.Interfaces[0].UpperExposed, "ceiling(0,"
	}
	for i := range count {
		if err := budget.SpendWork(ctx, 1); err != nil {
			return nil, false, err
		}
		wantLoop := len(target.Profile.Holes) + i + 1
		found := -1
		for j := range rows[slab] {
			if err := budget.SpendWork(ctx, 1); err != nil {
				return nil, false, err
			}
			entry := rows[slab][j]
			if entry.Region != 0 || entry.Loop != wantLoop {
				continue
			}
			if found >= 0 || entry.Column < 0 || entry.Column >= len(columns) {
				return nil, false, nil
			}
			found = entry.Column
		}
		if found < 0 {
			return nil, false, nil
		}
		if err := budget.SpendWork(ctx, 1); err != nil {
			return nil, false, err
		}
		column := columns[found]
		if top && column.Start != slab || !top && column.End != slab {
			return nil, false, nil
		}
		equal, err := replayEqualBlindLoop(ctx, column.Loop, section.Profile.Holes[wantLoop-1], budget)
		if err != nil || !equal {
			return nil, false, err
		}
		if err := budget.SpendWork(ctx, 128); err != nil {
			return nil, false, err
		}
		record := exposed[i]
		record.Outer.Segments = []sectionrecord.CurveSegment{record.Outer.Segments[0]}
		out = append(out, Patch{Role: role + strconv.Itoa(i) + ")", Floor: top, Record: record, Loops: []PatchLoop{{Column: found, Top: !top, Outer: true}}})
	}
	return out, true, nil
}
