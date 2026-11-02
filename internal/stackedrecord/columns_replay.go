package stackedrecord

import (
	"context"
	"math"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/units"
)

// replayBlindColumns groups the supported whole-curve blind-cut loops without
// allocating intermediate loop lists. It refuses duplicate loops within a slab.
func replayBlindColumns(ctx context.Context, sp Payload, budget *proofbound.ReplayBudget) ([]WallColumn, [][]SlabLoop, bool, error) {
	if budget == nil {
		return nil, nil, false, nil
	}
	if err := budget.SpendWork(ctx, 1); err != nil {
		return nil, nil, false, err
	}
	if len(sp.Slabs) != 2 {
		return nil, nil, false, nil
	}
	counts := [2]int{}
	total := 0
	for k := range sp.Slabs {
		if err := budget.SpendWork(ctx, 1); err != nil {
			return nil, nil, false, err
		}
		slab := sp.Slabs[k]
		for r := range slab.Regions {
			if err := budget.SpendWork(ctx, 1); err != nil {
				return nil, nil, false, err
			}
			region := slab.Regions[r]
			if len(region.Holes) > math.MaxInt-counts[k]-1 {
				return nil, nil, false, nil
			}
			counts[k] += 1 + len(region.Holes)
		}
		if counts[k] > math.MaxInt-total {
			return nil, nil, false, nil
		}
		total += counts[k]
	}
	if uint64(total) > (^uint64(0)-2)/2 {
		return nil, nil, false, nil
	}
	if err := budget.ReserveRecords(ctx, uint64(total)*2+2); err != nil {
		return nil, nil, false, err
	}
	if err := budget.ReserveArray(ctx, uint64(total), uint64(unsafe.Sizeof(WallColumn{}))); err != nil {
		return nil, nil, false, err
	}
	if err := budget.ReserveArray(ctx, 2, uint64(unsafe.Sizeof([]SlabLoop{}))); err != nil {
		return nil, nil, false, err
	}
	for _, count := range counts {
		if err := budget.ReserveArray(ctx, uint64(count), uint64(unsafe.Sizeof(SlabLoop{}))); err != nil {
			return nil, nil, false, err
		}
	}
	columns := make([]WallColumn, 0, total)
	bySlab := make([][]SlabLoop, 2)
	for k := range bySlab {
		bySlab[k] = make([]SlabLoop, 0, counts[k])
	}
	for k := range sp.Slabs {
		if err := budget.SpendWork(ctx, 1); err != nil {
			return nil, nil, false, err
		}
		for r := range sp.Slabs[k].Regions {
			if err := budget.SpendWork(ctx, 1); err != nil {
				return nil, nil, false, err
			}
			region := sp.Slabs[k].Regions[r]
			for i := 0; i <= len(region.Holes); i++ {
				if err := budget.SpendWork(ctx, 1); err != nil {
					return nil, nil, false, err
				}
				loop := region.Outer
				if i > 0 {
					loop = region.Holes[i-1]
				}
				if len(loop.Segments) == 0 {
					return nil, nil, false, nil
				}
				// Copy each admitted value before retaining it in a column. The
				// private column no longer aliases the source segment slice.
				if err := budget.ReserveRecords(ctx, uint64(len(loop.Segments))); err != nil {
					return nil, nil, false, err
				}
				if err := budget.ReserveArray(ctx, uint64(len(loop.Segments)), uint64(unsafe.Sizeof(sectionrecord.CurveSegment(nil)))); err != nil {
					return nil, nil, false, err
				}
				sealed := sectionrecord.LoopRecord{Segments: make([]sectionrecord.CurveSegment, len(loop.Segments))}
				finite := func(v float64) bool { return !math.IsNaN(v) && !math.IsInf(v, 0) }
				point := func(p sectionrecord.Point2) bool { return finite(p.U) && finite(p.V) }
				for j := range loop.Segments {
					if err := budget.SpendWork(ctx, 128); err != nil {
						return nil, nil, false, err
					}
					switch value := loop.Segments[j].(type) {
					case sectionrecord.LineSeg:
						if !point(value.Start) || !point(value.End) || !finite(value.TStart) || !finite(value.TEnd) {
							return nil, nil, false, nil
						}
					case sectionrecord.CircleSeg:
						if !point(value.Center) || value.Radius.Unit() != units.Millimeter || !finite(value.Radius.Mag()) || value.Radius.Mag() <= 0 || !finite(value.TStart) || !finite(value.TEnd) {
							return nil, nil, false, nil
						}
					case sectionrecord.ArcSeg:
						if !point(value.Center) || !point(value.Start) || !point(value.End) || !finite(value.TStart) || !finite(value.TEnd) {
							return nil, nil, false, nil
						}
					default:
						return nil, nil, false, nil
					}
					sealed.Segments[j] = loop.Segments[j]
				}
				loop = sealed
				for j := range bySlab[k] {
					if err := budget.SpendWork(ctx, 1); err != nil {
						return nil, nil, false, err
					}
					previous := bySlab[k][j].Column
					equal, err := replayEqualBlindLoop(ctx, loop, columns[previous].Loop, budget)
					if err != nil {
						return nil, nil, false, err
					}
					if equal {
						return nil, nil, false, nil
					}
				}
				found := -1
				if k > 0 {
					for j := range bySlab[k-1] {
						if err := budget.SpendWork(ctx, 1); err != nil {
							return nil, nil, false, err
						}
						previous := bySlab[k-1][j].Column
						equal, err := replayEqualBlindLoop(ctx, loop, columns[previous].Loop, budget)
						if err != nil {
							return nil, nil, false, err
						}
						if equal {
							found = previous
							break
						}
					}
				}
				if found < 0 {
					found = len(columns)
					columns = append(columns, WallColumn{Column: Column{Loop: loop, Start: k, End: k, Region: r, LoopIndex: i}}) //nolint:modernize // Named fields leave the remaining fields zero.
				} else if columns[found].End == k {
					return nil, nil, false, nil
				} else {
					columns[found].End = k
				}
				bySlab[k] = append(bySlab[k], SlabLoop{Column: found, Region: r, Loop: i})
			}
		}
	}
	return columns, bySlab, true, nil
}
