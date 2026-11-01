package stackedrecord

import (
	"context"
	"math"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/units"
)

// replayCopyBlindProfile copies supported immutable segment values into new
// slices. The caller must bind the result back to the certified source scene.
func replayCopyBlindProfile(ctx context.Context, profile momentinput.Profile, budget *proofbound.ReplayBudget) (momentinput.Profile, bool, error) {
	var out momentinput.Profile
	if budget == nil {
		return out, false, nil
	}
	copyLoop := func(loop sectionrecord.LoopRecord) (sectionrecord.LoopRecord, bool, error) {
		if len(loop.Segments) == 0 {
			return sectionrecord.LoopRecord{}, false, nil
		}
		if err := budget.ReserveRecords(ctx, uint64(len(loop.Segments))); err != nil {
			return sectionrecord.LoopRecord{}, false, err
		}
		if err := budget.ReserveArray(ctx, uint64(len(loop.Segments)), uint64(unsafe.Sizeof(sectionrecord.CurveSegment(nil)))); err != nil {
			return sectionrecord.LoopRecord{}, false, err
		}
		result := sectionrecord.LoopRecord{Segments: make([]sectionrecord.CurveSegment, len(loop.Segments))}
		finite := func(v float64) bool { return !math.IsNaN(v) && !math.IsInf(v, 0) }
		point := func(p sectionrecord.Point2) bool { return finite(p.U) && finite(p.V) }
		for i := range loop.Segments {
			if err := budget.SpendWork(ctx, 128); err != nil {
				return sectionrecord.LoopRecord{}, false, err
			}
			switch v := loop.Segments[i].(type) {
			case sectionrecord.LineSeg:
				if !point(v.Start) || !point(v.End) || !finite(v.TStart) || !finite(v.TEnd) {
					return sectionrecord.LoopRecord{}, false, nil
				}
			case sectionrecord.CircleSeg:
				if !point(v.Center) || v.Radius.Unit() != units.Millimeter || !finite(v.Radius.Mag()) || v.Radius.Mag() <= 0 || !finite(v.TStart) || !finite(v.TEnd) {
					return sectionrecord.LoopRecord{}, false, nil
				}
			case sectionrecord.ArcSeg:
				if !point(v.Center) || !point(v.Start) || !point(v.End) || !finite(v.TStart) || !finite(v.TEnd) {
					return sectionrecord.LoopRecord{}, false, nil
				}
			default:
				return sectionrecord.LoopRecord{}, false, nil
			}
			result.Segments[i] = loop.Segments[i]
		}
		return result, true, nil
	}
	outer, ok, err := copyLoop(profile.Outer)
	if err != nil || !ok {
		return out, false, err
	}
	if err := budget.ReserveRecords(ctx, uint64(len(profile.Holes))); err != nil {
		return out, false, err
	}
	if err := budget.ReserveArray(ctx, uint64(len(profile.Holes)), uint64(unsafe.Sizeof(sectionrecord.LoopRecord{}))); err != nil {
		return out, false, err
	}
	var holes []sectionrecord.LoopRecord
	if profile.Holes != nil {
		holes = make([]sectionrecord.LoopRecord, len(profile.Holes))
	}
	for i := range profile.Holes {
		if err := budget.SpendWork(ctx, 1); err != nil {
			return out, false, err
		}
		holes[i], ok, err = copyLoop(profile.Holes[i])
		if err != nil || !ok {
			return out, false, err
		}
	}
	return momentinput.Profile{Outer: outer, Holes: holes}, true, nil
}
