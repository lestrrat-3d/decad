package prismcells

import (
	"math"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionaudit"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// cutToolApart is §5.4's stability audit over every pair of a tool segment and
// a target segment with a nonzero h (agent-docs/carrier-record-design.md §6.3,
// ASA1: a clean-nesting match shares no vertex). It requires
// 4·max(h, c) < F over every segment and G > F for each pair.
func cutToolApart(budget *proofbound.WorkBudget, target, tool prismrecord.Payload,
	tr prismrecord.CarrierRecord, _ SceneDelta) (bool, error) {
	var all []sectionrecord.LoopRecord
	all = append(all, target.Profile.Outer)
	all = append(all, target.Profile.Holes...)
	all = append(all, tool.Profile.Outer)
	all = append(all, tool.Profile.Holes...)
	entries, err := sectionaudit.EntriesOf(budget, all)
	if err != nil {
		return false, err
	}
	floor, err := sectionaudit.ContactFloor(budget, entries)
	if err != nil {
		return false, err
	}
	type walked struct {
		w    survey2d.SegmentWalk
		c, h float64
	}
	var tgt, tl []walked
	for l, loop := range append([]sectionrecord.LoopRecord{target.Profile.Outer}, target.Profile.Holes...) {
		for j, seg := range loop.Segments {
			w, err := boundarywalk.WalkOf(seg, nil)
			if err != nil {
				return false, nil //nolint:nilerr // an unwalkable segment is not audited: no record
			}
			tgt = append(tgt, walked{w: w, c: loopWalkAllowance(loop, j, w), h: tr.SegmentH(l, j)})
		}
	}
	for _, loop := range append([]sectionrecord.LoopRecord{tool.Profile.Outer}, tool.Profile.Holes...) {
		for j, seg := range loop.Segments {
			w, err := boundarywalk.WalkOf(seg, nil)
			if err != nil {
				return false, nil //nolint:nilerr // see above
			}
			tl = append(tl, walked{w: w, c: loopWalkAllowance(loop, j, w)})
		}
	}
	for _, e := range append(append([]walked{}, tgt...), tl...) {
		if !(proofbound.ProductUpper(4, math.Max(e.h, e.c)) < floor) {
			return false, nil
		}
	}
	for _, g := range tgt {
		if g.h == 0 {
			continue
		}
		for _, t := range tl {
			if err := budget.Step(); err != nil {
				return false, err
			}
			if !GapAbove(t.w, g.w, t.c, g.c, floor) {
				return false, nil
			}
		}
	}
	return true, nil
}
