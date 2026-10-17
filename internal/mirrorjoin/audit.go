package mirrorjoin

import (
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionaudit"
	"github.com/lestrrat-3d/decad/internal/sketchrecord"
)

// AuditJoinedSection is §5.3: the modify §5 audit with an empty blend map,
// in prism-boolean §6's order. The junction falsifier runs on every junction
// of every loop (RB9, ErrUnrecordableProfile); S8 reads each loop's signed
// area — the outer positive, every hole negative (RB3, ErrDegenerate); S7
// refuses a crossing or contact of non-adjacent segments within the
// diameter-anchored floor (RB4); S9 proves every hole inside the outer and
// outside every other hole (RB5/RB6). Every check only refuses.
func AuditJoinedSection(budget *proofbound.WorkBudget, p momentinput.Profile, work *freeform.FreeformWork) error {
	loops := append([]LoopRecord{p.Outer}, p.Holes...)
	for li, loop := range loops {
		joins := make([]sketchrecord.LoopJoin, len(loop.Segments))
		for si, seg := range loop.Segments {
			if err := budget.Step(); err != nil {
				return err
			}
			ends, err := SegmentEnds(seg, work)
			if err != nil {
				return err
			}
			joins[si] = sketchrecord.RecordedJoin(
				ends.Start, ends.End, ends.ComputedStart, ends.ComputedEnd, ends.Closed)
		}
		if err := sketchrecord.FalsifyLoopJoins(fmt.Sprintf("mirror join loop %d", li), joins); err != nil {
			return err
		}
	}
	for li, loop := range loops {
		area, err := sectionaudit.LoopSignedArea(budget, loop)
		if err != nil {
			return err
		}
		if (li == 0) != (area > 0) || area == 0 {
			return fmt.Errorf(`%w: the join's spliced loop %d winds the wrong way or encloses nothing`, decaderr.ErrDegenerate, li)
		}
	}
	segs, err := sectionaudit.EntriesOf(budget, loops)
	if err != nil {
		return err
	}
	if err := sectionaudit.Crossing(budget, segs); err != nil {
		return err
	}
	return sectionaudit.Nesting(budget, segs, len(loops))
}

// Assemble splices a region and audits the resulting recorded section.
func (l Line) Assemble(budget *proofbound.WorkBudget, region Region) (momentinput.Profile, float64, error) {
	work := freeform.NewFreeformWork()
	spliced, err := l.Splice(budget, region)
	if err != nil {
		return momentinput.Profile{}, 0, err
	}
	out := momentinput.Profile{Outer: spliced.Outer, Holes: spliced.Holes}
	if err := AuditJoinedSection(budget, out, work); err != nil {
		return momentinput.Profile{}, 0, err
	}
	return out, spliced.Delta, nil
}
