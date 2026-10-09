package thickenaxis

import (
	"context"
	"errors"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionaudit"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// AuditRefusal maps a section topology refusal to Thicken's unsupported
// reach, while returning cancellation and existing unsupported errors unchanged.
func AuditRefusal(err error) error {
	if err == nil || errors.Is(err, context.Canceled) || errors.Is(err, context.DeadlineExceeded) ||
		errors.Is(err, decaderr.ErrUnsupported) {
		return err
	}
	return fmt.Errorf(`%w: the thicken offset audit refused: %v`, decaderr.ErrUnsupported, err)
}

// AuditSectionPair proves that the outer and reversed inner boundaries do not
// cross and are strictly nested at the requested offset.
func AuditSectionPair(ctx context.Context, budget *proofbound.WorkBudget, outer, inner momentinput.Profile) error {
	hole, err := offset2d.ReverseLoopRecordContext(ctx, inner.Outer)
	if err != nil {
		return err
	}
	entries, err := sectionaudit.EntriesOf(budget, []sectionrecord.LoopRecord{outer.Outer, hole})
	if err != nil {
		return err
	}
	if err := AuditRefusal(sectionaudit.Crossing(budget, entries)); err != nil {
		return err
	}
	return AuditRefusal(sectionaudit.Nesting(budget, entries, 2))
}

// RibbonProfile certifies the closed section one open axis-parallel walk
// sweeps when thickened. rightSteps and leftSteps are zero for the recorded
// side and one for its offset copy. The generated loop is audited for endpoint
// crossings after its whole-interval proof.
func RibbonProfile(ctx context.Context, chain sectionrecord.ChainRecord, rightSteps, leftSteps int,
	amount float64, budget *proofbound.WorkBudget, work *freeform.FreeformWork, radial *Radial,
) (momentinput.Profile, error) {
	raw := make([]survey2d.SideWalk, len(chain.Segments))
	for i, seg := range chain.Segments {
		if err := ctx.Err(); err != nil {
			return momentinput.Profile{}, err
		}
		if _, ok := seg.(sectionrecord.LineSeg); !ok {
			return momentinput.Profile{}, fmt.Errorf(`%w: the open walk requires line-only axis-parallel segments`, decaderr.ErrUnsupported)
		}
		w, err := boundarywalk.WalkOf(seg, work)
		if err != nil {
			return momentinput.Profile{}, err
		}
		raw[i] = survey2d.SideWalk{SegmentWalk: w, Segs: []int{i}}
	}
	walks, err := boundarywalk.CoalesceChainWalksContext(ctx, raw)
	if err != nil {
		return momentinput.Profile{}, err
	}
	loop, err := RibbonSection(ctx, walks, rightSteps, leftSteps, amount, budget, radial)
	if err != nil {
		return momentinput.Profile{}, err
	}
	section := momentinput.Profile{Outer: loop}
	entries, err := sectionaudit.EntriesOf(budget, []sectionrecord.LoopRecord{section.Outer})
	if err != nil {
		return momentinput.Profile{}, err
	}
	if err := AuditRefusal(sectionaudit.Crossing(budget, entries)); err != nil {
		return momentinput.Profile{}, err
	}
	return section, nil
}
