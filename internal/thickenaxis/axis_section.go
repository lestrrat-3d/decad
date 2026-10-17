package thickenaxis

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
)

// AxisSection certifies the two requested offsets of an axis-parallel sheet
// section. outer and inner are the caller's positive and negative side choices.
func AxisSection(ctx context.Context, profile momentinput.Profile, outer, inner bool,
	amount, tol float64, budget *proofbound.WorkBudget, radial *Radial) (SectionPair, error) {
	for _, seg := range profile.Outer.Segments {
		if err := ctx.Err(); err != nil {
			return SectionPair{}, err
		}
		if _, ok := seg.(sectionrecord.LineSeg); !ok {
			return SectionPair{}, fmt.Errorf(`%w: the sheet requires line-only axis-parallel walks`, decaderr.ErrUnsupported)
		}
	}
	loops, err := offset2d.ProfileCornerLoopsBudget(budget, profile)
	if err != nil {
		return SectionPair{}, err
	}
	if len(loops) != 1 {
		return SectionPair{}, fmt.Errorf(`%w: the sheet requires one outer loop`, decaderr.ErrUnsupported)
	}
	loop := loops[0]
	dirs, err := AxisDirections(loop.Walks, budget)
	if err != nil {
		return SectionPair{}, err
	}
	sec := NewSectionPair(profile)
	if outer {
		if sec.Outer, err = AxisOffset(budget, profile, loop, dirs, -1, amount, tol); err != nil {
			return SectionPair{}, err
		}
	}
	if inner {
		if sec.Inner, err = AxisOffset(budget, profile, loop, dirs, +1, amount, tol); err != nil {
			return SectionPair{}, err
		}
	}
	if outer {
		if err := AxisIntervalClear(ctx, loop.Walks, dirs, -1, amount, budget, radial); err != nil {
			return SectionPair{}, err
		}
	}
	if inner {
		if err := AxisIntervalClear(ctx, loop.Walks, dirs, +1, amount, budget, radial); err != nil {
			return SectionPair{}, err
		}
	}
	if err := AuditSectionPair(ctx, budget, sec.Outer, sec.Inner); err != nil {
		return SectionPair{}, err
	}
	return sec, nil
}
