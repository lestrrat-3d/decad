package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/thickenaxis"
)

func thickenAxisSection(ctx context.Context, profile profileRecord, side ThickenSide,
	amount float64, budget *proofbound.WorkBudget, radial *thickenRadial) (thickenaxis.SectionPair, error) {
	for _, seg := range profile.Outer.Segments {
		if err := ctx.Err(); err != nil {
			return thickenaxis.SectionPair{}, err
		}
		if _, ok := seg.(lineSeg); !ok {
			return thickenaxis.SectionPair{}, fmt.Errorf(`%w: the sheet requires line-only axis-parallel walks`, ErrUnsupported)
		}
	}
	loops, err := offset2d.ProfileCornerLoopsBudget(budget, profile)
	if err != nil {
		return thickenaxis.SectionPair{}, err
	}
	if len(loops) != 1 {
		return thickenaxis.SectionPair{}, fmt.Errorf(`%w: the sheet requires one outer loop`, ErrUnsupported)
	}
	loop := loops[0]
	dirs, err := thickenaxis.AxisDirections(loop.Walks, budget)
	if err != nil {
		return thickenaxis.SectionPair{}, err
	}
	sec := thickenaxis.NewSectionPair(profile)
	if side != ThickenNegative {
		if sec.Outer, err = thickenaxis.AxisOffset(budget, profile, loop, dirs, -1, amount, shellTol); err != nil {
			return thickenaxis.SectionPair{}, err
		}
	}
	if side != ThickenPositive {
		if sec.Inner, err = thickenaxis.AxisOffset(budget, profile, loop, dirs, +1, amount, shellTol); err != nil {
			return thickenaxis.SectionPair{}, err
		}
	}
	if side != ThickenNegative {
		if err := thickenaxis.AxisIntervalClear(ctx, loop.Walks, dirs, -1, amount, budget, radial); err != nil {
			return thickenaxis.SectionPair{}, err
		}
	}
	if side != ThickenPositive {
		if err := thickenaxis.AxisIntervalClear(ctx, loop.Walks, dirs, +1, amount, budget, radial); err != nil {
			return thickenaxis.SectionPair{}, err
		}
	}
	if err := thickenaxis.AuditSectionPair(ctx, budget, sec.Outer, sec.Inner); err != nil {
		return thickenaxis.SectionPair{}, err
	}
	return sec, nil
}
