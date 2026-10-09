package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/thickenaxis"
)

func thickenAxisSection(ctx context.Context, profile profileRecord, side ThickenSide,
	amount float64, budget *proofbound.WorkBudget, radial *thickenRadial) (thickenSection, error) {
	for _, seg := range profile.Outer.Segments {
		if err := ctx.Err(); err != nil {
			return thickenSection{}, err
		}
		if _, ok := seg.(lineSeg); !ok {
			return thickenSection{}, fmt.Errorf(`%w: the sheet requires line-only axis-parallel walks`, ErrUnsupported)
		}
	}
	loops, err := prismCornerLoopsBudget(budget, prismPayload{profile: profile})
	if err != nil {
		return thickenSection{}, err
	}
	if len(loops) != 1 {
		return thickenSection{}, fmt.Errorf(`%w: the sheet requires one outer loop`, ErrUnsupported)
	}
	loop := loops[0]
	dirs, err := thickenaxis.AxisDirections(loop.walks, budget)
	if err != nil {
		return thickenSection{}, err
	}
	sec := thickenSection{source: profile, outer: profile, inner: profile}
	if side != ThickenNegative {
		if sec.outer, err = thickenAxisOffset(budget, profile, loop, dirs, -1, amount); err != nil {
			return thickenSection{}, err
		}
	}
	if side != ThickenPositive {
		if sec.inner, err = thickenAxisOffset(budget, profile, loop, dirs, +1, amount); err != nil {
			return thickenSection{}, err
		}
	}
	if side != ThickenNegative {
		if err := thickenaxis.AxisIntervalClear(ctx, loop.walks, dirs, -1, amount, budget, radial); err != nil {
			return thickenSection{}, err
		}
	}
	if side != ThickenPositive {
		if err := thickenaxis.AxisIntervalClear(ctx, loop.walks, dirs, +1, amount, budget, radial); err != nil {
			return thickenSection{}, err
		}
	}
	if err := thickenaxis.AuditSectionPair(ctx, budget, sec.outer, sec.inner); err != nil {
		return thickenSection{}, err
	}
	return sec, nil
}

func thickenAxisOffset(budget *proofbound.WorkBudget, source profileRecord, loop cornerLoop,
	dirs []thickenaxis.AxisDir, sense int, amount float64) (profileRecord, error) {
	offset, err := offsetProfile(budget, source, float64(sense), amount)
	if err != nil {
		return profileRecord{}, thickenaxis.AuditRefusal(err)
	}
	if err := thickenaxis.CertifyAxisOffset(loop.walks, dirs, offset.Outer, sense, amount, budget); err != nil {
		return profileRecord{}, err
	}
	if err := auditOffsetSectionBudget(budget, source, offset); err != nil {
		return profileRecord{}, thickenaxis.AuditRefusal(err)
	}
	return offset, nil
}
