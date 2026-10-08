package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/thickenaxis"

	"github.com/lestrrat-3d/decad/internal/survey2d"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

type thickenAxisDir struct{ u, v int }

func thickenAxisSection(ctx context.Context, profile ProfileRecord, side ThickenSide,
	amount float64, budget *proofbound.WorkBudget, radial *thickenRadial) (thickenSection, error) {
	for _, seg := range profile.Outer.Segments {
		if err := ctx.Err(); err != nil {
			return thickenSection{}, err
		}
		if _, ok := seg.(LineSeg); !ok {
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
	dirs, err := thickenAxisDirections(loop, budget)
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
		if err := thickenAxisIntervalClear(ctx, loop, dirs, -1, amount, budget, radial); err != nil {
			return thickenSection{}, err
		}
	}
	if side != ThickenPositive {
		if err := thickenAxisIntervalClear(ctx, loop, dirs, +1, amount, budget, radial); err != nil {
			return thickenSection{}, err
		}
	}
	hole, err := reverseLoopRecordContext(ctx, sec.inner.Outer)
	if err != nil {
		return thickenSection{}, err
	}
	entries, err := buildSegEntriesBudget(budget, []LoopRecord{sec.outer.Outer, hole})
	if err != nil {
		return thickenSection{}, err
	}
	if err := thickenAuditRefusal(crossingAuditBudget(budget, entries)); err != nil {
		return thickenSection{}, err
	}
	if err := thickenAuditRefusal(nestingAuditBudget(budget, entries, 2)); err != nil {
		return thickenSection{}, err
	}
	return sec, nil
}

func thickenAxisDirections(loop cornerLoop, budget *proofbound.WorkBudget) ([]thickenAxisDir, error) {
	dirs, err := thickenaxis.AxisDirections(loop.walks, budget)
	if err != nil {
		return nil, err
	}
	out := make([]thickenAxisDir, len(dirs))
	for i, d := range dirs {
		out[i] = thickenAxisDir{u: d.U(), v: d.V()}
	}
	return out, nil
}

func thickenAxisOffset(budget *proofbound.WorkBudget, source ProfileRecord, loop cornerLoop,
	dirs []thickenAxisDir, sense int, amount float64) (ProfileRecord, error) {
	offset, err := offsetProfile(budget, source, float64(sense), amount)
	if err != nil {
		return ProfileRecord{}, thickenAuditRefusal(err)
	}
	if err := thickenCertifyAxisOffset(loop, dirs, offset.Outer, sense, amount, budget); err != nil {
		return ProfileRecord{}, err
	}
	if err := auditOffsetSectionBudget(budget, source, offset); err != nil {
		return ProfileRecord{}, thickenAuditRefusal(err)
	}
	return offset, nil
}

func thickenCertifyAxisOffset(loop cornerLoop, dirs []thickenAxisDir, generated LoopRecord,
	sense int, amount float64, budget *proofbound.WorkBudget) error {
	axisDirs := make([]thickenaxis.AxisDir, len(dirs))
	for i, d := range dirs {
		axisDirs[i] = thickenaxis.NewAxisDir(d.u, d.v)
	}
	return thickenaxis.CertifyAxisOffset(loop.walks, axisDirs, generated, sense, amount, budget)
}

func thickenAxisIntervalClear(ctx context.Context, loop cornerLoop, dirs []thickenAxisDir,
	sense int, amount float64, budget *proofbound.WorkBudget, radial *thickenRadial) error {
	axisDirs := make([]thickenaxis.AxisDir, len(dirs))
	for i, d := range dirs {
		axisDirs[i] = thickenaxis.NewAxisDir(d.u, d.v)
	}
	return thickenaxis.AxisIntervalClear(ctx, loop.walks, axisDirs, sense, amount, budget, radial)
}

// thickenRibbon certifies the closed section one open axis-parallel walk
// sweeps when it is thickened: the assembled boundary at the requested offset,
// proven simple there and proven free of any nonadjacent contact over the
// whole interval 0 < τ ≤ amount.
func thickenRibbon(ctx context.Context, chain ChainRecord, side ThickenSide, amount float64,
	budget *proofbound.WorkBudget, work *freeform.FreeformWork, radial *thickenRadial) (ProfileRecord, error) {
	raw := make([]survey2d.SideWalk, len(chain.Segments))
	for i, seg := range chain.Segments {
		if err := ctx.Err(); err != nil {
			return ProfileRecord{}, err
		}
		if _, ok := seg.(LineSeg); !ok {
			return ProfileRecord{}, fmt.Errorf(`%w: the open walk requires line-only axis-parallel segments`, ErrUnsupported)
		}
		w, err := boundarywalk.WalkOf(seg, work)
		if err != nil {
			return ProfileRecord{}, err
		}
		raw[i] = survey2d.SideWalk{SegmentWalk: w, Segs: []int{i}}
	}
	walks, err := boundarywalk.CoalesceChainWalksContext(ctx, raw)
	if err != nil {
		return ProfileRecord{}, err
	}
	rightSteps, leftSteps := 1, 0
	switch side {
	case ThickenNegative:
		rightSteps, leftSteps = 0, 1
	case ThickenCentered:
		rightSteps, leftSteps = 1, 1
	}
	loop, err := thickenaxis.RibbonSection(ctx, walks, rightSteps, leftSteps, amount, budget, radial)
	if err != nil {
		return ProfileRecord{}, err
	}
	section := ProfileRecord{Outer: loop}
	entries, err := buildSegEntriesBudget(budget, []LoopRecord{section.Outer})
	if err != nil {
		return ProfileRecord{}, err
	}
	if err := thickenAuditRefusal(crossingAuditBudget(budget, entries)); err != nil {
		return ProfileRecord{}, err
	}
	return section, nil
}
