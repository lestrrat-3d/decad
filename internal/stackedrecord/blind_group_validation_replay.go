package stackedrecord

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/prismplacement"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
)

func replayEqualBlindLoop(ctx context.Context, a, b sectionrecord.LoopRecord, budget proof.ReplayKeyBudget) (bool, error) {
	if len(a.Segments) != len(b.Segments) {
		return false, nil
	}
	for i := range a.Segments {
		// One value comparison covers type dispatch and every scalar field.
		if err := budget.SpendWork(ctx, 64); err != nil {
			return false, err
		}
		if !sectionrecord.IdenticalAnalyticSegment(a.Segments[i], b.Segments[i]) {
			return false, nil
		}
	}
	return true, nil
}

func replayEqualBlindProfile(ctx context.Context, a, b momentinput.Profile, budget proof.ReplayKeyBudget) (bool, error) {
	if len(a.Holes) != len(b.Holes) {
		return false, nil
	}
	ok, err := replayEqualBlindLoop(ctx, a.Outer, b.Outer, budget)
	if err != nil || !ok {
		return false, err
	}
	for i := range a.Holes {
		if err := budget.SpendWork(ctx, 1); err != nil {
			return false, err
		}
		ok, err = replayEqualBlindLoop(ctx, a.Holes[i], b.Holes[i], budget)
		if err != nil || !ok {
			return false, err
		}
	}
	return true, nil
}

// replayValidateBlindGroup checks the two-slab partition and each floor against
// the exact certified section. The caller still owns full topology verification.
func replayValidateBlindGroup(ctx context.Context, out Payload, target prismrecord.Payload,
	section prismcells.ReplayBlindSection, inner float64, top bool, budget proof.ReplayKeyBudget) (bool, error) {
	if err := budget.SpendWork(ctx, 1); err != nil {
		return false, err
	}
	if out.Frame != target.Frame || out.Xform != target.Xform || out.SectionDelta != section.SectionDelta || out.ClosedShell || len(out.Slabs) != 2 || len(out.Interfaces) != 1 {
		return false, nil
	}
	lower, upper := out.Slabs[0], out.Slabs[1]
	if lower.Z0 != target.Z0 || lower.Z1 != inner || upper.Z0 != inner || upper.Z1 != target.Z1 ||
		lower.Z0Delta != target.Z0Delta || upper.Z1Delta != target.Z1Delta || lower.Z1Delta != 0 || upper.Z0Delta != 0 || len(lower.Regions) != 1 || len(upper.Regions) != 1 {
		return false, nil
	}
	if !(target.Z0 < inner && inner < target.Z1) {
		return false, nil
	}
	zero, err := proof.ReplayFloatRat(ctx, 0, budget)
	if err != nil {
		return false, err
	}
	outer := prismplacement.Operand{Z0: lower.Z0, Z0Delta: lower.Z0Delta, Z0Denoted: lower.Z0Denoted,
		Z1: upper.Z1, Z1Delta: upper.Z1Delta, Z1Denoted: upper.Z1Denoted}
	same, err := prismplacement.ReplaySameShiftedInterval(ctx, target.PlacementOperand(), outer, zero, budget)
	if err != nil || !same {
		return false, err
	}
	middle := prismplacement.Operand{Z0: lower.Z1, Z0Delta: lower.Z1Delta, Z0Denoted: lower.Z1Denoted,
		Z1: upper.Z0, Z1Delta: upper.Z0Delta, Z1Denoted: upper.Z0Denoted}
	middleHeld := prismplacement.Operand{Z0: inner, Z1: inner}
	same, err = prismplacement.ReplaySameShiftedInterval(ctx, middleHeld, middle, zero, budget)
	if err != nil || !same {
		return false, err
	}
	uncut, cut := lower.Regions[0], upper.Regions[0]
	floors, other := out.Interfaces[0].LowerExposed, out.Interfaces[0].UpperExposed
	if !top {
		uncut, cut = upper.Regions[0], lower.Regions[0]
		floors, other = out.Interfaces[0].UpperExposed, out.Interfaces[0].LowerExposed
	}
	if len(other) != 0 || len(floors) != len(section.HoleRegions) || len(section.Profile.Holes) != len(target.Profile.Holes)+len(floors) {
		return false, nil
	}
	ok, err := replayEqualBlindProfile(ctx, uncut, target.Profile, budget)
	if err != nil || !ok {
		return false, err
	}
	ok, err = replayEqualBlindProfile(ctx, cut, section.Profile, budget)
	if err != nil || !ok {
		return false, err
	}
	for i := range floors {
		if err := budget.SpendWork(ctx, 1); err != nil {
			return false, err
		}
		floor := floors[i]
		if len(floor.Holes) != 0 || len(floor.Outer.Segments) != 1 {
			return false, nil
		}
		hole := section.Profile.Holes[len(target.Profile.Holes)+i]
		if len(hole.Segments) != 1 {
			return false, nil
		}
		circle, ok := hole.Segments[0].(sectionrecord.CircleSeg)
		if !ok {
			return false, nil
		}
		circle.CCW = !circle.CCW
		circle.TStart, circle.TEnd = circle.TEnd, circle.TStart
		got, ok := floor.Outer.Segments[0].(sectionrecord.CircleSeg)
		if !ok || got != circle {
			return false, nil
		}
	}
	return true, nil
}
