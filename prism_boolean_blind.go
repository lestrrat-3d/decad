package decad

import (
	"context"
	"fmt"
)

// tryBlindCupCut admits the first, hole-free blind-cut shape. The sketch
// arrangement proves that the tool is wholly inside the target section before
// the two axial intervals are assembled into a closed analytic pocket.
func tryBlindCupCut(ctx context.Context, a, b *Body) (cupPayload, bool, error) {
	budget := newWorkBudget(ctx)
	if err := budget.err(); err != nil {
		return cupPayload{}, false, err
	}
	target, tool, ok, err := admitPrismPairBudget(budget, a, b)
	if err != nil || !ok {
		return cupPayload{}, false, err
	}
	if len(target.profile.Holes) != 0 || len(tool.profile.Holes) != 0 ||
		target.sectionDelta != 0 || tool.sectionDelta != 0 {
		return cupPayload{}, false, nil
	}
	for _, profile := range []ProfileRecord{target.profile, tool.profile} {
		trimmed, err := prismProfileHasTrimmedCircularSource(budget, profile)
		if err != nil {
			return cupPayload{}, false, err
		}
		if trimmed {
			return cupPayload{}, false, nil
		}
	}
	z0, z1, ok := prismShiftedInterval(target, tool)
	if !ok {
		return cupPayload{}, false, nil
	}
	a0, a1 := floatRat(target.z0), floatRat(target.z1)
	if a0 == nil || a1 == nil {
		return cupPayload{}, false, nil
	}
	openAtTop := z0.Cmp(a0) > 0 && z0.Cmp(a1) < 0 && z1.Cmp(a1) >= 0
	openAtBottom := z0.Cmp(a0) <= 0 && z1.Cmp(a0) > 0 && z1.Cmp(a1) < 0
	if !openAtTop && !openAtBottom {
		return cupPayload{}, false, nil
	}
	segments, withinCap, err := prismSceneWithinWorkCap(budget, target, tool)
	if err != nil {
		return cupPayload{}, false, err
	}
	if !withinCap {
		return cupPayload{}, false, fmt.Errorf(
			`%w: the analytic cut scene charges %d arranger segments against the cap of %d`,
			ErrUnsupported, segments, prismMaxArrangementSegments)
	}
	reexpress, err := newPrismReexpression(target, tool)
	if err != nil {
		return cupPayload{}, false, err
	}
	if !reexpress.identity {
		return cupPayload{}, false, nil
	}
	_, _, sceneDelta, resolved, err := resolvePrismCut(ctx, budget, target, tool, reexpress)
	if err != nil || !resolved {
		return cupPayload{}, false, err
	}
	if sceneDelta.a != 0 || sceneDelta.b != 0 {
		return cupPayload{}, false, nil
	}
	inner := z0
	innerDelta := tool.z0Delta
	if openAtBottom {
		inner, innerDelta = z1, tool.z1Delta
	}
	innerHeld, _ := inner.Float64()
	innerDelta = absSumUpper(innerDelta, rationalFloatError(inner, innerHeld))
	cp := cupPayload{
		outer: target.profile, cavity: tool.profile,
		frame: target.frame, xform: target.xform,
		zCav: innerHeld, zCavDelta: innerDelta,
	}
	if openAtTop {
		cp.zOuter, cp.zOuterDelta = target.z0, target.z0Delta
		cp.zOpen, cp.zOpenDelta = target.z1, target.z1Delta
	} else {
		cp.zOuter, cp.zOuterDelta = target.z1, target.z1Delta
		cp.zOpen, cp.zOpenDelta = target.z0, target.z0Delta
	}
	return cp, true, nil
}
