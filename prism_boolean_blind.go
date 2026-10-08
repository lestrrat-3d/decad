package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/prismcells"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/stackedrecord"
	"github.com/lestrrat-3d/sketch"
)

// The structural match already proved that every target loop survives whole.
// Keep those records verbatim so a wall shared across slabs stays one column;
// take only the new tool hole from the authenticated arranged profile.
func canonicalizeStackedCutProfile(budget *proofbound.WorkBudget, target ProfileRecord, match *sketch.Profile,
	tags map[sketch.Entity]prismcells.Origin, candidate ProfileRecord) (ProfileRecord, error) {
	if len(match.Holes) != len(candidate.Holes) || len(candidate.Holes) != len(target.Holes)+1 {
		return ProfileRecord{}, fmt.Errorf(`%w: the cut profile has an unexpected hole count`, ErrUnsupported)
	}
	toolEntities, err := prismcells.LoopEntitySet(budget, tags, true, -1)
	if err != nil {
		return ProfileRecord{}, err
	}
	toolIndex := -1
	for j, hole := range match.Holes {
		isTool, err := prismcells.LoopMatchesOrigin(budget, hole, toolEntities)
		if err != nil {
			return ProfileRecord{}, err
		}
		if isTool {
			if toolIndex >= 0 {
				return ProfileRecord{}, fmt.Errorf(`%w: more than one result hole matches the tool`, ErrUnsupported)
			}
			toolIndex = j
		}
	}
	if toolIndex < 0 {
		return ProfileRecord{}, fmt.Errorf(`%w: no result hole matches the tool`, ErrUnsupported)
	}
	result := ProfileRecord{Outer: target.Outer, Holes: append([]LoopRecord(nil), target.Holes...)}
	result.Holes = append(result.Holes, candidate.Holes[toolIndex])
	return result, nil
}

// tryBlindStackedCut admits a cleanly nested blind tool against a prism. The
// private sketch scene proves the two slab sections before they are recorded.
func tryBlindStackedCut(ctx context.Context, a, b *Body) (stackedPrismPayload, bool, error) {
	budget := proofbound.NewWorkBudget(ctx)
	if err := budget.Err(); err != nil {
		return stackedPrismPayload{}, false, err
	}
	target, tool, ok, err := admitPrismPairBudget(budget, a, b)
	if err != nil || !ok {
		return stackedPrismPayload{}, false, err
	}
	if len(tool.profile.Holes) != 0 {
		return stackedPrismPayload{}, false, nil
	}
	for _, profile := range []ProfileRecord{target.profile, tool.profile} {
		trimmed, err := prismProfileHasTrimmedCircularSource(budget, profile)
		if err != nil {
			return stackedPrismPayload{}, false, err
		}
		if trimmed {
			return stackedPrismPayload{}, false, nil
		}
	}
	z0, z1, ok := prismShiftedInterval(target, tool)
	if !ok {
		return stackedPrismPayload{}, false, nil
	}
	a0, a1 := proofarith.FloatRat(target.z0), proofarith.FloatRat(target.z1)
	if a0 == nil || a1 == nil {
		return stackedPrismPayload{}, false, nil
	}
	openAtTop := z0.Cmp(a0) > 0 && z0.Cmp(a1) < 0 && z1.Cmp(a1) >= 0
	openAtBottom := z0.Cmp(a0) <= 0 && z1.Cmp(a0) > 0 && z1.Cmp(a1) < 0
	if !openAtTop && !openAtBottom {
		return stackedPrismPayload{}, false, nil
	}
	segments, withinCap, err := prismSceneWithinWorkCap(budget, target, tool)
	if err != nil {
		return stackedPrismPayload{}, false, err
	}
	if !withinCap {
		return stackedPrismPayload{}, false, fmt.Errorf(
			`%w: the analytic cut scene charges %d arranger segments against the cap of %d`,
			ErrUnsupported, segments, prismMaxArrangementSegments)
	}
	reexpress, err := newPrismReexpression(target, tool)
	if err != nil {
		return stackedPrismPayload{}, false, err
	}
	scene, match, tags, sceneDelta, resolved, err := resolvePrismCutWithTags(ctx, budget, target, tool, reexpress)
	if err != nil || !resolved {
		return stackedPrismPayload{}, false, err
	}
	_, err = prismRecordProfileContext(ctx, scene, match)
	if err != nil {
		return stackedPrismPayload{}, false, err
	}
	cutRegion, err := prismRecordArrangedProfileContext(ctx, match)
	if err != nil {
		return stackedPrismPayload{}, false, err
	}
	cutRegion, err = canonicalizeStackedCutProfile(budget, target.profile, match, tags, cutRegion)
	if err != nil {
		return stackedPrismPayload{}, false, err
	}
	inner := z0
	innerDelta := tool.z0Delta
	if openAtBottom {
		inner, innerDelta = z1, tool.z1Delta
	}
	innerHeld, _ := inner.Float64()
	shiftRound := proofarith.RationalFloatError(inner, innerHeld)
	if innerDelta == 0 {
		innerDelta = shiftRound
	} else {
		innerDelta = proofbound.AbsSumUpper(innerDelta, shiftRound)
	}
	sp := stackedPrismPayload{frame: target.frame, xform: target.xform, slabs: make([]prismSlab, 2),
		sectionDelta: max(proofbound.AbsSumUpper(target.sectionDelta, sceneDelta.a),
			proofbound.AbsSumUpper(tool.sectionDelta, sceneDelta.b, reexpress.delta))}
	if openAtTop {
		sp.slabs[0] = prismSlab{regions: []ProfileRecord{target.profile},
			z0: target.z0, z1: innerHeld, z0Delta: target.z0Delta, z1Delta: innerDelta}
		sp.slabs[1] = prismSlab{regions: []ProfileRecord{cutRegion},
			z0: innerHeld, z1: target.z1, z0Delta: innerDelta, z1Delta: target.z1Delta}
	} else {
		sp.slabs[0] = prismSlab{regions: []ProfileRecord{cutRegion},
			z0: target.z0, z1: innerHeld, z0Delta: target.z0Delta, z1Delta: innerDelta}
		sp.slabs[1] = prismSlab{regions: []ProfileRecord{target.profile},
			z0: innerHeld, z1: target.z1, z0Delta: innerDelta, z1Delta: target.z1Delta}
	}
	lowerOnly, upperOnly := stackedrecord.ExclusiveHoles(sp.slabs[0].regions[0], sp.slabs[1].regions[0])
	lowerExposed, err := stackedrecord.Exposed(ctx, upperOnly)
	if err != nil {
		return stackedPrismPayload{}, false, err
	}
	upperExposed, err := stackedrecord.Exposed(ctx, lowerOnly)
	if err != nil {
		return stackedPrismPayload{}, false, err
	}
	sp.interfaces = []prismSlabInterface{{lowerExposed: lowerExposed, upperExposed: upperExposed}}
	return sp, true, nil
}

// tryStackedThroughCut applies one cleanly nested, spanning prism tool to
// every slab. Each slab's private scene authenticates its own new region.
func tryStackedThroughCut(ctx context.Context, a, b *Body) (stackedPrismPayload, bool, error) {
	sp, ok := a.payload.(stackedPrismPayload)
	if !ok {
		return stackedPrismPayload{}, false, nil
	}
	// A union-built stack changes its outer loop at an interface. Its exposed
	// patches are not the exclusive-hole patches this cut re-derives below, so
	// it takes the mesh path (docs/stacked-prism-design.md §6).
	if runs, err := sp.outerRuns(); err != nil || len(runs) != 1 {
		return stackedPrismPayload{}, false, err
	}
	budget := proofbound.NewWorkBudget(ctx)
	outer := sp.outerPrism()
	proxy := &Body{payload: outer}
	_, tool, ok, err := admitPrismPairBudget(budget, proxy, b)
	if err != nil || !ok {
		return stackedPrismPayload{}, false, err
	}
	if len(tool.profile.Holes) != 0 || !prismCutZIntervalSpans(outer, tool) {
		return stackedPrismPayload{}, false, nil
	}
	trimmed, err := prismProfileHasTrimmedCircularSource(budget, tool.profile)
	if err != nil || trimmed {
		return stackedPrismPayload{}, false, err
	}
	reexpress, err := newPrismReexpression(outer, tool)
	if err != nil {
		return stackedPrismPayload{}, false, err
	}
	result := sp
	result.slabs = append([]prismSlab(nil), sp.slabs...)
	result.interfaces = make([]prismSlabInterface, len(sp.interfaces))
	for k, slab := range sp.slabs {
		if err := budget.Err(); err != nil {
			return stackedPrismPayload{}, false, err
		}
		target := outer
		target.profile = slab.regions[0]
		target.z0, target.z1 = slab.z0, slab.z1
		target.z0Delta, target.z1Delta = slab.z0Delta, slab.z1Delta
		trimmed, err := prismProfileHasTrimmedCircularSource(budget, target.profile)
		if err != nil || trimmed {
			return stackedPrismPayload{}, false, err
		}
		segments, withinCap, err := prismSceneWithinWorkCap(budget, target, tool)
		if err != nil {
			return stackedPrismPayload{}, false, err
		}
		if !withinCap {
			return stackedPrismPayload{}, false, fmt.Errorf(
				`%w: slab %d's analytic cut scene charges %d arranger segments against the cap of %d`,
				ErrUnsupported, k, segments, prismMaxArrangementSegments)
		}
		scene, match, tags, sceneDelta, resolved, err := resolvePrismCutWithTags(ctx, budget, target, tool, reexpress)
		if err != nil || !resolved {
			return stackedPrismPayload{}, false, err
		}
		_, err = prismRecordProfileContext(ctx, scene, match)
		if err != nil {
			return stackedPrismPayload{}, false, err
		}
		profile, err := prismRecordArrangedProfileContext(ctx, match)
		if err != nil {
			return stackedPrismPayload{}, false, err
		}
		profile, err = canonicalizeStackedCutProfile(budget, target.profile, match, tags, profile)
		if err != nil {
			return stackedPrismPayload{}, false, err
		}
		result.slabs[k].regions = []ProfileRecord{profile}
		result.sectionDelta = max(result.sectionDelta,
			proofbound.AbsSumUpper(target.sectionDelta, sceneDelta.a),
			proofbound.AbsSumUpper(tool.sectionDelta, sceneDelta.b, reexpress.delta))
	}
	for k := range result.interfaces {
		lowerOnly, upperOnly := stackedrecord.ExclusiveHoles(
			result.slabs[k].regions[0], result.slabs[k+1].regions[0])
		if len(lowerOnly) != 0 && len(upperOnly) != 0 {
			return stackedPrismPayload{}, false, fmt.Errorf(
				`%w: slab interface %d has exclusive holes on both sides`, ErrUnsupported, k)
		}
		lowerExposed, err := stackedrecord.Exposed(ctx, upperOnly)
		if err != nil {
			return stackedPrismPayload{}, false, err
		}
		upperExposed, err := stackedrecord.Exposed(ctx, lowerOnly)
		if err != nil {
			return stackedPrismPayload{}, false, err
		}
		result.interfaces[k] = prismSlabInterface{lowerExposed: lowerExposed, upperExposed: upperExposed}
	}
	if err := stackedrecord.Falsify(ctx, stackedRecordOf(result)); err != nil {
		return stackedPrismPayload{}, false, err
	}
	return result, true, nil
}
