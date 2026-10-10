package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/prismplacement"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/stackedrecord"
	"github.com/lestrrat-3d/sketch"
)

// The structural match already proved that every retained target loop survives whole.
// Keep those records verbatim so a wall shared across slabs stays one column;
// take only the new tool hole from the authenticated arranged profile.
func canonicalizeStackedCutProfile(budget *proofbound.WorkBudget, target profileRecord, match *sketch.Profile,
	tags map[sketch.Entity]prismcells.Origin, candidate profileRecord) (profileRecord, error) {
	if len(match.Holes) != len(candidate.Holes) || len(candidate.Holes) != len(target.Holes)+1 {
		return profileRecord{}, fmt.Errorf(`%w: the cut profile has an unexpected hole count`, ErrUnsupported)
	}
	toolEntities, err := prismcells.LoopEntitySet(budget, tags, true, -1)
	if err != nil {
		return profileRecord{}, err
	}
	toolIndex := -1
	for j, hole := range match.Holes {
		isTool, err := prismcells.LoopMatchesOrigin(budget, hole, toolEntities)
		if err != nil {
			return profileRecord{}, err
		}
		if isTool {
			if toolIndex >= 0 {
				return profileRecord{}, fmt.Errorf(`%w: more than one result hole matches the tool`, ErrUnsupported)
			}
			toolIndex = j
		}
	}
	if toolIndex < 0 {
		return profileRecord{}, fmt.Errorf(`%w: no result hole matches the tool`, ErrUnsupported)
	}
	result := profileRecord{Outer: target.Outer, Holes: append([]loopRecord(nil), target.Holes...)}
	result.Holes = append(result.Holes, candidate.Holes[toolIndex])
	return result, nil
}

// tryBlindStackedCut admits a cleanly nested blind tool against a prism,
// including one that encloses existing holes. The private sketch scene proves
// the two slab sections before they are recorded.
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
	for _, profile := range []profileRecord{target.profile, tool.profile} {
		trimmed, err := prismcells.ProfileHasTrimmedCircularSource(budget, profile.Outer, profile.Holes)
		if err != nil {
			return stackedPrismPayload{}, false, err
		}
		if trimmed {
			return stackedPrismPayload{}, false, nil
		}
	}
	z0, z1, ok := prismplacement.ShiftedInterval(prismPlacementOf(target), prismPlacementOf(tool))
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
	segments, withinCap, err := prismcells.RegionsWithinWorkCap(budget, target.profile, tool.profile)
	if err != nil {
		return stackedPrismPayload{}, false, err
	}
	if !withinCap {
		return stackedPrismPayload{}, false, fmt.Errorf(
			`%w: the analytic cut scene charges %d arranger segments against the cap of %d`,
			ErrUnsupported, segments, prismcells.MaxArrangementSegments)
	}
	reexpress, err := prismcells.NewReexpression(prismPlacementOf(target), prismPlacementOf(tool))
	if err != nil {
		return stackedPrismPayload{}, false, err
	}
	scene, profiles, tags, sceneDelta, err := prismCutCells(ctx, budget, target, tool, reexpress)
	if err != nil {
		return stackedPrismPayload{}, false, err
	}
	match, resolved, err := prismcells.MatchCut(budget, tags, profiles, len(target.profile.Holes))
	if err != nil {
		return stackedPrismPayload{}, false, err
	}
	var enclosing prismcells.EnclosingCutMatch
	if !resolved {
		enclosing, resolved, err = prismcells.MatchEnclosingCut(budget, tags, profiles, len(target.profile.Holes))
		if err != nil || !resolved {
			return stackedPrismPayload{}, false, err
		}
		match = enclosing.Outside
		if _, err := prismRecordProfileContext(ctx, scene, enclosing.Inside); err != nil {
			return stackedPrismPayload{}, false, err
		}
	}
	_, err = prismRecordProfileContext(ctx, scene, match)
	if err != nil {
		return stackedPrismPayload{}, false, err
	}
	cutRegion, err := prismRecordArrangedProfileContext(ctx, match)
	if err != nil {
		return stackedPrismPayload{}, false, err
	}
	kept := target.profile
	if len(enclosing.EnclosedHoles) != 0 {
		kept.Holes = make([]loopRecord, 0, len(enclosing.OutsideHoles))
		for _, i := range enclosing.OutsideHoles {
			kept.Holes = append(kept.Holes, target.profile.Holes[i])
		}
	}
	cutRegion, err = canonicalizeStackedCutProfile(budget, kept, match, tags, cutRegion)
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
	sp := stackedPrismPayload{frame: target.frame, xform: target.xform, slabs: make([]stackedrecord.Slab, 2),
		sectionDelta: max(proofbound.AbsSumUpper(target.sectionDelta, sceneDelta.A),
			proofbound.AbsSumUpper(tool.sectionDelta, sceneDelta.B, reexpress.Delta))}
	if openAtTop {
		sp.slabs[0] = stackedrecord.Slab{Regions: []profileRecord{target.profile},
			Z0: target.z0, Z1: innerHeld, Z0Delta: target.z0Delta, Z1Delta: innerDelta}
		sp.slabs[1] = stackedrecord.Slab{Regions: []profileRecord{cutRegion},
			Z0: innerHeld, Z1: target.z1, Z0Delta: innerDelta, Z1Delta: target.z1Delta}
	} else {
		sp.slabs[0] = stackedrecord.Slab{Regions: []profileRecord{cutRegion},
			Z0: target.z0, Z1: innerHeld, Z0Delta: target.z0Delta, Z1Delta: innerDelta}
		sp.slabs[1] = stackedrecord.Slab{Regions: []profileRecord{target.profile},
			Z0: innerHeld, Z1: target.z1, Z0Delta: innerDelta, Z1Delta: target.z1Delta}
	}
	if len(enclosing.EnclosedHoles) != 0 {
		innerHoles := make([]loopRecord, 0, len(enclosing.EnclosedHoles))
		for _, i := range enclosing.EnclosedHoles {
			innerHoles = append(innerHoles, target.profile.Holes[i])
		}
		toolHole := cutRegion.Holes[len(cutRegion.Holes)-1]
		exposed, err := stackedrecord.EnclosingExposed(ctx, toolHole, innerHoles)
		if err != nil {
			return stackedPrismPayload{}, false, err
		}
		if openAtTop {
			sp.interfaces = []stackedrecord.Interface{{LowerExposed: exposed}}
		} else {
			sp.interfaces = []stackedrecord.Interface{{UpperExposed: exposed}}
		}
	} else {
		lowerOnly, upperOnly := stackedrecord.ExclusiveHoles(sp.slabs[0].Regions[0], sp.slabs[1].Regions[0])
		lowerExposed, err := stackedrecord.Exposed(ctx, upperOnly)
		if err != nil {
			return stackedPrismPayload{}, false, err
		}
		upperExposed, err := stackedrecord.Exposed(ctx, lowerOnly)
		if err != nil {
			return stackedPrismPayload{}, false, err
		}
		sp.interfaces = []stackedrecord.Interface{{LowerExposed: lowerExposed, UpperExposed: upperExposed}}
	}
	return sp, true, nil
}

// tryStackedBlindCut splits the slab containing a blind tool's inner end.
// Every slab the tool reaches then needs its own whole-loop cut proof.
func tryStackedBlindCut(ctx context.Context, a, b *Body) (stackedPrismPayload, bool, error) {
	sp, ok := a.payload.(stackedPrismPayload)
	if !ok || len(sp.outerRuns()) != 1 {
		return stackedPrismPayload{}, false, nil
	}
	for _, slab := range sp.slabs {
		if len(slab.Regions) != 1 {
			return stackedPrismPayload{}, false, nil
		}
	}
	budget := proofbound.NewWorkBudget(ctx)
	outer := sp.outerPrism()
	proxy := &Body{payload: outer}
	_, tool, ok, err := admitPrismPairBudget(budget, proxy, b)
	if err != nil || !ok {
		return stackedPrismPayload{}, false, err
	}
	if len(tool.profile.Holes) != 0 {
		return stackedPrismPayload{}, false, nil
	}
	trimmed, err := prismcells.ProfileHasTrimmedCircularSource(budget, tool.profile.Outer, tool.profile.Holes)
	if err != nil || trimmed {
		return stackedPrismPayload{}, false, err
	}
	z0, z1, ok := prismplacement.ShiftedInterval(prismPlacementOf(outer), prismPlacementOf(tool))
	if !ok {
		return stackedPrismPayload{}, false, nil
	}
	first := proofarith.FloatRat(sp.slabs[0].Z0)
	last := proofarith.FloatRat(sp.slabs[len(sp.slabs)-1].Z1)
	if first == nil || last == nil {
		return stackedPrismPayload{}, false, nil
	}
	openAtTop := z0.Cmp(first) > 0 && z0.Cmp(last) < 0 && z1.Cmp(last) >= 0
	openAtBottom := z0.Cmp(first) <= 0 && z1.Cmp(first) > 0 && z1.Cmp(last) < 0
	if !openAtTop && !openAtBottom {
		return stackedPrismPayload{}, false, nil
	}
	inner, innerDelta := z0, tool.z0Delta
	if openAtBottom {
		inner, innerDelta = z1, tool.z1Delta
	}
	split := -1
	for k, slab := range sp.slabs {
		lower, upper := proofarith.FloatRat(slab.Z0), proofarith.FloatRat(slab.Z1)
		if lower != nil && upper != nil && inner.Cmp(lower) > 0 && inner.Cmp(upper) < 0 {
			split = k
			break
		}
	}
	if split < 0 {
		return stackedPrismPayload{}, false, nil
	}
	innerHeld, _ := inner.Float64()
	if innerHeld <= sp.slabs[split].Z0 || innerHeld >= sp.slabs[split].Z1 {
		return stackedPrismPayload{}, false, nil
	}
	innerDelta = proofbound.AbsSumUpper(innerDelta, proofarith.RationalFloatError(inner, innerHeld))
	reexpress, err := prismcells.NewReexpression(prismPlacementOf(outer), prismPlacementOf(tool))
	if err != nil {
		return stackedPrismPayload{}, false, err
	}
	lower, upper := sp.slabs[split], sp.slabs[split]
	lower.Z1, lower.Z1Delta = innerHeld, innerDelta
	upper.Z0, upper.Z0Delta = innerHeld, innerDelta
	result := sp
	result.slabs = make([]stackedrecord.Slab, len(sp.slabs)+1)
	copy(result.slabs, sp.slabs[:split])
	result.slabs[split], result.slabs[split+1] = lower, upper
	copy(result.slabs[split+2:], sp.slabs[split+1:])
	prior := make([]stackedrecord.Interface, len(result.slabs)-1)
	copy(prior, sp.interfaces[:split])
	copy(prior[split+1:], sp.interfaces[split:])
	active := make([]bool, len(result.slabs))
	for k := range active {
		active[k] = k <= split
		if openAtTop {
			active[k] = k > split
		}
	}
	return cutStackedSlabs(ctx, budget, result, outer, tool, reexpress, active, prior)
}

// tryStackedThroughCut applies a spanning prism tool to every slab. Each
// private scene proves either a new region or that the tool lies wholly in an
// existing hole and leaves that slab unchanged.
func tryStackedThroughCut(ctx context.Context, a, b *Body) (stackedPrismPayload, bool, error) {
	sp, ok := a.payload.(stackedPrismPayload)
	if !ok {
		return stackedPrismPayload{}, false, nil
	}
	// A union-built stack changes its outer loop at an interface. Its exposed
	// patches are not the exclusive-hole patches this cut re-derives below, so
	// it takes the mesh path (docs/stacked-prism-design.md §6).
	if len(sp.outerRuns()) != 1 {
		return stackedPrismPayload{}, false, nil
	}
	budget := proofbound.NewWorkBudget(ctx)
	outer := sp.outerPrism()
	proxy := &Body{payload: outer}
	_, tool, ok, err := admitPrismPairBudget(budget, proxy, b)
	if err != nil || !ok {
		return stackedPrismPayload{}, false, err
	}
	if len(tool.profile.Holes) != 0 ||
		!prismplacement.CutZIntervalSpans(prismPlacementOf(outer), prismPlacementOf(tool)) {
		return stackedPrismPayload{}, false, nil
	}
	trimmed, err := prismcells.ProfileHasTrimmedCircularSource(budget, tool.profile.Outer, tool.profile.Holes)
	if err != nil || trimmed {
		return stackedPrismPayload{}, false, err
	}
	reexpress, err := prismcells.NewReexpression(prismPlacementOf(outer), prismPlacementOf(tool))
	if err != nil {
		return stackedPrismPayload{}, false, err
	}
	active := make([]bool, len(sp.slabs))
	for i := range active {
		active[i] = true
	}
	return cutStackedSlabs(ctx, budget, sp, outer, tool, reexpress, active, sp.interfaces)
}

// cutStackedSlabs proves each affected section using its own sketch scene.
// Inactive slabs retain their section, and prior names the exposed side of
// any existing nonmonotone interface after an optional slab split.
func cutStackedSlabs(ctx context.Context, budget *proofbound.WorkBudget, sp stackedPrismPayload,
	outer, tool prismPayload, reexpress *prismReexpression, active []bool,
	prior []stackedrecord.Interface) (stackedPrismPayload, bool, error) {
	result := sp
	result.slabs = append([]stackedrecord.Slab(nil), sp.slabs...)
	result.interfaces = make([]stackedrecord.Interface, len(prior))
	for k, slab := range sp.slabs {
		if !active[k] {
			continue
		}
		if err := budget.Err(); err != nil {
			return stackedPrismPayload{}, false, err
		}
		target := outer
		target.profile = slab.Regions[0]
		target.z0, target.z1 = slab.Z0, slab.Z1
		target.z0Delta, target.z1Delta = slab.Z0Delta, slab.Z1Delta
		trimmed, err := prismcells.ProfileHasTrimmedCircularSource(budget, target.profile.Outer, target.profile.Holes)
		if err != nil || trimmed {
			return stackedPrismPayload{}, false, err
		}
		segments, withinCap, err := prismcells.RegionsWithinWorkCap(budget, target.profile, tool.profile)
		if err != nil {
			return stackedPrismPayload{}, false, err
		}
		if !withinCap {
			return stackedPrismPayload{}, false, fmt.Errorf(
				`%w: slab %d's analytic cut scene charges %d arranger segments against the cap of %d`,
				ErrUnsupported, k, segments, prismcells.MaxArrangementSegments)
		}
		scene, profiles, tags, sceneDelta, err := prismCutCells(ctx, budget, target, tool, reexpress)
		if err != nil {
			return stackedPrismPayload{}, false, err
		}
		match, resolved, err := prismcells.MatchCut(budget, tags, profiles, len(target.profile.Holes))
		if err != nil {
			return stackedPrismPayload{}, false, err
		}
		if !resolved {
			noOp, err := prismcells.MatchCutNoOp(budget, tags, profiles, len(target.profile.Holes))
			if err != nil || !noOp {
				return stackedPrismPayload{}, false, err
			}
			continue
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
		result.slabs[k].Regions = []profileRecord{profile}
		result.sectionDelta = max(result.sectionDelta,
			proofbound.AbsSumUpper(target.sectionDelta, sceneDelta.A),
			proofbound.AbsSumUpper(tool.sectionDelta, sceneDelta.B, reexpress.Delta))
	}
	var err error
	result.interfaces, err = stackedrecord.Derive(ctx, result.slabs, prior)
	if err != nil {
		return stackedPrismPayload{}, false, err
	}
	if err := stackedrecord.Falsify(ctx, stackedrecord.Record{Slabs: result.slabs, Interfaces: result.interfaces}); err != nil {
		return stackedPrismPayload{}, false, err
	}
	return result, true, nil
}
