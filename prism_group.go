package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/stackedrecord"

	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/sketch"
)

// This file is docs/general-boolean-design.md §3's class A5: prism-boolean's
// analytic paths over operands and results with several regions. A prism
// group (docs/mirror-pattern-design.md §6.3) is a one-slab
// stackedPrismPayload whose regions are pairwise disjoint lumps on one frame
// and one interval. A group tool enters Cut's private scene as one region per
// lump; a Union whose select-all survivors close into several loops proven
// pairwise disjoint becomes a group.
//
// Admission is prism-boolean's G1-G4 read on each operand's frame, placement
// and interval, with G4, G6 and the trimmed-circular refusal on every region.
// A miss is silent. Past the first private scene an unresolved topology is
// silent too (prism-boolean §4.4); only an invalid cell the result depends on
// (RB1), the arrangement cap (RB7) and the recording and audit refusals
// (RB2-RB9) are errors. A scene whose cuts carry a displaced operand's
// displacement charges every crossing (docs/general-boolean-design.md §3
// A6, prismSceneDelta.ChargeCrossings) and never refuses: a crossing with no
// charge, or any of those errors on such a scene, sends the pair to the mesh
// path (prismcells.AmplifiedFallback).

// prismGroupOperand is one operand read as regions over one interval: a prism
// is one region, a prism group one region per lump. proxy carries the frame,
// placement, interval, level displacements and section displacement the
// gates read, with the first region as its profile.
type prismGroupOperand struct {
	proxy   prismPayload
	regions []ProfileRecord
	group   bool
}

func prismGroupOperandOf(b *Body) (prismGroupOperand, bool) {
	switch p := b.payload.(type) {
	case prismPayload:
		return prismGroupOperand{proxy: p, regions: []ProfileRecord{p.profile}}, true
	case stackedPrismPayload:
		if !p.isGroup() {
			return prismGroupOperand{}, false
		}
		slab := p.slabs[0]
		proxy := prismPayload{profile: slab.regions[0], frame: p.frame, xform: p.xform,
			z0: slab.z0, z1: slab.z1, z0Delta: slab.z0Delta, z1Delta: slab.z1Delta,
			sectionDelta: p.sectionDelta}
		return prismGroupOperand{proxy: proxy, regions: slab.regions, group: true}, true
	default:
		return prismGroupOperand{}, false
	}
}

// admitPrismGroupPair is G1-G4 on the two proxies, then G4, G6 (when
// holeFree) and the trimmed-circular refusal on every region of both.
func admitPrismGroupPair(budget *proofbound.WorkBudget, oa, ob prismGroupOperand, holeFreeA, holeFreeB bool) (bool, error) {
	if _, _, ok, err := admitPrismPairBudget(budget, &Body{payload: oa.proxy}, &Body{payload: ob.proxy}); err != nil || !ok {
		return false, err
	}
	for _, op := range []struct {
		o        prismGroupOperand
		holeFree bool
	}{{oa, holeFreeA}, {ob, holeFreeB}} {
		for _, region := range op.o.regions {
			if op.holeFree && len(region.Holes) != 0 { // G6
				return false, nil
			}
			analytic, err := prismProfileIsAnalytic(budget, region) // G4
			if err != nil || !analytic {
				return false, err
			}
			trimmed, err := prismProfileHasTrimmedCircularSource(budget, region)
			if err != nil || trimmed {
				return false, err
			}
		}
	}
	return true, nil
}

func prismGroupWithinCap(budget *proofbound.WorkBudget, op string, regions ...[]ProfileRecord) error {
	var all []ProfileRecord
	for _, rs := range regions {
		all = append(all, rs...)
	}
	segments, withinCap, err := prismRegionsWithinWorkCap(budget, all...)
	if err != nil {
		return err
	}
	if !withinCap {
		return fmt.Errorf(
			`%w: the analytic %s scene charges at least %d arranger segments against this evaluator's cap of %d (each circle or arc costs 256, each line 1)`,
			ErrUnsupported, op, segments, prismMaxArrangementSegments)
	}
	return nil
}

// tryPrismGroupCut is A5's Cut with a prism-group tool on a prism target.
// The clean-nesting match requires the target's cell to carry the target's
// own loops whole plus one new hole per tool region, each reproducing that
// region's outer whole; that cell is the result, recorded through the seam.
// Otherwise the crossing sub-case classifies every cell against the target
// and against the group's regions together, and keeps target-and-not-tool.
func tryPrismGroupCut(ctx context.Context, a, b *Body) (prismPayload, bool, error) {
	target, aok := a.payload.(prismPayload)
	tool, bok := prismGroupOperandOf(b)
	if !aok || !bok || !tool.group {
		return prismPayload{}, false, nil
	}
	budget := proofbound.NewWorkBudget(ctx)
	if err := budget.Err(); err != nil {
		return prismPayload{}, false, err
	}
	targetOp := prismGroupOperand{proxy: target, regions: []ProfileRecord{target.profile}}
	if ok, err := admitPrismGroupPair(budget, targetOp, tool, false, true); err != nil || !ok {
		return prismPayload{}, false, err
	}
	if !prismCutZIntervalSpans(target, tool.proxy) { // G5: the tool spans the target
		return prismPayload{}, false, nil
	}
	if err := prismGroupWithinCap(budget, "cut", targetOp.regions, tool.regions); err != nil {
		return prismPayload{}, false, err
	}
	reexpress, err := prismcells.NewReexpression(prismPlacementOf(target), prismPlacementOf(tool.proxy))
	if err != nil {
		return prismPayload{}, false, err
	}
	s, tags, sceneDelta, err := buildPrismSceneRegions(budget, targetOp.regions, tool.regions, reexpress)
	if err != nil {
		return prismPayload{}, false, err
	}
	if err := budget.Err(); err != nil {
		return prismPayload{}, false, err
	}
	profiles, err := prismCellProfiles(ctx, budget, s)
	if err != nil {
		return prismPayload{}, false, err
	}
	if len(profiles) == 0 {
		return prismPayload{}, false, nil
	}
	// docs/general-boolean-design.md §3 A6: every cut a displaced operand can
	// move is charged; a crossing with no charge sends the pair to the mesh
	// path, and so does any later failure on an amplified cut
	// (prismcells.AmplifiedFallback).
	if ok, err := sceneDelta.ChargeCrossings(budget, tags, profiles, target.sectionDelta, tool.proxy.sectionDelta, reexpress.Delta); err != nil || !ok {
		return prismPayload{}, false, err
	}
	result := prismPayload{frame: target.frame, xform: target.xform,
		z0: target.z0, z1: target.z1, z0Delta: target.z0Delta, z1Delta: target.z1Delta}
	inA, inB := sceneDelta.Incoming(target.sectionDelta, tool.proxy.sectionDelta, reexpress.Delta)
	inputDelta := max(inA, inB)

	match, nested, err := prismGroupCutMatch(budget, profiles, tags, len(target.profile.Holes), len(tool.regions))
	if err != nil {
		return prismPayload{}, false, err
	}
	if nested {
		profile, err := prismRecordProfileContext(ctx, s, match)
		if err != nil {
			return prismPayload{}, false, err
		}
		// Every edge of the matched cell is whole: no cut charge.
		result.profile = profile
		result.sectionDelta = inputDelta
		return result, true, nil
	}

	// The crossing sub-case, scoped to a hole-free target as prism-boolean's.
	if len(target.profile.Holes) != 0 {
		return prismPayload{}, false, nil
	}
	matterA, matterB, resolved, err := prismcells.Classify(budget, tags, profiles)
	if err != nil || !resolved {
		return prismPayload{}, false, err
	}
	selected, err := prismcells.Select(budget, profiles, matterA, matterB, func(a, b bool) bool { return a && !b })
	if err != nil {
		return prismPayload{}, false, err
	}
	if ok, err := sceneDelta.SharedSpansBounded(budget, selected); err != nil || !ok {
		return prismPayload{}, false, err
	}
	merged, cutDelta, resolved, err := mergePrismCells(budget, selected, "cut")
	if fallBack, err := prismcells.AmplifiedFallback(sceneDelta.Amplified, err); fallBack || err != nil || !resolved {
		return prismPayload{}, false, err
	}
	if fallBack, err := prismcells.AmplifiedFallback(sceneDelta.Amplified, auditPrismMergeSection(budget, target, merged)); fallBack || err != nil {
		return prismPayload{}, false, err
	}
	result.profile = merged
	result.sectionDelta = sceneDelta.Merged(target.sectionDelta, tool.proxy.sectionDelta, reexpress.Delta, cutDelta)
	return result, true, nil
}

// prismGroupCutMatch is A5's clean-nesting search: the one cell whose outer
// reproduces the target's outer whole and whose holes reproduce the target's
// own holes plus every tool region's outer, each whole. A matched cell that
// sketch reports invalid is RB1.
func prismGroupCutMatch(budget *proofbound.WorkBudget, profiles []*sketch.Profile, tags map[sketch.Entity]prismcells.Origin, targetHoles, toolRegions int) (*sketch.Profile, bool, error) {
	targetOuter, err := prismcells.RegionLoopEntitySet(budget, tags, false, 0, -1)
	if err != nil {
		return nil, false, err
	}
	wantHoles := make([]map[sketch.Entity]struct{}, 0, targetHoles+toolRegions)
	for i := range targetHoles {
		hole, err := prismcells.RegionLoopEntitySet(budget, tags, false, 0, i)
		if err != nil {
			return nil, false, err
		}
		wantHoles = append(wantHoles, hole)
	}
	for r := range toolRegions {
		outer, err := prismcells.RegionLoopEntitySet(budget, tags, true, r, -1)
		if err != nil {
			return nil, false, err
		}
		wantHoles = append(wantHoles, outer)
	}
	match, resolved, err := prismcells.FindLoopMatch(budget, profiles, targetOuter, wantHoles)
	if err != nil || !resolved {
		return nil, false, err
	}
	if !match.Valid {
		return nil, false, prismcells.InvalidRegionError("cut")
	}
	return match, true, nil
}

// tryPrismGroupUnion is A5's Union over equal intervals where either operand
// is a prism group, or two prisms whose select-all survivors close into
// several loops. The survivors of the select-all merge are chained into
// closed loops. One loop is a prism. Two or more, proven pairwise disjoint by
// provePrismRegionsDisjoint, are a prism group.
func tryPrismGroupUnion(ctx context.Context, a, b *Body) (featurePayload, bool, error) {
	oa, aok := prismGroupOperandOf(a)
	ob, bok := prismGroupOperandOf(b)
	if !aok || !bok {
		return nil, false, nil
	}
	budget := proofbound.NewWorkBudget(ctx)
	if err := budget.Err(); err != nil {
		return nil, false, err
	}
	if ok, err := admitPrismGroupPair(budget, oa, ob, true, true); err != nil || !ok {
		return nil, false, err
	}
	if !prismUnionZIntervalMatches(oa.proxy, ob.proxy) { // G5 for Union: one interval
		return nil, false, nil
	}
	if err := prismGroupWithinCap(budget, "union", oa.regions, ob.regions); err != nil {
		return nil, false, err
	}
	reexpress, err := prismcells.NewReexpression(prismPlacementOf(oa.proxy), prismPlacementOf(ob.proxy))
	if err != nil {
		return nil, false, err
	}
	s, tags, sceneDelta, err := buildPrismSceneRegions(budget, oa.regions, ob.regions, reexpress)
	if err != nil {
		return nil, false, err
	}
	if err := budget.Err(); err != nil {
		return nil, false, err
	}
	profiles, err := prismCellProfiles(ctx, budget, s)
	if err != nil {
		return nil, false, err
	}
	if len(profiles) == 0 {
		return nil, false, nil
	}
	// docs/general-boolean-design.md §3 A6, as for the group Cut above.
	if ok, err := sceneDelta.ChargeCrossings(budget, tags, profiles, oa.proxy.sectionDelta, ob.proxy.sectionDelta, reexpress.Delta); err != nil || !ok {
		return nil, false, err
	}
	payload, ok, err := prismGroupUnionTail(ctx, budget, tags, profiles, oa, ob, reexpress, sceneDelta)
	if fallBack, err := prismcells.AmplifiedFallback(sceneDelta.Amplified, err); fallBack || err != nil {
		return nil, false, err
	}
	return payload, ok, nil
}

// prismGroupUnionTail is tryPrismGroupUnion past the crossing charge: the
// void check, the select-all merge into loops, §6's audit per loop, and the
// disjointness proof that makes several loops a group. Its errors are the
// caller's to route (prismcells.AmplifiedFallback).
func prismGroupUnionTail(ctx context.Context, budget *proofbound.WorkBudget, tags map[sketch.Entity]prismcells.Origin, profiles []*sketch.Profile,
	oa, ob prismGroupOperand, reexpress *prismReexpression, sceneDelta prismSceneDelta) (featurePayload, bool, error) {
	// Select-all keeps every bounded cell, which is the union only when no
	// cell is material of neither operand: a ring of overlapping operands
	// encloses such a cell.
	voidFree, err := prismcells.CellsHaveNoVoid(budget, tags, profiles)
	if err != nil || !voidFree {
		return nil, false, err
	}
	if ok, err := sceneDelta.SharedSpansBounded(budget, profiles); err != nil || !ok {
		return nil, false, err
	}
	loops, cutDelta, resolved, err := prismcells.MergeLoops(budget, profiles, "union")
	if err != nil || !resolved {
		return nil, false, err
	}
	// §7's incoming terms with A6's crossing charge.
	inA, inB := sceneDelta.Incoming(oa.proxy.sectionDelta, ob.proxy.sectionDelta, reexpress.Delta)
	inputDelta := max(inA, inB, sceneDelta.Crossing)
	regions := make([]ProfileRecord, len(loops))
	for i, loop := range loops {
		regions[i] = ProfileRecord{Outer: loop}
		// Point of no return (§3.4): §6's audit on each assembled loop.
		if err := auditPrismMergeSection(budget, oa.proxy, regions[i]); err != nil {
			return nil, false, err
		}
	}
	z0Delta, z1Delta := max(oa.proxy.z0Delta, ob.proxy.z0Delta), max(oa.proxy.z1Delta, ob.proxy.z1Delta)
	if len(regions) == 1 {
		return prismPayload{profile: regions[0], frame: oa.proxy.frame, xform: oa.proxy.xform,
			z0: oa.proxy.z0, z1: oa.proxy.z1, z0Delta: z0Delta, z1Delta: z1Delta,
			sectionDelta: proofbound.AbsSumUpper(inputDelta, cutDelta)}, true, nil
	}
	disjoint, disjointWalk, err := provePrismRegionsDisjoint(ctx, budget, regions)
	if err != nil || !disjoint {
		return nil, false, err
	}
	sp := stackedPrismPayload{
		slabs: []prismSlab{{regions: regions, z0: oa.proxy.z0, z1: oa.proxy.z1,
			z0Delta: z0Delta, z1Delta: z1Delta}},
		frame: oa.proxy.frame, xform: oa.proxy.xform,
		sectionDelta: proofbound.AbsSumUpper(max(inputDelta, disjointWalk), cutDelta),
	}
	if err := stackedrecord.Falsify(ctx, stackedRecordOf(sp)); err != nil {
		return nil, false, err
	}
	return sp, true, nil
}

// provePrismRegionsDisjoint is docs/mirror-pattern-design.md §6.3's
// disjointness read: a private scene holds every region's outer, and
// sketch's arrangement must return exactly one valid cell per region, each
// reproducing that region's outer with every edge whole and carrying no hole.
// Anything else — a Partial edge, a cell with a hole, an extra, missing or
// invalid cell — is "not proven", a silent miss. The check can only refuse.
// walk is the scene's walk charge over the regions' own segments.
func provePrismRegionsDisjoint(ctx context.Context, budget *proofbound.WorkBudget, regions []ProfileRecord) (bool, float64, error) {
	outers := make([]ProfileRecord, len(regions))
	for i, region := range regions {
		outers[i] = ProfileRecord{Outer: region.Outer}
	}
	if err := prismGroupWithinCap(budget, "disjointness", outers); err != nil {
		return false, 0, err
	}
	s, tags, sceneDelta, err := buildPrismSceneRegions(budget, outers, nil, &prismReexpression{Identity: true})
	if err != nil {
		return false, 0, err
	}
	profiles, err := prismProfilesContext(ctx, s.Profiles)
	if err != nil {
		return false, 0, err
	}
	if len(profiles) != len(regions) {
		return false, 0, nil
	}
	claimed := make([]bool, len(profiles))
	for r := range regions {
		want, err := prismcells.RegionLoopEntitySet(budget, tags, false, r, -1)
		if err != nil {
			return false, 0, err
		}
		match, resolved, err := prismcells.FindLoopMatch(budget, profiles, want, nil)
		if err != nil {
			return false, 0, err
		}
		if !resolved || !match.Valid {
			return false, 0, nil
		}
		for i, p := range profiles {
			if p != match {
				continue
			}
			if claimed[i] {
				return false, 0, nil
			}
			claimed[i] = true
		}
	}
	return true, sceneDelta.A, nil
}
