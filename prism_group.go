package decad

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/stackedrecord"

	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/prismplacement"
	"github.com/lestrrat-3d/decad/internal/proofbound"
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
	regions []profileRecord
	group   bool
}

func prismGroupOperandOf(b *Body) (prismGroupOperand, bool) {
	switch p := b.payload.(type) {
	case prismPayload:
		return prismGroupOperand{proxy: p, regions: []profileRecord{p.profile}}, true
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
		if ok, err := prismcells.AdmitGroupRegions(budget, op.o.regions, op.holeFree); err != nil || !ok {
			return false, err
		}
	}
	return true, nil
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
	targetOp := prismGroupOperand{proxy: target, regions: []profileRecord{target.profile}}
	if ok, err := admitPrismGroupPair(budget, targetOp, tool, false, true); err != nil || !ok {
		return prismPayload{}, false, err
	}
	if !prismplacement.CutZIntervalSpans(prismPlacementOf(target), prismPlacementOf(tool.proxy)) { // G5
		return prismPayload{}, false, nil
	}
	scene, ok, err := prismcells.BuildGroupScene(ctx, budget, targetOp.regions, tool.regions,
		prismPlacementOf(target), prismPlacementOf(tool.proxy), target.sectionDelta, tool.proxy.sectionDelta, "cut")
	if err != nil || !ok {
		return prismPayload{}, false, err
	}
	sceneDelta := scene.Delta
	result := prismPayload{frame: target.frame, xform: target.xform,
		z0: target.z0, z1: target.z1, z0Delta: target.z0Delta, z1Delta: target.z1Delta}
	inA, inB := sceneDelta.Incoming(target.sectionDelta, tool.proxy.sectionDelta, scene.ReexpressionDelta)
	inputDelta := max(inA, inB)

	match, nested, err := prismcells.GroupCutMatch(budget, scene.Profiles, scene.Tags,
		len(target.profile.Holes), len(tool.regions))
	if err != nil {
		return prismPayload{}, false, err
	}
	if nested {
		profile, err := prismRecordProfileContext(ctx, scene.Sketch, match)
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
	selected, resolved, err := prismcells.GroupCutCells(budget, scene)
	if err != nil || !resolved {
		return prismPayload{}, false, err
	}
	merged, cutDelta, resolved, err := prismcells.Merge(budget, selected, "cut")
	if fallBack, err := prismcells.AmplifiedFallback(sceneDelta.Amplified, err); fallBack || err != nil || !resolved {
		return prismPayload{}, false, err
	}
	if fallBack, err := prismcells.AmplifiedFallback(sceneDelta.Amplified, auditPrismMergeSection(budget, target, merged)); fallBack || err != nil {
		return prismPayload{}, false, err
	}
	result.profile = merged
	result.sectionDelta = sceneDelta.Merged(target.sectionDelta, tool.proxy.sectionDelta, scene.ReexpressionDelta, cutDelta)
	return result, true, nil
}

// tryPrismGroupUnion is A5's Union over equal intervals where either operand
// is a prism group, or two prisms whose select-all survivors close into
// several loops. The survivors of the select-all merge are chained into
// closed loops. One loop is a prism. Two or more loops proven pairwise
// disjoint by prismcells.ProveGroupDisjoint form a prism group.
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
	if !prismplacement.UnionZIntervalMatches(prismPlacementOf(oa.proxy), prismPlacementOf(ob.proxy)) { // G5
		return nil, false, nil
	}
	scene, ok, err := prismcells.BuildGroupScene(ctx, budget, oa.regions, ob.regions,
		prismPlacementOf(oa.proxy), prismPlacementOf(ob.proxy), oa.proxy.sectionDelta, ob.proxy.sectionDelta, "union")
	if err != nil || !ok {
		return nil, false, err
	}
	payload, ok, err := prismGroupUnionTail(ctx, budget, scene, oa, ob)
	if fallBack, err := prismcells.AmplifiedFallback(scene.Delta.Amplified, err); fallBack || err != nil {
		return nil, false, err
	}
	return payload, ok, nil
}

// prismGroupUnionTail audits each assembled loop and publishes one prism or
// a group after proving its regions disjoint. Its errors are the
// caller's to route (prismcells.AmplifiedFallback).
func prismGroupUnionTail(ctx context.Context, budget *proofbound.WorkBudget, scene prismcells.GroupScene,
	oa, ob prismGroupOperand) (featurePayload, bool, error) {
	loops, cutDelta, resolved, err := prismcells.GroupUnionLoops(budget, scene)
	if err != nil || !resolved {
		return nil, false, err
	}
	// §7's incoming terms with A6's crossing charge.
	inA, inB := scene.Delta.Incoming(oa.proxy.sectionDelta, ob.proxy.sectionDelta, scene.ReexpressionDelta)
	inputDelta := max(inA, inB, scene.Delta.Crossing)
	regions := make([]profileRecord, len(loops))
	for i, loop := range loops {
		regions[i] = profileRecord{Outer: loop}
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
	disjoint, disjointWalk, err := prismcells.ProveGroupDisjoint(ctx, budget, regions)
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
