package prismboolean

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/prismplacement"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/stackedrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// This file is agent-docs/general-boolean-design.md §3's class A5: prism-boolean's
// analytic paths over operands and results with several regions. A prism
// group (agent-docs/mirror-pattern-design.md §6.3) is a one-slab
// stackedrecord.Payload whose regions are pairwise disjoint lumps on one frame
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
// displacement charges every crossing (agent-docs/general-boolean-design.md §3
// A6, prismSceneDelta.ChargeCrossings) and never refuses: a crossing with no
// charge, or any of those errors on such a scene, sends the pair to the mesh
// path (prismcells.AmplifiedFallback).

// groupOperand is one operand read as regions over one interval: a prism is
// one region, a prism group one region per lump. proxy carries the frame,
// placement, interval, level displacements and section displacement the
// gates read, with the first region as its profile.
type groupOperand struct {
	proxy   prismrecord.Payload
	regions []momentinput.Profile
	group   bool
}

func groupOperandOf(payload topo.Payload) (groupOperand, bool) {
	switch p := payload.(type) {
	case prismrecord.Payload:
		return groupOperand{proxy: p, regions: []momentinput.Profile{p.Profile}}, true
	case stackedrecord.Payload:
		if !p.IsGroup() {
			return groupOperand{}, false
		}
		slab := p.Slabs[0]
		proxy := prismrecord.Payload{Profile: slab.Regions[0], Frame: p.Frame, Xform: p.Xform,
			Z0: slab.Z0, Z1: slab.Z1, Z0Delta: slab.Z0Delta, Z1Delta: slab.Z1Delta,
			Z0Denoted: slab.Z0Denoted, Z1Denoted: slab.Z1Denoted, SectionDelta: p.SectionDelta}
		return groupOperand{proxy: proxy, regions: slab.Regions, group: true}, true
	default:
		return groupOperand{}, false
	}
}

// admitGroupPair is G1-G4 on the two proxies, then G4, G6 (when holeFree) and
// the trimmed-circular refusal on every region of both.
func admitGroupPair(budget *proofbound.WorkBudget, op meshbool.OperationKind,
	oa, ob groupOperand, holeFreeA, holeFreeB bool) (bool, error) {
	if _, _, ok, err := prismcells.AdmitPairBudget(budget, op, oa.proxy, ob.proxy); err != nil || !ok {
		return false, err
	}
	for _, op := range []struct {
		o        groupOperand
		holeFree bool
	}{{oa, holeFreeA}, {ob, holeFreeB}} {
		if ok, err := prismcells.AdmitGroupRegions(budget, op.o.regions, op.holeFree); err != nil || !ok {
			return false, err
		}
	}
	return true, nil
}

// TryGroupCut is A5's Cut with a prism-group tool on a prism target. The
// clean-nesting match requires the target's cell to carry the target's own
// loops whole plus one new hole per tool region, each reproducing that
// region's outer whole; that cell is the result, recorded through the seam.
// Otherwise the crossing sub-case classifies every cell against the target
// and against the group's regions together, and keeps target-and-not-tool.
func TryGroupCut(ctx context.Context, a, b topo.Payload) (prismrecord.Payload, bool, error) {
	target, aok := a.(prismrecord.Payload)
	tool, bok := groupOperandOf(b)
	if !aok || !bok || !tool.group {
		return prismrecord.Payload{}, false, nil
	}
	budget := proofbound.NewWorkBudget(ctx)
	if err := budget.Err(); err != nil {
		return prismrecord.Payload{}, false, err
	}
	targetOp := groupOperand{proxy: target, regions: []momentinput.Profile{target.Profile}}
	if ok, err := admitGroupPair(budget, meshbool.OpCut, targetOp, tool, false, true); err != nil || !ok {
		return prismrecord.Payload{}, false, err
	}
	if !prismplacement.CutZIntervalSpans(target.PlacementOperand(), tool.proxy.PlacementOperand()) { // G5
		return prismrecord.Payload{}, false, nil
	}
	scene, ok, err := prismcells.BuildGroupScene(ctx, budget, targetOp.regions, tool.regions,
		target.PlacementOperand(), tool.proxy.PlacementOperand(), target.SectionDelta, tool.proxy.SectionDelta, "cut")
	if err != nil || !ok {
		return prismrecord.Payload{}, false, err
	}
	sceneDelta := scene.Delta
	result := prismrecord.Payload{Frame: target.Frame, Xform: target.Xform,
		Z0: target.Z0, Z1: target.Z1, Z0Delta: target.Z0Delta, Z1Delta: target.Z1Delta}
	inA, inB := sceneDelta.Incoming(target.SectionDelta, tool.proxy.SectionDelta, scene.ReexpressionDelta)
	inputDelta := max(inA, inB)

	match, nested, err := prismcells.GroupCutMatch(budget, scene.Profiles, scene.Tags,
		len(target.Profile.Holes), len(tool.regions))
	if err != nil {
		return prismrecord.Payload{}, false, err
	}
	if nested {
		profile, err := prismcells.RecordProfileContext(ctx, scene.Sketch, match)
		if err != nil {
			return prismrecord.Payload{}, false, err
		}
		// Every edge of the matched cell is whole: no cut charge.
		result.Profile = profile
		result.SectionDelta = inputDelta
		return result, true, nil
	}

	// The crossing sub-case, scoped to a hole-free target as prism-boolean's.
	if len(target.Profile.Holes) != 0 {
		return prismrecord.Payload{}, false, nil
	}
	selected, resolved, err := prismcells.GroupCutCells(budget, scene)
	if err != nil || !resolved {
		return prismrecord.Payload{}, false, err
	}
	merged, cutDelta, resolved, err := prismcells.Merge(budget, selected, "cut")
	if fallBack, err := prismcells.AmplifiedFallback(sceneDelta.Amplified, err); fallBack || err != nil || !resolved {
		return prismrecord.Payload{}, false, err
	}
	if fallBack, err := prismcells.AmplifiedFallback(sceneDelta.Amplified, prismcells.AuditMergeSection(budget, target, merged)); fallBack || err != nil {
		return prismrecord.Payload{}, false, err
	}
	result.Profile = merged
	result.SectionDelta = sceneDelta.Merged(target.SectionDelta, tool.proxy.SectionDelta, scene.ReexpressionDelta, cutDelta)
	return result, true, nil
}

// TryGroupUnion is A5's Union over equal intervals where either operand is a
// prism group, or two prisms whose select-all survivors close into several
// loops. The survivors of the select-all merge are chained into closed loops.
// One loop is a prism. Two or more loops proven pairwise apart by
// prismcells.ProveGroupApart form a prism group. Loops that touch at one
// point, which the arrangement leaves uncut, are a silent miss to the mesh
// path.
func TryGroupUnion(ctx context.Context, a, b topo.Payload) (topo.Payload, bool, error) {
	oa, aok := groupOperandOf(a)
	ob, bok := groupOperandOf(b)
	if !aok || !bok {
		return nil, false, nil
	}
	budget := proofbound.NewWorkBudget(ctx)
	if err := budget.Err(); err != nil {
		return nil, false, err
	}
	if ok, err := admitGroupPair(budget, meshbool.OpUnion, oa, ob, true, true); err != nil || !ok {
		return nil, false, err
	}
	if !prismplacement.UnionZIntervalMatches(oa.proxy.PlacementOperand(), ob.proxy.PlacementOperand()) { // G5
		return nil, false, nil
	}
	scene, ok, err := prismcells.BuildGroupScene(ctx, budget, oa.regions, ob.regions,
		oa.proxy.PlacementOperand(), ob.proxy.PlacementOperand(), oa.proxy.SectionDelta, ob.proxy.SectionDelta, "union")
	if err != nil || !ok {
		return nil, false, err
	}
	payload, ok, err := stackedrecord.GroupUnionTail(ctx, budget, scene, oa.proxy, ob.proxy)
	if fallBack, err := prismcells.AmplifiedFallback(scene.Delta.Amplified, err); fallBack || err != nil {
		return nil, false, err
	}
	return payload, ok, nil
}
