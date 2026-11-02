package stackedrecord

import (
	"context"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/prismplacement"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// replayBlindBuildPlan is private preparation only. It contains no body and
// cannot be published before topology and output verification are complete.
type replayBlindBuildPlan struct {
	columns    []WallColumn
	rows       [][]SlabLoop
	patches    []Patch
	walks      []*momentinput.ProfileWalks
	target     prismrecord.Payload
	sourceBody *topo.Body
	cert       *prismcells.ReplayBlindScene
}

func prepareBlindBuildPlanParts(ctx context.Context, sp Payload, target prismrecord.Payload, section prismcells.ReplayBlindSection,
	inner float64, top bool, budget *proofbound.ReplayBudget) (*replayBlindBuildPlan, bool, error) {
	if budget == nil {
		return nil, false, nil
	}
	columns, rows, ok, err := replayBlindColumns(ctx, sp, budget)
	if err != nil || !ok {
		return nil, false, err
	}
	patches, ok, err := replayBlindPatches(ctx, sp, target, section, inner, top, columns, rows, budget)
	if err != nil || !ok {
		return nil, false, err
	}
	walks, ok, err := replayBlindSourceWalks(ctx, columns, target, budget)
	if err != nil || !ok {
		return nil, false, err
	}
	if err := budget.SpendWork(ctx, 1); err != nil {
		return nil, false, err
	}
	if err := budget.ReserveRecords(ctx, 1); err != nil {
		return nil, false, err
	}
	if err := budget.ReserveArray(ctx, 1, uint64(unsafe.Sizeof(replayBlindBuildPlan{}))); err != nil {
		return nil, false, err
	}
	return &replayBlindBuildPlan{columns: columns, rows: rows, patches: patches, walks: walks, target: target}, true, nil
}

// prepareBlindBuildReplay rebinds both source placements and records to the
// original scene before and after preparing the isolated column and patch data.
func prepareBlindBuildReplay(ctx context.Context, sp Payload, target prismrecord.Payload, tool Payload,
	section prismcells.ReplayBlindSection, inner float64, top bool, cert *prismcells.ReplayBlindScene,
	budget *proofbound.ReplayBudget) (*replayBlindBuildPlan, bool, error) {
	if budget == nil {
		return nil, false, nil
	}
	if err := budget.SpendWork(ctx, 1); err != nil {
		return nil, false, err
	}
	if !tool.IsGroup() {
		return nil, false, nil
	}
	slab := tool.Slabs[0]
	place := prismplacement.Operand{Frame: tool.Frame, Xform: tool.Xform, Z0: slab.Z0, Z1: slab.Z1,
		Z0Delta: slab.Z0Delta, Z1Delta: slab.Z1Delta, Z0Denoted: slab.Z0Denoted, Z1Denoted: slab.Z1Denoted}
	bind := func() (bool, error) {
		return prismcells.ReplayBlindSceneMatches(ctx, cert, target.Profile, slab.Regions,
			target.PlacementOperand(), place, target.SectionDelta, tool.SectionDelta, budget)
	}
	ok, err := bind()
	if err != nil || !ok {
		return nil, false, err
	}
	certified, ok, err := prismcells.ReplayBlindGroupSection(ctx, cert, budget)
	if err != nil || !ok {
		return nil, false, err
	}
	if certified.SectionDelta != section.SectionDelta || len(certified.HoleRegions) != len(section.HoleRegions) {
		return nil, false, nil
	}
	for i := range certified.HoleRegions {
		if err := budget.SpendWork(ctx, 1); err != nil {
			return nil, false, err
		}
		if certified.HoleRegions[i] != section.HoleRegions[i] {
			return nil, false, nil
		}
	}
	same, err := replayEqualBlindProfile(ctx, certified.Profile, section.Profile, budget)
	if err != nil || !same {
		return nil, false, err
	}
	plan, ok, err := prepareBlindBuildPlanParts(ctx, sp, target, certified, inner, top, budget)
	if err != nil || !ok {
		return nil, false, err
	}
	ok, err = bind()
	if err != nil || !ok {
		return nil, false, err
	}
	return plan, true, nil
}
