package stackedrecord

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/prismplacement"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// ReplayBlindGroupBuild uses the already charged two-slab validation and
// prepared columns/patches instead of repeating those unmetered algorithms.
// Its wall geometry, measurements and output checks still need complete
// accounting before this result can enter a public Boolean.
func ReplayBlindGroupBuild(ctx context.Context, env topo.Env, out Payload, target prismrecord.Payload,
	tool Payload, cert *prismcells.ReplayBlindScene, budget *proofbound.ReplayBudget,
) (*topo.Body, bool, error) {
	result, _, ok, err := ReplayBlindGroupBuildWithEvents(ctx, env, out, target, tool, cert, budget)
	return result, ok, err
}

// ReplayBlindGroupBuildWithEvents returns the sealed construction face log
// beside the uncommitted body for a separate bounded ancestry proof.
func ReplayBlindGroupBuildWithEvents(ctx context.Context, env topo.Env, out Payload, target prismrecord.Payload,
	tool Payload, cert *prismcells.ReplayBlindScene, budget *proofbound.ReplayBudget,
) (*topo.Body, *ReplayBlindFaceLog, bool, error) {
	return replayBlindGroupBuildWithEvents(ctx, env, out, target, nil, tool, cert, budget)
}

// ReplayBlindGroupBuildWithSource binds the target producer's original box to
// the cut's unchanged outer wall and returns the same sealed face log.
func ReplayBlindGroupBuildWithSource(ctx context.Context, env topo.Env, out Payload, targetBody *topo.Body,
	tool Payload, cert *prismcells.ReplayBlindScene, budget *proofbound.ReplayBudget,
) (*topo.Body, *ReplayBlindFaceLog, bool, error) {
	if targetBody == nil {
		return nil, nil, false, nil
	}
	target, ok := targetBody.Payload.(prismrecord.Payload)
	if !ok {
		return nil, nil, false, nil
	}
	return replayBlindGroupBuildWithEvents(ctx, env, out, target, targetBody, tool, cert, budget)
}

func replayBlindGroupBuildWithEvents(ctx context.Context, env topo.Env, out Payload, target prismrecord.Payload,
	targetBody *topo.Body, tool Payload, cert *prismcells.ReplayBlindScene, budget *proofbound.ReplayBudget,
) (*topo.Body, *ReplayBlindFaceLog, bool, error) {
	if budget == nil {
		return nil, nil, false, nil
	}
	if err := budget.SpendWork(ctx, 1); err != nil {
		return nil, nil, false, err
	}
	if !tool.IsGroup() || len(out.Slabs) != 2 || len(out.Interfaces) != 1 {
		return nil, nil, false, nil
	}
	toolSlab := tool.Slabs[0]
	place := prismplacement.Operand{Frame: tool.Frame, Xform: tool.Xform, Z0: toolSlab.Z0, Z1: toolSlab.Z1,
		Z0Delta: toolSlab.Z0Delta, Z1Delta: toolSlab.Z1Delta,
		Z0Denoted: toolSlab.Z0Denoted, Z1Denoted: toolSlab.Z1Denoted}
	mapping, ok, err := prismplacement.ReplayParallelSignedMap(ctx, target.PlacementOperand(), place, budget)
	if err != nil || !ok {
		return nil, nil, false, err
	}
	top, ok, err := prismplacement.ReplayBlindLevels(ctx, target.PlacementOperand(), place, mapping.Shift, budget)
	if err != nil || !ok {
		return nil, nil, false, err
	}
	section, ok, err := prismcells.ReplayBlindGroupSection(ctx, cert, budget)
	if err != nil || !ok {
		return nil, nil, false, err
	}
	inner := out.Slabs[0].Z1
	valid, err := replayValidateBlindGroup(ctx, out, target, section, inner, top, budget)
	if err != nil || !valid {
		return nil, nil, false, err
	}
	prepared, ok, err := prepareBlindBuildReplay(ctx, out, target, tool, section, inner, top, cert, budget)
	if err != nil || !ok {
		return nil, nil, false, err
	}
	prepared.sourceBody, prepared.cert = targetBody, cert
	if err := ctx.Err(); err != nil {
		return nil, nil, false, err
	}
	capacity := len(out.Slabs[0].Regions) + len(out.Slabs[len(out.Slabs)-1].Regions) + len(prepared.patches)
	for _, col := range prepared.columns {
		if err := budget.SpendWork(ctx, 1); err != nil {
			return nil, nil, false, err
		}
		if len(col.Loop.Segments) > int(^uint(0)>>1)-capacity {
			return nil, nil, false, decaderr.ErrUnsupported
		}
		capacity += len(col.Loop.Segments)
	}
	log, err := NewReplayBlindFaceLog(ctx, budget, capacity)
	if err != nil {
		return nil, nil, false, err
	}
	result, err := build(ctx, env, out, OwnPlan{}, budget, prepared, log, nil, nil)
	if err != nil {
		return nil, nil, false, err
	}
	return result, log, true, nil
}
