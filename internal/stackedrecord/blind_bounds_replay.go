package stackedrecord

import (
	"context"
	"math"

	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// replayBlindSourceBounds reuses the authenticated target producer's box only
// when its full outer wall and both end levels survive the blind internal cut.
// The clearance certificate proves every new hole stays inside that wall.
func replayBlindSourceBounds(ctx context.Context, sp Payload, source *topo.Body,
	prepared *replayBlindBuildPlan, cert *prismcells.ReplayBlindScene,
	budget *proofbound.ReplayBudget) (measurement.Box, bool, error) {
	if source == nil || prepared == nil || budget == nil || cert == nil ||
		!source.Solid || source.Kind != topo.BodySolid ||
		source.Origin.Role != provenance.RoleBody || len(source.OperationWarnings) != 0 ||
		len(sp.Slabs) != 2 || len(sp.Slabs[0].Regions) != 1 || len(sp.Slabs[1].Regions) != 1 {
		return measurement.Box{}, false, nil
	}
	if err := budget.SpendWork(ctx, 64); err != nil {
		return measurement.Box{}, false, err
	}
	actual, ok := source.Payload.(prismrecord.Payload)
	if !ok {
		return measurement.Box{}, false, nil
	}
	target := prepared.target
	if !replaySameFrameBits(actual.Frame, target.Frame) ||
		!replaySameTransformBits(actual.Xform, target.Xform) ||
		math.Float64bits(actual.Z0) != math.Float64bits(target.Z0) ||
		math.Float64bits(actual.Z1) != math.Float64bits(target.Z1) ||
		math.Float64bits(actual.SectionDelta) != math.Float64bits(target.SectionDelta) ||
		math.Float64bits(actual.Z0Delta) != math.Float64bits(target.Z0Delta) ||
		math.Float64bits(actual.Z1Delta) != math.Float64bits(target.Z1Delta) ||
		!replaySameFrameBits(sp.Frame, target.Frame) ||
		!replaySameTransformBits(sp.Xform, target.Xform) ||
		math.Float64bits(sp.Slabs[0].Z0) != math.Float64bits(target.Z0) ||
		math.Float64bits(sp.Slabs[1].Z1) != math.Float64bits(target.Z1) {
		return measurement.Box{}, false, nil
	}
	same, err := replayEqualBlindProfile(ctx, actual.Profile, target.Profile, budget)
	if err != nil || !same {
		return measurement.Box{}, false, err
	}
	for _, slab := range sp.Slabs {
		same, err = replayEqualBlindLoop(ctx, slab.Regions[0].Outer, target.Profile.Outer, budget)
		if err != nil || !same {
			return measurement.Box{}, false, err
		}
	}
	outerColumns := 0
	for _, col := range prepared.columns {
		if err := budget.SpendWork(ctx, 1); err != nil {
			return measurement.Box{}, false, err
		}
		if col.LoopIndex != 0 {
			continue
		}
		same, err = replayEqualBlindLoop(ctx, col.Loop, target.Profile.Outer, budget)
		if err != nil || !same || col.Start != 0 || col.End != 1 || col.Region != 0 {
			return measurement.Box{}, false, err
		}
		outerColumns++
	}
	if outerColumns != 1 {
		return measurement.Box{}, false, nil
	}
	clearance, err := prismcells.ReplayGroupSourceClearance(ctx, cert, budget)
	if err != nil || !clearance {
		return measurement.Box{}, false, err
	}
	box, sealed, err := source.ReplayPrismBuiltBounds(ctx, budget)
	if err != nil || !sealed {
		return measurement.Box{}, false, err
	}
	if err := budget.SpendWork(ctx, 8); err != nil {
		return measurement.Box{}, false, err
	}
	if !measurement.FiniteValues(box.Min.X, box.Min.Y, box.Min.Z,
		box.Max.X, box.Max.Y, box.Max.Z, box.Bound.Base()) ||
		box.Min.X > box.Max.X || box.Min.Y > box.Max.Y || box.Min.Z > box.Max.Z ||
		box.Bound.Base() < 0 || box.Exactness != measurement.ExactnessOf(box.Bound.Base()) {
		return measurement.Box{}, false, nil
	}
	return box, true, nil
}
