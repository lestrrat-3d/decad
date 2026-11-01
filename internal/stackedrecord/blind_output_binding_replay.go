package stackedrecord

import (
	"context"
	"math"

	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/prismplacement"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

func replaySameVecBits(a, b r3.Vec) bool {
	return math.Float64bits(a.X) == math.Float64bits(b.X) && math.Float64bits(a.Y) == math.Float64bits(b.Y) && math.Float64bits(a.Z) == math.Float64bits(b.Z)
}
func replaySameFrameBits(a, b r3.Frame) bool {
	return replaySameVecBits(a.Origin(), b.Origin()) && replaySameVecBits(a.U(), b.U()) && replaySameVecBits(a.V(), b.V()) && replaySameVecBits(a.N(), b.N())
}
func replaySameTransformBits(a, b r3.Transform) bool {
	x, y := a.Basis(), b.Basis()
	return replaySameVecBits(x.EX, y.EX) && replaySameVecBits(x.EY, y.EY) && replaySameVecBits(x.EZ, y.EZ) && replaySameVecBits(a.Translation(), b.Translation())
}

// ReplayBlindOutputBinding revalidates the output's stored two-slab payload
// against its original scene. It does not validate topology or face lineage.
func ReplayBlindOutputBinding(ctx context.Context, out *topo.Body, target prismrecord.Payload, tool Payload, cert *prismcells.ReplayBlindScene, budget *proofbound.ReplayBudget) ([]int, bool, bool, error) {
	if budget == nil {
		return nil, false, false, nil
	}
	if err := budget.SpendWork(ctx, 128); err != nil {
		return nil, false, false, err
	}
	if out == nil || !tool.IsGroup() {
		return nil, false, false, nil
	}
	p, ok := out.Payload.(Payload)
	if !ok {
		return nil, false, false, nil
	}
	slab := tool.Slabs[0]
	place := prismplacement.Operand{Frame: tool.Frame, Xform: tool.Xform, Z0: slab.Z0, Z1: slab.Z1, Z0Delta: slab.Z0Delta, Z1Delta: slab.Z1Delta, Z0Denoted: slab.Z0Denoted, Z1Denoted: slab.Z1Denoted}
	m, ok, err := prismplacement.ReplayParallelSignedMap(ctx, target.PlacementOperand(), place, budget)
	if err != nil || !ok {
		return nil, false, false, err
	}
	top, ok, err := prismplacement.ReplayBlindLevels(ctx, target.PlacementOperand(), place, m.Shift, budget)
	if err != nil || !ok {
		return nil, false, false, err
	}
	expected, regions, ok, err := ReplayBlindGroupPayload(ctx, target, tool, cert, budget)
	if err != nil || !ok {
		return nil, false, false, err
	}
	section, ok, err := prismcells.ReplayBlindGroupSection(ctx, cert, budget)
	if err != nil || !ok {
		return nil, false, false, err
	}
	if len(p.Slabs) != 2 || len(expected.Slabs) != 2 || len(p.Interfaces) != 1 || len(expected.Interfaces) != 1 || len(p.Upstream) != 0 || !replaySameFrameBits(p.Frame, expected.Frame) || !replaySameTransformBits(p.Xform, expected.Xform) || math.Float64bits(p.SectionDelta) != math.Float64bits(expected.SectionDelta) {
		return nil, false, false, nil
	}
	for side := range 2 {
		actual, wanted := p.Interfaces[0].LowerExposed, expected.Interfaces[0].LowerExposed
		if side == 1 {
			actual, wanted = p.Interfaces[0].UpperExposed, expected.Interfaces[0].UpperExposed
		}
		if len(actual) != len(wanted) {
			return nil, false, false, nil
		}
		for i := range actual {
			if err := budget.SpendWork(ctx, 32); err != nil {
				return nil, false, false, err
			}
			same, err := replayEqualBlindProfile(ctx, actual[i], wanted[i], budget)
			if err != nil || !same {
				return nil, false, false, err
			}
		}
	}
	for i := range p.Slabs {
		if err := budget.SpendWork(ctx, 64); err != nil {
			return nil, false, false, err
		}
		a, b := p.Slabs[i], expected.Slabs[i]
		if math.Float64bits(a.Z0) != math.Float64bits(b.Z0) || math.Float64bits(a.Z1) != math.Float64bits(b.Z1) || math.Float64bits(a.Z0Delta) != math.Float64bits(b.Z0Delta) || math.Float64bits(a.Z1Delta) != math.Float64bits(b.Z1Delta) {
			return nil, false, false, nil
		}
	}
	valid, err := replayValidateBlindGroup(ctx, p, target, section, expected.Slabs[0].Z1, top, budget)
	if err != nil || !valid {
		return nil, false, false, err
	}
	return regions, top, true, nil
}
