package stackedrecord

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/prismplacement"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// GroupUnionTail audits each assembled loop and publishes one prism or a
// group after proving its regions apart: disjoint, and touching nowhere, not
// even at one tangent point (prismcells.ProveGroupApart). The caller routes its errors
// through prismcells.AmplifiedFallback.
func GroupUnionTail(ctx context.Context, budget *proofbound.WorkBudget, scene prismcells.GroupScene,
	a, b prismrecord.Payload) (topo.Payload, bool, error) {
	loops, cutDelta, resolved, err := prismcells.GroupUnionLoops(budget, scene)
	if err != nil || !resolved {
		return nil, false, err
	}
	// §7's incoming terms with A6's crossing charge.
	inA, inB := scene.Delta.Incoming(a.SectionDelta, b.SectionDelta, scene.ReexpressionDelta)
	inputDelta := max(inA, inB, scene.Delta.Crossing)
	regions := make([]momentinput.Profile, len(loops))
	for i, loop := range loops {
		regions[i] = momentinput.Profile{Outer: loop}
		// Point of no return (§3.4): §6's audit on each assembled loop.
		if err := prismcells.AuditMergeSection(budget, a, regions[i]); err != nil {
			return nil, false, err
		}
	}
	z0Delta, z1Delta := max(a.Z0Delta, b.Z0Delta), max(a.Z1Delta, b.Z1Delta)
	// G5 proved both operands' levels denote one rational at each end
	// (prismplacement.UnionZIntervalMatches), so the shared level keeps it.
	z0Denoted := prismplacement.DenoteLevel(a.DenotedZ0(), a.Z0, z0Delta)
	z1Denoted := prismplacement.DenoteLevel(a.DenotedZ1(), a.Z1, z1Delta)
	if len(regions) == 1 {
		return prismrecord.Payload{Profile: regions[0], Frame: a.Frame, Xform: a.Xform,
			Z0: a.Z0, Z1: a.Z1, Z0Delta: z0Delta, Z1Delta: z1Delta, Z0Denoted: z0Denoted, Z1Denoted: z1Denoted,
			SectionDelta: proofbound.AbsSumUpper(inputDelta, cutDelta)}, true, nil
	}
	disjoint, disjointWalk, err := prismcells.ProveGroupApart(ctx, budget, regions)
	if err != nil || !disjoint {
		return nil, false, err
	}
	sp := Payload{
		Slabs: []Slab{{Regions: regions, Z0: a.Z0, Z1: a.Z1,
			Z0Delta: z0Delta, Z1Delta: z1Delta, Z0Denoted: z0Denoted, Z1Denoted: z1Denoted}},
		Frame: a.Frame, Xform: a.Xform,
		SectionDelta: proofbound.AbsSumUpper(max(inputDelta, disjointWalk), cutDelta),
	}
	if err := Falsify(ctx, Record{Slabs: sp.Slabs, Interfaces: sp.Interfaces}); err != nil {
		return nil, false, err
	}
	return sp, true, nil
}
