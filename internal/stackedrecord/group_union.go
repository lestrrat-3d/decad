package stackedrecord

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// GroupUnionTail audits each assembled loop and publishes one prism or a
// group after proving its regions disjoint. The caller routes its errors
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
	if len(regions) == 1 {
		return prismrecord.Payload{Profile: regions[0], Frame: a.Frame, Xform: a.Xform,
			Z0: a.Z0, Z1: a.Z1, Z0Delta: z0Delta, Z1Delta: z1Delta,
			SectionDelta: proofbound.AbsSumUpper(inputDelta, cutDelta)}, true, nil
	}
	disjoint, disjointWalk, err := prismcells.ProveGroupDisjoint(ctx, budget, regions)
	if err != nil || !disjoint {
		return nil, false, err
	}
	sp := Payload{
		Slabs: []Slab{{Regions: regions, Z0: a.Z0, Z1: a.Z1,
			Z0Delta: z0Delta, Z1Delta: z1Delta}},
		Frame: a.Frame, Xform: a.Xform,
		SectionDelta: proofbound.AbsSumUpper(max(inputDelta, disjointWalk), cutDelta),
	}
	if err := Falsify(ctx, Record{Slabs: sp.Slabs, Interfaces: sp.Interfaces}); err != nil {
		return nil, false, err
	}
	return sp, true, nil
}
