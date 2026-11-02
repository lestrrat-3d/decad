package stackedrecord

import (
	"context"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/prismplacement"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// GroupUnionTail audits each assembled loop and publishes one prism or a
// group after proving its regions apart: disjoint, and touching nowhere, not
// even at one tangent point (prismcells.ProveGroupApart). The caller routes its errors
// through prismcells.AmplifiedFallback.
func GroupUnionTail(ctx context.Context, budget *proofbound.WorkBudget, scene prismcells.GroupScene,
	a, b prismrecord.Payload) (topo.Payload, bool, error) {
	return groupUnionTail(ctx, budget, scene, a, b, nil)
}

// GroupUnionTailWithOrigins returns the general union's surviving edge origins.
func GroupUnionTailWithOrigins(ctx context.Context, budget *proofbound.WorkBudget, scene prismcells.GroupScene,
	a, b prismrecord.Payload) (topo.Payload, [][]prismcells.Origin, bool, error) {
	var origins [][]prismcells.Origin
	payload, ok, err := groupUnionTail(ctx, budget, scene, a, b, &origins)
	return payload, origins, ok, err
}

func groupUnionTail(ctx context.Context, budget *proofbound.WorkBudget, scene prismcells.GroupScene,
	a, b prismrecord.Payload, origins *[][]prismcells.Origin) (topo.Payload, bool, error) {
	var loops []sectionrecord.LoopRecord
	var cutDelta float64
	var resolved bool
	var err error
	if origins == nil {
		loops, cutDelta, resolved, err = prismcells.GroupUnionLoops(budget, scene)
	} else {
		loops, *origins, cutDelta, resolved, err = prismcells.GroupUnionLoopsWithOrigins(budget, scene)
	}
	if err != nil || !resolved {
		return nil, false, err
	}
	return groupUnionTailFromLoops(ctx, budget, scene, a, b, loops, cutDelta)
}

// GroupWholeCircleUnionTailWithOrigins retains the source tag for each segment in a
// disjoint whole-circle group. The caller must authenticate the input source
// records and carry these tags across private Union steps.
func GroupWholeCircleUnionTailWithOrigins(ctx context.Context, budget *proofbound.WorkBudget, scene prismcells.GroupScene,
	a, b prismrecord.Payload) (topo.Payload, [][]prismcells.Origin, bool, error) {
	loops, origins, cutDelta, resolved, err := prismcells.GroupWholeCircleOrigins(budget, scene)
	if err != nil || !resolved {
		return nil, nil, false, err
	}
	payload, ok, err := groupUnionTailFromLoops(ctx, budget, scene, a, b, loops, cutDelta)
	if err != nil || !ok {
		return nil, nil, false, err
	}
	return payload, origins, true, nil
}

func groupUnionTailFromLoops(ctx context.Context, budget *proofbound.WorkBudget, scene prismcells.GroupScene,
	a, b prismrecord.Payload, loops []sectionrecord.LoopRecord, cutDelta float64) (topo.Payload, bool, error) {
	// §7's incoming terms with A6's crossing charge.
	inA, inB := scene.Delta.Incoming(a.SectionDelta, b.SectionDelta, scene.ReexpressionDelta)
	inputDelta := max(inA, inB, scene.Delta.Crossing)
	if err := budget.Reserve(uint64(len(loops))); err != nil {
		return nil, false, err
	}
	if err := budget.ReserveBytesMul(uint64(len(loops)), uint64(unsafe.Sizeof(momentinput.Profile{}))); err != nil {
		return nil, false, err
	}
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
