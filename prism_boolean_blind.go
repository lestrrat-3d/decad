package decad

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/prismplacement"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/stackedrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// tryBlindStackedCut admits a cleanly nested blind tool against a prism,
// including one that encloses existing holes. The private sketch scene proves
// the two slab sections before they are recorded.
func tryBlindStackedCut(ctx context.Context, a, b *Body) (stackedPrismPayload, bool, error) {
	budget := proofbound.NewWorkBudget(ctx)
	if err := budget.Err(); err != nil {
		return stackedPrismPayload{}, false, err
	}
	target, tool, ok, err := admitPrismPairBudget(budget, meshbool.OpCut, a, b)
	if err != nil || !ok {
		return stackedPrismPayload{}, false, err
	}
	return stackedrecord.BuildBlindCut(ctx, budget, target, tool)
}

// tryStackedBlindCut splits the slab containing a blind tool's inner end, or
// reuses an exact existing interface. Every reached slab needs its own
// whole-loop cut proof, and the resulting interfaces must pass the audit.
func tryStackedBlindCut(ctx context.Context, a, b *Body) (stackedPrismPayload, bool, error) {
	sp, ok := a.n.Payload.(stackedPrismPayload)
	if !ok || len(sp.OuterRuns()) != 1 {
		return stackedPrismPayload{}, false, nil
	}
	for _, slab := range sp.Slabs {
		if len(slab.Regions) != 1 {
			return stackedPrismPayload{}, false, nil
		}
	}
	budget := proofbound.NewWorkBudget(ctx)
	outer := sp.OuterPrism()
	proxy := bornBody(&Body{n: topo.Body{Payload: outer}})
	_, tool, ok, err := admitPrismPairBudget(budget, meshbool.OpCut, proxy, b)
	if err != nil || !ok {
		return stackedPrismPayload{}, false, err
	}
	return stackedrecord.ApplyBlindCut(ctx, budget, sp, outer, tool)
}

// tryStackedThroughCut applies a spanning prism tool to every slab. Each
// private scene proves either a new region or that the tool lies wholly in an
// existing hole and leaves that slab unchanged.
func tryStackedThroughCut(ctx context.Context, a, b *Body) (stackedPrismPayload, bool, error) {
	sp, ok := a.n.Payload.(stackedPrismPayload)
	if !ok {
		return stackedPrismPayload{}, false, nil
	}
	// A union-built stack changes its outer loop at an interface. Its exposed
	// patches are not the exclusive-hole patches this cut re-derives below, so
	// it takes the mesh path (docs/stacked-prism-design.md §6).
	if len(sp.OuterRuns()) != 1 {
		return stackedPrismPayload{}, false, nil
	}
	budget := proofbound.NewWorkBudget(ctx)
	outer := sp.OuterPrism()
	proxy := bornBody(&Body{n: topo.Body{Payload: outer}})
	_, tool, ok, err := admitPrismPairBudget(budget, meshbool.OpCut, proxy, b)
	if err != nil || !ok {
		return stackedPrismPayload{}, false, err
	}
	if len(tool.Profile.Holes) != 0 ||
		!prismplacement.CutZIntervalSpans(outer.PlacementOperand(), tool.PlacementOperand()) {
		return stackedPrismPayload{}, false, nil
	}
	trimmed, err := prismcells.ProfileHasTrimmedCircularSource(budget, tool.Profile.Outer, tool.Profile.Holes)
	if err != nil || trimmed {
		return stackedPrismPayload{}, false, err
	}
	reexpress, err := prismcells.NewReexpression(outer.PlacementOperand(), tool.PlacementOperand())
	if err != nil {
		return stackedPrismPayload{}, false, err
	}
	active := make([]bool, len(sp.Slabs))
	for i := range active {
		active[i] = true
	}
	return stackedrecord.CutSlabs(ctx, budget, sp, outer, tool, reexpress, active, sp.Interfaces, -1)
}
