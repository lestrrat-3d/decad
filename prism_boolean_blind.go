package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/prismplacement"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
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
	if len(tool.Profile.Holes) != 0 {
		return stackedPrismPayload{}, false, nil
	}
	for _, profile := range []profileRecord{target.Profile, tool.Profile} {
		trimmed, err := prismcells.ProfileHasTrimmedCircularSource(budget, profile.Outer, profile.Holes)
		if err != nil {
			return stackedPrismPayload{}, false, err
		}
		if trimmed {
			return stackedPrismPayload{}, false, nil
		}
	}
	z0, z1, ok := prismplacement.ShiftedInterval(target.PlacementOperand(), tool.PlacementOperand())
	if !ok {
		return stackedPrismPayload{}, false, nil
	}
	a0, a1 := proofarith.FloatRat(target.Z0), proofarith.FloatRat(target.Z1)
	if a0 == nil || a1 == nil {
		return stackedPrismPayload{}, false, nil
	}
	openAtTop := z0.Cmp(a0) > 0 && z0.Cmp(a1) < 0 && z1.Cmp(a1) >= 0
	openAtBottom := z0.Cmp(a0) <= 0 && z1.Cmp(a0) > 0 && z1.Cmp(a1) < 0
	if !openAtTop && !openAtBottom {
		return stackedPrismPayload{}, false, nil
	}
	segments, withinCap, err := prismcells.RegionsWithinWorkCap(budget, target.Profile, tool.Profile)
	if err != nil {
		return stackedPrismPayload{}, false, err
	}
	if !withinCap {
		return stackedPrismPayload{}, false, fmt.Errorf(
			`%w: the analytic cut scene charges %d arranger segments against the cap of %d`,
			ErrUnsupported, segments, prismcells.MaxArrangementSegments)
	}
	reexpress, err := prismcells.NewReexpression(target.PlacementOperand(), tool.PlacementOperand())
	if err != nil {
		return stackedPrismPayload{}, false, err
	}
	scene, profiles, tags, sceneDelta, err := prismcells.CutCells(ctx, budget, target.Profile, tool.Profile, reexpress)
	if err != nil {
		return stackedPrismPayload{}, false, err
	}
	match, resolved, err := prismcells.MatchCut(budget, tags, profiles, len(target.Profile.Holes))
	if err != nil {
		return stackedPrismPayload{}, false, err
	}
	var enclosing prismcells.EnclosingCutMatch
	if !resolved {
		enclosing, resolved, err = prismcells.MatchEnclosingCut(budget, tags, profiles, len(target.Profile.Holes))
		if err != nil || !resolved {
			return stackedPrismPayload{}, false, err
		}
		match = enclosing.Outside
		if _, err := prismcells.RecordProfileContext(ctx, scene, enclosing.Inside); err != nil {
			return stackedPrismPayload{}, false, err
		}
	}
	_, err = prismcells.RecordProfileContext(ctx, scene, match)
	if err != nil {
		return stackedPrismPayload{}, false, err
	}
	cutRegion, err := prismcells.RecordArrangedProfileContext(ctx, match)
	if err != nil {
		return stackedPrismPayload{}, false, err
	}
	kept := target.Profile
	if len(enclosing.EnclosedHoles) != 0 {
		kept.Holes = make([]loopRecord, 0, len(enclosing.OutsideHoles))
		for _, i := range enclosing.OutsideHoles {
			kept.Holes = append(kept.Holes, target.Profile.Holes[i])
		}
	}
	cutRegion, err = prismcells.CanonicalStackedCut(budget, kept, match, tags, cutRegion)
	if err != nil {
		return stackedPrismPayload{}, false, err
	}
	inner := z0
	innerDelta := tool.Z0Delta
	if openAtBottom {
		inner, innerDelta = z1, tool.Z1Delta
	}
	innerHeld, _ := inner.Float64()
	shiftRound := proofarith.RationalFloatError(inner, innerHeld)
	if innerDelta == 0 {
		innerDelta = shiftRound
	} else {
		innerDelta = proofbound.AbsSumUpper(innerDelta, shiftRound)
	}
	sp := stackedPrismPayload{Frame: target.Frame, Xform: target.Xform, Slabs: make([]stackedrecord.Slab, 2),
		SectionDelta: max(proofbound.AbsSumUpper(target.SectionDelta, sceneDelta.A),
			proofbound.AbsSumUpper(tool.SectionDelta, sceneDelta.B, reexpress.Delta))}
	if openAtTop {
		sp.Slabs[0] = stackedrecord.Slab{Regions: []profileRecord{target.Profile},
			Z0: target.Z0, Z1: innerHeld, Z0Delta: target.Z0Delta, Z1Delta: innerDelta}
		sp.Slabs[1] = stackedrecord.Slab{Regions: []profileRecord{cutRegion},
			Z0: innerHeld, Z1: target.Z1, Z0Delta: innerDelta, Z1Delta: target.Z1Delta}
	} else {
		sp.Slabs[0] = stackedrecord.Slab{Regions: []profileRecord{cutRegion},
			Z0: target.Z0, Z1: innerHeld, Z0Delta: target.Z0Delta, Z1Delta: innerDelta}
		sp.Slabs[1] = stackedrecord.Slab{Regions: []profileRecord{target.Profile},
			Z0: innerHeld, Z1: target.Z1, Z0Delta: innerDelta, Z1Delta: target.Z1Delta}
	}
	if len(enclosing.EnclosedHoles) != 0 {
		innerHoles := make([]loopRecord, 0, len(enclosing.EnclosedHoles))
		for _, i := range enclosing.EnclosedHoles {
			innerHoles = append(innerHoles, target.Profile.Holes[i])
		}
		toolHole := cutRegion.Holes[len(cutRegion.Holes)-1]
		exposed, err := stackedrecord.EnclosingExposed(ctx, toolHole, innerHoles)
		if err != nil {
			return stackedPrismPayload{}, false, err
		}
		if openAtTop {
			sp.Interfaces = []stackedrecord.Interface{{LowerExposed: exposed}}
		} else {
			sp.Interfaces = []stackedrecord.Interface{{UpperExposed: exposed}}
		}
	} else {
		lowerOnly, upperOnly := stackedrecord.ExclusiveHoles(sp.Slabs[0].Regions[0], sp.Slabs[1].Regions[0])
		lowerExposed, err := stackedrecord.Exposed(ctx, upperOnly)
		if err != nil {
			return stackedPrismPayload{}, false, err
		}
		upperExposed, err := stackedrecord.Exposed(ctx, lowerOnly)
		if err != nil {
			return stackedPrismPayload{}, false, err
		}
		sp.Interfaces = []stackedrecord.Interface{{LowerExposed: lowerExposed, UpperExposed: upperExposed}}
	}
	return sp, true, nil
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
	if len(tool.Profile.Holes) != 0 {
		return stackedPrismPayload{}, false, nil
	}
	trimmed, err := prismcells.ProfileHasTrimmedCircularSource(budget, tool.Profile.Outer, tool.Profile.Holes)
	if err != nil || trimmed {
		return stackedPrismPayload{}, false, err
	}
	z0, z1, ok := prismplacement.ShiftedInterval(outer.PlacementOperand(), tool.PlacementOperand())
	if !ok {
		return stackedPrismPayload{}, false, nil
	}
	first := proofarith.FloatRat(sp.Slabs[0].Z0)
	last := proofarith.FloatRat(sp.Slabs[len(sp.Slabs)-1].Z1)
	if first == nil || last == nil {
		return stackedPrismPayload{}, false, nil
	}
	openAtTop := z0.Cmp(first) > 0 && z0.Cmp(last) < 0 && z1.Cmp(last) >= 0
	openAtBottom := z0.Cmp(first) <= 0 && z1.Cmp(first) > 0 && z1.Cmp(last) < 0
	if !openAtTop && !openAtBottom {
		return stackedPrismPayload{}, false, nil
	}
	inner, innerDelta := z0, tool.Z0Delta
	if openAtBottom {
		inner, innerDelta = z1, tool.Z1Delta
	}
	split, interfaceAt := -1, -1
	for k, slab := range sp.Slabs {
		lower, upper := proofarith.FloatRat(slab.Z0), proofarith.FloatRat(slab.Z1)
		if lower != nil && upper != nil && inner.Cmp(lower) > 0 && inner.Cmp(upper) < 0 {
			split = k
			break
		}
		if upper != nil && k < len(sp.Slabs)-1 && inner.Cmp(upper) == 0 {
			interfaceAt = k
			break
		}
	}
	if split < 0 && interfaceAt < 0 {
		return stackedPrismPayload{}, false, nil
	}
	reexpress, err := prismcells.NewReexpression(outer.PlacementOperand(), tool.PlacementOperand())
	if err != nil {
		return stackedPrismPayload{}, false, err
	}
	if interfaceAt >= 0 {
		// With a displaced level, the nominal equality does not prove which
		// slabs the true tool reaches. Keep this arm to exactly shared levels.
		if innerDelta != 0 || sp.Slabs[interfaceAt].Z1Delta != 0 ||
			sp.Slabs[interfaceAt+1].Z0Delta != 0 {
			return stackedPrismPayload{}, false, nil
		}
		active := make([]bool, len(sp.Slabs))
		for k := range active {
			active[k] = k <= interfaceAt
			if openAtTop {
				active[k] = k > interfaceAt
			}
		}
		return cutStackedSlabs(ctx, budget, sp, outer, tool, reexpress, active, sp.Interfaces, interfaceAt)
	}
	innerHeld, _ := inner.Float64()
	if innerHeld <= sp.Slabs[split].Z0 || innerHeld >= sp.Slabs[split].Z1 {
		return stackedPrismPayload{}, false, nil
	}
	innerDelta = proofbound.AbsSumUpper(innerDelta, proofarith.RationalFloatError(inner, innerHeld))
	lower, upper := sp.Slabs[split], sp.Slabs[split]
	lower.Z1, lower.Z1Delta = innerHeld, innerDelta
	upper.Z0, upper.Z0Delta = innerHeld, innerDelta
	result := sp
	result.Slabs = make([]stackedrecord.Slab, len(sp.Slabs)+1)
	copy(result.Slabs, sp.Slabs[:split])
	result.Slabs[split], result.Slabs[split+1] = lower, upper
	copy(result.Slabs[split+2:], sp.Slabs[split+1:])
	prior := make([]stackedrecord.Interface, len(result.Slabs)-1)
	copy(prior, sp.Interfaces[:split])
	copy(prior[split+1:], sp.Interfaces[split:])
	active := make([]bool, len(result.Slabs))
	for k := range active {
		active[k] = k <= split
		if openAtTop {
			active[k] = k > split
		}
	}
	return cutStackedSlabs(ctx, budget, result, outer, tool, reexpress, active, prior, -1)
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
	return cutStackedSlabs(ctx, budget, sp, outer, tool, reexpress, active, sp.Interfaces, -1)
}

// cutStackedSlabs proves each affected section using its own sketch scene.
// Inactive slabs retain their section, and prior names the exposed side of
// any existing nonmonotone interface after an optional slab split. A
// nonnegative monotoneAt admits new opposed holes only when their exact
// sections prove separate at that interface.
func cutStackedSlabs(ctx context.Context, budget *proofbound.WorkBudget, sp stackedPrismPayload,
	outer, tool prismPayload, reexpress *prismReexpression, active []bool,
	prior []stackedrecord.Interface, monotoneAt int) (stackedPrismPayload, bool, error) {
	result := sp
	result.Slabs = append([]stackedrecord.Slab(nil), sp.Slabs...)
	result.Interfaces = make([]stackedrecord.Interface, len(prior))
	for k, slab := range sp.Slabs {
		if !active[k] {
			continue
		}
		if err := budget.Err(); err != nil {
			return stackedPrismPayload{}, false, err
		}
		target := outer
		target.Profile = slab.Regions[0]
		trimmed, err := prismcells.ProfileHasTrimmedCircularSource(budget, target.Profile.Outer, target.Profile.Holes)
		if err != nil || trimmed {
			return stackedPrismPayload{}, false, err
		}
		segments, withinCap, err := prismcells.RegionsWithinWorkCap(budget, target.Profile, tool.Profile)
		if err != nil {
			return stackedPrismPayload{}, false, err
		}
		if !withinCap {
			return stackedPrismPayload{}, false, fmt.Errorf(
				`%w: slab %d's analytic cut scene charges %d arranger segments against the cap of %d`,
				ErrUnsupported, k, segments, prismcells.MaxArrangementSegments)
		}
		scene, profiles, tags, sceneDelta, err := prismcells.CutCells(ctx, budget, target.Profile, tool.Profile, reexpress)
		if err != nil {
			return stackedPrismPayload{}, false, err
		}
		match, resolved, err := prismcells.MatchCut(budget, tags, profiles, len(target.Profile.Holes))
		if err != nil {
			return stackedPrismPayload{}, false, err
		}
		if !resolved {
			noOp, err := prismcells.MatchCutNoOp(budget, tags, profiles, len(target.Profile.Holes))
			if err != nil || !noOp {
				return stackedPrismPayload{}, false, err
			}
			continue
		}
		_, err = prismcells.RecordProfileContext(ctx, scene, match)
		if err != nil {
			return stackedPrismPayload{}, false, err
		}
		profile, err := prismcells.RecordArrangedProfileContext(ctx, match)
		if err != nil {
			return stackedPrismPayload{}, false, err
		}
		profile, err = prismcells.CanonicalStackedCut(budget, target.Profile, match, tags, profile)
		if err != nil {
			return stackedPrismPayload{}, false, err
		}
		result.Slabs[k].Regions = []profileRecord{profile}
		result.SectionDelta = max(result.SectionDelta,
			proofbound.AbsSumUpper(target.SectionDelta, sceneDelta.A),
			proofbound.AbsSumUpper(tool.SectionDelta, sceneDelta.B, reexpress.Delta))
	}
	if monotoneAt >= 0 {
		lowerOnly, upperOnly := stackedrecord.ExclusiveHoles(
			result.Slabs[monotoneAt].Regions[0], result.Slabs[monotoneAt+1].Regions[0])
		if len(lowerOnly) != 0 && len(upperOnly) != 0 {
			if result.SectionDelta != 0 {
				return stackedPrismPayload{}, false, nil
			}
			_, separate, err := stackedrecord.SeparatedOpposed(ctx, budget, lowerOnly, upperOnly)
			if err != nil || !separate {
				return stackedPrismPayload{}, false, err
			}
		}
	}
	var err error
	result.Interfaces, err = stackedrecord.Derive(ctx, result.Slabs, prior)
	if err != nil {
		return stackedPrismPayload{}, false, err
	}
	if !result.OpposedSectionExact() {
		return stackedPrismPayload{}, false, nil
	}
	if err := stackedrecord.Falsify(ctx, stackedrecord.Record{Slabs: result.Slabs, Interfaces: result.Interfaces}); err != nil {
		return stackedPrismPayload{}, false, err
	}
	return result, true, nil
}
