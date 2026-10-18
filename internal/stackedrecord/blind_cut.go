package stackedrecord

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/prismplacement"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
)

// BuildBlindCut records the first two-slab pocket from admitted prism records.
// The caller owns live-body admission and passes its shared work budget.
func BuildBlindCut(ctx context.Context, budget *proofbound.WorkBudget,
	target, tool prismrecord.Payload) (Payload, bool, error) {
	if len(tool.Profile.Holes) != 0 {
		return Payload{}, false, nil
	}
	for _, profile := range []momentinput.Profile{target.Profile, tool.Profile} {
		trimmed, err := prismcells.ProfileHasTrimmedCircularSource(budget, profile.Outer, profile.Holes)
		if err != nil {
			return Payload{}, false, err
		}
		if trimmed {
			return Payload{}, false, nil
		}
	}
	z0, z1, ok := prismplacement.ShiftedInterval(target.PlacementOperand(), tool.PlacementOperand())
	if !ok {
		return Payload{}, false, nil
	}
	a0, a1 := proofarith.FloatRat(target.Z0), proofarith.FloatRat(target.Z1)
	if a0 == nil || a1 == nil {
		return Payload{}, false, nil
	}
	openAtTop := z0.Cmp(a0) > 0 && z0.Cmp(a1) < 0 && z1.Cmp(a1) >= 0
	openAtBottom := z0.Cmp(a0) <= 0 && z1.Cmp(a0) > 0 && z1.Cmp(a1) < 0
	if !openAtTop && !openAtBottom {
		return Payload{}, false, nil
	}
	segments, withinCap, err := prismcells.RegionsWithinWorkCap(budget, target.Profile, tool.Profile)
	if err != nil {
		return Payload{}, false, err
	}
	if !withinCap {
		return Payload{}, false, fmt.Errorf(
			`%w: the analytic cut scene charges %d arranger segments against the cap of %d`,
			decaderr.ErrUnsupported, segments, prismcells.MaxArrangementSegments)
	}
	reexpress, err := prismcells.NewReexpression(target.PlacementOperand(), tool.PlacementOperand())
	if err != nil {
		return Payload{}, false, err
	}
	scene, profiles, tags, sceneDelta, err := prismcells.CutCells(ctx, budget, target.Profile, tool.Profile, reexpress)
	if err != nil {
		return Payload{}, false, err
	}
	match, resolved, err := prismcells.MatchCut(budget, tags, profiles, len(target.Profile.Holes))
	if err != nil {
		return Payload{}, false, err
	}
	var enclosing prismcells.EnclosingCutMatch
	if !resolved {
		enclosing, resolved, err = prismcells.MatchEnclosingCut(budget, tags, profiles, len(target.Profile.Holes))
		if err != nil || !resolved {
			return Payload{}, false, err
		}
		match = enclosing.Outside
		if _, err := prismcells.RecordProfileContext(ctx, scene, enclosing.Inside); err != nil {
			return Payload{}, false, err
		}
	}
	_, err = prismcells.RecordProfileContext(ctx, scene, match)
	if err != nil {
		return Payload{}, false, err
	}
	cutRegion, err := prismcells.RecordArrangedProfileContext(ctx, match)
	if err != nil {
		return Payload{}, false, err
	}
	kept := target.Profile
	if len(enclosing.EnclosedHoles) != 0 {
		kept.Holes = make([]sectionrecord.LoopRecord, 0, len(enclosing.OutsideHoles))
		for _, i := range enclosing.OutsideHoles {
			kept.Holes = append(kept.Holes, target.Profile.Holes[i])
		}
	}
	cutRegion, err = prismcells.CanonicalStackedCut(budget, kept, match, tags, cutRegion)
	if err != nil {
		return Payload{}, false, err
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
	sp := Payload{Frame: target.Frame, Xform: target.Xform, Slabs: make([]Slab, 2),
		SectionDelta: max(proofbound.AbsSumUpper(target.SectionDelta, sceneDelta.A),
			proofbound.AbsSumUpper(tool.SectionDelta, sceneDelta.B, reexpress.Delta))}
	if openAtTop {
		sp.Slabs[0] = Slab{Regions: []momentinput.Profile{target.Profile},
			Z0: target.Z0, Z1: innerHeld, Z0Delta: target.Z0Delta, Z1Delta: innerDelta}
		sp.Slabs[1] = Slab{Regions: []momentinput.Profile{cutRegion},
			Z0: innerHeld, Z1: target.Z1, Z0Delta: innerDelta, Z1Delta: target.Z1Delta}
	} else {
		sp.Slabs[0] = Slab{Regions: []momentinput.Profile{cutRegion},
			Z0: target.Z0, Z1: innerHeld, Z0Delta: target.Z0Delta, Z1Delta: innerDelta}
		sp.Slabs[1] = Slab{Regions: []momentinput.Profile{target.Profile},
			Z0: innerHeld, Z1: target.Z1, Z0Delta: innerDelta, Z1Delta: target.Z1Delta}
	}
	if len(enclosing.EnclosedHoles) != 0 {
		innerHoles := make([]sectionrecord.LoopRecord, 0, len(enclosing.EnclosedHoles))
		for _, i := range enclosing.EnclosedHoles {
			innerHoles = append(innerHoles, target.Profile.Holes[i])
		}
		toolHole := cutRegion.Holes[len(cutRegion.Holes)-1]
		exposed, err := EnclosingExposed(ctx, toolHole, innerHoles)
		if err != nil {
			return Payload{}, false, err
		}
		if openAtTop {
			sp.Interfaces = []Interface{{LowerExposed: exposed}}
		} else {
			sp.Interfaces = []Interface{{UpperExposed: exposed}}
		}
	} else {
		lowerOnly, upperOnly := ExclusiveHoles(sp.Slabs[0].Regions[0], sp.Slabs[1].Regions[0])
		lowerExposed, err := Exposed(ctx, upperOnly)
		if err != nil {
			return Payload{}, false, err
		}
		upperExposed, err := Exposed(ctx, lowerOnly)
		if err != nil {
			return Payload{}, false, err
		}
		sp.Interfaces = []Interface{{LowerExposed: lowerExposed, UpperExposed: upperExposed}}
	}
	return sp, true, nil
}

// ApplyBlindCut splits the slab containing a blind tool's inner end, or
// reuses an exact existing interface. Every reached slab needs its own
// whole-loop cut proof, and the resulting interfaces must pass the audit.
func ApplyBlindCut(ctx context.Context, budget *proofbound.WorkBudget, sp Payload,
	outer, tool prismrecord.Payload) (Payload, bool, error) {
	if len(tool.Profile.Holes) != 0 {
		return Payload{}, false, nil
	}
	trimmed, err := prismcells.ProfileHasTrimmedCircularSource(budget, tool.Profile.Outer, tool.Profile.Holes)
	if err != nil || trimmed {
		return Payload{}, false, err
	}
	z0, z1, ok := prismplacement.ShiftedInterval(outer.PlacementOperand(), tool.PlacementOperand())
	if !ok {
		return Payload{}, false, nil
	}
	first := proofarith.FloatRat(sp.Slabs[0].Z0)
	last := proofarith.FloatRat(sp.Slabs[len(sp.Slabs)-1].Z1)
	if first == nil || last == nil {
		return Payload{}, false, nil
	}
	openAtTop := z0.Cmp(first) > 0 && z0.Cmp(last) < 0 && z1.Cmp(last) >= 0
	openAtBottom := z0.Cmp(first) <= 0 && z1.Cmp(first) > 0 && z1.Cmp(last) < 0
	if !openAtTop && !openAtBottom {
		return Payload{}, false, nil
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
		return Payload{}, false, nil
	}
	reexpress, err := prismcells.NewReexpression(outer.PlacementOperand(), tool.PlacementOperand())
	if err != nil {
		return Payload{}, false, err
	}
	if interfaceAt >= 0 {
		// With a displaced level, the nominal equality does not prove which
		// slabs the true tool reaches. Keep this arm to exactly shared levels.
		if innerDelta != 0 || sp.Slabs[interfaceAt].Z1Delta != 0 ||
			sp.Slabs[interfaceAt+1].Z0Delta != 0 {
			return Payload{}, false, nil
		}
		active := make([]bool, len(sp.Slabs))
		for k := range active {
			active[k] = k <= interfaceAt
			if openAtTop {
				active[k] = k > interfaceAt
			}
		}
		return CutSlabs(ctx, budget, sp, outer, tool, reexpress, active, sp.Interfaces, interfaceAt)
	}
	innerHeld, _ := inner.Float64()
	if innerHeld <= sp.Slabs[split].Z0 || innerHeld >= sp.Slabs[split].Z1 {
		return Payload{}, false, nil
	}
	innerDelta = proofbound.AbsSumUpper(innerDelta, proofarith.RationalFloatError(inner, innerHeld))
	lower, upper := sp.Slabs[split], sp.Slabs[split]
	lower.Z1, lower.Z1Delta = innerHeld, innerDelta
	upper.Z0, upper.Z0Delta = innerHeld, innerDelta
	result := sp
	result.Slabs = make([]Slab, len(sp.Slabs)+1)
	copy(result.Slabs, sp.Slabs[:split])
	result.Slabs[split], result.Slabs[split+1] = lower, upper
	copy(result.Slabs[split+2:], sp.Slabs[split+1:])
	prior := make([]Interface, len(result.Slabs)-1)
	copy(prior, sp.Interfaces[:split])
	copy(prior[split+1:], sp.Interfaces[split:])
	active := make([]bool, len(result.Slabs))
	for k := range active {
		active[k] = k <= split
		if openAtTop {
			active[k] = k > split
		}
	}
	return CutSlabs(ctx, budget, result, outer, tool, reexpress, active, prior, -1)
}
