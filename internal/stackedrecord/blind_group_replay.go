package stackedrecord

import (
	"context"
	"math"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/prismplacement"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
)

// ReplayBlindGroupPayload constructs the two-slab record for one matched
// whole-circle group. The caller must separately authenticate bodies, cap
// planes, provenance and the final built output before publication.
func ReplayBlindGroupPayload(ctx context.Context, target prismrecord.Payload, tool Payload,
	cert *prismcells.ReplayBlindScene, budget *proofbound.ReplayBudget) (Payload, []int, bool, error) {
	var out Payload
	if budget == nil {
		return out, nil, false, nil
	}
	if err := budget.SpendWork(ctx, 1); err != nil {
		return out, nil, false, err
	}
	if !tool.IsGroup() || len(tool.Interfaces) != 0 || target.Z0Delta != 0 || target.Z1Delta != 0 {
		return out, nil, false, nil
	}
	slab := tool.Slabs[0]
	if slab.Z0Delta != 0 || slab.Z1Delta != 0 {
		return out, nil, false, nil
	}
	groupPlacement := prismplacement.Operand{Frame: tool.Frame, Xform: tool.Xform, Z0: slab.Z0, Z1: slab.Z1, Z0Delta: slab.Z0Delta, Z1Delta: slab.Z1Delta, Z0Denoted: slab.Z0Denoted, Z1Denoted: slab.Z1Denoted}
	targetPlacement := target.PlacementOperand()
	bound, err := prismcells.ReplayBlindSceneMatches(ctx, cert, target.Profile, slab.Regions, targetPlacement, groupPlacement, target.SectionDelta, tool.SectionDelta, budget)
	if err != nil || !bound {
		return out, nil, false, err
	}
	mapping, ok, err := prismplacement.ReplayParallelSignedMap(ctx, targetPlacement, groupPlacement, budget)
	if err != nil || !ok {
		return out, nil, false, err
	}
	top, ok, err := prismplacement.ReplayBlindLevels(ctx, targetPlacement, groupPlacement, mapping.Shift, budget)
	if err != nil || !ok {
		return out, nil, false, err
	}
	inner := slab.Z0
	if !top {
		inner = slab.Z1
	}
	innerRat, err := proof.ReplayFloatRat(ctx, inner, budget)
	if err != nil {
		return out, nil, false, err
	}
	innerRat, err = proof.ReplayRatArithmetic(ctx, innerRat, mapping.Shift, '+', budget)
	if err != nil {
		return out, nil, false, err
	}
	held, err := proof.ReplayRatFloat64(ctx, innerRat, budget)
	if err != nil {
		return out, nil, false, err
	}
	if math.IsNaN(held) || math.IsInf(held, 0) {
		return out, nil, false, nil
	}
	heldRat, err := proof.ReplayFloatRat(ctx, held, budget)
	if err != nil {
		return out, nil, false, err
	}
	cmp, err := proof.ReplayRatCompare(ctx, heldRat, innerRat, budget)
	if err != nil || cmp != 0 {
		return out, nil, false, err
	}
	section, ok, err := prismcells.ReplayBlindGroupSection(ctx, cert, budget)
	if err != nil || !ok {
		return out, nil, false, err
	}
	if len(section.HoleRegions) != len(slab.Regions) || len(section.Profile.Holes) != len(target.Profile.Holes)+len(slab.Regions) {
		return out, nil, false, nil
	}
	uncut, ok, err := replayCopyBlindProfile(ctx, target.Profile, budget)
	if err != nil || !ok {
		return out, nil, false, err
	}
	// Rebind the copied values to the same certified scene before retaining
	// them in either slab.
	bound, err = prismcells.ReplayBlindSceneMatches(ctx, cert, uncut, slab.Regions, targetPlacement, groupPlacement, target.SectionDelta, tool.SectionDelta, budget)
	if err != nil || !bound {
		return out, nil, false, err
	}
	if err := reserveBlindPayloadArrays(ctx, budget, len(slab.Regions)); err != nil {
		return out, nil, false, err
	}
	exposed := make([]momentinput.Profile, 0, len(slab.Regions))
	for i := len(target.Profile.Holes); i < len(section.Profile.Holes); i++ {
		if err := budget.SpendWork(ctx, 1); err != nil {
			return Payload{}, nil, false, err
		}
		hole := section.Profile.Holes[i]
		if len(hole.Segments) != 1 {
			return Payload{}, nil, false, nil
		}
		circle, yes := hole.Segments[0].(sectionrecord.CircleSeg)
		if !yes {
			return Payload{}, nil, false, nil
		}
		circle.CCW = !circle.CCW
		circle.TStart, circle.TEnd = circle.TEnd, circle.TStart
		if err := budget.ReserveRecords(ctx, 1); err != nil {
			return Payload{}, nil, false, err
		}
		if err := budget.ReserveArray(ctx, 1, 16); err != nil {
			return Payload{}, nil, false, err
		}
		if err := budget.ReserveBytes(ctx, 256); err != nil {
			return Payload{}, nil, false, err
		}
		exposed = append(exposed, momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{circle}}})
	}
	lower := Slab{Z0: target.Z0, Z1: held, Z0Delta: target.Z0Delta, Z0Denoted: target.Z0Denoted}
	upper := Slab{Z0: held, Z1: target.Z1, Z1Delta: target.Z1Delta, Z1Denoted: target.Z1Denoted}
	boundary := Interface{}
	if top {
		lower.Regions = []momentinput.Profile{uncut}
		upper.Regions = []momentinput.Profile{section.Profile}
		boundary.LowerExposed = exposed
	} else {
		lower.Regions = []momentinput.Profile{section.Profile}
		upper.Regions = []momentinput.Profile{uncut}
		boundary.UpperExposed = exposed
	}
	out = Payload{Frame: target.Frame, Xform: target.Xform, SectionDelta: section.SectionDelta, Slabs: []Slab{lower, upper}, Interfaces: []Interface{boundary}}
	valid, err := replayValidateBlindGroup(ctx, out, target, section, held, top, budget)
	if err != nil || !valid {
		return Payload{}, nil, false, err
	}
	return out, section.HoleRegions, true, nil
}

// reserveBlindPayloadArrays charges the two slabs, interface, two region
// profiles and every exposed floor profile before allocating the result.
func reserveBlindPayloadArrays(ctx context.Context, budget *proofbound.ReplayBudget, count int) error {
	if count < 0 || uint64(count) > ^uint64(0)-5 {
		return decaderr.ErrUnsupported
	}
	if err := budget.ReserveRecords(ctx, uint64(count)+5); err != nil {
		return err
	}
	if err := budget.ReserveArray(ctx, 2, uint64(unsafe.Sizeof(Slab{}))); err != nil {
		return err
	}
	if err := budget.ReserveArray(ctx, 1, uint64(unsafe.Sizeof(Interface{}))); err != nil {
		return err
	}
	if err := budget.ReserveArray(ctx, 2, uint64(unsafe.Sizeof(momentinput.Profile{}))); err != nil {
		return err
	}
	return budget.ReserveArray(ctx, uint64(count), 64)
}
