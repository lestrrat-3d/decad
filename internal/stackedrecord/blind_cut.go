package stackedrecord

import (
	"context"
	"fmt"
	"math/big"

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
	arms, ok := blindArmsOf(target.PlacementOperand(), tool.PlacementOperand())
	if !ok {
		return Payload{}, false, nil
	}
	openAtTop, openAtBottom := arms.openAtTop, arms.openAtBottom
	inner, innerDenoted := toolInnerEnd(target.PlacementOperand(), tool, z0, z1, openAtBottom)
	segments, withinCap, err := prismcells.RegionsWithinWorkCap(budget, target.Profile, tool.Profile)
	if err != nil {
		return Payload{}, false, err
	}
	if !withinCap {
		return Payload{}, false, prismcells.WithWorkLimit(fmt.Errorf(
			`%w: the analytic cut scene charges %d arranger segments against the cap of %d`,
			decaderr.ErrUnsupported, segments, prismcells.MaxArrangementSegments), segments)
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
	innerDelta := tool.Z0Delta
	if openAtBottom {
		innerDelta = tool.Z1Delta
	}
	innerHeld, _ := inner.Float64()
	// The ranges prove the inner end strictly inside, but its held float may
	// round onto a cap, which leaves no slab between them.
	if innerHeld <= target.Z0 || innerHeld >= target.Z1 {
		return Payload{}, false, nil
	}
	shiftRound := proofarith.RationalFloatError(inner, innerHeld)
	if innerDelta == 0 {
		innerDelta = shiftRound
	} else {
		innerDelta = proofbound.AbsSumUpper(innerDelta, shiftRound)
	}
	level := prismplacement.DenoteLevel(innerDenoted, innerHeld, innerDelta)
	sp := Payload{Frame: target.Frame, Xform: target.Xform, Slabs: make([]Slab, 2),
		SectionDelta: max(proofbound.AbsSumUpper(target.SectionDelta, sceneDelta.A),
			proofbound.AbsSumUpper(tool.SectionDelta, sceneDelta.B, reexpress.Delta))}
	lower := Slab{Z0: target.Z0, Z1: innerHeld, Z0Delta: target.Z0Delta, Z1Delta: innerDelta,
		Z0Denoted: target.Z0Denoted, Z1Denoted: level}
	upper := Slab{Z0: innerHeld, Z1: target.Z1, Z0Delta: innerDelta, Z1Delta: target.Z1Delta,
		Z0Denoted: level, Z1Denoted: target.Z1Denoted}
	if openAtTop {
		lower.Regions, upper.Regions = []momentinput.Profile{target.Profile}, []momentinput.Profile{cutRegion}
	} else {
		lower.Regions, upper.Regions = []momentinput.Profile{cutRegion}, []momentinput.Profile{target.Profile}
	}
	sp.Slabs[0], sp.Slabs[1] = lower, upper
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

// toolInnerEnd is a blind tool's inner end on the target's axis: the exact
// lift of its held level, and the level it denotes, or nil when no record
// proves that level.
func toolInnerEnd(target prismplacement.Operand, tool prismrecord.Payload, z0, z1 *big.Rat,
	openAtBottom bool) (*big.Rat, *big.Rat) {
	if openAtBottom {
		return z1, prismplacement.DenotedShiftedLevel(target, tool.PlacementOperand(), tool.Z1, tool.Z1Delta, tool.Z1Denoted)
	}
	return z0, prismplacement.DenotedShiftedLevel(target, tool.PlacementOperand(), tool.Z0, tool.Z0Delta, tool.Z0Denoted)
}

// denotedInterface is the interface whose denoted level equals level, or -1.
// Both slabs meeting there must record that level. Equal held floats never
// stand in for it, since each may round a different denoted level.
func denotedInterface(slabs []Slab, level *big.Rat) int {
	if level == nil {
		return -1
	}
	for k := 0; k+1 < len(slabs); k++ {
		below, above := slabs[k].DenotedZ1(), slabs[k+1].DenotedZ0()
		if below != nil && above != nil && below.Cmp(above) == 0 && below.Cmp(level) == 0 {
			return k
		}
	}
	return -1
}

// blindArms is which cap a blind tool opens at, read on the exact level
// ranges (prismplacement.LevelRange): every level the held floats,
// displacements and denotations allow must agree. openAtTop needs the tool's
// lower end strictly inside the target and its upper end at or past the
// target's top; openAtBottom mirrors it. inner is the range of the tool end
// that lies inside.
type blindArms struct {
	openAtTop, openAtBottom bool
	inner                   prismplacement.LevelRange
}

// blindArmsOf reads target and tool's level ranges on the target's axis.
// ok is false when neither arm is proven, so a tool end within its
// displacement of a target cap, or an inner end within one of either cap,
// declines.
func blindArmsOf(target, tool prismplacement.Operand) (blindArms, bool) {
	a0, a1, z0, z1, ok := prismplacement.IntervalRanges(target, tool)
	if !ok {
		return blindArms{}, false
	}
	switch {
	case a0.Below(z0) && z0.Below(a1) && a1.AtOrBelow(z1):
		return blindArms{openAtTop: true, inner: z0}, true
	case z0.AtOrBelow(a0) && a0.Below(z1) && z1.Below(a1):
		return blindArms{openAtBottom: true, inner: z1}, true
	}
	return blindArms{}, false
}

// rangeSplit is the slab whose level ranges strictly contain inner's, or -1.
// An inner end that does not lie certainly above a slab's lower level, before
// it lies certainly below that slab's upper one, is -1: denotedInterface alone
// decides an inner end at an interface.
func rangeSplit(slabs []Slab, inner prismplacement.LevelRange) int {
	for k, slab := range slabs {
		lower, okLo := prismplacement.LevelRangeOf(slab.Z0, slab.Z0Delta, slab.Z0Denoted, new(big.Rat))
		upper, okHi := prismplacement.LevelRangeOf(slab.Z1, slab.Z1Delta, slab.Z1Denoted, new(big.Rat))
		if !okLo || !okHi || !lower.Below(inner) {
			return -1
		}
		if inner.Below(upper) {
			return k
		}
	}
	return -1
}

// ApplyBlindCut splits the slab containing a blind tool's inner end, or
// reuses an existing interface whose denoted level equals the tool's. Every
// reached slab needs its own whole-loop cut proof, and the resulting
// interfaces must pass the audit.
func ApplyBlindCut(ctx context.Context, budget *proofbound.WorkBudget, sp Payload,
	outer, tool prismrecord.Payload) (Payload, bool, error) {
	reach, ok, err := reachBlindCut(budget, sp, outer, tool)
	if err != nil || !ok {
		return Payload{}, false, err
	}
	reexpress, err := prismcells.NewReexpression(outer.PlacementOperand(), tool.PlacementOperand())
	if err != nil {
		return Payload{}, false, err
	}
	if reach.interfaceAt >= 0 {
		return CutSlabs(ctx, budget, sp, outer, tool, reexpress, reach.interfaceActive(len(sp.Slabs)),
			sp.Interfaces, reach.interfaceAt)
	}
	split := reach.split
	innerHeld, _ := reach.inner.Float64()
	if innerHeld <= sp.Slabs[split].Z0 || innerHeld >= sp.Slabs[split].Z1 {
		return Payload{}, false, nil
	}
	innerDelta := proofbound.AbsSumUpper(reach.innerDelta, proofarith.RationalFloatError(reach.inner, innerHeld))
	level := prismplacement.DenoteLevel(reach.innerDenoted, innerHeld, innerDelta)
	lower, upper := sp.Slabs[split], sp.Slabs[split]
	lower.Z1, lower.Z1Delta, lower.Z1Denoted = innerHeld, innerDelta, level
	upper.Z0, upper.Z0Delta, upper.Z0Denoted = innerHeld, innerDelta, level
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
		if reach.openAtTop {
			active[k] = k > split
		}
	}
	return CutSlabs(ctx, budget, result, outer, tool, reexpress, active, prior, -1)
}

// blindCutReach is where a blind tool's inner end lies in a stack: strictly
// inside slab split, or at the interface after slab interfaceAt whose denoted
// level equals the tool's. The other index is -1.
type blindCutReach struct {
	openAtTop           bool
	inner, innerDenoted *big.Rat
	innerDelta          float64
	split, interfaceAt  int
}

// reachBlindCut reads a hole-free, untrimmed blind tool's interval against
// the stack's levels exactly. ok=false is a silent miss.
func reachBlindCut(budget *proofbound.WorkBudget, sp Payload,
	outer, tool prismrecord.Payload) (blindCutReach, bool, error) {
	if len(tool.Profile.Holes) != 0 {
		return blindCutReach{}, false, nil
	}
	trimmed, err := prismcells.ProfileHasTrimmedCircularSource(budget, tool.Profile.Outer, tool.Profile.Holes)
	if err != nil || trimmed {
		return blindCutReach{}, false, err
	}
	z0, z1, ok := prismplacement.ShiftedInterval(outer.PlacementOperand(), tool.PlacementOperand())
	if !ok {
		return blindCutReach{}, false, nil
	}
	// outer is the stack's outer prism, whose ends are the first and last
	// slabs' own levels with their denotations.
	arms, ok := blindArmsOf(outer.PlacementOperand(), tool.PlacementOperand())
	if !ok {
		return blindCutReach{}, false, nil
	}
	openAtTop, openAtBottom := arms.openAtTop, arms.openAtBottom
	reach := blindCutReach{openAtTop: openAtTop, innerDelta: tool.Z0Delta, split: -1}
	reach.inner, reach.innerDenoted = toolInnerEnd(outer.PlacementOperand(), tool, z0, z1, openAtBottom)
	if openAtBottom {
		reach.innerDelta = tool.Z1Delta
	}
	reach.interfaceAt = denotedInterface(sp.Slabs, reach.innerDenoted)
	if reach.interfaceAt < 0 {
		reach.split = rangeSplit(sp.Slabs, arms.inner)
	}
	if reach.split < 0 && reach.interfaceAt < 0 {
		return blindCutReach{}, false, nil
	}
	return reach, true, nil
}

// interfaceActive names the slabs a tool ending at interface interfaceAt
// reaches: those below it for a pocket from the bottom, above it for one
// from the top.
func (r blindCutReach) interfaceActive(n int) []bool {
	active := make([]bool, n)
	for k := range active {
		active[k] = k <= r.interfaceAt
		if r.openAtTop {
			active[k] = k > r.interfaceAt
		}
	}
	return active
}
