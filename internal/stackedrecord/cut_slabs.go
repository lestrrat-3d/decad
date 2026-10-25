package stackedrecord

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// CutSlabs proves each affected section using its own sketch scene.
// Inactive slabs retain their section, and prior names the exposed side of
// any existing nonmonotone interface after an optional slab split. A
// nonnegative monotoneAt admits new opposed holes only when their exact
// sections prove separate at that interface.
func CutSlabs(ctx context.Context, budget *proofbound.WorkBudget, sp Payload,
	outer, tool prismrecord.Payload, reexpress *prismcells.Reexpression, active []bool,
	prior []Interface, monotoneAt int) (Payload, bool, error) {
	result := sp
	result.Slabs = append([]Slab(nil), sp.Slabs...)
	result.Interfaces = make([]Interface, len(prior))
	for k, slab := range sp.Slabs {
		if !active[k] {
			continue
		}
		if err := budget.Err(); err != nil {
			return Payload{}, false, err
		}
		target := outer
		target.Profile = slab.Regions[0]
		trimmed, err := prismcells.ProfileHasTrimmedCircularSource(budget, target.Profile.Outer, target.Profile.Holes)
		if err != nil || trimmed {
			return Payload{}, false, err
		}
		segments, withinCap, err := prismcells.RegionsWithinWorkCap(budget, target.Profile, tool.Profile)
		if err != nil {
			return Payload{}, false, err
		}
		if !withinCap {
			return Payload{}, false, prismcells.WithWorkLimit(fmt.Errorf(
				`%w: slab %d's analytic cut scene charges %d arranger segments against the cap of %d`,
				decaderr.ErrUnsupported, k, segments, prismcells.MaxArrangementSegments), segments)
		}
		scene, profiles, tags, sceneDelta, err := prismcells.CutCells(ctx, budget, target.Profile, tool.Profile, reexpress)
		if err != nil {
			return Payload{}, false, err
		}
		match, resolved, err := prismcells.MatchCut(budget, tags, profiles, len(target.Profile.Holes))
		if err != nil {
			return Payload{}, false, err
		}
		if !resolved {
			noOp, err := prismcells.MatchCutNoOp(budget, tags, profiles, len(target.Profile.Holes))
			if err != nil || !noOp {
				return Payload{}, false, err
			}
			continue
		}
		_, err = prismcells.RecordProfileContext(ctx, scene, match)
		if err != nil {
			return Payload{}, false, err
		}
		profile, err := prismcells.RecordArrangedProfileContext(ctx, match)
		if err != nil {
			return Payload{}, false, err
		}
		profile, err = prismcells.CanonicalStackedCut(budget, target.Profile, match, tags, profile)
		if err != nil {
			return Payload{}, false, err
		}
		result.Slabs[k].Regions = []momentinput.Profile{profile}
		result.SectionDelta = max(result.SectionDelta,
			proofbound.AbsSumUpper(target.SectionDelta, sceneDelta.A),
			proofbound.AbsSumUpper(tool.SectionDelta, sceneDelta.B, reexpress.Delta))
	}
	if monotoneAt >= 0 {
		lowerOnly, upperOnly := ExclusiveHoles(
			result.Slabs[monotoneAt].Regions[0], result.Slabs[monotoneAt+1].Regions[0])
		if len(lowerOnly) != 0 && len(upperOnly) != 0 {
			if result.SectionDelta != 0 {
				return Payload{}, false, nil
			}
			_, separate, err := SeparatedOpposed(ctx, budget, lowerOnly, upperOnly)
			if err != nil || !separate {
				return Payload{}, false, err
			}
		}
	}
	var err error
	result.Interfaces, err = Derive(ctx, result.Slabs, prior)
	if err != nil {
		return Payload{}, false, err
	}
	if !result.OpposedSectionExact() {
		return Payload{}, false, nil
	}
	if err := Falsify(ctx, Record{Slabs: result.Slabs, Interfaces: result.Interfaces}); err != nil {
		return Payload{}, false, err
	}
	return result, true, nil
}
