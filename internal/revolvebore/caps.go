package revolvebore

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/units"
)

// CapProfiles asks Sketch to certify each circular cap with the tool hole.
// The tool must already be mapped into the Revolve's axial frame.
func CapProfiles(ctx context.Context, mer Meridian, tool prismrecord.Payload) ([2]momentinput.Profile, bool, error) {
	var result [2]momentinput.Profile
	budget := proofbound.NewWorkBudget(ctx)
	for i, radius := range [2]float64{mer.LowRadius, mer.HighRadius} {
		circle := sectionrecord.CircleSeg{Radius: units.Millimeters(radius), CCW: true, TEnd: 1}
		outer := momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{circle}}}
		_, within, err := prismcells.RegionsWithinWorkCap(budget, outer, tool.Profile)
		if err != nil || !within {
			return result, false, err
		}
		reexpress, err := prismcells.NewReexpression(tool.PlacementOperand(), tool.PlacementOperand())
		if err != nil || !reexpress.Identity {
			return result, false, err
		}
		scene, match, tags, delta, resolved, err := prismcells.ResolveCutWithTags(ctx, budget, outer, tool.Profile, reexpress)
		if err != nil || !resolved || delta.A != 0 || delta.B != 0 {
			return result, false, err
		}
		if _, err := prismcells.RecordProfileContext(ctx, scene, match); err != nil {
			return result, false, err
		}
		profile, err := prismcells.RecordArrangedProfileContext(ctx, match)
		if err != nil {
			return result, false, err
		}
		profile, err = prismcells.CanonicalStackedCut(budget, outer, match, tags, profile)
		if err != nil {
			return result, false, err
		}
		if len(profile.Holes) != 1 {
			return result, false, nil
		}
		result[i] = profile
	}
	return result, true, budget.Err()
}
