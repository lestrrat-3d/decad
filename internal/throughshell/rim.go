package throughshell

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/brepgeom"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionaudit"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
)

// RimRegions classifies the loops left when a cavity trace cancels part of a
// removed face's outer loop. A nonempty reason identifies an SG7 refusal;
// other errors are returned unchanged for the shell caller to map.
func RimRegions(ctx context.Context, budget *proofbound.WorkBudget, outer sectionrecord.LoopRecord,
	cavity []sectionrecord.LoopRecord) ([]momentinput.Profile, string, error) {
	trace, err := brepgeom.TraceRim(ctx, outer, cavity)
	if err != nil {
		if reason, ok := err.(brepgeom.RimTraceError); ok {
			return nil, reason.Error(), nil
		}
		return nil, "", err
	}
	if !trace.Cancelled {
		return []momentinput.Profile{{Outer: trace.Loops[0], Holes: trace.Loops[1:]}}, "", nil
	}
	var outs, inner []sectionrecord.LoopRecord
	for _, rec := range trace.Loops {
		area, err := sectionaudit.LoopSignedArea(budget, rec)
		if err != nil {
			return nil, "", err
		}
		switch {
		case area > 0:
			outs = append(outs, rec)
		case area < 0:
			inner = append(inner, rec)
		default:
			return nil, "a loop left by the cavity's trace encloses no area", nil
		}
	}
	switch {
	case len(inner) == 0:
		out := make([]momentinput.Profile, len(outs))
		for i, o := range outs {
			out[i] = momentinput.Profile{Outer: o}
		}
		return out, "", nil
	case len(outs) == 1:
		return []momentinput.Profile{{Outer: outs[0], Holes: inner}}, "", nil
	}
	return nil, "the loops left by the cavity's trace are not one outer loop with holes", nil
}
