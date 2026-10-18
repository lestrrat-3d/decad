package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/prismplacement"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// tryPrismHoledIntersect admits one cleanly nested, one-hole prism under a
// hole-free prism. Intersect is symmetric, so put the hole-free operand first.
// The shared Intersect admission used by the overlap-area reading stays
// hole-free; this body-only arm uses the same gates and work cap explicitly.
func tryPrismHoledIntersect(ctx context.Context, a, b *Body) (prismPayload, bool, error) {
	ah, aok := a.n.Payload.(prismPayload)
	bh, bok := b.n.Payload.(prismPayload)
	if !aok || !bok {
		return prismPayload{}, false, nil
	}
	if len(ah.Profile.Holes) == 1 && len(bh.Profile.Holes) == 0 {
		a, b = b, a
	} else if len(ah.Profile.Holes) != 0 || len(bh.Profile.Holes) != 1 {
		return prismPayload{}, false, nil
	}

	budget := proofbound.NewWorkBudget(ctx)
	if err := budget.Err(); err != nil {
		return prismPayload{}, false, err
	}
	pa, pb, ok, err := admitPrismPairBudget(budget, meshbool.OpIntersect, a, b)
	if err != nil || !ok {
		return prismPayload{}, false, err
	}
	for _, profile := range []profileRecord{pa.Profile, pb.Profile} {
		trimmed, err := prismcells.ProfileHasTrimmedCircularSource(budget, profile.Outer, profile.Holes)
		if err != nil {
			return prismPayload{}, false, err
		}
		if trimmed {
			return prismPayload{}, false, nil
		}
	}
	if !prismplacement.IntersectZIntervalOverlaps(pa.PlacementOperand(), pb.PlacementOperand()) {
		return prismPayload{}, false, nil
	}
	segments, withinCap, err := prismcells.RegionsWithinWorkCap(budget, pa.Profile, pb.Profile)
	if err != nil {
		return prismPayload{}, false, err
	}
	if !withinCap {
		return prismPayload{}, false, fmt.Errorf(
			`%w: the analytic %s scene charges at least %d arranger segments against this evaluator's cap of %d`,
			ErrUnsupported, meshbool.OpIntersect, segments, prismcells.MaxArrangementSegments)
	}
	reexpress, err := prismcells.NewReexpression(pa.PlacementOperand(), pb.PlacementOperand())
	if err != nil {
		return prismPayload{}, false, err
	}
	return prismcells.ResolveAndBuildIntersect(ctx, budget, pa, pb, reexpress)
}
