package filletband

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/capband"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
)

// LoopContourDelta bounds the contour that remains after a sphere pole
// consumes an inward arc. The surviving offset carriers meet at the recorded
// centre, so the ordinary contour proof reads that centre as their miter. The
// contour holds an LF5′ corner at its G1 foot, and the true contour's corner
// points lie within NearTangentReach of both feet, so the bound adds that
// reach (agent-docs/loop-fillet-design.md §5.5).
func LoopContourDelta(ctx context.Context, loop sectionrecord.LoopRecord, r, rDelta, tol float64) (float64, error) {
	budget := proofbound.NewWorkBudget(ctx)
	walks, err := capband.LoopWalks(budget, loop, freeform.NewFreeformWork())
	if err != nil {
		return 0, err
	}
	if len(walks) == 1 && walks[0].Closed {
		return capband.LoopContourDelta(ctx, loop, r, rDelta, tol)
	}
	delta, err := ContourDelta(budget, walks, r, rDelta, tol)
	if err != nil {
		return 0, err
	}
	rho, err := LoopNearTangentRho(budget, loop, walks, r, rDelta, tol)
	if err != nil {
		return 0, err
	}
	if rho == nil {
		return delta, nil
	}
	return proofbound.AbsSumUpper(delta, NearTangentReach(rho)), nil
}
