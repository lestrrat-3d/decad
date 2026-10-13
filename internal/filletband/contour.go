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
// centre, so the ordinary contour proof reads that centre as their miter.
func LoopContourDelta(ctx context.Context, loop sectionrecord.LoopRecord, r, rDelta, tol float64) (float64, error) {
	budget := proofbound.NewWorkBudget(ctx)
	walks, err := capband.LoopWalks(budget, loop, freeform.NewFreeformWork())
	if err != nil {
		return 0, err
	}
	if len(walks) == 1 && walks[0].Closed {
		return capband.LoopContourDelta(ctx, loop, r, rDelta, tol)
	}
	return ContourDelta(budget, walks, r, rDelta, tol)
}
