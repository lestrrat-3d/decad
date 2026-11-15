package filletband

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/decaderr"

	"github.com/lestrrat-3d/decad/internal/capband"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
)

// LoopContourDelta bounds the contour that remains after a sphere pole
// consumes an inward arc. The surviving offset carriers meet at the recorded
// centre, so the ordinary contour proof reads that centre as their miter. The
// contour holds an LF5′ corner, or a corner beside an off-radius arc, at its
// G1 foot or its recorded join, and the true contour lies within that
// corner's local reach of it (Loop.LocalReach), so the bound adds the loop's
// largest reach (agent-docs/loop-fillet-design.md §5.7). A loop whose corners
// Table LF refuses is ErrUnsupported: no band holds it.
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
	read, _, _, bad, why, err := ReadLoop(budget, loop, walks, r, rDelta, tol)
	if err != nil {
		return 0, err
	}
	if bad >= 0 {
		return 0, fmt.Errorf(`%w: a fillet contour's corner %d has no supported join: %s (loop-fillet SF1)`, decaderr.ErrUnsupported, bad, why)
	}
	rIv, err := RadiusInterval(r, rDelta)
	if err != nil {
		return 0, err
	}
	reach := 0.0
	for _, d := range read.LocalReach(rIv) {
		reach = math.Max(reach, d)
	}
	if reach == 0 {
		return delta, nil
	}
	return proofbound.AbsSumUpper(delta, reach), nil
}
