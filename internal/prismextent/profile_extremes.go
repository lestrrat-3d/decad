package prismextent

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// ProfileBoundaryExtremes reads the boundary extremes of a recorded profile's
// outer loop and holes along (gu, gv), resolving each segment walk or reading
// it from walks.
func ProfileBoundaryExtremes(ctx context.Context, profile momentinput.Profile, gu, gv float64, work *freeform.FreeformWork, walks *momentinput.ProfileWalks) (float64, float64, float64, error) {
	if err := freeform.RequireFiniteDirection(gu, gv); err != nil {
		return 0, 0, 0, err
	}
	if walks != nil && !walks.Matches(profile) {
		return 0, 0, 0, momentinput.ErrResolvedWalksMismatch
	}
	loops := append([]sectionrecord.LoopRecord{profile.Outer}, profile.Holes...)
	counts := make([]int, len(loops))
	for li, loop := range loops {
		counts[li] = len(loop.Segments)
	}
	return BoundaryExtremesBoundedContext(ctx, gu, gv, work, counts, func(li, si int) (survey2d.SegmentWalk, error) {
		return momentinput.ResolveOrRead(loops[li].Segments[si], work, walks, li, si)
	})
}
