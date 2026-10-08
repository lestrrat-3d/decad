package revolveaxis

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// ResolvedWalks holds the plane and axis-coordinate readings of one recorded
// loop or chain. Segs has the same indices as Plane, while Walks may coalesce
// adjacent segments. Plane stays beside Walks because a bound against the
// recorded point needs the plane coordinates before axis re-expression and
// near-axis snapping. SingleClosed is true only for a whole closed loop walk.
type ResolvedWalks struct {
	Walks        []survey2d.SideWalk
	Kinds        []WallKind
	Plane        []survey2d.SegmentWalk
	Segs         []sectionrecord.CurveSegment
	SingleClosed bool
}

type WalkCharge func(sectionrecord.CurveSegment, survey2d.SegmentWalk) (survey2d.SegmentWalk, error)

// ResolveLoop resolves a closed recorded loop for both the revolve body and
// its tessellation. charge folds the payload's section displacement into each
// axis-coordinate walk before adjacent walks are coalesced.
func ResolveLoop(ctx context.Context, loop sectionrecord.LoopRecord, work *freeform.FreeformWork,
	what string, charge WalkCharge, snapTol float64,
) (ResolvedWalks, error) {
	if err := ctx.Err(); err != nil {
		return ResolvedWalks{}, err
	}
	if len(loop.Segments) == 0 {
		return ResolvedWalks{}, fmt.Errorf(`%w: a recorded loop holds no segments`, decaderr.ErrDegenerate)
	}
	return resolveRecordedWalks(ctx, loop.Segments, false, work, what, charge, snapTol)
}

// ResolveChain resolves an open recorded chain without joining its last
// segment to its first.
func ResolveChain(ctx context.Context, chain sectionrecord.ChainRecord, work *freeform.FreeformWork,
	what string, charge WalkCharge, snapTol float64,
) (ResolvedWalks, error) {
	if len(chain.Segments) == 0 {
		return ResolvedWalks{}, fmt.Errorf(`%w: a recorded chain holds no segments`, decaderr.ErrDegenerate)
	}
	return resolveRecordedWalks(ctx, chain.Segments, true, work, what, charge, snapTol)
}

func resolveRecordedWalks(ctx context.Context, segs []sectionrecord.CurveSegment, open bool,
	work *freeform.FreeformWork, what string, charge WalkCharge, snapTol float64,
) (ResolvedWalks, error) {
	raw := make([]survey2d.SideWalk, len(segs))
	plane := make([]survey2d.SegmentWalk, len(segs))
	for i, seg := range segs {
		if err := ctx.Err(); err != nil {
			return ResolvedWalks{}, err
		}
		w, err := boundarywalk.WalkOf(seg, work)
		if err != nil {
			return ResolvedWalks{}, err
		}
		if err := boundarywalk.RequireAnalyticWalk(w, what); err != nil {
			return ResolvedWalks{}, err
		}
		plane[i] = w
		axisWalk, err := charge(seg, w)
		if err != nil {
			return ResolvedWalks{}, err
		}
		raw[i] = survey2d.SideWalk{SegmentWalk: axisWalk, Segs: []int{i}}
	}
	var walks []survey2d.SideWalk
	var err error
	if open {
		walks, err = boundarywalk.CoalesceChainWalksContext(ctx, raw)
	} else {
		walks, err = boundarywalk.CoalesceWalksContext(ctx, raw)
	}
	if err != nil {
		return ResolvedWalks{}, err
	}
	kinds := make([]WallKind, len(walks))
	for i, w := range walks {
		kinds[i] = Classify(w.SegmentWalk, snapTol)
	}
	return ResolvedWalks{Walks: walks, Kinds: kinds, Plane: plane, Segs: segs,
		SingleClosed: !open && len(walks) == 1 && walks[0].Closed}, nil
}

// RequireChainAxisIncidence checks an open walk without wrapping its last
// junction onto its first. A free pole has one off-axis incident wall;
// an interior axis junction needs one swept wall and one axis line.
func RequireChainAxisIncidence(resolved ResolvedWalks) error {
	walks, kinds := resolved.Walks, resolved.Kinds
	n := len(walks)
	startPole, endPole := walks[0].StartV == 0, walks[n-1].EndV == 0
	if startPole && endPole {
		return fmt.Errorf(`%w: a chain with both free ends on the revolve axis needs closed-sheet pole topology`, decaderr.ErrUnsupported)
	}
	if startPole && kinds[0] == WallAxis || endPole && kinds[n-1] == WallAxis {
		return fmt.Errorf(`%w: a chain free end on the revolve axis has no incident swept wall`, decaderr.ErrUnsupported)
	}
	seen := map[float64]struct{}{}
	for i, w := range walks {
		if w.StartV != 0 {
			continue
		}
		if _, duplicate := seen[w.StartU]; duplicate {
			return fmt.Errorf(`%w: two chain junctions meet the revolve axis at the same point`, decaderr.ErrDegenerate)
		}
		seen[w.StartU] = struct{}{}
		if i == 0 {
			continue
		}
		if walks[i-1].EndV != 0 || (kinds[i-1] == WallAxis) == (kinds[i] == WallAxis) {
			return fmt.Errorf(`%w: a chain interior axis junction needs one swept wall and one axis line`, decaderr.ErrDegenerate)
		}
	}
	if endPole {
		if _, duplicate := seen[walks[n-1].EndU]; duplicate {
			return fmt.Errorf(`%w: two chain junctions meet the revolve axis at the same point`, decaderr.ErrDegenerate)
		}
	}
	return nil
}
