package boundarywalk

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// JunctionVertexAtReplay reads the ordinary junction only when every arc
// natural end it reaches has zero exact radial residual.
func JunctionVertexAtReplay(ctx context.Context, segs []sectionrecord.CurveSegment,
	prev, next survey2d.SideWalk, budget *proofbound.ReplayBudget) (float64, float64, float64, error) {
	if budget == nil || len(prev.Segs) == 0 || len(next.Segs) == 0 {
		return 0, 0, 0, decaderr.ErrUnsupported
	}
	if err := budget.SpendWork(ctx, 64); err != nil {
		return 0, 0, 0, err
	}
	i, j := prev.Segs[len(prev.Segs)-1], next.Segs[0]
	if i < 0 || i >= len(segs) || j < 0 || j >= len(segs) {
		return 0, 0, 0, decaderr.ErrUnsupported
	}
	end, start := segs[i], segs[j]
	if err := replayArcNaturalEnd(ctx, end, false, budget); err != nil {
		return 0, 0, 0, err
	}
	if err := replayArcNaturalEnd(ctx, start, true, budget); err != nil {
		return 0, 0, 0, err
	}
	u, v, bound := junctionVertexWithBounds(end, prev.SegmentWalk, start,
		next.SegmentWalk, next.StartBound, prev.EndBound)
	return u, v, proofbound.WalkEndBoundAllow(bound), nil
}

// JunctionStartBoundReplay keeps the componentwise station bound used by
// tessellation.SampleLoop. The arc-end check precedes its scalar arithmetic.
func JunctionStartBoundReplay(ctx context.Context, segs []sectionrecord.CurveSegment,
	prev, next survey2d.SideWalk, budget *proofbound.ReplayBudget) (proofbound.WalkEndBound, error) {
	if budget == nil || len(prev.Segs) == 0 || len(next.Segs) == 0 {
		return proofbound.WalkEndBound{}, decaderr.ErrUnsupported
	}
	if err := budget.SpendWork(ctx, 64); err != nil {
		return proofbound.WalkEndBound{}, err
	}
	i, j := prev.Segs[len(prev.Segs)-1], next.Segs[0]
	if i < 0 || i >= len(segs) || j < 0 || j >= len(segs) {
		return proofbound.WalkEndBound{}, decaderr.ErrUnsupported
	}
	if err := replayArcNaturalEnd(ctx, segs[i], false, budget); err != nil {
		return proofbound.WalkEndBound{}, err
	}
	if err := replayArcNaturalEnd(ctx, segs[j], true, budget); err != nil {
		return proofbound.WalkEndBound{}, err
	}
	return JunctionStartBound(segs[i], prev.SegmentWalk, segs[j], next.SegmentWalk), nil
}

func replayArcNaturalEnd(ctx context.Context, segment sectionrecord.CurveSegment,
	start bool, budget *proofbound.ReplayBudget) error {
	if err := budget.SpendWork(ctx, 8); err != nil {
		return err
	}
	arc, ok := segment.(sectionrecord.ArcSeg)
	if !ok {
		return nil
	}
	t := arc.TEnd
	if start {
		t = arc.TStart
	}
	if t != 1 {
		return nil
	}
	squared := func(p sectionrecord.Point2) (*big.Rat, error) {
		u, err := proofarith.ReplayFloatRat(ctx, p.U, budget)
		if err != nil {
			return nil, err
		}
		v, err := proofarith.ReplayFloatRat(ctx, p.V, budget)
		if err != nil {
			return nil, err
		}
		cu, err := proofarith.ReplayFloatRat(ctx, arc.Center.U, budget)
		if err != nil {
			return nil, err
		}
		cv, err := proofarith.ReplayFloatRat(ctx, arc.Center.V, budget)
		if err != nil {
			return nil, err
		}
		du, err := proofarith.ReplayRatArithmetic(ctx, u, cu, '-', budget)
		if err != nil {
			return nil, err
		}
		dv, err := proofarith.ReplayRatArithmetic(ctx, v, cv, '-', budget)
		if err != nil {
			return nil, err
		}
		u2, err := proofarith.ReplayRatArithmetic(ctx, du, du, '*', budget)
		if err != nil {
			return nil, err
		}
		v2, err := proofarith.ReplayRatArithmetic(ctx, dv, dv, '*', budget)
		if err != nil {
			return nil, err
		}
		return proofarith.ReplayRatArithmetic(ctx, u2, v2, '+', budget)
	}
	a, err := squared(arc.Start)
	if err != nil {
		return err
	}
	b, err := squared(arc.End)
	if err != nil {
		return err
	}
	cmp, err := proofarith.ReplayRatCompare(ctx, a, b, budget)
	if err != nil {
		return err
	}
	if cmp != 0 {
		return decaderr.ErrUnsupported
	}
	return nil
}
