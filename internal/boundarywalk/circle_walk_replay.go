package boundarywalk

import (
	"context"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/units"
)

// WholeCircleWalkReplay resolves only a whole recorded circle. All exact
// endpoint and length errors use the caller's replay meter.
func WholeCircleWalkReplay(ctx context.Context, seg sectionrecord.CircleSeg,
	budget *proofbound.ReplayBudget) (survey2d.SegmentWalk, error) {
	if budget == nil {
		return survey2d.SegmentWalk{}, decaderr.ErrUnsupported
	}
	if err := budget.SpendWork(ctx, 32); err != nil {
		return survey2d.SegmentWalk{}, err
	}
	if seg.Radius.Unit() != units.Millimeter || !finiteReplayCircle(seg) ||
		!((seg.TStart == 0 && seg.TEnd == 1 && seg.CCW) ||
			(seg.TStart == 1 && seg.TEnd == 0 && !seg.CCW)) {
		return survey2d.SegmentWalk{}, decaderr.ErrUnsupported
	}
	r := seg.Radius.Mag()
	th0, th1 := 2*math.Pi*seg.TStart, 2*math.Pi*seg.TEnd
	w := circularWalk(seg.Center.U, seg.Center.V, r, th0, th1,
		math.Abs(r), proofbound.CircularSweepUpper(seg.TStart, seg.TEnd))
	w.Closed = math.Abs(math.Abs(th1-th0)-2*math.Pi) < 1e-12
	cu, err := proofarith.ReplayFloatRat(ctx, seg.Center.U, budget)
	if err != nil {
		return survey2d.SegmentWalk{}, err
	}
	cv, err := proofarith.ReplayFloatRat(ctx, seg.Center.V, budget)
	if err != nil {
		return survey2d.SegmentWalk{}, err
	}
	radius, err := proofarith.ReplayFloatRat(ctx, r, budget)
	if err != nil {
		return survey2d.SegmentWalk{}, err
	}
	u, err := proofarith.ReplayRatArithmetic(ctx, cu, radius, '+', budget)
	if err != nil {
		return survey2d.SegmentWalk{}, err
	}
	for _, end := range []struct {
		heldU, heldV float64
		out          *proofbound.WalkEndBound
	}{
		{w.StartU, w.StartV, &w.StartBound},
		{w.EndU, w.EndV, &w.EndBound},
	} {
		end.out.U, err = proofarith.ReplayRationalFloatError(ctx, u, end.heldU, budget)
		if err != nil {
			return survey2d.SegmentWalk{}, err
		}
		end.out.V, err = proofarith.ReplayRationalFloatError(ctx, cv, end.heldV, budget)
		if err != nil {
			return survey2d.SegmentWalk{}, err
		}
	}
	two, err := proofarith.ReplayFloatRat(ctx, 2, budget)
	if err != nil {
		return survey2d.SegmentWalk{}, err
	}
	intervalError := 0.0
	for _, pi := range [...]*big.Rat{proofbound.PiLower, proofbound.PiUpper} {
		angle, e := proofarith.ReplayRatArithmetic(ctx, pi, two, '*', budget)
		if e != nil {
			return survey2d.SegmentWalk{}, e
		}
		length, e := proofarith.ReplayRatArithmetic(ctx, angle, radius, '*', budget)
		if e != nil {
			return survey2d.SegmentWalk{}, e
		}
		bound, e := proofarith.ReplayRationalFloatError(ctx, length, w.Length, budget)
		if e != nil {
			return survey2d.SegmentWalk{}, e
		}
		intervalError = math.Max(intervalError, bound)
	}
	w.LengthBound = math.Min(w.LengthBound, intervalError)
	return w, nil
}

func finiteReplayCircle(seg sectionrecord.CircleSeg) bool {
	for _, v := range [...]float64{seg.Center.U, seg.Center.V, seg.Radius.Mag(), seg.TStart, seg.TEnd} {
		if math.IsNaN(v) || math.IsInf(v, 0) {
			return false
		}
	}
	return seg.Radius.Mag() > 0
}
