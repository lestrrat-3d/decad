package circularbounds

import (
	"context"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// ReplayArcEndpointPlan holds one recorded arc's exact centre, radius bracket
// and endpoint-angle brackets for every station in the same replay call.
type ReplayArcEndpointPlan struct {
	centerU, centerV *big.Rat
	radius           proofbound.RatInterval
	start, sweep     proofbound.RatInterval
}

// NewReplayArcEndpointPlan charges the source readings once before any
// station uses them. The caller must authenticate seg against its source seal.
func NewReplayArcEndpointPlan(ctx context.Context, seg ArcSeg,
	budget *proofbound.ReplayBudget) (*ReplayArcEndpointPlan, error) {
	if err := budget.ReserveBundle(ctx, 1, 1, 512); err != nil {
		return nil, err
	}
	read := func(value float64) (*big.Rat, error) {
		return proofarith.ReplayFloatRat(ctx, value, budget)
	}
	cu, err := read(seg.Center.U)
	if err != nil {
		return nil, err
	}
	cv, err := read(seg.Center.V)
	if err != nil {
		return nil, err
	}
	su, err := read(seg.Start.U)
	if err != nil {
		return nil, err
	}
	sv, err := read(seg.Start.V)
	if err != nil {
		return nil, err
	}
	eu, err := read(seg.End.U)
	if err != nil {
		return nil, err
	}
	ev, err := read(seg.End.V)
	if err != nil {
		return nil, err
	}
	dx0, err := proofarith.ReplayRatArithmetic(ctx, su, cu, '-', budget)
	if err != nil {
		return nil, err
	}
	dy0, err := proofarith.ReplayRatArithmetic(ctx, sv, cv, '-', budget)
	if err != nil {
		return nil, err
	}
	dx1, err := proofarith.ReplayRatArithmetic(ctx, eu, cu, '-', budget)
	if err != nil {
		return nil, err
	}
	dy1, err := proofarith.ReplayRatArithmetic(ctx, ev, cv, '-', budget)
	if err != nil {
		return nil, err
	}
	dxSquared, err := proofarith.ReplayRatArithmetic(ctx, dx0, dx0, '*', budget)
	if err != nil {
		return nil, err
	}
	dySquared, err := proofarith.ReplayRatArithmetic(ctx, dy0, dy0, '*', budget)
	if err != nil {
		return nil, err
	}
	r2, err := proofarith.ReplayRatArithmetic(ctx, dxSquared, dySquared, '+', budget)
	if err != nil {
		return nil, err
	}
	if r2.Sign() == 0 {
		return nil, decaderr.ErrUnsupported
	}
	rLoFloat, err := proofbound.ReplayRatSqrtDown(ctx, r2, budget)
	if err != nil {
		return nil, err
	}
	rHiFloat, err := proofbound.ReplayRatSqrtUp(ctx, r2, budget)
	if err != nil {
		return nil, err
	}
	rLo, err := read(rLoFloat)
	if err != nil {
		return nil, err
	}
	rHi, err := read(rHiFloat)
	if err != nil {
		return nil, err
	}
	if err := budget.SpendWork(ctx, 10); err != nil {
		return nil, err
	}
	heldDY0 := seg.Start.V - seg.Center.V
	heldDY1 := seg.End.V - seg.Center.V
	a0, err := proofbound.ReplayAtan2Interval(ctx, dy0, dx0,
		heldDY0 == 0 && math.Signbit(heldDY0), budget)
	if err != nil {
		return nil, err
	}
	a1, err := proofbound.ReplayAtan2Interval(ctx, dy1, dx1,
		heldDY1 == 0 && math.Signbit(heldDY1), budget)
	if err != nil {
		return nil, err
	}
	sweep, err := proofbound.ReplayIntervalSub(ctx, a1, a0, budget)
	if err != nil {
		return nil, err
	}
	heldA0 := math.Atan2(heldDY0, seg.Start.U-seg.Center.U)
	heldA1 := math.Atan2(heldDY1, seg.End.U-seg.Center.U)
	if heldA1-heldA0 <= 0 {
		sweep, err = proofbound.ReplayIntervalAdd(ctx, sweep, proofbound.TwoPiIv, budget)
		if err != nil {
			return nil, err
		}
	}
	return &ReplayArcEndpointPlan{centerU: cu, centerV: cv,
		radius: proofbound.RatInterval{Lo: rLo, Hi: rHi}, start: a0, sweep: sweep}, nil
}

// Endpoint encloses the source arc's point at an exact parameter. One plan
// can serve all stations, with every station operation charged separately.
func (plan *ReplayArcEndpointPlan) Endpoint(ctx context.Context, rt *big.Rat,
	budget *proofbound.ReplayBudget) (proofbound.RatInterval, proofbound.RatInterval, error) {
	if plan == nil || rt == nil || budget == nil {
		return proofbound.RatInterval{}, proofbound.RatInterval{}, decaderr.ErrUnsupported
	}
	step, err := proofbound.ReplayIntervalScale(ctx, plan.sweep, rt, budget)
	if err != nil {
		return proofbound.RatInterval{}, proofbound.RatInterval{}, err
	}
	angle, err := proofbound.ReplayIntervalAdd(ctx, plan.start, step, budget)
	if err != nil {
		return proofbound.RatInterval{}, proofbound.RatInterval{}, err
	}
	sin, cos, ok, err := proofbound.ReplayRadSinCosSpan(ctx, angle, budget)
	if err != nil {
		return proofbound.RatInterval{}, proofbound.RatInterval{}, err
	}
	if !ok {
		return proofbound.RatInterval{}, proofbound.RatInterval{}, decaderr.ErrUnsupported
	}
	uRadius, err := proofbound.ReplayIntervalMul(ctx, plan.radius, cos, budget)
	if err != nil {
		return proofbound.RatInterval{}, proofbound.RatInterval{}, err
	}
	vRadius, err := proofbound.ReplayIntervalMul(ctx, plan.radius, sin, budget)
	if err != nil {
		return proofbound.RatInterval{}, proofbound.RatInterval{}, err
	}
	u, err := proofbound.ReplayIntervalAdd(ctx,
		proofbound.RatInterval{Lo: plan.centerU, Hi: plan.centerU}, uRadius, budget)
	if err != nil {
		return proofbound.RatInterval{}, proofbound.RatInterval{}, err
	}
	v, err := proofbound.ReplayIntervalAdd(ctx,
		proofbound.RatInterval{Lo: plan.centerV, Hi: plan.centerV}, vRadius, budget)
	return u, v, err
}
