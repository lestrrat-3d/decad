package circularbounds

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/units"
)

// ReplayCircleEndpointPlan holds a recorded circle's exact centre and radius
// for all stations in one replay call.
type ReplayCircleEndpointPlan struct {
	centerU, centerV, radius *big.Rat
}

var replayCircleOne = big.NewRat(1, 1)

// NewReplayCircleEndpointPlan charges the source scalar readings once. The
// caller must authenticate seg against its source seal.
func NewReplayCircleEndpointPlan(ctx context.Context, seg CircleSeg,
	budget *proofbound.ReplayBudget) (*ReplayCircleEndpointPlan, error) {
	if err := budget.ReserveBundle(ctx, 2, 1, 384); err != nil {
		return nil, err
	}
	radius, err := seg.Radius.In(units.Millimeter)
	if err != nil {
		return nil, err
	}
	r, err := proofarith.ReplayFloatRat(ctx, radius, budget)
	if err != nil {
		return nil, err
	}
	cu, err := proofarith.ReplayFloatRat(ctx, seg.Center.U, budget)
	if err != nil {
		return nil, err
	}
	cv, err := proofarith.ReplayFloatRat(ctx, seg.Center.V, budget)
	if err != nil {
		return nil, err
	}
	return &ReplayCircleEndpointPlan{centerU: cu, centerV: cv, radius: r}, nil
}

// Endpoint encloses the recorded circle at one exact turn parameter.
func (plan *ReplayCircleEndpointPlan) Endpoint(ctx context.Context, rt *big.Rat,
	budget *proofbound.ReplayBudget) (proofbound.RatInterval, proofbound.RatInterval, error) {
	if plan == nil || rt == nil || budget == nil {
		return proofbound.RatInterval{}, proofbound.RatInterval{}, decaderr.ErrUnsupported
	}
	if plan.radius.Sign() == 0 {
		u, err := proofbound.ReplayIntervalScale(ctx,
			proofbound.RatInterval{Lo: plan.centerU, Hi: plan.centerU},
			replayCircleOne, budget)
		if err != nil {
			return proofbound.RatInterval{}, proofbound.RatInterval{}, err
		}
		v, err := proofbound.ReplayIntervalScale(ctx,
			proofbound.RatInterval{Lo: plan.centerV, Hi: plan.centerV},
			replayCircleOne, budget)
		return u, v, err
	}
	sin, cos, err := proofbound.ReplayQuarterTurnSinCos(ctx, rt, budget)
	if err != nil {
		return proofbound.RatInterval{}, proofbound.RatInterval{}, err
	}
	uRadius, err := proofbound.ReplayIntervalScale(ctx, cos, plan.radius, budget)
	if err != nil {
		return proofbound.RatInterval{}, proofbound.RatInterval{}, err
	}
	vRadius, err := proofbound.ReplayIntervalScale(ctx, sin, plan.radius, budget)
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
