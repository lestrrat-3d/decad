package circularbounds

import (
	"context"
	"fmt"
	"math"
	"math/big"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/units"
)

// CircleStationReplayCache keeps exact circle constants under one replay meter.
// It does not authenticate the supplied station intervals.
type CircleStationReplayCache struct {
	meter        *proofbound.ReplayBudget
	radius, u, v *big.Rat
}

// These refusals are constructed once so rejected replay does not allocate.
var (
	errCircleStationCacheReplay1 = fmt.Errorf("%w: invalid circle station cache", decaderr.ErrUnsupported)
	errCircleStationCacheReplay2 = fmt.Errorf("%w: invalid circle radius", decaderr.ErrUnsupported)
	errCircleStationCacheReplay3 = fmt.Errorf("%w: invalid circle station cache use", decaderr.ErrUnsupported)
)

func NewCircleStationReplayCache(ctx context.Context, seg sectionrecord.CircleSeg, b *proofbound.ReplayBudget) (*CircleStationReplayCache, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	if b == nil || !seg.CCW || seg.TStart != 0 || seg.TEnd != 1 || seg.Radius.Unit() != units.Millimeter {
		return nil, errCircleStationCacheReplay1
	}
	if err := b.SpendWork(ctx, 64); err != nil {
		return nil, err
	}
	r, err := seg.Radius.In(units.Millimeter)
	if err != nil || !(r > 0) || math.IsInf(r, 0) {
		return nil, errCircleStationCacheReplay2
	}
	if err := b.ReserveRecords(ctx, 1); err != nil {
		return nil, err
	}
	if err := b.ReserveArray(ctx, 1, uint64(unsafe.Sizeof(CircleStationReplayCache{}))); err != nil {
		return nil, err
	}
	radius, err := proof.ReplayFloatRat(ctx, r, b)
	if err != nil {
		return nil, err
	}
	u, err := proof.ReplayFloatRat(ctx, seg.Center.U, b)
	if err != nil {
		return nil, err
	}
	v, err := proof.ReplayFloatRat(ctx, seg.Center.V, b)
	if err != nil {
		return nil, err
	}
	return &CircleStationReplayCache{meter: b, radius: radius, u: u, v: v}, nil
}

func (c *CircleStationReplayCache) Bound(ctx context.Context, heldU, heldV float64, s, co proofbound.RatInterval, b *proofbound.ReplayBudget) (proofbound.WalkEndBound, error) {
	if err := ctx.Err(); err != nil {
		return proofbound.WalkEndBound{}, err
	}
	if c == nil || b == nil || c.meter != b || s.Lo == nil || s.Hi == nil || co.Lo == nil || co.Hi == nil {
		return proofbound.WalkEndBound{}, errCircleStationCacheReplay3
	}
	if err := b.SpendWork(ctx, 64); err != nil {
		return proofbound.WalkEndBound{}, err
	}
	calc := func(iv proofbound.RatInterval, origin *big.Rat, held float64) (float64, error) {
		heldRat, err := proof.ReplayFloatRat(ctx, held, b)
		if err != nil {
			return 0, err
		}
		var gap [2]float64
		for i, endpoint := range [2]*big.Rat{iv.Lo, iv.Hi} {
			value, err := proof.ReplayRatArithmetic(ctx, endpoint, c.radius, '*', b)
			if err != nil {
				return 0, err
			}
			value, err = proof.ReplayRatArithmetic(ctx, value, origin, '+', b)
			if err != nil {
				return 0, err
			}
			gap[i], err = proof.ReplayRationalRatError(ctx, value, heldRat, b)
			if err != nil {
				return 0, err
			}
		}
		return math.Max(gap[0], gap[1]), nil
	}
	u, err := calc(co, c.u, heldU)
	if err != nil {
		return proofbound.WalkEndBound{}, err
	}
	v, err := calc(s, c.v, heldV)
	if err != nil {
		return proofbound.WalkEndBound{}, err
	}
	return proofbound.WalkEndBound{U: u, V: v}, nil
}
