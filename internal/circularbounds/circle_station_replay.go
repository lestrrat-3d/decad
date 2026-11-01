package circularbounds

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/units"
)

// CircleStationReplay bounds a held interior station of an authenticated whole circle.
func CircleStationReplay(ctx context.Context, seg sectionrecord.CircleSeg, k, n int, heldU, heldV float64,
	b *proofbound.ReplayBudget) (proofbound.WalkEndBound, error) {
	if b == nil || !seg.CCW || seg.TStart != 0 || seg.TEnd != 1 || seg.Radius.Unit() != units.Millimeter {
		return proofbound.WalkEndBound{}, fmt.Errorf("%w: unsupported bounded circle station", decaderr.ErrUnsupported)
	}
	r, err := seg.Radius.In(units.Millimeter)
	if err != nil || !(r > 0) || math.IsInf(r, 0) {
		return proofbound.WalkEndBound{}, fmt.Errorf("%w: invalid bounded circle radius", decaderr.ErrUnsupported)
	}
	s, c, err := proofbound.CircleTurnFractionReplay(ctx, k, n, b)
	if err != nil {
		return proofbound.WalkEndBound{}, err
	}
	return circleStationFromIntervals(ctx, seg, heldU, heldV, s, c, b)
}

// circleStationFromIntervals bounds the held point against certified
// sine and cosine enclosures of the recorded turn.
func circleStationFromIntervals(ctx context.Context, seg sectionrecord.CircleSeg, heldU, heldV float64,
	s, c proofbound.RatInterval, b *proofbound.ReplayBudget) (proofbound.WalkEndBound, error) {
	if b == nil || !seg.CCW || seg.TStart != 0 || seg.TEnd != 1 || seg.Radius.Unit() != units.Millimeter || s.Lo == nil || s.Hi == nil || c.Lo == nil || c.Hi == nil {
		return proofbound.WalkEndBound{}, fmt.Errorf("%w: invalid bounded circle station", decaderr.ErrUnsupported)
	}
	r, err := seg.Radius.In(units.Millimeter)
	if err != nil || !(r > 0) || math.IsInf(r, 0) {
		return proofbound.WalkEndBound{}, fmt.Errorf("%w: invalid bounded circle radius", decaderr.ErrUnsupported)
	}
	radius, err := proof.ReplayFloatRat(ctx, r, b)
	if err != nil {
		return proofbound.WalkEndBound{}, err
	}
	calc := func(iv proofbound.RatInterval, center, held float64) (float64, error) {
		origin, err := proof.ReplayFloatRat(ctx, center, b)
		if err != nil {
			return 0, err
		}
		var gap [2]float64
		for i, endpoint := range [2]*big.Rat{iv.Lo, iv.Hi} {
			value, err := proof.ReplayRatArithmetic(ctx, endpoint, radius, '*', b)
			if err != nil {
				return 0, err
			}
			value, err = proof.ReplayRatArithmetic(ctx, value, origin, '+', b)
			if err != nil {
				return 0, err
			}
			gap[i], err = proof.ReplayRationalFloatError(ctx, value, held, b)
			if err != nil {
				return 0, err
			}
		}
		return math.Max(gap[0], gap[1]), nil
	}
	u, err := calc(c, seg.Center.U, heldU)
	if err != nil {
		return proofbound.WalkEndBound{}, err
	}
	v, err := calc(s, seg.Center.V, heldV)
	if err != nil {
		return proofbound.WalkEndBound{}, err
	}
	return proofbound.WalkEndBound{U: u, V: v}, nil
}
