package proofbound

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
)

var replayAtanOne = big.NewRat(1, 1)

func replayIntervalCopy(ctx context.Context, iv RatInterval,
	budget *ReplayBudget) (RatInterval, error) {
	lo, err := proofarith.ReplayReducedFraction(ctx, iv.Lo.Num(), iv.Lo.Denom(), budget)
	if err != nil {
		return RatInterval{}, err
	}
	hi, err := proofarith.ReplayReducedFraction(ctx, iv.Hi.Num(), iv.Hi.Denom(), budget)
	if err != nil {
		return RatInterval{}, err
	}
	return IntervalOwned(lo, hi), nil
}

func replayIntervalSum(ctx context.Context, a, b RatInterval, subtract bool,
	budget *ReplayBudget) (RatInterval, error) {
	op := byte('+')
	bLo, bHi := b.Lo, b.Hi
	if subtract {
		op = '-'
		bLo, bHi = b.Hi, b.Lo
	}
	lo, err := proofarith.ReplayRatArithmetic(ctx, a.Lo, bLo, op, budget)
	if err != nil {
		return RatInterval{}, err
	}
	hi, err := proofarith.ReplayRatArithmetic(ctx, a.Hi, bHi, op, budget)
	if err != nil {
		return RatInterval{}, err
	}
	return IntervalOwned(lo, hi), nil
}

func replayAtanPositiveInterval(ctx context.Context, x *big.Rat,
	budget *ReplayBudget) (RatInterval, error) {
	cmp, err := proofarith.ReplayRatCompare(ctx, x, replayFixedOneHalf, budget)
	if err != nil {
		return RatInterval{}, err
	}
	if cmp <= 0 {
		return ReplayAtanSmallInterval(ctx, x, budget)
	}
	cmp, err = proofarith.ReplayRatCompare(ctx, x, replayAtanOne, budget)
	if err != nil {
		return RatInterval{}, err
	}
	if cmp > 0 {
		return RatInterval{}, decaderr.ErrUnsupported
	}
	numerator, err := proofarith.ReplayRatArithmetic(ctx, x, replayAtanOne, '-', budget)
	if err != nil {
		return RatInterval{}, err
	}
	denominator, err := proofarith.ReplayRatArithmetic(ctx, x, replayAtanOne, '+', budget)
	if err != nil {
		return RatInterval{}, err
	}
	q, err := proofarith.ReplayRatArithmetic(ctx, numerator, denominator, '/', budget)
	if err != nil {
		return RatInterval{}, err
	}
	small, err := ReplayAtanSmallInterval(ctx, q, budget)
	if err != nil {
		return RatInterval{}, err
	}
	return replayIntervalSum(ctx, QuarterPiIv, small, false, budget)
}

// ReplayAtan2Interval evaluates the ordinary axis and octant formulas without
// process-wide memo state, charging all reached rational and series work.
func ReplayAtan2Interval(ctx context.Context, y, x *big.Rat, negativeZeroY bool,
	budget *ReplayBudget) (RatInterval, error) {
	if x == nil || y == nil || budget == nil {
		return RatInterval{}, decaderr.ErrUnsupported
	}
	if x.Sign() == 0 {
		if y.Sign() < 0 {
			return replayTurnNegInterval(ctx, HalfPiIv, budget)
		}
		return replayIntervalCopy(ctx, HalfPiIv, budget)
	}
	if y.Sign() == 0 {
		if x.Sign() < 0 {
			pi := RatInterval{Lo: PiLower, Hi: PiUpper}
			if negativeZeroY {
				return replayTurnNegInterval(ctx, pi, budget)
			}
			return replayIntervalCopy(ctx, pi, budget)
		}
		return replayIntervalCopy(ctx, RatInterval{Lo: replayTurnZero, Hi: replayTurnZero}, budget)
	}
	ax, ay := x, y
	var err error
	if x.Sign() < 0 {
		ax, err = proofarith.ReplayRatArithmetic(ctx, replayTurnZero, x, '-', budget)
		if err != nil {
			return RatInterval{}, err
		}
	}
	if y.Sign() < 0 {
		ay, err = proofarith.ReplayRatArithmetic(ctx, replayTurnZero, y, '-', budget)
		if err != nil {
			return RatInterval{}, err
		}
	}
	cmp, err := proofarith.ReplayRatCompare(ctx, ay, ax, budget)
	if err != nil {
		return RatInterval{}, err
	}
	var base RatInterval
	if cmp <= 0 {
		ratio, e := proofarith.ReplayRatArithmetic(ctx, ay, ax, '/', budget)
		if e != nil {
			return RatInterval{}, e
		}
		base, err = replayAtanPositiveInterval(ctx, ratio, budget)
	} else {
		ratio, e := proofarith.ReplayRatArithmetic(ctx, ax, ay, '/', budget)
		if e != nil {
			return RatInterval{}, e
		}
		small, e := replayAtanPositiveInterval(ctx, ratio, budget)
		if e != nil {
			return RatInterval{}, e
		}
		base, err = replayIntervalSum(ctx, HalfPiIv, small, true, budget)
	}
	if err != nil {
		return RatInterval{}, err
	}
	switch {
	case x.Sign() > 0 && y.Sign() > 0:
		return base, nil
	case x.Sign() > 0 && y.Sign() < 0:
		return replayTurnNegInterval(ctx, base, budget)
	case x.Sign() < 0 && y.Sign() > 0:
		return replayIntervalSum(ctx, RatInterval{Lo: PiLower, Hi: PiUpper}, base, true, budget)
	default:
		negPi, e := replayTurnNegInterval(ctx, RatInterval{Lo: PiLower, Hi: PiUpper}, budget)
		if e != nil {
			return RatInterval{}, e
		}
		return replayIntervalSum(ctx, negPi, base, false, budget)
	}
}
