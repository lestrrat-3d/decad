package proofbound

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
)

var replayTurnZero = new(big.Rat)
var replayTurnEight = big.NewRat(8, 1)
var replayTurnOctants = [8]*big.Rat{
	big.NewRat(0, 8), big.NewRat(1, 8), big.NewRat(2, 8), big.NewRat(3, 8),
	big.NewRat(4, 8), big.NewRat(5, 8), big.NewRat(6, 8), big.NewRat(7, 8),
}

// ReplayTurnSinCosInterval evaluates an exact rational turn without using the
// ordinary unmetered trig memo. Each period/octant operation and interval
// negation charges the supplied per-call replay budget.
func ReplayTurnSinCosInterval(ctx context.Context, t *big.Rat,
	budget *ReplayBudget) (RatInterval, RatInterval, error) {
	if t == nil || budget == nil {
		return RatInterval{}, RatInterval{}, decaderr.ErrUnsupported
	}
	fl, err := proofarith.ReplayFixedRatQuo(ctx, t, budget)
	if err != nil {
		return RatInterval{}, RatInterval{}, err
	}
	flRat, err := proofarith.ReplayReducedFraction(ctx, fl, replayFixedOne, budget)
	if err != nil {
		return RatInterval{}, RatInterval{}, err
	}
	cmp, err := proofarith.ReplayRatCompare(ctx, flRat, t, budget)
	if err != nil {
		return RatInterval{}, RatInterval{}, err
	}
	if cmp > 0 {
		fl, err = replayFixedIntOp(ctx, fl, replayFixedOne, '-', 0, budget)
		if err != nil {
			return RatInterval{}, RatInterval{}, err
		}
		flRat, err = proofarith.ReplayReducedFraction(ctx, fl, replayFixedOne, budget)
		if err != nil {
			return RatInterval{}, RatInterval{}, err
		}
	}
	f, err := proofarith.ReplayRatArithmetic(ctx, t, flRat, '-', budget)
	if err != nil {
		return RatInterval{}, RatInterval{}, err
	}
	scaled, err := proofarith.ReplayRatArithmetic(ctx, f, replayTurnEight, '*', budget)
	if err != nil {
		return RatInterval{}, RatInterval{}, err
	}
	kBig, err := proofarith.ReplayFixedRatQuo(ctx, scaled, budget)
	if err != nil {
		return RatInterval{}, RatInterval{}, err
	}
	if err := budget.SpendWork(ctx, 1); err != nil {
		return RatInterval{}, RatInterval{}, err
	}
	if !kBig.IsInt64() || kBig.Sign() < 0 || kBig.Int64() >= 8 {
		return RatInterval{}, RatInterval{}, decaderr.ErrUnsupported
	}
	k := int(kBig.Int64())
	s, err := proofarith.ReplayRatArithmetic(ctx, f, replayTurnOctants[k], '-', budget)
	if err != nil {
		return RatInterval{}, RatInterval{}, err
	}
	var sn, cs RatInterval
	if k%2 == 0 {
		sn, cs, err = ReplayTurnFirstOctantSinCos(ctx, s, budget)
	} else {
		complement, e := proofarith.ReplayRatArithmetic(ctx, replayFixedEighth, s, '-', budget)
		if e != nil {
			return RatInterval{}, RatInterval{}, e
		}
		sn, cs, err = ReplayTurnFirstOctantSinCos(ctx, complement, budget)
	}
	if err != nil {
		return RatInterval{}, RatInterval{}, err
	}
	switch k {
	case 0:
		return sn, cs, nil
	case 1:
		return cs, sn, nil
	case 2:
		neg, e := replayTurnNegInterval(ctx, sn, budget)
		return cs, neg, e
	case 3:
		neg, e := replayTurnNegInterval(ctx, cs, budget)
		return sn, neg, e
	case 4:
		negSn, e := replayTurnNegInterval(ctx, sn, budget)
		if e != nil {
			return RatInterval{}, RatInterval{}, e
		}
		negCs, e := replayTurnNegInterval(ctx, cs, budget)
		return negSn, negCs, e
	case 5:
		negCs, e := replayTurnNegInterval(ctx, cs, budget)
		if e != nil {
			return RatInterval{}, RatInterval{}, e
		}
		negSn, e := replayTurnNegInterval(ctx, sn, budget)
		return negCs, negSn, e
	case 6:
		negCs, e := replayTurnNegInterval(ctx, cs, budget)
		return negCs, sn, e
	default:
		negSn, e := replayTurnNegInterval(ctx, sn, budget)
		return negSn, cs, e
	}
}

func replayTurnNegInterval(ctx context.Context, iv RatInterval,
	budget *ReplayBudget) (RatInterval, error) {
	lo, err := proofarith.ReplayRatArithmetic(ctx, replayTurnZero, iv.Hi, '-', budget)
	if err != nil {
		return RatInterval{}, err
	}
	hi, err := proofarith.ReplayRatArithmetic(ctx, replayTurnZero, iv.Lo, '-', budget)
	if err != nil {
		return RatInterval{}, err
	}
	return IntervalOwned(lo, hi), nil
}
