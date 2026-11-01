package proofbound

import (
	"context"
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
)

// ReplayIntervalAdd performs the ordinary endpoint sum with charged rational arithmetic.
func ReplayIntervalAdd(ctx context.Context, a, b RatInterval,
	budget *ReplayBudget) (RatInterval, error) {
	return replayIntervalSum(ctx, a, b, false, budget)
}

// ReplayIntervalSub performs the ordinary endpoint difference with charged rational arithmetic.
func ReplayIntervalSub(ctx context.Context, a, b RatInterval,
	budget *ReplayBudget) (RatInterval, error) {
	return replayIntervalSum(ctx, a, b, true, budget)
}

// ReplayIntervalScale multiplies both endpoints by an exact scale, swapping
// them when the scale is negative.
func ReplayIntervalScale(ctx context.Context, a RatInterval, scale *big.Rat,
	budget *ReplayBudget) (RatInterval, error) {
	lo, hi := a.Lo, a.Hi
	if scale.Sign() < 0 {
		lo, hi = hi, lo
	}
	lo, err := proofarith.ReplayRatArithmetic(ctx, lo, scale, '*', budget)
	if err != nil {
		return RatInterval{}, err
	}
	hi, err = proofarith.ReplayRatArithmetic(ctx, hi, scale, '*', budget)
	if err != nil {
		return RatInterval{}, err
	}
	return IntervalOwned(lo, hi), nil
}

// ReplayIntervalMul computes all four corner products before selecting the
// exact low and high endpoints. This also covers intervals crossing zero.
func ReplayIntervalMul(ctx context.Context, a, b RatInterval,
	budget *ReplayBudget) (RatInterval, error) {
	pairs := [4][2]*big.Rat{{a.Lo, b.Lo}, {a.Lo, b.Hi}, {a.Hi, b.Lo}, {a.Hi, b.Hi}}
	var products [4]*big.Rat
	for i, pair := range pairs {
		value, err := proofarith.ReplayRatArithmetic(ctx, pair[0], pair[1], '*', budget)
		if err != nil {
			return RatInterval{}, err
		}
		products[i] = value
	}
	lo, hi := 0, 0
	for i := 1; i < len(products); i++ {
		less, err := proofarith.ReplayRatCompare(ctx, products[i], products[lo], budget)
		if err != nil {
			return RatInterval{}, err
		}
		if less < 0 {
			lo = i
		}
		greater, err := proofarith.ReplayRatCompare(ctx, products[i], products[hi], budget)
		if err != nil {
			return RatInterval{}, err
		}
		if greater > 0 {
			hi = i
		}
	}
	if lo == hi {
		// The two result endpoints must not share mutable rational storage.
		return replayIntervalCopy(ctx, RatInterval{Lo: products[lo], Hi: products[hi]}, budget)
	}
	return IntervalOwned(products[lo], products[hi]), nil
}
