package proof

import (
	"context"
	"math"
	"math/big"
)

// replaySqrtSearch finds the tight directed float by binary search over the
// ordered, nonnegative finite float encodings. Each square comparison is exact
// and charged by cmp; the search makes at most 64 comparisons plus one tie
// check, including for roots outside the finite or normal float range.
func replaySqrtSearch(ctx context.Context, down bool, m ReplayKeyBudget,
	cmp func(float64) (int, error)) (float64, error) {
	lo, hi := uint64(0), math.Float64bits(math.MaxFloat64)
	for lo < hi {
		if err := m.SpendWork(ctx, 1); err != nil {
			return 0, err
		}
		mid := lo + (hi-lo+1)/2
		c, err := cmp(math.Float64frombits(mid))
		if err != nil {
			return 0, err
		}
		if c <= 0 {
			lo = mid
		} else {
			hi = mid - 1
		}
	}
	f := math.Float64frombits(lo)
	if down {
		return f, nil
	}
	c, err := cmp(f)
	if err != nil {
		return 0, err
	}
	if c == 0 {
		return f, nil
	}
	if lo == math.Float64bits(math.MaxFloat64) {
		return math.Inf(1), nil
	}
	return math.Float64frombits(lo + 1), nil
}

func replaySqrtDyadicSearch(ctx context.Context, d Dyadic, down bool, m ReplayKeyBudget) (float64, error) {
	return replaySqrtSearch(ctx, down, m, func(f float64) (int, error) {
		v, _ := DyOf(f)
		sq, err := ReplayDyadicArithmetic(ctx, v, v, '*', m)
		if err != nil {
			return 0, err
		}
		diff, err := ReplayDyadicArithmetic(ctx, sq, d, '-', m)
		if err != nil {
			return 0, err
		}
		return diff.Sign(), nil
	})
}

func replaySqrtRatSearch(ctx context.Context, q *big.Rat, down bool, m ReplayKeyBudget) (float64, error) {
	return replaySqrtSearch(ctx, down, m, func(f float64) (int, error) {
		v, _ := DyOf(f)
		sq, err := ReplayDyadicArithmetic(ctx, v, v, '*', m)
		if err != nil {
			return 0, err
		}
		r, err := ReplayDyadicRat(ctx, sq, m)
		if err != nil {
			return 0, err
		}
		return ReplayRatCompare(ctx, r, q, m)
	})
}
