package proof

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
)

// These refusals are constructed once so rejected replay does not allocate.
var (
	errExactErrorReplay3 = fmt.Errorf("%w: missing rational error input", decaderr.ErrUnsupported)
)

// ReplayRationalRatError bounds an exact rational difference without lifting
// the same held rational repeatedly. It does not mutate either input.
func ReplayRationalRatError(ctx context.Context, exact, held *big.Rat, m ReplayKeyBudget) (float64, error) {
	if exact == nil || held == nil || m == nil {
		return 0, errExactErrorReplay3
	}
	if err := m.SpendWork(ctx, 4); err != nil {
		return 0, err
	}
	return replayRationalRatError(ctx, exact, held, m)
}

func replayRationalRatError(ctx context.Context, exact, held *big.Rat, m ReplayKeyBudget) (float64, error) {
	d, err := ReplayRatArithmetic(ctx, exact, held, '-', m)
	if err != nil {
		return 0, err
	}
	if d.Sign() < 0 {
		d.Neg(d)
	}
	if d.Sign() == 0 {
		return 0, nil
	}
	f, err := ReplayFractionFloat64(ctx, d.Num(), d.Denom(), m)
	if err != nil {
		return 0, err
	}
	if math.IsInf(f, 1) {
		return f, nil
	}
	r, err := ReplayFloatRat(ctx, f, m)
	if err != nil {
		return 0, err
	}
	cmp, err := ReplayRatCompare(ctx, r, d, m)
	if err != nil {
		return 0, err
	}
	if cmp != 0 {
		f = math.Nextafter(f, math.Inf(1))
	}
	return f, nil
}

// ReplaySqrtDyadicRat matches DySqrtUp using bounded exact comparisons.
func ReplaySqrtDyadicRat(ctx context.Context, q *big.Rat, m ReplayKeyBudget) (float64, error) {
	if q == nil || m == nil {
		return 0, errReplayBoundedSquareRootNeedsAnOperandAnd
	}
	if err := m.SpendWork(ctx, 1); err != nil {
		return 0, err
	}
	if q.Sign() <= 0 {
		return 0, nil
	}
	exp, ok := powerOfTwoExp(q.Denom())
	if !ok {
		return 0, errReplayReplaySquareRootIsNotDyadic
	}
	num, err := ReplayProductDifference(ctx, q.Num(), replayOne, replayZero, replayZero, m)
	if err != nil {
		return 0, err
	}
	d := fromBig(num, -exp)
	var f float64
	if d.big == nil {
		f = dySqrtSeedInline(d)
	} else {
		var certain bool
		f, certain = RatioSqrtSeed(d.big, replayOne, -d.exp)
		if !certain {
			return replaySqrtDyadicSearch(ctx, d, false, m)
		}
	}
	if math.IsInf(f, 0) || math.IsNaN(f) {
		f = math.MaxFloat64
	}
	for range SqrtAdjustLimit {
		r, err := ReplayFloatRat(ctx, f, m)
		if err != nil {
			return 0, err
		}
		square, err := ReplayRatArithmetic(ctx, r, r, '*', m)
		if err != nil {
			return 0, err
		}
		cmp, err := ReplayRatCompare(ctx, square, q, m)
		if err != nil {
			return 0, err
		}
		if cmp >= 0 {
			return f, nil
		}
		f = math.Nextafter(f, math.Inf(1))
		if math.IsInf(f, 1) {
			return f, nil
		}
	}
	return math.Inf(1), nil
}

// ReplaySqrtDownRat follows RatSqrtDown's seed and charged exact square tests.
func ReplaySqrtDownRat(ctx context.Context, q *big.Rat, m ReplayKeyBudget) (float64, error) {
	if q == nil || m == nil {
		return 0, errReplayBoundedSquareRootNeedsAnOperandAnd
	}
	if err := m.SpendWork(ctx, 1); err != nil {
		return 0, err
	}
	if q.Sign() <= 0 {
		return 0, nil
	}
	f, ok := RatSqrtSeed(q)
	if !ok {
		return replaySqrtRatSearch(ctx, q, true, m)
	}
	if math.IsInf(f, 0) || math.IsNaN(f) {
		f = math.MaxFloat64
	}
	for range SqrtAdjustLimit {
		r, err := ReplayFloatRat(ctx, f, m)
		if err != nil {
			return 0, err
		}
		square, err := ReplayRatArithmetic(ctx, r, r, '*', m)
		if err != nil {
			return 0, err
		}
		cmp, err := ReplayRatCompare(ctx, square, q, m)
		if err != nil {
			return 0, err
		}
		if cmp <= 0 {
			return f, nil
		}
		f = math.Nextafter(f, 0)
	}
	return 0, nil
}
