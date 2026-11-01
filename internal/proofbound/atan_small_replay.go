package proofbound

import (
	"context"
	"math/big"
	"math/bits"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
)

var replayAtanDivisors = func() [130]*big.Int {
	var out [130]*big.Int
	for i := range out {
		out[i] = big.NewInt(int64(i))
	}
	return out
}()

func replayFixedMul(ctx context.Context, a, b *big.Int, up bool,
	budget *ReplayBudget) (*big.Int, error) {
	p, err := replayFixedIntOp(ctx, a, b, '*', 0, budget)
	if err != nil {
		return nil, err
	}
	if err := budget.SpendWork(ctx, uint64((p.BitLen()+bits.UintSize-1)/bits.UintSize+1)); err != nil {
		return nil, err
	}
	roundUp := up && p.Sign() != 0 && p.TrailingZeroBits() < TrigFixedBits
	out, err := replayFixedIntOp(ctx, p, nil, 's', TrigFixedBits, budget)
	if err != nil {
		return nil, err
	}
	if roundUp {
		return replayFixedIntOp(ctx, out, replayFixedOne, '+', 0, budget)
	}
	return out, nil
}

func replayFixedDiv(ctx context.Context, a *big.Int, divisor int, up bool,
	budget *ReplayBudget) (*big.Int, error) {
	if divisor <= 0 || divisor >= len(replayAtanDivisors) {
		return nil, decaderr.ErrUnsupported
	}
	if up {
		var err error
		a, err = replayFixedIntOp(ctx, a, replayAtanDivisors[divisor-1], '+', 0, budget)
		if err != nil {
			return nil, err
		}
	}
	return replayFixedIntOp(ctx, a, replayAtanDivisors[divisor], '/', 0, budget)
}

// ReplayAtanSmallInterval evaluates the ordinary 64-term arctangent
// enclosure using charged fixed-point operations and reduced fractions.
func ReplayAtanSmallInterval(ctx context.Context, x *big.Rat,
	budget *ReplayBudget) (RatInterval, error) {
	if x == nil || budget == nil {
		return RatInterval{}, decaderr.ErrUnsupported
	}
	negative := x.Sign() < 0
	if negative {
		var err error
		x, err = proofarith.ReplayRatArithmetic(ctx, replayTurnZero, x, '-', budget)
		if err != nil {
			return RatInterval{}, err
		}
	}
	cmp, err := proofarith.ReplayRatCompare(ctx, x, replayFixedOneHalf, budget)
	if err != nil {
		return RatInterval{}, err
	}
	if cmp > 0 {
		return RatInterval{}, decaderr.ErrUnsupported
	}
	xLo, err := ReplayFixedFloor(ctx, x, budget)
	if err != nil {
		return RatInterval{}, err
	}
	xHi, err := ReplayFixedCeil(ctx, x, budget)
	if err != nil {
		return RatInterval{}, err
	}
	x2Lo, err := replayFixedMul(ctx, xLo, xLo, false, budget)
	if err != nil {
		return RatInterval{}, err
	}
	x2Hi, err := replayFixedMul(ctx, xHi, xHi, true, budget)
	if err != nil {
		return RatInterval{}, err
	}
	powerLo, err := replayFixedIntOp(ctx, xLo, nil, 'c', 0, budget)
	if err != nil {
		return RatInterval{}, err
	}
	powerHi, err := replayFixedIntOp(ctx, xHi, nil, 'c', 0, budget)
	if err != nil {
		return RatInterval{}, err
	}
	var zero big.Int
	lo, err := replayFixedIntOp(ctx, &zero, nil, 'c', 0, budget)
	if err != nil {
		return RatInterval{}, err
	}
	hi, err := replayFixedIntOp(ctx, &zero, nil, 'c', 0, budget)
	if err != nil {
		return RatInterval{}, err
	}
	for n := range 64 {
		d := 2*n + 1
		tLo, e := replayFixedDiv(ctx, powerLo, d, false, budget)
		if e != nil {
			return RatInterval{}, e
		}
		tHi, e := replayFixedDiv(ctx, powerHi, d, true, budget)
		if e != nil {
			return RatInterval{}, e
		}
		if n%2 == 0 {
			lo, e = replayFixedIntOp(ctx, lo, tLo, '+', 0, budget)
			if e == nil {
				hi, e = replayFixedIntOp(ctx, hi, tHi, '+', 0, budget)
			}
		} else {
			lo, e = replayFixedIntOp(ctx, lo, tHi, '-', 0, budget)
			if e == nil {
				hi, e = replayFixedIntOp(ctx, hi, tLo, '-', 0, budget)
			}
		}
		if e != nil {
			return RatInterval{}, e
		}
		powerLo, e = replayFixedMul(ctx, powerLo, x2Lo, false, budget)
		if e != nil {
			return RatInterval{}, e
		}
		powerHi, e = replayFixedMul(ctx, powerHi, x2Hi, true, budget)
		if e != nil {
			return RatInterval{}, e
		}
	}
	remainder, err := replayFixedDiv(ctx, powerHi, 129, true, budget)
	if err != nil {
		return RatInterval{}, err
	}
	hi, err = replayFixedIntOp(ctx, hi, remainder, '+', 0, budget)
	if err != nil {
		return RatInterval{}, err
	}
	loRat, err := proofarith.ReplayReducedFraction(ctx, lo, TrigFixedOne, budget)
	if err != nil {
		return RatInterval{}, err
	}
	hiRat, err := proofarith.ReplayReducedFraction(ctx, hi, TrigFixedOne, budget)
	if err != nil {
		return RatInterval{}, err
	}
	iv := IntervalOwned(loRat, hiRat)
	if negative {
		return replayTurnNegInterval(ctx, iv, budget)
	}
	return iv, nil
}

var replayFixedOneHalf = big.NewRat(1, 2)
