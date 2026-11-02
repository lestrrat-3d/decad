package proofbound

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
)

func replayRatFloorGrid(ctx context.Context, x *big.Rat, shift uint,
	budget *ReplayBudget) (*big.Rat, error) {
	scale, err := replayFixedIntOp(ctx, replayFixedOne, nil, 'l', shift, budget)
	if err != nil {
		return nil, err
	}
	num, err := proofarith.ReplayFixedRatInt(ctx, x, true, scale, true, '*', 0, budget)
	if err != nil {
		return nil, err
	}
	quotient, err := proofarith.ReplayFixedRatInt(ctx, x, false, num, false, '/', 0, budget)
	if err != nil {
		return nil, err
	}
	remainder, err := proofarith.ReplayFixedRatInt(ctx, x, false, num, false, '%', 0, budget)
	if err != nil {
		return nil, err
	}
	if remainder.Sign() < 0 {
		quotient, err = replayFixedIntOp(ctx, quotient, replayFixedOne, '-', 0, budget)
		if err != nil {
			return nil, err
		}
	}
	return proofarith.ReplayReducedFraction(ctx, quotient, scale, budget)
}

func replayTrigWidening(ctx context.Context, q *big.Rat,
	budget *ReplayBudget) (*big.Rat, error) {
	if q.Sign() < 0 {
		return nil, decaderr.ErrUnsupported
	}
	if q.Sign() == 0 {
		return proofarith.ReplayCopyRat(ctx, replayTurnZero, budget)
	}
	shift := TrigFixedBits - 1 + q.Denom().BitLen() - q.Num().BitLen()
	if shift > 2048 || shift < -2048 {
		return nil, decaderr.ErrUnsupported
	}
	n, err := proofarith.ReplayFixedRatInt(ctx, q, true, nil, true, 'c', 0, budget)
	if err != nil {
		return nil, err
	}
	d, err := proofarith.ReplayFixedRatInt(ctx, q, false, nil, true, 'c', 0, budget)
	if err != nil {
		return nil, err
	}
	if shift >= 0 {
		n, err = replayFixedIntOp(ctx, n, nil, 'l', uint(shift), budget)
	} else {
		d, err = replayFixedIntOp(ctx, d, nil, 'l', uint(-shift), budget)
	}
	if err != nil {
		return nil, err
	}
	m, err := replayFixedIntOp(ctx, n, d, '/', 0, budget)
	if err != nil {
		return nil, err
	}
	remainder, err := replayFixedIntOp(ctx, n, d, '%', 0, budget)
	if err != nil {
		return nil, err
	}
	if remainder.Sign() > 0 {
		m, err = replayFixedIntOp(ctx, m, replayFixedOne, '+', 0, budget)
		if err != nil {
			return nil, err
		}
	}
	if shift < 0 {
		m, err = replayFixedIntOp(ctx, m, nil, 'l', uint(-shift), budget)
		if err != nil {
			return nil, err
		}
		return proofarith.ReplayReducedFraction(ctx, m, replayFixedOne, budget)
	}
	scale, err := replayFixedIntOp(ctx, replayFixedOne, nil, 'l', uint(shift), budget)
	if err != nil {
		return nil, err
	}
	return proofarith.ReplayReducedFraction(ctx, m, scale, budget)
}

func replayIntervalWiden(ctx context.Context, iv RatInterval, width *big.Rat,
	budget *ReplayBudget) (RatInterval, error) {
	lo, err := proofarith.ReplayRatArithmetic(ctx, iv.Lo, width, '-', budget)
	if err != nil {
		return RatInterval{}, err
	}
	hi, err := proofarith.ReplayRatArithmetic(ctx, iv.Hi, width, '+', budget)
	if err != nil {
		return RatInterval{}, err
	}
	return IntervalOwned(lo, hi), nil
}

// ReplayRadSinCosInterval evaluates the ordinary dyadic-turn enclosure for
// one exact radian value without the process-wide memo.
func ReplayRadSinCosInterval(ctx context.Context, x *big.Rat,
	budget *ReplayBudget) (RatInterval, RatInterval, error) {
	if x == nil || budget == nil {
		return RatInterval{}, RatInterval{}, decaderr.ErrUnsupported
	}
	if x.Sign() == 0 {
		zero := RatInterval{Lo: replayTurnZero, Hi: replayTurnZero}
		one := RatInterval{Lo: replayAtanOne, Hi: replayAtanOne}
		sin, err := replayIntervalCopy(ctx, zero, budget)
		if err != nil {
			return RatInterval{}, RatInterval{}, err
		}
		cos, err := replayIntervalCopy(ctx, one, budget)
		return sin, cos, err
	}
	loDen, hiDen := TwoPiIv.Hi, TwoPiIv.Lo
	if x.Sign() < 0 {
		loDen, hiDen = TwoPiIv.Lo, TwoPiIv.Hi
	}
	turnLo, err := proofarith.ReplayRatArithmetic(ctx, x, loDen, '/', budget)
	if err != nil {
		return RatInterval{}, RatInterval{}, err
	}
	turnHi, err := proofarith.ReplayRatArithmetic(ctx, x, hiDen, '/', budget)
	if err != nil {
		return RatInterval{}, RatInterval{}, err
	}
	base, err := replayRatFloorGrid(ctx, turnLo, TurnGridShift, budget)
	if err != nil {
		return RatInterval{}, RatInterval{}, err
	}
	gap, err := proofarith.ReplayRatArithmetic(ctx, turnHi, base, '-', budget)
	if err != nil {
		return RatInterval{}, RatInterval{}, err
	}
	if gap.Sign() < 0 {
		return RatInterval{}, RatInterval{}, decaderr.ErrUnsupported
	}
	sin, cos, err := ReplayTurnSinCosInterval(ctx, base, budget)
	if err != nil {
		return RatInterval{}, RatInterval{}, err
	}
	slop, err := proofarith.ReplayRatArithmetic(ctx, TwoPiIv.Hi, gap, '*', budget)
	if err != nil {
		return RatInterval{}, RatInterval{}, err
	}
	slop, err = replayTrigWidening(ctx, slop, budget)
	if err != nil {
		return RatInterval{}, RatInterval{}, err
	}
	sin, err = replayIntervalWiden(ctx, sin, slop, budget)
	if err != nil {
		return RatInterval{}, RatInterval{}, err
	}
	cos, err = replayIntervalWiden(ctx, cos, slop, budget)
	return sin, cos, err
}

// ReplayRadSinCosSpan widens a charged lower-endpoint reading by the
// interval's exact width rounded upward on the ordinary relative grid.
func ReplayRadSinCosSpan(ctx context.Context, x RatInterval,
	budget *ReplayBudget) (RatInterval, RatInterval, bool, error) {
	if x.Lo == nil || x.Hi == nil || budget == nil {
		return RatInterval{}, RatInterval{}, false, decaderr.ErrUnsupported
	}
	width, err := proofarith.ReplayRatArithmetic(ctx, x.Hi, x.Lo, '-', budget)
	if err != nil {
		return RatInterval{}, RatInterval{}, false, err
	}
	if width.Sign() < 0 {
		return RatInterval{}, RatInterval{}, false, nil
	}
	width, err = replayTrigWidening(ctx, width, budget)
	if err != nil {
		return RatInterval{}, RatInterval{}, false, err
	}
	sin, cos, err := ReplayRadSinCosInterval(ctx, x.Lo, budget)
	if err != nil {
		return RatInterval{}, RatInterval{}, false, err
	}
	sin, err = replayIntervalWiden(ctx, sin, width, budget)
	if err != nil {
		return RatInterval{}, RatInterval{}, false, err
	}
	cos, err = replayIntervalWiden(ctx, cos, width, budget)
	return sin, cos, err == nil, err
}
