package proofbound

import (
	"context"
	"math/big"
	"math/bits"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
)

// replayFixedIntOp admits at most 2,048-bit operands. At that width Go 1.27.3
// math/big uses basic multiplication and division (both switch to recursive
// algorithms at 40 words); the quadratic work charge and 512 bytes per word
// cover their limb loops, output and temporary limb slices before arithmetic.
func replayFixedIntOp(ctx context.Context, a, b *big.Int, op byte, shift uint,
	budget *ReplayBudget) (*big.Int, error) {
	if budget == nil || a == nil || op != 'c' && op != 's' && op != 'l' && op != 'a' && b == nil {
		return nil, decaderr.ErrUnsupported
	}
	maxBits := a.BitLen()
	if b != nil {
		maxBits = max(maxBits, b.BitLen())
	}
	if op == 'l' {
		if shift > 2048 || a.BitLen() > 2048-int(shift) {
			return nil, decaderr.ErrUnsupported
		}
		maxBits = a.BitLen() + int(shift)
	}
	if maxBits > 2048 || op == '*' && a.BitLen()+b.BitLen() > 2048 ||
		(op == '/' || op == '%') && b.Sign() == 0 {
		return nil, decaderr.ErrUnsupported
	}
	words := uint64((maxBits+bits.UintSize-1)/bits.UintSize + 1)
	if err := budget.ReserveBundle(ctx, 16*words*words+16, 1, 4096+512*words); err != nil {
		return nil, err
	}
	out := new(big.Int)
	switch op {
	case 'c':
		out.Set(a)
	case 's':
		out.Rsh(a, shift)
	case 'l':
		out.Lsh(a, shift)
	case '+':
		out.Add(a, b)
	case '-':
		out.Sub(a, b)
	case '*':
		out.Mul(a, b)
	case '/':
		out.Quo(a, b)
	case '%':
		out.Rem(a, b)
	case 'a':
		out.Abs(a)
	default:
		return nil, decaderr.ErrUnsupported
	}
	return out, nil
}

var replayFixedOne = big.NewInt(1)
var replayFixedTwo = big.NewRat(2, 1)
var replayFixedEighth = big.NewRat(1, 8)

// ReplayFixedFloor charges the fixed-grid product, quotient, remainder and
// possible outward correction before each exact integer operation.
func ReplayFixedFloor(ctx context.Context, q *big.Rat, budget *ReplayBudget) (*big.Int, error) {
	return replayFixedRound(ctx, q, false, budget)
}

// ReplayFixedCeil is the outward-rounding mirror of ReplayFixedFloor.
func ReplayFixedCeil(ctx context.Context, q *big.Rat, budget *ReplayBudget) (*big.Int, error) {
	return replayFixedRound(ctx, q, true, budget)
}

func replayFixedRound(ctx context.Context, q *big.Rat, ceil bool, budget *ReplayBudget) (*big.Int, error) {
	if q == nil || budget == nil {
		return nil, decaderr.ErrUnsupported
	}
	num, err := proofarith.ReplayFixedRatInt(ctx, q, true, TrigFixedOne, true, '*', 0, budget)
	if err != nil {
		return nil, err
	}
	quo, err := proofarith.ReplayFixedRatInt(ctx, q, false, num, false, '/', 0, budget)
	if err != nil {
		return nil, err
	}
	rem, err := proofarith.ReplayFixedRatInt(ctx, q, false, num, false, '%', 0, budget)
	if err != nil {
		return nil, err
	}
	if ceil && rem.Sign() > 0 {
		return replayFixedIntOp(ctx, quo, replayFixedOne, '+', 0, budget)
	}
	if !ceil && rem.Sign() < 0 {
		return replayFixedIntOp(ctx, quo, replayFixedOne, '-', 0, budget)
	}
	return quo, nil
}

// ReplayTurnFirstOctantSinCos computes the ordinary first-octant enclosure
// with charged rational and fixed-grid arithmetic on one shared meter.
func ReplayTurnFirstOctantSinCos(ctx context.Context, u *big.Rat,
	budget *ReplayBudget) (RatInterval, RatInterval, error) {
	if u == nil || budget == nil || u.Sign() < 0 {
		return RatInterval{}, RatInterval{}, decaderr.ErrUnsupported
	}
	cmp, err := proofarith.ReplayRatCompare(ctx, u, replayFixedEighth, budget)
	if err != nil {
		return RatInterval{}, RatInterval{}, err
	}
	if cmp > 0 {
		return RatInterval{}, RatInterval{}, decaderr.ErrUnsupported
	}
	twoU, err := proofarith.ReplayRatArithmetic(ctx, replayFixedTwo, u, '*', budget)
	if err != nil {
		return RatInterval{}, RatInterval{}, err
	}
	loAngle, err := proofarith.ReplayRatArithmetic(ctx, PiLower, twoU, '*', budget)
	if err != nil {
		return RatInterval{}, RatInterval{}, err
	}
	hiAngle, err := proofarith.ReplayRatArithmetic(ctx, PiUpper, twoU, '*', budget)
	if err != nil {
		return RatInterval{}, RatInterval{}, err
	}
	lo, err := ReplayFixedFloor(ctx, loAngle, budget)
	if err != nil {
		return RatInterval{}, RatInterval{}, err
	}
	hi, err := ReplayFixedCeil(ctx, hiAngle, budget)
	if err != nil {
		return RatInterval{}, RatInterval{}, err
	}
	sLo, cLo, eLo, err := ReplayTrigFixedSeries(ctx, lo, budget)
	if err != nil {
		return RatInterval{}, RatInterval{}, err
	}
	sHi, cHi, eHi, err := ReplayTrigFixedSeries(ctx, hi, budget)
	if err != nil {
		return RatInterval{}, RatInterval{}, err
	}
	if err := budget.SpendWork(ctx, 1); err != nil {
		return RatInterval{}, RatInterval{}, err
	}
	e := eLo
	if eHi.Cmp(e) > 0 {
		e = eHi
	}
	sinLo, err := replayFixedIntOp(ctx, sLo, e, '-', 0, budget)
	if err != nil {
		return RatInterval{}, RatInterval{}, err
	}
	sinHi, err := replayFixedIntOp(ctx, sHi, e, '+', 0, budget)
	if err != nil {
		return RatInterval{}, RatInterval{}, err
	}
	cosLo, err := replayFixedIntOp(ctx, cHi, e, '-', 0, budget)
	if err != nil {
		return RatInterval{}, RatInterval{}, err
	}
	cosHi, err := replayFixedIntOp(ctx, cLo, e, '+', 0, budget)
	if err != nil {
		return RatInterval{}, RatInterval{}, err
	}
	sinLoRat, err := proofarith.ReplayReducedFraction(ctx, sinLo, TrigFixedOne, budget)
	if err != nil {
		return RatInterval{}, RatInterval{}, err
	}
	sinHiRat, err := proofarith.ReplayReducedFraction(ctx, sinHi, TrigFixedOne, budget)
	if err != nil {
		return RatInterval{}, RatInterval{}, err
	}
	cosLoRat, err := proofarith.ReplayReducedFraction(ctx, cosLo, TrigFixedOne, budget)
	if err != nil {
		return RatInterval{}, RatInterval{}, err
	}
	cosHiRat, err := proofarith.ReplayReducedFraction(ctx, cosHi, TrigFixedOne, budget)
	if err != nil {
		return RatInterval{}, RatInterval{}, err
	}
	return IntervalOwned(sinLoRat, sinHiRat), IntervalOwned(cosLoRat, cosHiRat), nil
}

// ReplayTrigFixedSeries evaluates the ordinary 24-term fixed-point series
// through the charged integer helper above.
func ReplayTrigFixedSeries(ctx context.Context, x *big.Int,
	budget *ReplayBudget) (sin, cos, errUnits *big.Int, err error) {
	if budget == nil || x == nil || x.Sign() < 0 || x.BitLen() > TrigFixedBits+1 {
		return nil, nil, nil, decaderr.ErrUnsupported
	}
	if err := budget.SpendWork(ctx, uint64((x.BitLen()+bits.UintSize-1)/bits.UintSize+1)); err != nil {
		return nil, nil, nil, err
	}
	if x.Cmp(TrigFixedOne) > 0 {
		return nil, nil, nil, decaderr.ErrUnsupported
	}
	x2, err := replayFixedIntOp(ctx, x, x, '*', 0, budget)
	if err != nil {
		return nil, nil, nil, err
	}
	x2, err = replayFixedIntOp(ctx, x2, nil, 's', TrigFixedBits, budget)
	if err != nil {
		return nil, nil, nil, err
	}
	series := func(first *big.Int, k0 int64) (*big.Int, *big.Int, error) {
		var zero big.Int
		sum, e := replayFixedIntOp(ctx, &zero, nil, 'c', 0, budget)
		if e != nil {
			return nil, nil, e
		}
		term, e := replayFixedIntOp(ctx, first, nil, 'c', 0, budget)
		if e != nil {
			return nil, nil, e
		}
		k := k0
		for n := range TrigSeriesTerms {
			op := byte('+')
			if n%2 != 0 {
				op = '-'
			}
			sum, e = replayFixedIntOp(ctx, sum, term, op, 0, budget)
			if e != nil {
				return nil, nil, e
			}
			term, e = replayFixedIntOp(ctx, term, x2, '*', 0, budget)
			if e != nil {
				return nil, nil, e
			}
			term, e = replayFixedIntOp(ctx, term, nil, 's', TrigFixedBits, budget)
			if e != nil {
				return nil, nil, e
			}
			if e := budget.ReserveBundle(ctx, 1, 1, 128); e != nil {
				return nil, nil, e
			}
			divisor := big.NewInt(k * (k + 1))
			term, e = replayFixedIntOp(ctx, term, divisor, '/', 0, budget)
			if e != nil {
				return nil, nil, e
			}
			k += 2
		}
		return sum, term, nil
	}
	sin, sinRem, err := series(x, 2)
	if err != nil {
		return nil, nil, nil, err
	}
	cos, cosRem, err := series(TrigFixedOne, 1)
	if err != nil {
		return nil, nil, nil, err
	}
	rem := sinRem
	if err := budget.SpendWork(ctx, uint64((max(cosRem.BitLen(), rem.BitLen())+bits.UintSize-1)/bits.UintSize+1)); err != nil {
		return nil, nil, nil, err
	}
	if cosRem.CmpAbs(rem) > 0 {
		rem = cosRem
	}
	rem, err = replayFixedIntOp(ctx, rem, nil, 'a', 0, budget)
	if err != nil {
		return nil, nil, nil, err
	}
	if err := budget.ReserveBundle(ctx, 1, 1, 128); err != nil {
		return nil, nil, nil, err
	}
	margin := big.NewInt(3 * TrigSeriesTerms * (TrigSeriesTerms + 1) / 2)
	errUnits, err = replayFixedIntOp(ctx, rem, margin, '+', 0, budget)
	if err != nil {
		return nil, nil, nil, err
	}
	return sin, cos, errUnits, nil
}
