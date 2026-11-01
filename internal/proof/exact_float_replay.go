package proof

import (
	"context"
	"math/big"
	"math/bits"

	"github.com/lestrrat-3d/r3"
)

// ReplayXptVec rounds a point using charged fixed-capacity integer division.
func ReplayXptVec(ctx context.Context, p Xpt, budget ReplayKeyBudget) (r3.Vec, error) {
	if budget == nil || p.X == nil || p.Y == nil || p.Z == nil || p.W == nil || p.W.Sign() <= 0 {
		return r3.Vec{}, errReplayInvalidExactPoint
	}
	var out [3]float64
	for i, num := range [3]*big.Int{p.X, p.Y, p.Z} {
		f, err := replayQuotientFloat64(ctx, num, p.W, budget)
		if err != nil {
			return r3.Vec{}, err
		}
		out[i] = f
	}
	return r3.Vec{X: out[0], Y: out[1], Z: out[2]}, nil
}

func replayQuotientFloat64(ctx context.Context, num, den *big.Int, budget ReplayKeyBudget) (float64, error) {
	if err := budget.SpendWork(ctx, 1); err != nil {
		return 0, err
	}
	if num.Sign() == 0 {
		return 0, nil
	}
	exp := num.BitLen() - den.BitLen()
	if isPositivePowerOfTwo(den) {
		words := uint64(len(num.Bits())) + uint64(len(den.Bits()))
		if err := budget.SpendWork(ctx, words+1); err != nil {
			return 0, err
		}
		mantissa, rem := powerOfTwoQuotient(num)
		f, _ := roundQuotient(num.Sign() < 0, exp, mantissa, rem)
		return f, nil
	}
	shift := 54 - exp
	aBits, bBits := num.BitLen(), den.BitLen()
	if shift > 0 {
		if shift > int(^uint(0)>>1)-aBits {
			return 0, errReplayExactFloatWidthOverflow
		}
		aBits += shift
	} else if shift < 0 {
		if -shift > int(^uint(0)>>1)-bBits {
			return 0, errReplayExactFloatWidthOverflow
		}
		bBits -= shift
	}
	if bBits > int(^uint(0)>>1)-54 {
		return 0, errReplayExactFloatWidthOverflow
	}
	maxBits := max(aBits, bBits+54)
	if maxBits > int(^uint(0)>>1)-bits.UintSize {
		return 0, errReplayExactFloatWidthOverflow
	}
	n := 1 + (maxBits+bits.UintSize-1)/bits.UintSize
	if n > int(^uint(0)>>1)/2 {
		return 0, errReplayExactFloatWorkspaceOverflow
	}
	bytes := uint64(2*n) * uint64(bits.UintSize/8)
	if err := budget.ReserveRecords(ctx, 1); err != nil {
		return 0, err
	}
	if bytes > (^uint64(0)-128)/2 {
		return 0, errReplayExactFloatStorageOverflow
	}
	if err := budget.ReserveBytes(ctx, 2*bytes+128); err != nil {
		return 0, err
	}
	if err := budget.SpendWork(ctx, uint64(2*n)); err != nil {
		return 0, err
	}
	storage := make([]big.Word, 2*n)
	a, b := storage[:n], storage[n:]
	copy(a, num.Bits())
	copy(b, den.Bits())
	if shift > 0 {
		if err := keyShiftLeft(ctx, a, uint(shift), budget); err != nil {
			return 0, err
		}
	} else if shift < 0 {
		if err := keyShiftLeft(ctx, b, uint(-shift), budget); err != nil {
			return 0, err
		}
	}
	// The normalized quotient is below 2^55. Compare against the divisor
	// shifted by each of its 55 possible bits, retaining a as the remainder.
	if err := keyShiftLeft(ctx, b, 54, budget); err != nil {
		return 0, err
	}
	var quotient uint64
	for bit := 54; bit >= 0; bit-- {
		if err := budget.SpendWork(ctx, uint64(2*n)); err != nil {
			return 0, err
		}
		if keyCompare(a, b) >= 0 {
			keySubtract(a, b)
			quotient |= uint64(1) << uint(bit)
		}
		if bit > 0 {
			if err := keyShift(ctx, b, 1, budget); err != nil {
				return 0, err
			}
		}
	}
	if err := budget.SpendWork(ctx, uint64(n)); err != nil {
		return 0, err
	}
	f, _ := roundQuotient(num.Sign() < 0, exp, quotient, !keyZero(a))
	return f, nil
}

// ReplayFractionFloat64 rounds an exact fraction with bounded workspace.
func ReplayFractionFloat64(ctx context.Context, num, den *big.Int, budget ReplayKeyBudget) (float64, error) {
	if budget == nil || num == nil || den == nil || den.Sign() <= 0 {
		return 0, errReplayInvalidExactFraction
	}
	return replayQuotientFloat64(ctx, num, den, budget)
}
