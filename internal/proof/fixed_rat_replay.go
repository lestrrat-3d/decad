package proof

import (
	"context"
	"math/big"
	"math/bits"

	"github.com/lestrrat-3d/decad/internal/decaderr"
)

// ReplayBundleBudget reserves the three counters atomically.
type ReplayBundleBudget interface {
	ReserveBundle(context.Context, uint64, uint64, uint64) error
}

// ReplayFixedRatInt computes a bounded integer operation using one rational
// component without exposing that component's mutable storage.
func ReplayFixedRatInt(ctx context.Context, q *big.Rat, numerator bool, other *big.Int, ratFirst bool, op byte, shift uint, budget ReplayBundleBudget) (*big.Int, error) {
	if q == nil || budget == nil {
		return nil, decaderr.ErrUnsupported
	}
	part := q.Denom()
	if numerator {
		part = q.Num()
	}
	a, b := part, other
	if !ratFirst {
		a, b = other, part
	}
	return replayFixedRatInteger(ctx, a, b, op, shift, budget)
}

// ReplayFixedRatQuo computes a bounded integer quotient of q's components.
func ReplayFixedRatQuo(ctx context.Context, q *big.Rat, budget ReplayBundleBudget) (*big.Int, error) {
	if q == nil || budget == nil {
		return nil, decaderr.ErrUnsupported
	}
	return replayFixedRatInteger(ctx, q.Num(), q.Denom(), '/', 0, budget)
}

func replayFixedRatInteger(ctx context.Context, a, b *big.Int, op byte, shift uint, budget ReplayBundleBudget) (*big.Int, error) {
	if a == nil || op != 'c' && op != 's' && op != 'l' && op != 'a' && b == nil {
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
	if maxBits > 2048 || op == '*' && a.BitLen()+b.BitLen() > 2048 || (op == '/' || op == '%') && b.Sign() == 0 {
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

// ReplayCopyRat copies q through the existing bounded fraction kernel.
func ReplayCopyRat(ctx context.Context, q *big.Rat, budget ReplayKeyBudget) (*big.Rat, error) {
	if q == nil {
		return nil, decaderr.ErrUnsupported
	}
	return ReplayReducedFraction(ctx, q.Num(), q.Denom(), budget)
}
