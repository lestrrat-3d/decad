package proofbound

import (
	"context"
	"math"
	"math/big"
	"math/bits"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
)

func replayRatSquareCmp(ctx context.Context, f float64, q *big.Rat,
	budget *ReplayBudget) (int, bool, error) {
	if IsNonFinite(f) {
		return 0, false, nil
	}
	if err := budget.ReserveBundle(ctx, 8, 1, 128); err != nil {
		return 0, false, err
	}
	frac, exp := math.Frexp(math.Abs(f))
	m := big.NewInt(int64(frac * (1 << 53)))
	lhs, err := replayFixedIntOp(ctx, m, m, '*', 0, budget)
	if err != nil {
		return 0, false, err
	}
	lhs, err = replayFixedIntOp(ctx, lhs, q.Denom(), '*', 0, budget)
	if err != nil {
		return 0, false, err
	}
	e2 := 2 * (exp - 53)
	rhs := q.Num()
	if e2 >= 0 {
		lhs, err = replayFixedIntOp(ctx, lhs, nil, 'l', uint(e2), budget)
	} else {
		rhs, err = replayFixedIntOp(ctx, rhs, nil, 'l', uint(-e2), budget)
	}
	if err != nil {
		return 0, false, err
	}
	work := uint64((max(lhs.BitLen(), rhs.BitLen())+bits.UintSize-1)/bits.UintSize + 1)
	if err := budget.SpendWork(ctx, work); err != nil {
		return 0, false, err
	}
	return lhs.Cmp(rhs), true, nil
}

// ReplayRatSqrtDown follows the ordinary directed float walk when its
// allocation-free leading-limb seed suffices. Other seeds refuse before the
// ordinary big.Float fallback.
func ReplayRatSqrtDown(ctx context.Context, q *big.Rat,
	budget *ReplayBudget) (float64, error) {
	if q == nil || budget == nil {
		return 0, decaderr.ErrUnsupported
	}
	if q.Sign() <= 0 {
		if err := budget.SpendWork(ctx, 1); err != nil {
			return 0, err
		}
		return 0, nil
	}
	if err := budget.SpendWork(ctx, 12); err != nil {
		return 0, err
	}
	f, ok := proofarith.RatSqrtSeed(q)
	if !ok {
		return 0, decaderr.ErrUnsupported
	}
	if IsNonFinite(f) {
		f = math.MaxFloat64
	}
	for range SqrtAdjustLimit {
		cmp, finite, err := replayRatSquareCmp(ctx, f, q, budget)
		if err != nil {
			return 0, err
		}
		if finite && cmp <= 0 {
			return f, nil
		}
		if err := budget.SpendWork(ctx, 1); err != nil {
			return 0, err
		}
		f = math.Nextafter(f, 0)
	}
	return 0, nil
}

// ReplayRatSqrtUp is ReplayRatSqrtDown's upward directed walk.
func ReplayRatSqrtUp(ctx context.Context, q *big.Rat,
	budget *ReplayBudget) (float64, error) {
	if q == nil || budget == nil {
		return 0, decaderr.ErrUnsupported
	}
	if q.Sign() <= 0 {
		if err := budget.SpendWork(ctx, 1); err != nil {
			return 0, err
		}
		return 0, nil
	}
	if err := budget.SpendWork(ctx, 12); err != nil {
		return 0, err
	}
	f, ok := proofarith.RatSqrtSeed(q)
	if !ok {
		return 0, decaderr.ErrUnsupported
	}
	if IsNonFinite(f) {
		f = math.MaxFloat64
	}
	for range SqrtAdjustLimit {
		cmp, finite, err := replayRatSquareCmp(ctx, f, q, budget)
		if err != nil {
			return 0, err
		}
		if !finite || cmp >= 0 {
			return f, nil
		}
		if err := budget.SpendWork(ctx, 1); err != nil {
			return 0, err
		}
		f = proofarith.NextUp(f)
	}
	return math.Inf(1), nil
}
