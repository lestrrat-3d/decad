package proof

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
)

var errRationalFloatBudget = fmt.Errorf("%w: missing exact error budget", decaderr.ErrUnsupported)

// ReplayRationalFloatError charges the exact difference and directed float
// rounding before returning the same upper error as RationalFloatError.
func ReplayRationalFloatError(ctx context.Context, exact *big.Rat, held float64,
	budget ReplayKeyBudget) (float64, error) {
	if budget == nil {
		return 0, errRationalFloatBudget
	}
	if err := ctx.Err(); err != nil {
		return 0, err
	}
	if exact == nil || math.IsNaN(held) || math.IsInf(held, 0) {
		return math.Inf(1), nil
	}
	h, err := ReplayFloatRat(ctx, held, budget)
	if err != nil {
		return 0, err
	}
	diff, err := ReplayRatArithmetic(ctx, exact, h, '-', budget)
	if err != nil {
		return 0, err
	}
	// A sign read and in-place sign flip need no new scratch. Charge both
	// before using the reduced difference to select its magnitude.
	if err := budget.SpendWork(ctx, 2); err != nil {
		return 0, err
	}
	if diff.Sign() == 0 {
		return 0, nil
	}
	if diff.Sign() < 0 {
		diff.Neg(diff)
	}
	out, err := ReplayFractionFloat64(ctx, diff.Num(), diff.Denom(), budget)
	if err != nil {
		return 0, err
	}
	if math.IsInf(out, 1) {
		return out, nil
	}
	if out == 0 {
		return math.SmallestNonzeroFloat64, nil
	}
	rounded, err := ReplayFloatRat(ctx, out, budget)
	if err != nil {
		return 0, err
	}
	comparison, err := ReplayRatCompare(ctx, rounded, diff, budget)
	if err != nil {
		return 0, err
	}
	// RationalFloatError raises every inexact nearest result once, even
	// when nearest already lies above the exact value.
	if comparison != 0 {
		return NextUp(out), nil
	}
	return out, nil
}
