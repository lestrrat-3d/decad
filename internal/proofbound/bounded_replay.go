package proofbound

import (
	"context"
	"math"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proof"
)

// BoundedAddReplay performs the same interval addition as BoundedAdd, with
// every exact rounding-error operation charged before it runs.
func BoundedAddReplay(ctx context.Context, a, b BoundedScalar, budget *ReplayBudget) (BoundedScalar, error) {
	if budget == nil {
		return BoundedScalar{}, decaderr.ErrUnsupported
	}
	if err := budget.SpendWork(ctx, 1); err != nil {
		return BoundedScalar{}, err
	}
	value := a.Value + b.Value
	if IsNonFinite(a.Value) || IsNonFinite(b.Value) || IsNonFinite(value) {
		return MeasuredScalar(value, math.Inf(1)), nil
	}
	ra, err := proof.ReplayFloatRat(ctx, a.Value, budget)
	if err != nil {
		return BoundedScalar{}, err
	}
	rb, err := proof.ReplayFloatRat(ctx, b.Value, budget)
	if err != nil {
		return BoundedScalar{}, err
	}
	exact, err := proof.ReplayRatArithmetic(ctx, ra, rb, '+', budget)
	if err != nil {
		return BoundedScalar{}, err
	}
	round, err := proof.ReplayRationalFloatError(ctx, exact, value, budget)
	if err != nil {
		return BoundedScalar{}, err
	}
	return MeasuredScalar(value, AbsSumUpper(a.Bound, b.Bound, round)), nil
}

// BoundedSubReplay charges the exact error of the held subtraction.
func BoundedSubReplay(ctx context.Context, a, b BoundedScalar, budget *ReplayBudget) (BoundedScalar, error) {
	return BoundedAddReplay(ctx, a, BoundedNeg(b), budget)
}

// BoundedMulReplay charges the exact error of the held product.
func BoundedMulReplay(ctx context.Context, a, b BoundedScalar, budget *ReplayBudget) (BoundedScalar, error) {
	if budget == nil {
		return BoundedScalar{}, decaderr.ErrUnsupported
	}
	if err := budget.SpendWork(ctx, 1); err != nil {
		return BoundedScalar{}, err
	}
	value := a.Value * b.Value
	if IsNonFinite(a.Value) || IsNonFinite(b.Value) || IsNonFinite(value) {
		return MeasuredScalar(value, math.Inf(1)), nil
	}
	ra, err := proof.ReplayFloatRat(ctx, a.Value, budget)
	if err != nil {
		return BoundedScalar{}, err
	}
	rb, err := proof.ReplayFloatRat(ctx, b.Value, budget)
	if err != nil {
		return BoundedScalar{}, err
	}
	exact, err := proof.ReplayRatArithmetic(ctx, ra, rb, '*', budget)
	if err != nil {
		return BoundedScalar{}, err
	}
	round, err := proof.ReplayRationalFloatError(ctx, exact, value, budget)
	if err != nil {
		return BoundedScalar{}, err
	}
	return MeasuredScalar(value, AbsSumUpper(
		ProductUpper(math.Abs(a.Value), b.Bound),
		ProductUpper(math.Abs(b.Value), a.Bound),
		ProductUpper(a.Bound, b.Bound), round)), nil
}

// BoundedQuotientReplay charges the quotient's exact float-rounding error.
func BoundedQuotientReplay(ctx context.Context, num, numBound, den, denBound float64,
	budget *ReplayBudget) (BoundedScalar, error) {
	if budget == nil {
		return BoundedScalar{}, decaderr.ErrUnsupported
	}
	if err := budget.SpendWork(ctx, 1); err != nil {
		return BoundedScalar{}, err
	}
	value := num / den
	clearance := math.Nextafter(math.Abs(den)-denBound, math.Inf(-1))
	if clearance <= 0 || IsNonFinite(num) || IsNonFinite(den) || IsNonFinite(value) {
		return MeasuredScalar(value, math.Inf(1)), nil
	}
	ra, err := proof.ReplayFloatRat(ctx, num, budget)
	if err != nil {
		return BoundedScalar{}, err
	}
	rb, err := proof.ReplayFloatRat(ctx, den, budget)
	if err != nil {
		return BoundedScalar{}, err
	}
	exact, err := proof.ReplayRatArithmetic(ctx, ra, rb, '/', budget)
	if err != nil {
		return BoundedScalar{}, err
	}
	centralRound, err := proof.ReplayRationalFloatError(ctx, exact, value, budget)
	if err != nil {
		return BoundedScalar{}, err
	}
	centralUpper := AbsSumUpper(value, centralRound)
	numerator := AbsSumUpper(numBound, ProductUpper(centralUpper, denBound))
	bound := UpRound(numerator / clearance)
	return MeasuredScalar(value, AbsSumUpper(bound, centralRound)), nil
}

func BoundedDivReplay(ctx context.Context, a, b BoundedScalar,
	budget *ReplayBudget) (BoundedScalar, error) {
	return BoundedQuotientReplay(ctx, a.Value, a.Bound, b.Value, b.Bound, budget)
}
