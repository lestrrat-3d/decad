package denotation

import (
	"context"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/proof"
)

// ReplayLevelRange computes the bounded rational range for a recorded level.
func ReplayLevelRange(ctx context.Context, held, delta float64, denoted LevelDenotation, shift *big.Rat, budget proof.ReplayKeyBudget) (*big.Rat, *big.Rat, bool, error) {
	if err := budget.SpendWork(ctx, 1); err != nil {
		return nil, nil, false, err
	}
	if math.IsNaN(delta) || math.IsInf(delta, 0) || delta < 0 {
		return nil, nil, false, nil
	}
	var exact *big.Rat
	if denoted.exact != nil && denoted.held == held && denoted.delta == delta {
		exact = denoted.exact
	} else if delta == 0 {
		var err error
		exact, err = proof.ReplayFloatRat(ctx, held, budget)
		if err != nil {
			return nil, nil, false, err
		}
	}
	if exact != nil {
		shifted, err := proof.ReplayRatArithmetic(ctx, exact, shift, '+', budget)
		if err != nil {
			return nil, nil, false, err
		}
		return shifted, shifted, true, nil
	}
	h, err := proof.ReplayFloatRat(ctx, held, budget)
	if err != nil {
		return nil, nil, false, err
	}
	d, err := proof.ReplayFloatRat(ctx, delta, budget)
	if err != nil {
		return nil, nil, false, err
	}
	h, err = proof.ReplayRatArithmetic(ctx, h, shift, '+', budget)
	if err != nil {
		return nil, nil, false, err
	}
	lo, err := proof.ReplayRatArithmetic(ctx, h, d, '-', budget)
	if err != nil {
		return nil, nil, false, err
	}
	hi, err := proof.ReplayRatArithmetic(ctx, h, d, '+', budget)
	if err != nil {
		return nil, nil, false, err
	}
	return lo, hi, true, nil
}

// ReplayDenotedLevel reads an immutable denotation under budget. The returned
// rational must not be changed by the caller.
func ReplayDenotedLevel(ctx context.Context, held, delta float64, denoted LevelDenotation, budget proof.ReplayKeyBudget) (*big.Rat, bool, error) {
	if budget == nil {
		return nil, false, nil
	}
	if err := budget.SpendWork(ctx, 1); err != nil {
		return nil, false, err
	}
	if math.IsNaN(delta) || math.IsInf(delta, 0) || delta < 0 {
		return nil, false, nil
	}
	if denoted.exact != nil && denoted.held == held && denoted.delta == delta {
		return denoted.exact, true, nil
	}
	if delta != 0 {
		return nil, false, nil
	}
	q, err := proof.ReplayFloatRat(ctx, held, budget)
	return q, err == nil, err
}
