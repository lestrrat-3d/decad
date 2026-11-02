package prismplacement

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/denotation"
	"github.com/lestrrat-3d/decad/internal/proof"
)

func replayLevelRange(ctx context.Context, held, delta float64, denoted LevelDenotation, shift *big.Rat, budget proof.ReplayKeyBudget) (LevelRange, bool, error) {
	lo, hi, ok, err := denotation.ReplayLevelRange(ctx, held, delta, denoted, shift, budget)
	return LevelRange{Lo: lo, Hi: hi}, ok, err
}

// ReplayBlindLevels proves one coincident cap and a strictly interior opposite
// tool level using the certified exact shift. top reports the coincident end.
func ReplayBlindLevels(ctx context.Context, target, tool Operand, shift *big.Rat, budget proof.ReplayKeyBudget) (bool, bool, error) {
	if budget == nil || shift == nil {
		return false, false, nil
	}
	zero, err := proof.ReplayFloatRat(ctx, 0, budget)
	if err != nil {
		return false, false, err
	}
	a0, ok, err := replayLevelRange(ctx, target.Z0, target.Z0Delta, target.Z0Denoted, zero, budget)
	if err != nil || !ok {
		return false, false, err
	}
	a1, ok, err := replayLevelRange(ctx, target.Z1, target.Z1Delta, target.Z1Denoted, zero, budget)
	if err != nil || !ok {
		return false, false, err
	}
	b0, ok, err := replayLevelRange(ctx, tool.Z0, tool.Z0Delta, tool.Z0Denoted, shift, budget)
	if err != nil || !ok {
		return false, false, err
	}
	b1, ok, err := replayLevelRange(ctx, tool.Z1, tool.Z1Delta, tool.Z1Denoted, shift, budget)
	if err != nil || !ok {
		return false, false, err
	}
	below := func(a, b LevelRange) (bool, error) {
		c, e := proof.ReplayRatCompare(ctx, a.Hi, b.Lo, budget)
		return c < 0, e
	}
	same := func(a, b LevelRange) (bool, error) {
		c, e := proof.ReplayRatCompare(ctx, a.Lo, a.Hi, budget)
		if e != nil || c != 0 {
			return false, e
		}
		c, e = proof.ReplayRatCompare(ctx, b.Lo, b.Hi, budget)
		if e != nil || c != 0 {
			return false, e
		}
		c, e = proof.ReplayRatCompare(ctx, a.Lo, b.Lo, budget)
		return c == 0, e
	}
	ordered, err := below(a0, a1)
	if err != nil || !ordered {
		return false, false, err
	}
	ordered, err = below(b0, b1)
	if err != nil || !ordered {
		return false, false, err
	}
	upper, err := same(a1, b1)
	if err != nil {
		return false, false, err
	}
	if upper {
		inside, err := below(a0, b0)
		if err != nil || !inside {
			return false, false, err
		}
		inside, err = below(b0, a1)
		return true, inside, err
	}
	lower, err := same(a0, b0)
	if err != nil || !lower {
		return false, false, err
	}
	inside, err := below(a0, b1)
	if err != nil || !inside {
		return false, false, err
	}
	inside, err = below(b1, a1)
	return false, inside, err
}

// ReplayDenotedLevel reads an immutable denotation under budget.
func ReplayDenotedLevel(ctx context.Context, held, delta float64, denoted LevelDenotation, budget proof.ReplayKeyBudget) (*big.Rat, bool, error) {
	return denotation.ReplayDenotedLevel(ctx, held, delta, denoted, budget)
}
