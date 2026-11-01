package proof

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/r3"
)

var replayZero = big.NewInt(0)
var replayNegativeOne = big.NewInt(-1)

// ReplayOrientSignPrepared preserves the float filter and bounds exact scratch.
func ReplayOrientSignPrepared(ctx context.Context, a, b, c, d r3.Vec, xa, xd, n Xpt, budget ReplayKeyBudget) (int, error) {
	if budget == nil {
		return 0, errReplayMissingOrientationBudget
	}
	if err := budget.SpendWork(ctx, 256); err != nil {
		return 0, err
	}
	if sign, certain := OrientSignFloat(a, b, c, d); certain {
		return sign, nil
	}
	return ReplayXdotDiffSign(ctx, n, xd, xa, budget)
}

// ReplayOrientSignMixed computes the exact plane-side sign on the shared budget.
func ReplayOrientSignMixed(ctx context.Context, a, b, c r3.Vec, d Xpt, budget ReplayKeyBudget) (int, error) {
	if budget == nil {
		return 0, errReplayMissingOrientationBudget
	}
	if err := ctx.Err(); err != nil {
		return 0, err
	}
	reserve := func(blocks uint64) error {
		if err := budget.ReserveRecords(ctx, blocks); err != nil {
			return err
		}
		if blocks > ^uint64(0)/64 {
			return errReplayOrientationAllocationOverflow
		}
		return budget.ReserveBytes(ctx, blocks*64)
	}
	spend := func(n uint64) error { return budget.SpendWork(ctx, n) }
	normal, err := TriangleNormalReplay(a, b, c, reserve, spend)
	if err != nil {
		return 0, err
	}
	if err := spend(1); err != nil {
		return 0, err
	}
	anchor, err := XptOfBounded(a, reserve)
	if err != nil {
		return 0, err
	}
	return ReplayXdotDiffSign(ctx, normal, d, anchor, budget)
}

// ReplayXdotDiffSign decides n dot (p-q) using fixed-limb products.
func ReplayXdotDiffSign(ctx context.Context, n, p, q Xpt, budget ReplayKeyBudget) (int, error) {
	if budget == nil || n.X == nil || n.Y == nil || n.Z == nil || p.X == nil || p.Y == nil || p.Z == nil || q.X == nil || q.Y == nil || q.Z == nil || p.W == nil || q.W == nil || p.W.Sign() <= 0 || q.W.Sign() <= 0 {
		return 0, errReplayInvalidOrientationInputs
	}
	var sum *big.Int
	for i, pn := range [3]*big.Int{p.X, p.Y, p.Z} {
		qn := [3]*big.Int{q.X, q.Y, q.Z}[i]
		nn := [3]*big.Int{n.X, n.Y, n.Z}[i]
		d, err := ReplayProductDifference(ctx, pn, q.W, qn, p.W, budget)
		if err != nil {
			return 0, err
		}
		if sum == nil {
			sum, err = ReplayProductDifference(ctx, nn, d, replayZero, replayZero, budget)
		} else {
			sum, err = ReplayProductDifference(ctx, nn, d, replayNegativeOne, sum, budget)
		}
		if err != nil {
			return 0, err
		}
	}
	return sum.Sign(), nil
}
