package proof

import (
	"context"
	"encoding/binary"
	"math/big"
)

// ReplayReducedFraction builds the same reduced ratio as RatFrac without GCD scratch.
func ReplayReducedFraction(ctx context.Context, num, den *big.Int, budget ReplayKeyBudget) (*big.Rat, error) {
	if budget == nil || num == nil || den == nil || den.Sign() <= 0 {
		return nil, errReplayInvalidReplayFraction
	}
	key, err := ReplayExactKey(ctx, Xpt{X: num, Y: replayZero, Z: replayZero, W: den}, budget)
	if err != nil {
		return nil, err
	}
	var starts, lengths [4]int
	var signs [4]byte
	at := 0
	for i := range 4 {
		if at > len(key)-9 {
			return nil, errReplayInvalidExactKey
		}
		signs[i] = key[at]
		size := binary.LittleEndian.Uint64([]byte(key[at+1 : at+9]))
		at += 9
		if size > uint64(len(key)-at) {
			return nil, errReplayInvalidExactKeyLength
		}
		starts[i], lengths[i] = at, int(size)
		at += int(size)
	}
	bytes := uint64(lengths[0]) + uint64(lengths[3])
	if bytes > (^uint64(0)-1024)/4 {
		return nil, errReplayReplayFractionStorageOverflow
	}
	if err := budget.ReserveRecords(ctx, 3); err != nil {
		return nil, err
	}
	if err := budget.ReserveBytes(ctx, 4*bytes+1024); err != nil {
		return nil, err
	}
	if err := budget.SpendWork(ctx, bytes+4); err != nil {
		return nil, err
	}
	out := new(big.Rat).SetInt64(0)
	out.Num().SetBytes([]byte(key[starts[0] : starts[0]+lengths[0]]))
	if signs[0] == 0 {
		out.Num().Neg(out.Num())
	}
	out.Denom().SetBytes([]byte(key[starts[3] : starts[3]+lengths[3]]))
	return out, nil
}

var replayOne = big.NewInt(1)

// ReplayRatCompare compares two rationals using bounded products.
func ReplayRatCompare(ctx context.Context, a, b *big.Rat, budget ReplayKeyBudget) (int, error) {
	if a == nil || b == nil {
		return 0, errReplayInvalidReplayRational
	}
	d, err := ReplayProductDifference(ctx, a.Num(), b.Denom(), b.Num(), a.Denom(), budget)
	if err != nil {
		return 0, err
	}
	return d.Sign(), nil
}

// ReplayRatArithmetic performs a reduced sum, difference, product or quotient.
func ReplayRatArithmetic(ctx context.Context, a, b *big.Rat, op byte, budget ReplayKeyBudget) (*big.Rat, error) {
	if a == nil || b == nil {
		return nil, errReplayInvalidReplayRational
	}
	var num, den *big.Int
	var err error
	switch op {
	case '+':
		first, e := ReplayProductDifference(ctx, a.Num(), b.Denom(), replayZero, replayZero, budget)
		if e != nil {
			return nil, e
		}
		num, err = ReplayProductDifference(ctx, b.Num(), a.Denom(), replayNegativeOne, first, budget)
	case '-':
		num, err = ReplayProductDifference(ctx, a.Num(), b.Denom(), b.Num(), a.Denom(), budget)
	case '*':
		num, err = ReplayProductDifference(ctx, a.Num(), b.Num(), replayZero, replayZero, budget)
	case '/':
		if b.Sign() == 0 {
			return nil, errReplayReplayRationalDivisionByZero
		}
		num, err = ReplayProductDifference(ctx, a.Num(), b.Denom(), replayZero, replayZero, budget)
	default:
		return nil, errReplayInvalidReplayRationalOperation
	}
	if err != nil {
		return nil, err
	}
	if op == '/' {
		den, err = ReplayProductDifference(ctx, a.Denom(), b.Num(), replayZero, replayZero, budget)
	} else {
		den, err = ReplayProductDifference(ctx, a.Denom(), b.Denom(), replayZero, replayZero, budget)
	}
	if err != nil {
		return nil, err
	}
	if den.Sign() < 0 {
		num.Neg(num)
		den.Neg(den)
	}
	return ReplayReducedFraction(ctx, num, den, budget)
}
