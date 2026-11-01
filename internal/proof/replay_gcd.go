package proof

import (
	"context"
	"math/big"
	"math/bits"
)

// ReplayGCD computes a positive gcd using fixed arrays and charged passes.
func ReplayGCD(ctx context.Context, a, b *big.Int, m ReplayKeyBudget) (*big.Int, error) {
	if a == nil || b == nil || m == nil || a.Sign() <= 0 || b.Sign() <= 0 {
		return nil, errReplayBoundedGcdOperands
	}
	n := (max(a.BitLen(), b.BitLen())+bits.UintSize-1)/bits.UintSize + 1
	if n > 65 {
		return nil, errReplayBoundedGcdWidth
	}
	if err := m.ReserveRecords(ctx, 2); err != nil {
		return nil, err
	}
	if err := m.ReserveBytes(ctx, uint64(4*n*bits.UintSize/8+512)); err != nil {
		return nil, err
	}
	if err := m.SpendWork(ctx, uint64(2*n)); err != nil {
		return nil, err
	}
	storage := make([]big.Word, 2*n)
	u, v := storage[:n], storage[n:]
	copy(u, a.Bits())
	copy(v, b.Bits())
	common := min(keyTrailing(u), keyTrailing(v))
	if err := keyShift(ctx, u, keyTrailing(u), m); err != nil {
		return nil, err
	}
	for {
		if err := m.SpendWork(ctx, uint64(2*n)); err != nil {
			return nil, err
		}
		if keyZero(v) {
			break
		}
		if err := keyShift(ctx, v, keyTrailing(v), m); err != nil {
			return nil, err
		}
		if err := m.SpendWork(ctx, uint64(2*n)); err != nil {
			return nil, err
		}
		if keyCompare(u, v) > 0 {
			u, v = v, u
		}
		keySubtract(v, u)
	}
	if err := keyShiftLeft(ctx, u, common, m); err != nil {
		return nil, err
	}
	return new(big.Int).SetBits(u), nil
}

// ReplayDivideExact divides positive integers using fixed shifted-divisor
// scratch. It refuses if the division has a remainder.
func ReplayDivideExact(ctx context.Context, a, b *big.Int, m ReplayKeyBudget) (*big.Int, error) {
	if a == nil || b == nil || m == nil || a.Sign() < 0 || b.Sign() <= 0 {
		return nil, errReplayBoundedDivisionOperands
	}
	n := (max(a.BitLen(), b.BitLen())+bits.UintSize-1)/bits.UintSize + 1
	if n > 65 {
		return nil, errReplayBoundedDivisionWidth
	}
	if err := m.ReserveRecords(ctx, 2); err != nil {
		return nil, err
	}
	if err := m.ReserveBytes(ctx, uint64(6*n*bits.UintSize/8+512)); err != nil {
		return nil, err
	}
	if err := m.SpendWork(ctx, uint64(3*n)); err != nil {
		return nil, err
	}
	storage := make([]big.Word, 3*n)
	rem, divisor, q := storage[:n], storage[n:2*n], storage[2*n:]
	copy(rem, a.Bits())
	copy(divisor, b.Bits())
	shift := max(0, a.BitLen()-b.BitLen())
	if err := keyShiftLeft(ctx, divisor, uint(shift), m); err != nil {
		return nil, err
	}
	for bit := shift; bit >= 0; bit-- {
		if err := m.SpendWork(ctx, uint64(2*n)); err != nil {
			return nil, err
		}
		if keyCompare(rem, divisor) >= 0 {
			keySubtract(rem, divisor)
			q[bit/bits.UintSize] |= big.Word(1) << uint(bit%bits.UintSize)
		}
		if bit > 0 {
			if err := keyShift(ctx, divisor, 1, m); err != nil {
				return nil, err
			}
		}
	}
	if err := m.SpendWork(ctx, uint64(n)); err != nil {
		return nil, err
	}
	if !keyZero(rem) {
		return nil, errReplayBoundedDivisionHasRemainder
	}
	return new(big.Int).SetBits(q), nil
}
