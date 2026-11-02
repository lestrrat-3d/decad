package proof

import (
	"context"
	"math/big"
	"math/bits"
	"unsafe"
)

// ReplayRatPartToWords copies one Rat component into caller-owned fixed storage.
// It never returns or retains the Rat's numerator or denominator storage.
func ReplayRatPartToWords(ctx context.Context, q *big.Rat, numerator bool, dst []uint64, budget ReplayKeyBudget) error {
	if q == nil || budget == nil || bits.UintSize != 64 || len(dst) == 0 || len(dst) > 4096/bits.UintSize {
		return errReplayInvalidExactFraction
	}
	part := q.Denom()
	if numerator {
		part = q.Num()
	}
	if part.BitLen() > len(dst)*bits.UintSize {
		return errReplayInvalidExactFraction
	}
	if err := budget.SpendWork(ctx, uint64(len(dst))); err != nil {
		return err
	}
	clear(dst)
	for i, word := range part.Bits() {
		dst[i] = uint64(word)
	}
	return nil
}

// ReplayDyadicRatFromWords makes a reduced Rat from a bounded signed dyadic
// magnitude. It copies the caller's words before normalizing the owned copy.
func ReplayDyadicRatFromWords(ctx context.Context, magnitude []uint64, exponent int, negative bool, budget ReplayKeyBudget) (*big.Rat, error) {
	if budget == nil || bits.UintSize != 64 || exponent < -4095 || exponent > 4095 || len(magnitude) > 4096/bits.UintSize {
		return nil, errReplayBoundedDyadicWidthOverflow
	}
	bitLen := 0
	if len(magnitude) != 0 {
		bitLen = (len(magnitude)-1)*bits.UintSize + bits.Len(uint(magnitude[len(magnitude)-1]))
	}
	if bitLen+max(exponent, 0) > 4096 || 1+max(-exponent, 0) > 4096 {
		return nil, errReplayBoundedDyadicWidthOverflow
	}
	if err := budget.ReserveRecords(ctx, 1); err != nil {
		return nil, err
	}
	if err := budget.ReserveBytes(ctx, uint64(unsafe.Sizeof(big.Int{}))); err != nil {
		return nil, err
	}
	if err := budget.ReserveBytes(ctx, uint64(len(magnitude))*uint64(unsafe.Sizeof(big.Word(0)))); err != nil {
		return nil, err
	}
	if err := budget.SpendWork(ctx, 3*uint64(len(magnitude))+1); err != nil {
		return nil, err
	}
	words := make([]big.Word, len(magnitude))
	for i, word := range magnitude {
		words[i] = big.Word(word)
	}
	value := new(big.Int).SetBits(words)
	if negative {
		value.Neg(value)
	}
	return ReplayDyadicRat(ctx, fromBig(value, exponent), budget)
}
