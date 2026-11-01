package proof

import (
	"context"
	"encoding/binary"
	"fmt"
	"math/big"
	"math/bits"
	"slices"

	"github.com/lestrrat-3d/decad/internal/decaderr"
)

// ReplayKeyBudget reserves all workspace and charges each arithmetic pass.
type ReplayKeyBudget interface {
	ReserveRecords(context.Context, uint64) error
	ReserveBytes(context.Context, uint64) error
	SpendWork(context.Context, uint64) error
}

// These refusals are constructed once so rejected replay does not allocate.
var (
	errExactKeyReplay1 = fmt.Errorf("%w: invalid bounded exact point", decaderr.ErrUnsupported)
	errExactKeyReplay2 = fmt.Errorf("%w: exact key size overflow", decaderr.ErrUnsupported)
	errExactKeyReplay3 = fmt.Errorf("%w: exact key width overflow", decaderr.ErrUnsupported)
	errExactKeyReplay4 = fmt.Errorf("%w: exact key workspace overflow", decaderr.ErrUnsupported)
	errExactKeyReplay5 = fmt.Errorf("%w: exact key work overflow", decaderr.ErrUnsupported)
	errExactKeyReplay6 = fmt.Errorf("%w: exact key allocation envelope overflow", decaderr.ErrUnsupported)
	errExactKeyReplay7 = fmt.Errorf("%w: exact key workspace exceeds address space", decaderr.ErrUnsupported)
	errExactKeyReplay8 = fmt.Errorf("%w: exact key division has remainder", decaderr.ErrUnsupported)
)

// ReplayExactKey encodes the same canonical point as Key with bounded scratch.
func ReplayExactKey(ctx context.Context, p Xpt, budget ReplayKeyBudget) (string, error) {
	if budget == nil || p.X == nil || p.Y == nil || p.Z == nil || p.W == nil || p.W.Sign() <= 0 {
		return "", errExactKeyReplay1
	}
	values := [4]*big.Int{p.X, p.Y, p.Z, p.W}
	var maxBits, rawBytes uint64
	rawBytes = 36
	for _, v := range values {
		n := uint64(v.BitLen())
		maxBits = max(maxBits, n)
		if n > ^uint64(0)-7 || rawBytes > ^uint64(0)-(n+7)/8 {
			return "", errExactKeyReplay2
		}
		rawBytes += (n + 7) / 8
	}
	if maxBits > ^uint64(0)-uint64(bits.UintSize-1) {
		return "", errExactKeyReplay3
	}
	words := (maxBits + uint64(bits.UintSize-1)) / uint64(bits.UintSize)
	// Eight equal-size arrays hold gcd operands, division scratch and outputs.
	if words == 0 || words > (^uint64(0)/uint64(bits.UintSize/8)-8)/8 || rawBytes > ^uint64(0)/2 {
		return "", errExactKeyReplay4
	}
	if words > (^uint64(0)-rawBytes)/8 {
		return "", errExactKeyReplay5
	}
	workspaceBytes := (8*words + 8) * uint64(bits.UintSize/8)
	if err := budget.ReserveRecords(ctx, 3); err != nil {
		return "", err
	}
	if workspaceBytes > (^uint64(0)-128)/2 || rawBytes > (^uint64(0)-256)/4 {
		return "", errExactKeyReplay6
	}
	if err := budget.ReserveBytes(ctx, 2*workspaceBytes+128); err != nil {
		return "", err
	}
	// Byte slice and independently allocated immutable result string.
	if err := budget.ReserveBytes(ctx, rawBytes*4+256); err != nil {
		return "", err
	}
	if err := budget.SpendWork(ctx, 8*words+rawBytes); err != nil {
		return "", err
	}
	if words > uint64(^uint(0)>>1)/8-1 || rawBytes > uint64(^uint(0)>>1) {
		return "", errExactKeyReplay7
	}
	n := int(words) + 1
	storage := make([]big.Word, 8*n)
	g, u, v, rem := storage[:n], storage[n:2*n], storage[2*n:3*n], storage[3*n:4*n]
	var q [4][]big.Word
	for i := range q {
		q[i] = storage[(4+i)*n : (5+i)*n]
	}
	if p.W.BitLen()-1 == int(p.W.TrailingZeroBits()) {
		shift := p.W.TrailingZeroBits()
		for _, value := range values[:3] {
			if value.Sign() != 0 {
				shift = min(shift, value.TrailingZeroBits())
			}
		}
		for i, value := range values {
			copy(q[i], value.Bits())
			if err := keyShift(ctx, q[i], shift, budget); err != nil {
				return "", err
			}
		}
	} else {
		copy(g, p.W.Bits())
		for _, value := range values[:3] {
			if value.Sign() == 0 {
				continue
			}
			clear(u)
			clear(v)
			copy(u, g)
			copy(v, value.Bits())
			// Binary GCD keeps the common power of two and decreases one operand.
			if err := budget.SpendWork(ctx, uint64(2*n)); err != nil {
				return "", err
			}
			common := min(keyTrailing(u), keyTrailing(v))
			if err := keyShift(ctx, u, keyTrailing(u), budget); err != nil {
				return "", err
			}
			for {
				if err := budget.SpendWork(ctx, uint64(2*n)); err != nil {
					return "", err
				}
				if keyZero(v) {
					break
				}
				if err := keyShift(ctx, v, keyTrailing(v), budget); err != nil {
					return "", err
				}
				if err := budget.SpendWork(ctx, uint64(2*n)); err != nil {
					return "", err
				}
				if keyCompare(u, v) > 0 {
					u, v = v, u
				}
				keySubtract(v, u)
			}
			clear(g)
			copy(g, u)
			if err := keyShiftLeft(ctx, g, common, budget); err != nil {
				return "", err
			}
		}
		for i, value := range values {
			clear(rem)
			// Restoring long division emits one bit per source bit and never
			// stores more than N+1 words in the remainder.
			for bit := value.BitLen() - 1; bit >= 0; bit-- {
				if err := budget.SpendWork(ctx, uint64(3*n)); err != nil {
					return "", err
				}
				keyLeftOne(rem)
				rem[0] |= (value.Bits()[bit/bits.UintSize] >> uint(bit%bits.UintSize)) & 1
				if keyCompare(rem, g) >= 0 {
					keySubtract(rem, g)
					q[i][bit/bits.UintSize] |= big.Word(1) << uint(bit%bits.UintSize)
				}
			}
			if err := budget.SpendWork(ctx, uint64(n)); err != nil {
				return "", err
			}
			if !keyZero(rem) {
				return "", errExactKeyReplay8
			}
		}
	}
	if err := budget.SpendWork(ctx, uint64(4*n)); err != nil {
		return "", err
	}
	out := make([]byte, 0, int(rawBytes))
	for i, value := range values {
		mag := q[i]
		length := keyBitLen(mag)
		byteLen := (length + 7) / 8
		sign := value.Sign()
		out = append(out, byte(sign+1))
		var header [8]byte
		binary.LittleEndian.PutUint64(header[:], uint64(byteLen))
		out = append(out, header[:]...)
		for j := byteLen - 1; j >= 0; j-- {
			out = append(out, byte(mag[j/(bits.UintSize/8)]>>uint((j%(bits.UintSize/8))*8)))
		}
	}
	return string(out), nil
}

func keyZero(a []big.Word) bool {
	for _, x := range a {
		if x != 0 {
			return false
		}
	}
	return true
}
func keyBitLen(a []big.Word) int {
	for i, word := range slices.Backward(a) {
		if word != 0 {
			return i*bits.UintSize + bits.Len(uint(word))
		}
	}
	return 0
}
func keyTrailing(a []big.Word) uint {
	for i, x := range a {
		if x != 0 {
			return uint(i*bits.UintSize + bits.TrailingZeros(uint(x)))
		}
	}
	return 0
}
func keyCompare(a, b []big.Word) int {
	for i, word := range slices.Backward(a) {
		if word < b[i] {
			return -1
		}
		if word > b[i] {
			return 1
		}
	}
	return 0
}
func keySubtract(a, b []big.Word) {
	var borrow uint
	for i := range a {
		x, c := bits.Sub(uint(a[i]), uint(b[i]), borrow)
		a[i] = big.Word(x)
		borrow = c
	}
}
func keyLeftOne(a []big.Word) {
	var carry uint
	for i, x := range a {
		next := uint(x) >> (bits.UintSize - 1)
		a[i] = big.Word(uint(x)<<1 | carry)
		carry = next
	}
}
func keyShift(ctx context.Context, a []big.Word, shift uint, budget ReplayKeyBudget) error {
	if shift == 0 {
		return nil
	}
	if err := budget.SpendWork(ctx, uint64(len(a))); err != nil {
		return err
	}
	whole, part := int(shift/uint(bits.UintSize)), shift%uint(bits.UintSize)
	for i := range a {
		j := i + whole
		var x uint
		if j < len(a) {
			x = uint(a[j]) >> part
			if part > 0 && j+1 < len(a) {
				x |= uint(a[j+1]) << (uint(bits.UintSize) - part)
			}
		}
		a[i] = big.Word(x)
	}
	return nil
}
func keyShiftLeft(ctx context.Context, a []big.Word, shift uint, budget ReplayKeyBudget) error {
	if shift == 0 {
		return nil
	}
	if err := budget.SpendWork(ctx, uint64(len(a))); err != nil {
		return err
	}
	whole, part := int(shift/uint(bits.UintSize)), shift%uint(bits.UintSize)
	for i := len(a) - 1; i >= 0; i-- {
		j := i - whole
		var x uint
		if j >= 0 {
			x = uint(a[j]) << part
			if part > 0 && j > 0 {
				x |= uint(a[j-1]) >> (uint(bits.UintSize) - part)
			}
		}
		a[i] = big.Word(x)
	}
	return nil
}
