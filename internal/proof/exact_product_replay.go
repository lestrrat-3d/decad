package proof

import (
	"context"
	"math/big"
	"math/bits"
	"unsafe"
)

// ReplayProductDifference computes a*b-c*d using charged fixed limb buffers.
func ReplayProductDifference(ctx context.Context, a, b, c, d *big.Int, budget ReplayKeyBudget) (*big.Int, error) {
	if arena, ok := budget.(interface {
		ProductDifference(context.Context, *big.Int, *big.Int, *big.Int, *big.Int) (*big.Int, error)
	}); ok {
		return arena.ProductDifference(ctx, a, b, c, d)
	}
	if budget == nil || a == nil || b == nil || c == nil || d == nil {
		return nil, errReplayInvalidExactProduct
	}
	aw, bw, cw, dw := len(a.Bits()), len(b.Bits()), len(c.Bits()), len(d.Bits())
	maxInt := int(^uint(0) >> 1)
	if aw > maxInt-bw || cw > maxInt-dw {
		return nil, errReplayProductWidthOverflow
	}
	width := max(aw+bw, cw+dw)
	if width >= maxInt/5 {
		return nil, errReplayProductStorageOverflow
	}
	n := width + 1
	if n > maxInt/5 {
		return nil, errReplayProductStorageOverflow
	}
	bytes := uint64(3*n) * uint64(bits.UintSize/8)
	if err := budget.ReserveRecords(ctx, 2); err != nil {
		return nil, err
	}
	if bytes > (^uint64(0)-256)/2 {
		return nil, errReplayProductStorageOverflow
	}
	if err := budget.ReserveBytes(ctx, 2*bytes+256); err != nil {
		return nil, err
	}
	if aw != 0 && uint64(bw) > ^uint64(0)/uint64(aw) || cw != 0 && uint64(dw) > ^uint64(0)/uint64(cw) {
		return nil, errReplayProductWorkOverflow
	}
	w1, w2 := uint64(aw)*uint64(bw), uint64(cw)*uint64(dw)
	if w1 > ^uint64(0)-w2 || w1+w2 > ^uint64(0)-uint64(5*n) {
		return nil, errReplayProductWorkOverflow
	}
	if err := budget.SpendWork(ctx, w1+w2+uint64(5*n)); err != nil {
		return nil, err
	}
	storage := make([]big.Word, 3*n)
	x, y, out := storage[:n], storage[n:2*n], storage[2*n:]
	multiply := func(dst []big.Word, left, right []big.Word) {
		for i, l := range left {
			var carry uint
			for j, r := range right {
				hi, lo := bits.Mul(uint(l), uint(r))
				var c uint
				lo, c = bits.Add(lo, uint(dst[i+j]), 0)
				hi, _ = bits.Add(hi, 0, c)
				lo, c = bits.Add(lo, carry, 0)
				hi, _ = bits.Add(hi, 0, c)
				dst[i+j] = big.Word(lo)
				carry = hi
			}
			dst[i+len(right)] = big.Word(carry)
		}
	}
	multiply(x, a.Bits(), b.Bits())
	multiply(y, c.Bits(), d.Bits())
	sx, sy := a.Sign()*b.Sign(), -c.Sign()*d.Sign()
	negative := false
	switch {
	case sx == 0:
		copy(out, y)
		negative = sy < 0
	case sy == 0:
		copy(out, x)
		negative = sx < 0
	case sx == sy:
		var carry uint
		for i := range out {
			v, k := bits.Add(uint(x[i]), uint(y[i]), carry)
			out[i] = big.Word(v)
			carry = k
		}
		negative = sx < 0
	default:
		if keyCompare(x, y) >= 0 {
			copy(out, x)
			keySubtract(out, y)
			negative = sx < 0
		} else {
			copy(out, y)
			keySubtract(out, x)
			negative = sy < 0
		}
	}
	result := new(big.Int).SetBits(out)
	if negative {
		result.Neg(result)
	}
	return result, nil
}

// ReplayRatProductDifference computes q.Num()*b-c*q.Denom() with bounded scratch.
func ReplayRatProductDifference(ctx context.Context, q *big.Rat, b, c *big.Int, budget ReplayKeyBudget) (*big.Int, error) {
	if q == nil || b == nil || c == nil {
		return nil, errReplayInvalidExactProduct
	}
	return ReplayProductDifference(ctx, q.Num(), b, c, q.Denom(), budget)
}

// smallReplayProduct owns both the integer and its limbs in one allocation.
type smallReplayProduct struct {
	value big.Int
	words [4]big.Word
}

// ReplayProductArena owns serial scratch for exact product differences.
type ReplayProductArena struct {
	storage         []big.Word
	width, maxWidth int
	busy            bool
}

func (arena *ReplayProductArena) MaxWidth() int { return arena.maxWidth }

// ProductDifference computes a product with retained, independently owned scratch.
func (arena *ReplayProductArena) ProductDifference(ctx context.Context, a, b, c, d *big.Int, budget ReplayKeyBudget) (*big.Int, error) {
	if budget == nil || a == nil || b == nil || c == nil || d == nil {
		return nil, errReplayInvalidExactProduct
	}
	if err := budget.SpendWork(ctx, 1); err != nil {
		return nil, err
	}
	aw, bw, cw, dw := len(a.Bits()), len(b.Bits()), len(c.Bits()), len(d.Bits())
	maxInt := int(^uint(0) >> 1)
	if aw > maxInt-bw || cw > maxInt-dw {
		return nil, errReplayProductWidthOverflow
	}
	width := max(aw+bw, cw+dw)
	if width >= maxInt/8 {
		return nil, errReplayProductStorageOverflow
	}
	n := width + 1
	if n > maxInt/8 {
		return nil, errReplayProductStorageOverflow
	}
	if arena.busy {
		return nil, errReplayReentrantProductArena
	}
	arena.busy = true
	defer func() { arena.busy = false }()
	words := uint64(3 * n)
	wordBytes := uint64(unsafe.Sizeof(big.Word(0)))
	if words > (^uint64(0)-uint64(unsafe.Sizeof(big.Int{})))/wordBytes {
		return nil, errReplayProductStorageOverflow
	}
	if n > arena.width {
		if err := budget.ReserveRecords(ctx, 1); err != nil {
			return nil, err
		}
		if err := budget.ReserveBytes(ctx, words*wordBytes); err != nil {
			return nil, err
		}
	}
	smallBudget, small := budget.(interface {
		ReserveProductSmall(context.Context, uint64) error
	})
	small = small && n <= len(smallReplayProduct{}.words)
	resultRecords := uint64(2)
	if small {
		resultRecords = 1
	}
	if err := budget.ReserveRecords(ctx, resultRecords); err != nil {
		return nil, err
	}
	if small {
		if err := smallBudget.ReserveProductSmall(ctx, uint64(unsafe.Sizeof(smallReplayProduct{}))); err != nil {
			return nil, err
		}
	} else if err := budget.ReserveBytes(ctx, uint64(n)*wordBytes+uint64(unsafe.Sizeof(big.Int{}))); err != nil {
		return nil, err
	}
	if aw != 0 && uint64(bw) > ^uint64(0)/uint64(aw) || cw != 0 && uint64(dw) > ^uint64(0)/uint64(cw) {
		return nil, errReplayProductWorkOverflow
	}
	w1, w2 := uint64(aw)*uint64(bw), uint64(cw)*uint64(dw)
	if w1 > ^uint64(0)-w2 || w1+w2 > ^uint64(0)-uint64(7*n) {
		return nil, errReplayProductWorkOverflow
	}
	if err := budget.SpendWork(ctx, w1+w2+uint64(7*n)); err != nil {
		return nil, err
	}
	grow := n > arena.width
	if grow {
		arena.storage = make([]big.Word, 3*n)
		arena.width = n
	}
	arena.maxWidth = max(arena.maxWidth, n)
	w := arena.width
	x, y, out := arena.storage[:n], arena.storage[w:w+n], arena.storage[2*w:2*w+n]
	if !grow {
		clear(x)
		clear(y)
		clear(out)
	}
	multiply := func(dst []big.Word, left, right []big.Word) {
		for i, l := range left {
			var carry uint
			for j, r := range right {
				hi, lo := bits.Mul(uint(l), uint(r))
				var c uint
				lo, c = bits.Add(lo, uint(dst[i+j]), 0)
				hi, _ = bits.Add(hi, 0, c)
				lo, c = bits.Add(lo, carry, 0)
				hi, _ = bits.Add(hi, 0, c)
				dst[i+j] = big.Word(lo)
				carry = hi
			}
			dst[i+len(right)] = big.Word(carry)
		}
	}
	multiply(x, a.Bits(), b.Bits())
	multiply(y, c.Bits(), d.Bits())
	sx, sy := a.Sign()*b.Sign(), -c.Sign()*d.Sign()
	negative := false
	switch {
	case sx == 0:
		copy(out, y)
		negative = sy < 0
	case sy == 0:
		copy(out, x)
		negative = sx < 0
	case sx == sy:
		var carry uint
		for i := range out {
			v, k := bits.Add(uint(x[i]), uint(y[i]), carry)
			out[i] = big.Word(v)
			carry = k
		}
		negative = sx < 0
	default:
		if keyCompare(x, y) >= 0 {
			copy(out, x)
			keySubtract(out, y)
			negative = sx < 0
		} else {
			copy(out, y)
			keySubtract(out, x)
			negative = sy < 0
		}
	}
	var result *big.Int
	if small {
		owned := &smallReplayProduct{}
		copy(owned.words[:n], out)
		result = owned.value.SetBits(owned.words[:n])
	} else {
		resultWords := make([]big.Word, n)
		copy(resultWords, out)
		result = new(big.Int).SetBits(resultWords)
	}
	if negative {
		result.Neg(result)
	}
	return result, nil
}

// ReplayPlaneCrossing computes the owned crossing with charged limb arithmetic.
func ReplayPlaneCrossing(ctx context.Context, a, b Xpt, an, bn *big.Int, budget ReplayKeyBudget) (Xpt, error) {
	var out Xpt
	for i, pair := range [4][2]*big.Int{{a.X, b.X}, {a.Y, b.Y}, {a.Z, b.Z}, {a.W, b.W}} {
		v, err := ReplayProductDifference(ctx, an, pair[1], bn, pair[0], budget)
		if err != nil {
			return Xpt{}, err
		}
		switch i {
		case 0:
			out.X = v
		case 1:
			out.Y = v
		case 2:
			out.Z = v
		case 3:
			out.W = v
		}
	}
	if out.W.Sign() == 0 {
		return Xpt{}, errReplayZeroCrossingWeight
	}
	if out.W.Sign() < 0 {
		out.X.Neg(out.X)
		out.Y.Neg(out.Y)
		out.Z.Neg(out.Z)
		out.W.Neg(out.W)
	}
	if err := replayStripTwos(ctx, out, budget); err != nil {
		return Xpt{}, err
	}
	return out, nil
}

func replayStripTwos(ctx context.Context, out Xpt, budget ReplayKeyBudget) error {
	values := [4]*big.Int{out.X, out.Y, out.Z, out.W}
	var words uint64
	for _, v := range values {
		if uint64(len(v.Bits())) > ^uint64(0)-words {
			return errReplayCrossingWorkOverflow
		}
		words += uint64(len(v.Bits()))
	}
	if err := budget.SpendWork(ctx, words+4); err != nil {
		return err
	}
	shift := out.W.TrailingZeroBits()
	for _, v := range values[:3] {
		if v.Sign() != 0 {
			shift = min(shift, v.TrailingZeroBits())
		}
	}
	if shift != 0 {
		for _, v := range values {
			negative := v.Sign() < 0
			if err := keyShift(ctx, v.Bits(), shift, budget); err != nil {
				return err
			}
			v.SetBits(v.Bits())
			if negative {
				v.Neg(v)
			}
		}
	}
	return nil
}

// ReplayXlerp computes Xlerp with fixed-limb products and charged storage.
func ReplayXlerp(ctx context.Context, a, b Xpt, num, den *big.Int, budget ReplayKeyBudget) (Xpt, error) {
	if budget == nil || num == nil || den == nil || den.Sign() <= 0 || a.W == nil || b.W == nil || a.W.Sign() <= 0 || b.W.Sign() <= 0 {
		return Xpt{}, errReplayInvalidReplayInterpolation
	}
	difference, err := ReplayProductDifference(ctx, den, replayOne, num, replayOne, budget)
	if err != nil {
		return Xpt{}, err
	}
	var out Xpt
	for i, pair := range [3][2]*big.Int{{a.X, b.X}, {a.Y, b.Y}, {a.Z, b.Z}} {
		left, err := ReplayProductDifference(ctx, pair[0], difference, replayZero, replayZero, budget)
		if err != nil {
			return Xpt{}, err
		}
		left, err = ReplayProductDifference(ctx, left, b.W, replayZero, replayZero, budget)
		if err != nil {
			return Xpt{}, err
		}
		right, err := ReplayProductDifference(ctx, pair[1], num, replayZero, replayZero, budget)
		if err != nil {
			return Xpt{}, err
		}
		right, err = ReplayProductDifference(ctx, right, a.W, replayZero, replayZero, budget)
		if err != nil {
			return Xpt{}, err
		}
		value, err := ReplayProductDifference(ctx, left, replayOne, replayNegativeOne, right, budget)
		if err != nil {
			return Xpt{}, err
		}
		switch i {
		case 0:
			out.X = value
		case 1:
			out.Y = value
		case 2:
			out.Z = value
		}
	}
	weight, err := ReplayProductDifference(ctx, a.W, b.W, replayZero, replayZero, budget)
	if err != nil {
		return Xpt{}, err
	}
	out.W, err = ReplayProductDifference(ctx, weight, den, replayZero, replayZero, budget)
	if err != nil {
		return Xpt{}, err
	}
	if err := replayStripTwos(ctx, out, budget); err != nil {
		return Xpt{}, err
	}
	return out, nil
}

// ReplayRatXlerp interpolates exact points using a bounded rational parameter.
func ReplayRatXlerp(ctx context.Context, a, b Xpt, t *big.Rat, budget ReplayKeyBudget) (Xpt, error) {
	if t == nil {
		return Xpt{}, errReplayInvalidReplayInterpolation
	}
	return ReplayXlerp(ctx, a, b, t.Num(), t.Denom(), budget)
}
