package proofbound

import (
	"context"
	"fmt"
	"math/big"
	"math/bits"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proof"
)

// replayTrigInteger is one unsigned integer in the private 4096-bit arena.
const replayTrigWords = 64
const replayTrigSlots = 32

type replayTrigInteger [replayTrigWords]uint64

//nolint:containedctx // The context is confined to this synchronous replay.
type replayTrigArena struct {
	slots  [replayTrigSlots]replayTrigInteger
	budget *ReplayBudget
	ctx    context.Context
}

func newReplayTrigArena(ctx context.Context, budget *ReplayBudget) (*replayTrigArena, error) {
	if budget == nil || bits.UintSize != 64 {
		return nil, fmt.Errorf("%w: unsupported bounded trig arena", decaderr.ErrUnsupported)
	}
	if err := budget.ReserveRecords(ctx, replayTrigSlots); err != nil {
		return nil, err
	}
	if err := budget.ReserveArray(ctx, 1, uint64(unsafe.Sizeof(replayTrigArena{}))); err != nil {
		return nil, err
	}
	if err := budget.SpendWork(ctx, replayTrigSlots*replayTrigWords); err != nil {
		return nil, err
	}
	return &replayTrigArena{budget: budget, ctx: ctx}, nil
}
func (a *replayTrigArena) charge(n uint64) error { return a.budget.SpendWork(a.ctx, n) }
func (a *replayTrigArena) load(slot int, x *big.Int) error {
	if x == nil || x.BitLen() > 64*replayTrigWords {
		return fmt.Errorf("%w: bounded trig integer width", decaderr.ErrUnsupported)
	}
	if err := a.charge(replayTrigWords); err != nil {
		return err
	}
	v := &a.slots[slot]
	*v = replayTrigInteger{}
	for i, w := range x.Bits() {
		v[i] = uint64(w)
	}
	return nil
}
func replayTrigCmp(x, y *replayTrigInteger) int {
	for i := replayTrigWords - 1; i >= 0; i-- {
		if x[i] < y[i] {
			return -1
		}
		if x[i] > y[i] {
			return 1
		}
	}
	return 0
}
func (a *replayTrigArena) add(out, x, y int) error {
	if err := a.charge(replayTrigWords); err != nil {
		return err
	}
	var carry uint64
	for i := range replayTrigWords {
		a.slots[out][i], carry = bits.Add64(a.slots[x][i], a.slots[y][i], carry)
	}
	if carry != 0 {
		return fmt.Errorf("%w: bounded trig addition overflow", decaderr.ErrUnsupported)
	}
	return nil
}
func (a *replayTrigArena) sub(out, x, y int) error {
	if err := a.charge(replayTrigWords); err != nil {
		return err
	}
	var borrow uint64
	for i := range replayTrigWords {
		a.slots[out][i], borrow = bits.Sub64(a.slots[x][i], a.slots[y][i], borrow)
	}
	if borrow != 0 {
		return fmt.Errorf("%w: bounded trig subtraction underflow", decaderr.ErrUnsupported)
	}
	return nil
}
func (a *replayTrigArena) mul(out, x, y int) error {
	if out == x || out == y {
		return fmt.Errorf("%w: bounded trig product aliases input", decaderr.ErrUnsupported)
	}
	if err := a.charge(replayTrigWords); err != nil {
		return err
	}
	nonzero := uint64(0)
	for _, word := range a.slots[x] {
		if word != 0 {
			nonzero++
		}
	}
	if err := a.charge(replayTrigWords + nonzero*replayTrigWords); err != nil {
		return err
	}
	z := &a.slots[out]
	*z = replayTrigInteger{}
	for i, xi := range a.slots[x] {
		if xi == 0 {
			continue
		}
		var carry uint64
		for j, yj := range a.slots[y] {
			if i+j >= replayTrigWords {
				if yj != 0 || carry != 0 {
					return fmt.Errorf("%w: bounded trig product overflow", decaderr.ErrUnsupported)
				}
				continue
			}
			hi, lo := bits.Mul64(xi, yj)
			lo, c := bits.Add64(lo, z[i+j], 0)
			hi, c1 := bits.Add64(hi, 0, c)
			lo, c = bits.Add64(lo, carry, 0)
			hi, c2 := bits.Add64(hi, 0, c)
			if c1|c2 != 0 {
				return fmt.Errorf("%w: bounded trig product carry overflow", decaderr.ErrUnsupported)
			}
			z[i+j] = lo
			carry = hi
		}
		if carry != 0 {
			return fmt.Errorf("%w: bounded trig product overflow", decaderr.ErrUnsupported)
		}
	}
	return nil
}

func replayTrigBitLen(x *replayTrigInteger) int {
	for i := replayTrigWords - 1; i >= 0; i-- {
		if x[i] != 0 {
			return 64*i + bits.Len64(x[i])
		}
	}
	return 0
}

// div computes quotient and remainder without allocating division scratch.
func (a *replayTrigArena) div(q, r, n, d int) error {
	if q == r || q == n || q == d || r == n || r == d {
		return fmt.Errorf("%w: bounded trig division aliases input", decaderr.ErrUnsupported)
	}
	if err := a.charge(2 * replayTrigWords); err != nil {
		return err
	}
	nbits, dbits := replayTrigBitLen(&a.slots[n]), replayTrigBitLen(&a.slots[d])
	if dbits == 0 {
		return fmt.Errorf("%w: bounded trig division by zero", decaderr.ErrUnsupported)
	}
	if err := a.charge(2 * replayTrigWords); err != nil {
		return err
	}
	a.slots[q], a.slots[r] = replayTrigInteger{}, replayTrigInteger{}
	words := min(replayTrigWords, (max(nbits, dbits)+63)/64+1)
	for bit := nbits - 1; bit >= 0; bit-- {
		if err := a.charge(3 * uint64(words)); err != nil {
			return err
		}
		var carry uint64
		for i := range words {
			next := a.slots[r][i] >> 63
			a.slots[r][i] = a.slots[r][i]<<1 | carry
			carry = next
		}
		if carry != 0 {
			return fmt.Errorf("%w: bounded trig remainder overflow", decaderr.ErrUnsupported)
		}
		a.slots[r][0] |= (a.slots[n][bit/64] >> uint(bit%64)) & 1
		cmp := 0
		for i := words - 1; i >= 0; i-- {
			if a.slots[r][i] < a.slots[d][i] {
				cmp = -1
				break
			}
			if a.slots[r][i] > a.slots[d][i] {
				cmp = 1
				break
			}
		}
		if cmp >= 0 {
			var borrow uint64
			for i := range words {
				a.slots[r][i], borrow = bits.Sub64(a.slots[r][i], a.slots[d][i], borrow)
			}
			a.slots[q][bit/64] |= uint64(1) << uint(bit%64)
		}
	}
	return nil
}

func (a *replayTrigArena) left(slot int, shift uint) error {
	x := &a.slots[slot]
	if n := replayTrigBitLen(x); n > 0 && (shift >= 4096 || uint(n) > 4096-shift) {
		return fmt.Errorf("%w: bounded trig shift overflow", decaderr.ErrUnsupported)
	}
	if err := a.charge(replayTrigWords); err != nil {
		return err
	}
	whole, part := int(shift/64), shift%64
	for i := replayTrigWords - 1; i >= 0; i-- {
		j := i - whole
		var v uint64
		if j >= 0 {
			v = x[j] << part
			if part != 0 && j > 0 {
				v |= x[j-1] >> (64 - part)
			}
		}
		x[i] = v
	}
	return nil
}
func (a *replayTrigArena) right(slot int, shift uint) error {
	if err := a.charge(replayTrigWords); err != nil {
		return err
	}
	x := &a.slots[slot]
	whole, part := int(shift/64), shift%64
	for i := range replayTrigWords {
		j := i + whole
		var v uint64
		if j < replayTrigWords {
			v = x[j] >> part
			if part != 0 && j+1 < replayTrigWords {
				v |= x[j+1] << (64 - part)
			}
		}
		x[i] = v
	}
	return nil
}

func (a *replayTrigArena) setWord(slot int, value uint64) error {
	if err := a.charge(replayTrigWords); err != nil {
		return err
	}
	a.slots[slot] = replayTrigInteger{}
	a.slots[slot][0] = value
	return nil
}
func (a *replayTrigArena) copySlot(dst, src int) error {
	if err := a.charge(replayTrigWords); err != nil {
		return err
	}
	a.slots[dst] = a.slots[src]
	return nil
}
func (a *replayTrigArena) divWord(slot int, divisor uint64) error {
	if divisor == 0 {
		return fmt.Errorf("%w: bounded trig division by zero", decaderr.ErrUnsupported)
	}
	if err := a.charge(replayTrigWords); err != nil {
		return err
	}
	var rem uint64
	for i := replayTrigWords - 1; i >= 0; i-- {
		a.slots[slot][i], rem = bits.Div64(rem, a.slots[slot][i], divisor)
	}
	return nil
}

// fixedSeries evaluates the two first-octant alternating sums on the
// 2^-200 grid. Slots 20 through 25 are its private scratch.
func (a *replayTrigArena) fixedSeries(input, sin, cos, margin int) error {
	if input >= 20 || sin >= 20 || cos >= 20 || margin >= 20 || sin == cos || sin == margin || cos == margin {
		return fmt.Errorf("%w: invalid bounded trig series slots", decaderr.ErrUnsupported)
	}
	const square, term, tmp, sinRem, cosRem, c = 20, 21, 22, 23, 24, 25
	if err := a.mul(square, input, input); err != nil {
		return err
	}
	if err := a.right(square, TrigFixedBits); err != nil {
		return err
	}
	series := func(first, out, rem int, k uint64) error {
		if err := a.setWord(out, 0); err != nil {
			return err
		}
		if err := a.copySlot(term, first); err != nil {
			return err
		}
		for i := range TrigSeriesTerms {
			var err error
			if i%2 == 0 {
				err = a.add(out, out, term)
			} else {
				err = a.sub(out, out, term)
			}
			if err != nil {
				return err
			}
			if err = a.mul(tmp, term, square); err != nil {
				return err
			}
			if err = a.right(tmp, TrigFixedBits); err != nil {
				return err
			}
			if err = a.divWord(tmp, k*(k+1)); err != nil {
				return err
			}
			if err = a.copySlot(term, tmp); err != nil {
				return err
			}
			k += 2
		}
		return a.copySlot(rem, term)
	}
	if err := series(input, sin, sinRem, 2); err != nil {
		return err
	}
	if err := a.setWord(c, 1); err != nil {
		return err
	}
	if err := a.left(c, TrigFixedBits); err != nil {
		return err
	}
	if err := series(c, cos, cosRem, 1); err != nil {
		return err
	}
	rem := sinRem
	if replayTrigCmp(&a.slots[cosRem], &a.slots[sinRem]) > 0 {
		rem = cosRem
	}
	if err := a.setWord(c, 3*TrigSeriesTerms*(TrigSeriesTerms+1)/2); err != nil {
		return err
	}
	return a.add(margin, rem, c)
}

// replayTrigSigned stores a sign separately from one arena magnitude.
type replayTrigSigned struct {
	slot     int
	negative bool
}

type replayTrigSpan struct{ lo, hi replayTrigSigned }

func (a *replayTrigArena) signedAdd(out int, x, y replayTrigSigned) (replayTrigSigned, error) {
	result := replayTrigSigned{slot: out}
	if x.negative == y.negative {
		if err := a.add(out, x.slot, y.slot); err != nil {
			return result, err
		}
		result.negative = x.negative
	} else {
		if err := a.charge(replayTrigWords); err != nil {
			return result, err
		}
		if replayTrigCmp(&a.slots[x.slot], &a.slots[y.slot]) >= 0 {
			if err := a.sub(out, x.slot, y.slot); err != nil {
				return result, err
			}
			result.negative = x.negative
		} else {
			if err := a.sub(out, y.slot, x.slot); err != nil {
				return result, err
			}
			result.negative = y.negative
		}
	}
	if err := a.charge(replayTrigWords); err != nil {
		return result, err
	}
	if replayTrigBitLen(&a.slots[out]) == 0 {
		result.negative = false
	}
	return result, nil
}

func (a *replayTrigArena) dyadicRat(value replayTrigSigned, scale int) (*big.Rat, error) {
	if scale < 0 || scale > 4095 {
		return nil, fmt.Errorf("%w: bounded trig result scale", decaderr.ErrUnsupported)
	}
	if err := a.copySlot(18, value.slot); err != nil {
		return nil, err
	}
	if err := a.charge(replayTrigWords); err != nil {
		return nil, err
	}
	if err := a.charge(uint64(scale)); err != nil {
		return nil, err
	}
	tz := 0
	for tz < scale && (a.slots[18][tz/64]>>(uint(tz)%64))&1 == 0 {
		tz++
	}
	if err := a.right(18, uint(tz)); err != nil {
		return nil, err
	}
	scale -= tz
	if err := a.charge(replayTrigWords); err != nil {
		return nil, err
	}
	nwords := (replayTrigBitLen(&a.slots[18]) + 63) / 64
	dwords := scale/64 + 1
	if err := a.budget.ReserveRecords(a.ctx, 3); err != nil {
		return nil, err
	}
	for _, words := range [2]int{nwords, dwords} {
		if err := a.budget.ReserveArray(a.ctx, uint64(words), uint64(unsafe.Sizeof(big.Word(0)))); err != nil {
			return nil, err
		}
	}
	if err := a.budget.ReserveArray(a.ctx, 1, uint64(unsafe.Sizeof(big.Rat{}))); err != nil {
		return nil, err
	}
	if err := a.charge(uint64(nwords + dwords)); err != nil {
		return nil, err
	}
	n := make([]big.Word, nwords)
	for i := range n {
		n[i] = big.Word(a.slots[18][i])
	}
	d := make([]big.Word, dwords)
	d[scale/64] = big.Word(uint64(1) << uint(scale%64))
	if err := a.budget.ReserveRecords(a.ctx, 2); err != nil {
		return nil, err
	}
	for range 2 {
		if err := a.budget.ReserveArray(a.ctx, 1, uint64(unsafe.Sizeof(big.Int{}))); err != nil {
			return nil, err
		}
	}
	if err := a.charge(64); err != nil {
		return nil, err
	}
	num, den := new(big.Int).SetBits(n), new(big.Int).SetBits(d)
	if value.negative && nwords > 0 {
		num.Neg(num)
	}
	return proof.ReplayReducedFraction(a.ctx, num, den, a.budget)
}
