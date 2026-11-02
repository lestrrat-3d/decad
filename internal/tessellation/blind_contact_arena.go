package tessellation

import (
	"math/bits"

	"github.com/lestrrat-3d/decad/internal/decaderr"
)

// addContactInto writes a sum or difference into a distinct arena slot.
// The inputs must not alias the output; the caller discards it on error.
func (a blindContactArithmetic) addContactInto(z, x, y *blindContactInt, subtract bool) error {
	if z == x || z == y {
		return decaderr.ErrUnsupported
	}
	n, m, err := a.widths(x, y)
	if err != nil {
		return err
	}
	w := max(n, m)
	clearCount := len(z.limbs)
	if z.known {
		clearCount = int(z.used)
	}
	if err := a.budget.SpendWork(a.ctx, uint64(clearCount+1+3*w+2)); err != nil {
		return err
	}
	for i := range clearCount {
		z.limbs[i] = 0
	}
	z.negative, z.used = false, 0
	z.known = false
	xneg := n != 0 && x.negative
	yneg := m != 0 && (y.negative != subtract)
	if xneg == yneg {
		var carry uint64
		for i := range w {
			z.limbs[i], carry = bits.Add64(x.limbs[i], y.limbs[i], carry)
		}
		if carry != 0 {
			if w == len(z.limbs) {
				return decaderr.ErrUnsupported
			}
			z.limbs[w] = carry
		}
		z.used = uint8(w)
		if carry != 0 {
			z.used++
		}
		z.negative = xneg && (w != 0 || carry != 0)
		z.known = true
		return nil
	}
	order := 0
	for i := w - 1; i >= 0; i-- {
		if x.limbs[i] > y.limbs[i] {
			order = 1
			break
		}
		if x.limbs[i] < y.limbs[i] {
			order = -1
			break
		}
	}
	if order == 0 {
		z.known = true
		return nil
	}
	large, small, negative := x, y, xneg
	if order < 0 {
		large, small, negative = y, x, yneg
	}
	var borrow uint64
	for i := range w {
		z.limbs[i], borrow = bits.Sub64(large.limbs[i], small.limbs[i], borrow)
	}
	for w > 0 && z.limbs[w-1] == 0 {
		w--
	}
	z.used = uint8(w)
	z.negative = negative
	z.known = true
	return nil
}

// multiplyContactInto writes a product into a distinct arena slot.
func (a blindContactArithmetic) multiplyContactInto(z, x, y *blindContactInt) error {
	if z == x || z == y {
		return decaderr.ErrUnsupported
	}
	n, m, err := a.widths(x, y)
	if err != nil {
		return err
	}
	clearCount := len(z.limbs)
	if z.known {
		clearCount = int(z.used)
	}
	if err := a.budget.SpendWork(a.ctx, uint64(clearCount+1+n*m+min(n+m, 32))); err != nil {
		return err
	}
	for i := range clearCount {
		z.limbs[i] = 0
	}
	z.negative, z.used = false, 0
	z.known = false
	if n == 0 || m == 0 {
		z.known = true
		return nil
	}
	if n+m > len(z.limbs)+1 {
		return decaderr.ErrUnsupported
	}
	for i := range n {
		var carry uint64
		for j := range m {
			k := i + j
			hi, lo := bits.Mul64(x.limbs[i], y.limbs[j])
			if k >= len(z.limbs) {
				if hi != 0 || lo != 0 || carry != 0 {
					return decaderr.ErrUnsupported
				}
				continue
			}
			var c uint64
			lo, c = bits.Add64(lo, z.limbs[k], 0)
			hi, _ = bits.Add64(hi, 0, c)
			lo, c = bits.Add64(lo, carry, 0)
			hi, _ = bits.Add64(hi, 0, c)
			z.limbs[k], carry = lo, hi
		}
		k := i + m
		if k == len(z.limbs) {
			if carry != 0 {
				return decaderr.ErrUnsupported
			}
		} else {
			z.limbs[k] = carry
		}
	}
	width := min(n+m, len(z.limbs))
	for width > 0 && z.limbs[width-1] == 0 {
		width--
	}
	z.used = uint8(width)
	z.negative = x.negative != y.negative
	z.known = true
	return nil
}

func (a blindContactArithmetic) differenceContactInto(z, x, y *blindContactVector) error {
	if z == x || z == y {
		return decaderr.ErrUnsupported
	}
	for i := range z {
		if err := a.addContactInto(&z[i], &x[i], &y[i], true); err != nil {
			return err
		}
	}
	return nil
}

func (a blindContactArithmetic) crossContactInto(z, x, y *blindContactVector, slots *[2]blindContactInt) error {
	if err := a.budget.SpendWork(a.ctx, 32); err != nil {
		return err
	}
	if z == x || z == y {
		return decaderr.ErrUnsupported
	}
	for i := range 3 {
		for j := range slots {
			if &slots[j] == &x[i] || &slots[j] == &y[i] || &slots[j] == &z[i] {
				return decaderr.ErrUnsupported
			}
		}
	}
	for i := range z {
		j, k := (i+1)%3, (i+2)%3
		if err := a.multiplyContactInto(&slots[0], &x[j], &y[k]); err != nil {
			return err
		}
		if err := a.multiplyContactInto(&slots[1], &x[k], &y[j]); err != nil {
			return err
		}
		if err := a.addContactInto(&z[i], &slots[0], &slots[1], true); err != nil {
			return err
		}
	}
	return nil
}

func (a blindContactArithmetic) dotContactInto(z *blindContactInt, x, y *blindContactVector,
	slots *[3]blindContactInt) error {
	if err := a.budget.SpendWork(a.ctx, 32); err != nil {
		return err
	}
	for i := range 3 {
		if z == &x[i] || z == &y[i] || z == &slots[i] {
			return decaderr.ErrUnsupported
		}
		for j := range 3 {
			if &slots[i] == &x[j] || &slots[i] == &y[j] {
				return decaderr.ErrUnsupported
			}
		}
	}
	if err := a.multiplyContactInto(&slots[0], &x[0], &y[0]); err != nil {
		return err
	}
	if err := a.multiplyContactInto(&slots[1], &x[1], &y[1]); err != nil {
		return err
	}
	if err := a.addContactInto(&slots[2], &slots[0], &slots[1], false); err != nil {
		return err
	}
	if err := a.multiplyContactInto(&slots[0], &x[2], &y[2]); err != nil {
		return err
	}
	return a.addContactInto(z, &slots[2], &slots[0], false)
}

func (a blindContactArithmetic) projectContactInto(out *blindContactInt, endpoint *[4]blindContactInt,
	direction *blindContactVector, scratch *blindContactScratch) error {
	if err := a.budget.SpendWork(a.ctx, 128); err != nil {
		return err
	}
	for i := range 3 {
		if err := a.requireBits(&endpoint[i], 600); err != nil {
			return err
		}
		if err := a.requireBits(&direction[i], 600); err != nil {
			return err
		}
		scratch.vectors[0][i] = endpoint[i]
	}
	if err := a.dotContactInto(out, &scratch.vectors[0], direction, &scratch.dot); err != nil {
		return err
	}
	return a.requireBits(out, 1202)
}

func (a blindContactArithmetic) compareContactFractionsArena(x, xw, y, yw *blindContactInt,
	scratch *blindContactScratch) (int, error) {
	for _, w := range [...]*blindContactInt{xw, yw} {
		order, err := a.compare(w, &scratch.zero)
		if err != nil {
			return 0, err
		}
		if order <= 0 {
			return 0, decaderr.ErrUnsupported
		}
	}
	if err := a.multiplyContactInto(&scratch.products[0], x, yw); err != nil {
		return 0, err
	}
	if err := a.multiplyContactInto(&scratch.products[1], y, xw); err != nil {
		return 0, err
	}
	if err := a.requireBits(&scratch.products[0], 1699); err != nil {
		return 0, err
	}
	if err := a.requireBits(&scratch.products[1], 1699); err != nil {
		return 0, err
	}
	return a.compare(&scratch.products[0], &scratch.products[1])
}
