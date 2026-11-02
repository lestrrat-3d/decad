package tessellation

import (
	"math/bits"

	"github.com/lestrrat-3d/decad/internal/decaderr"
)

// compareContactFractions compares numerators over positive denominators.
func (a blindContactArithmetic) compareContactFractions(x, xw, y, yw *blindContactInt) (int, error) {
	var zero blindContactInt
	for _, w := range [...]*blindContactInt{xw, yw} {
		order, err := a.compare(w, &zero)
		if err != nil {
			return 0, err
		}
		if order <= 0 {
			return 0, decaderr.ErrUnsupported
		}
	}
	left, err := a.mul(x, yw)
	if err != nil {
		return 0, err
	}
	right, err := a.mul(y, xw)
	if err != nil {
		return 0, err
	}
	if err := a.requireBits(&left, 1699); err != nil {
		return 0, err
	}
	if err := a.requireBits(&right, 1699); err != nil {
		return 0, err
	}
	return a.compare(&left, &right)
}

func (a blindContactArithmetic) add(x, y *blindContactInt) (blindContactInt, error) {
	var z blindContactInt
	n, m, err := a.widths(x, y)
	if err != nil {
		return z, err
	}
	w := max(n, m)
	if err := a.budget.SpendWork(a.ctx, uint64(2*w+1)); err != nil {
		return z, err
	}
	if x.negative == y.negative {
		var carry uint64
		for i := range w {
			z.limbs[i], carry = bits.Add64(x.limbs[i], y.limbs[i], carry)
		}
		if carry != 0 {
			if w == len(z.limbs) {
				return blindContactInt{}, decaderr.ErrUnsupported
			}
			z.limbs[w] = carry
		}
		z.negative = x.negative && (w != 0 || carry != 0)
		return z, nil
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
		return z, nil
	}
	large, small := x, y
	if order < 0 {
		large, small = y, x
	}
	var borrow uint64
	for i := range w {
		z.limbs[i], borrow = bits.Sub64(large.limbs[i], small.limbs[i], borrow)
	}
	z.negative = large.negative
	return z, nil
}

func (a blindContactArithmetic) mul(x, y *blindContactInt) (blindContactInt, error) {
	var z blindContactInt
	n, m, err := a.widths(x, y)
	if err != nil {
		return z, err
	}
	if err := a.budget.SpendWork(a.ctx, uint64(n*m)); err != nil {
		return z, err
	}
	if n == 0 || m == 0 {
		return z, nil
	}
	if n+m > len(z.limbs)+1 {
		return z, decaderr.ErrUnsupported
	}
	for i := range n {
		var carry uint64
		for j := range m {
			k := i + j
			// Product, destination limb and incoming carry sum to at most
			// (2^64-1)^2 + 2*(2^64-1) = 2^128-1.
			hi, lo := bits.Mul64(x.limbs[i], y.limbs[j])
			if k >= len(z.limbs) {
				if hi != 0 || lo != 0 || carry != 0 {
					return blindContactInt{}, decaderr.ErrUnsupported
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
				return blindContactInt{}, decaderr.ErrUnsupported
			}
		} else {
			z.limbs[k] = carry
		}
	}
	z.negative = x.negative != y.negative
	return z, nil
}

func (a blindContactArithmetic) sub(x, y *blindContactInt) (blindContactInt, error) {
	if err := a.budget.SpendWork(a.ctx, 32); err != nil {
		return blindContactInt{}, err
	}
	neg := *y
	var nonzero uint64
	for _, word := range neg.limbs {
		nonzero |= word
	}
	if nonzero != 0 {
		neg.negative = !neg.negative
	}
	return a.add(x, &neg)
}

func (a blindContactArithmetic) difference(x, y *blindContactVector) (blindContactVector, error) {
	var out blindContactVector
	for i := range out {
		v, err := a.sub(&x[i], &y[i])
		if err != nil {
			return blindContactVector{}, err
		}
		out[i] = v
	}
	return out, nil
}

func (a blindContactArithmetic) cross(x, y *blindContactVector) (blindContactVector, error) {
	var out blindContactVector
	for i := range out {
		j, k := (i+1)%3, (i+2)%3
		left, err := a.mul(&x[j], &y[k])
		if err != nil {
			return blindContactVector{}, err
		}
		right, err := a.mul(&x[k], &y[j])
		if err != nil {
			return blindContactVector{}, err
		}
		v, err := a.sub(&left, &right)
		if err != nil {
			return blindContactVector{}, err
		}
		out[i] = v
	}
	return out, nil
}

func (a blindContactArithmetic) dot(x, y *blindContactVector) (blindContactInt, error) {
	var out blindContactInt
	for i := range x {
		term, err := a.mul(&x[i], &y[i])
		if err != nil {
			return blindContactInt{}, err
		}
		out, err = a.add(&out, &term)
		if err != nil {
			return blindContactInt{}, err
		}
	}
	return out, nil
}

func (a blindContactArithmetic) projectContactEndpoint(endpoint *[4]blindContactInt,
	direction *blindContactVector) (blindContactInt, error) {
	var point blindContactVector
	if err := a.budget.SpendWork(a.ctx, 96); err != nil {
		return blindContactInt{}, err
	}
	for i := range point {
		if err := a.requireBits(&endpoint[i], 600); err != nil {
			return blindContactInt{}, err
		}
		if err := a.requireBits(&direction[i], 600); err != nil {
			return blindContactInt{}, err
		}
		point[i] = endpoint[i]
	}
	value, err := a.dot(&point, direction)
	if err != nil {
		return blindContactInt{}, err
	}
	if err := a.requireBits(&value, 1202); err != nil {
		return blindContactInt{}, err
	}
	return value, nil
}
