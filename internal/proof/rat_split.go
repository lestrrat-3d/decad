package proof

import (
	"cmp"
	"math/big"
	"math/bits"
)

// ratSplitAdd sets z = a+b (a-b with sub) in lowest terms with Knuth's
// split GCD (TAOCP 4.5.1): d1 = gcd(q1, q2) and d2 = gcd(t, d1) replace the
// one GCD of the full cross-multiplied numerator and denominator.
func ratSplitAdd(z, a, b *big.Rat, sub bool) *big.Rat {
	p1, q1 := a.Num(), a.Denom()
	p2, q2 := b.Num(), b.Denom()
	d1 := ratGCD(new(big.Int), q1, q2)
	if isOne(d1) {
		n := new(big.Int).Mul(p1, q2)
		m := new(big.Int).Mul(p2, q1)
		if sub {
			n.Sub(n, m)
		} else {
			n.Add(n, m)
		}
		if n.Sign() == 0 {
			return z.SetInt64(0)
		}
		d := m.Mul(q1, q2)
		z.SetInt(n)
		z.Denom().Set(d)
		return z
	}
	q1d := new(big.Int).Quo(q1, d1)
	q2d := new(big.Int).Quo(q2, d1)
	t := new(big.Int).Mul(p1, q2d)
	u := new(big.Int).Mul(p2, q1d)
	if sub {
		t.Sub(t, u)
	} else {
		t.Add(t, u)
	}
	if t.Sign() == 0 {
		return z.SetInt64(0)
	}
	d2 := ratGCD(u, t, d1)
	if !isOne(d2) {
		t.Quo(t, d2)
		q2d.Quo(q2, d2)
	} else {
		q2d.Set(q2)
	}
	d := q1d.Mul(q1d, q2d)
	z.SetInt(t)
	z.Denom().Set(d)
	return z
}

// ratSplitMul sets z = a·b in lowest terms: g1 = gcd(p1, q2) and
// g2 = gcd(p2, q1) cancel before the products are formed.
func ratSplitMul(z, a, b *big.Rat) *big.Rat {
	p1, q1 := a.Num(), a.Denom()
	p2, q2 := b.Num(), b.Denom()
	if p1.Sign() == 0 || p2.Sign() == 0 {
		return z.SetInt64(0)
	}
	n1, d2 := p1, q2
	if g := ratGCD(new(big.Int), p1, q2); !isOne(g) {
		n1 = new(big.Int).Quo(p1, g)
		d2 = new(big.Int).Quo(q2, g)
	}
	n2, d1 := p2, q1
	if g := ratGCD(new(big.Int), p2, q1); !isOne(g) {
		n2 = new(big.Int).Quo(p2, g)
		d1 = new(big.Int).Quo(q1, g)
	}
	n := new(big.Int).Mul(n1, n2)
	d := new(big.Int).Mul(d1, d2)
	z.SetInt(n)
	z.Denom().Set(d)
	return z
}

// QuoRat sets z = a/b in lowest terms: g1 = gcd(p1, p2) and
// g2 = gcd(q1, q2) cancel first. b must be nonzero.
//
// Operands must be in lowest terms, as for AddRat: a non-reduced operand gives
// the exact quotient, but not always in lowest terms, and an identity read
// from the quotient's text (RatTextEqual, as AddVert in internal/meshbool
// reads it) would then name one value two ways.
func QuoRat(z, a, b *big.Rat) *big.Rat {
	p1, q1 := a.Num(), a.Denom()
	p2, q2 := b.Num(), b.Denom()
	if p2.Sign() == 0 {
		return z.Quo(a, b)
	}
	if p1.Sign() == 0 {
		return z.SetInt64(0)
	}
	n1, n2 := p1, p2
	if g := ratGCD(new(big.Int), p1, p2); !isOne(g) {
		n1 = new(big.Int).Quo(p1, g)
		n2 = new(big.Int).Quo(p2, g)
	}
	m1, m2 := q2, q1
	if g := ratGCD(new(big.Int), q1, q2); !isOne(g) {
		m1 = new(big.Int).Quo(q2, g)
		m2 = new(big.Int).Quo(q1, g)
	}
	n := new(big.Int).Mul(n1, m1)
	d := new(big.Int).Mul(m2, n2)
	if d.Sign() < 0 {
		n.Neg(n)
		d.Neg(d)
	}
	z.SetInt(n)
	z.Denom().Set(d)
	return z
}

// CmpRat compares a and b and returns -1, 0 or +1 as a is less than, equal to
// or greater than b, the result big.Rat.Cmp returns. It reads a and b and
// writes neither: it never stores into a Rat's numerator or denominator, and
// it asks Denom for a fraction only, so a Rat whose integer denominator is
// unstored (Denom would return a fresh detached 1) allocates nothing.
//
// Signs decide first. Two integers, and two fractions with equal denominators,
// compare their numerators. Two power-of-two denominators 2^i and 2^j put the
// numerators over the larger one, which shifts the numerator with the smaller
// exponent left by |i-j|, and compare those. Any other pair compares the two
// cross products a.Num·b.Denom and b.Num·a.Denom, whose sign equals the sign
// of the difference because both denominators are positive. The integers the
// shifts and products need are stack-declared big.Int values, and no branch
// reaches math/big's pooled scratch stack below the Karatsuba size.
func CmpRat(a, b *big.Rat) int {
	if sa, sb := a.Sign(), b.Sign(); sa != sb {
		return cmp.Compare(sa, sb)
	} else if sa == 0 {
		return 0
	}
	an, bn := a.Num(), b.Num()
	aInt, bInt := a.IsInt(), b.IsInt()
	if aInt && bInt {
		return an.Cmp(bn)
	}
	i, okA := denomExp(a)
	j, okB := denomExp(b)
	if okA && okB {
		var shifted big.Int
		if i < j {
			return shifted.Lsh(an, j-i).Cmp(bn)
		}
		return an.Cmp(shifted.Lsh(bn, i-j))
	}
	if !aInt && !bInt && a.Denom().Cmp(b.Denom()) == 0 {
		return an.Cmp(bn)
	}
	var x, y big.Int
	return mulDenom(&x, an, b).Cmp(mulDenom(&y, bn, a))
}

// mulDenom sets z to n times r's denominator and returns it. An integer r has
// denominator 1, so z is n without a call to Denom.
func mulDenom(z, n *big.Int, r *big.Rat) *big.Int {
	if r.IsInt() {
		return z.Set(n)
	}
	return z.Mul(n, r.Denom())
}

// isOne reports whether x is 1.
func isOne(x *big.Int) bool { return x.IsInt64() && x.Int64() == 1 }

// ratGCD is GCD(nil, nil, a, b). When either magnitude is a power of two,
// their greatest common divisor is 2^min(tz(a), tz(b)). Count those bits instead
// of running the Euclidean algorithm. Other operands skip complete low zero
// words before GCD; their original common binary factor is restored afterwards.
// z owns its result and may alias either input, as with big.Int.GCD.
func ratGCD(z, a, b *big.Int) *big.Int {
	if a.Sign() == 0 || b.Sign() == 0 {
		return z.GCD(nil, nil, a, b)
	}
	at, bt := a.TrailingZeroBits(), b.TrailingZeroBits()
	if a.BitLen() == int(at)+1 || b.BitLen() == int(bt)+1 {
		return z.Lsh(z.SetInt64(1), min(at, bt))
	}
	if at >= bits.UintSize || bt >= bits.UintSize {
		// Discard complete zero words without copying the immutable operand
		// limbs. GCD writes a separate result before z changes, so the views
		// stay read-only even when z aliases a or b. No view is returned.
		var av, bv big.Int
		aa, bb := a, b
		if at >= bits.UintSize {
			aa = av.SetBits(a.Bits()[at/bits.UintSize:])
		}
		if bt >= bits.UintSize {
			bb = bv.SetBits(b.Bits()[bt/bits.UintSize:])
		}
		var reduced big.Int
		reduced.GCD(nil, nil, aa, bb)
		// The odd parts determine the odd gcd. Remove any residual binary
		// factor from the word-aligned views and restore the original one.
		reduced.Rsh(&reduced, reduced.TrailingZeroBits())
		reduced.Lsh(&reduced, min(at, bt))
		return z.Set(&reduced)
	}
	return z.GCD(nil, nil, a, b)
}
