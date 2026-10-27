package proof

import (
	"math"
	"math/big"
	"math/bits"
)

// Directed float64 roundings of a quotient of
// two Dyadics, without forming a big.Rat: no GCD, one integer QuoRem.
//
// dyQuoRound rounds n/d (n >= 0, d > 0) to a float64 either to nearest-even
// (up=false; what big.Rat.Float64 answers, and xhpFractionFloat64's algorithm)
// or toward +Inf (up=true; the smallest float64 at or above n/d, which is what
// proofbound.RatFloatUp answers). ok is false where the result would leave the
// normal float64 range; the caller then takes the big.Rat path, so the fast
// path never has to reproduce subnormal or overflow rounding.
func dyQuoRound(n, d Dyadic, up bool) (float64, bool, bool) {
	if n.IsZero() {
		return 0, true, true
	}
	exp := n.bitLen() - d.bitLen()
	mant, sticky, native := dyQuoMantissa128(n, d, exp)
	if !native {
		mant, sticky, native = dyQuoMantissaWords(n, d, exp)
	}
	if !native {
		var numerator, denominator big.Int
		a := n.MantInto(&numerator)
		b := d.MantInto(&denominator)
		if shift := 54 - exp; shift > 0 {
			a.Lsh(a, uint(shift))
		} else if shift < 0 {
			b.Lsh(b, uint(-shift))
		}
		var q, r big.Int
		q.QuoRem(a, b, &r)
		mant, sticky = q.Uint64(), r.Sign() != 0
	}
	return dyQuoFinish(mant, sticky, exp, n.exp-d.exp, up)
}

// dyQuoFinish rounds the quotient bits of dyQuoRound to a float64: mant is
// the floor of the scaled quotient, of 54 or 55 bits, sticky reports a
// nonzero remainder, exp is the bit-length difference the scaling used, and
// expDiff is the numerator's exponent less the denominator's.
func dyQuoFinish(mant uint64, sticky bool, exp, expDiff int, up bool) (float64, bool, bool) {
	if mant>>54 == 1 {
		if mant&1 != 0 {
			sticky = true
		}
		mant >>= 1
		exp++
	}
	// mant has 54 bits; its value is mant·2^(exp-54) scaled by 2^(n.exp-d.exp).
	exact := !sticky && mant&1 == 0
	if up {
		mant >>= 1
		if !exact {
			mant++
			if mant >= 1<<53 {
				mant >>= 1
				exp++
			}
		}
	} else {
		if mant&1 != 0 && (sticky || mant&2 != 0) {
			mant++
			if mant >= 1<<54 {
				mant >>= 1
				exp++
			}
		}
		mant >>= 1
	}
	// Leading bit of the 53-bit mant sits at 2^52; the value is mant·2^scale.
	scale := exp - 53 + expDiff
	if top := scale + 52; top < -1022 || top > 1023 {
		return 0, false, false
	}
	if scale >= -dyPow2Bias && scale <= dyPow2Bias {
		return float64(mant) * dyPow2[scale+dyPow2Bias], exact, true
	}
	return math.Ldexp(float64(mant), scale), exact, true
}

// dyQuoMantissa128 forms the same quotient bits and remainder flag as the
// big.Int path for an inline numerator and a 64-bit denominator. A positive
// numerator shift leaves at most 64+54 bits. A negative shift must keep the
// denominator in 64 bits. The quotient has at most 55 bits, so Div64's high
// numerator word is strictly below the denominator.
func dyQuoMantissa128(n, d Dyadic, exp int) (uint64, bool, bool) {
	if n.big != nil || d.big != nil || d.hi&^dySign != 0 {
		return 0, false, false
	}
	lo, hi, denominator := n.lo, n.hi&^dySign, d.lo
	if shift := 54 - exp; shift > 0 {
		lo, hi, _ = shl128(lo, hi, shift, n.bitLen())
	} else if shift < 0 {
		if bits.Len64(denominator)-shift > 64 {
			return 0, false, false
		}
		denominator <<= uint(-shift)
	}
	quotient, remainder := bits.Div64(hi, lo, denominator)
	return quotient, remainder != 0, true
}

// DyQuoFloatUp is proofbound.RatFloatUp over the exact quotient n/d, for
// n >= 0 and d > 0: the smallest float64 at or above n/d, bit for bit. Outside
// the normal range it takes the big.Rat path itself.
func DyQuoFloatUp(n, d Dyadic) float64 {
	if n.Sign() >= 0 && d.Sign() > 0 {
		if f, _, ok := dyQuoRound(n, d, true); ok {
			return f
		}
	}
	q := new(big.Rat).Quo(n.Rat(), d.Rat())
	f, _ := q.Float64()
	if math.IsNaN(f) || math.IsInf(f, 0) {
		return f
	}
	if fr := FloatRat(f); fr != nil && fr.Cmp(q) >= 0 {
		return f
	}
	return math.Nextafter(f, math.Inf(1))
}

// DyQuoFloatUpMul is DyQuoFloatUp(n, DyMul(a, b)), bit for bit, without
// building the product as a Dyadic. For inline factors of one sign whose
// product passes 64 bits, and a positive n, DyQuoFloatUp(n, DyMul(a, b)) takes
// dyQuoMantissaWords and dyQuoFinish. The product of two odd mantissas is odd,
// so its four stack words are the magnitude of that canonical Dyadic, with
// exponent a.exp+b.exp; dyQuoMantissaProduct reads from them the normalized
// denominator words dyMagnitudeBits reads from the Dyadic, and the same
// division and rounding follow. Every other case, and a quotient outside the
// normal range, takes DyQuoFloatUp(n, DyMul(a, b)) itself.
func DyQuoFloatUpMul(n, a, b Dyadic) float64 {
	if a.big != nil || b.big != nil || a.IsZero() || b.IsZero() || a.neg() != b.neg() || n.Sign() <= 0 {
		return DyQuoFloatUp(n, DyMul(a, b))
	}
	product := mulMagnitudes(a, b)
	denBits := 0
	for i := len(product) - 1; i >= 0; i-- {
		if product[i] != 0 {
			denBits = i*64 + bits.Len64(product[i])
			break
		}
	}
	if denBits <= 64 {
		return DyQuoFloatUp(n, DyMul(a, b))
	}
	exp := n.bitLen() - denBits
	mant, sticky := dyQuoMantissaProduct(n, &product, denBits, exp)
	if f, _, ok := dyQuoFinish(mant, sticky, exp, n.exp-(a.exp+b.exp), true); ok {
		return f
	}
	return DyQuoFloatUp(n, DyMul(a, b))
}

// mulMagnitudes is the full product of two inline magnitudes, least
// significant word first: the four words DyMul forms for the same factors.
func mulMagnitudes(a, b Dyadic) [4]uint64 {
	alo, ahi := a.lo, a.hi&^dySign
	blo, bhi := b.lo, b.hi&^dySign
	h0, p0 := bits.Mul64(alo, blo)
	h1, l1 := bits.Mul64(alo, bhi)
	h2, l2 := bits.Mul64(ahi, blo)
	h3, l3 := bits.Mul64(ahi, bhi)
	p1, c := bits.Add64(h0, l1, 0)
	p2, c2 := bits.Add64(h1, l3, c)
	p3 := h3 + c2
	p1, c = bits.Add64(p1, l2, 0)
	p2, c2 = bits.Add64(p2, h2, c)
	p3 += c2
	return [4]uint64{p0, p1, p2, p3}
}

// DyQuoFloatDown is proofbound.RatFloatDown over the exact quotient n/d, for
// d > 0 and n of either sign: the largest float64 at or below n/d, bit for
// bit, or the saturated nearest float when n/d leaves the range. A negative
// n answers −ceil(|n|/d), since negation is exact. A nonnegative n answers
// ceil(n/d) when that is exact and the float below it otherwise: dyQuoRound
// rounds at 53 bits in the quotient's own binade, so an inexact ceiling f has
// n/d strictly between f's 53-bit predecessor and f, and that predecessor is
// NextDown(f), the floor. At f = 2^-1022 the predecessor lies above the largest
// subnormal, NextDown(f), so the floor is still NextDown(f). A quotient whose
// ceiling leaves the normal range takes RatFloatDown's own big.Rat path.
func DyQuoFloatDown(n, d Dyadic) float64 {
	if d.Sign() > 0 {
		if n.Sign() < 0 {
			if f, _, ok := dyQuoRound(DyAbs(n), d, true); ok {
				return -f
			}
		} else if f, exact, ok := dyQuoRound(n, d, true); ok {
			if exact {
				return f
			}
			return NextDown(f)
		}
	}
	q := new(big.Rat).Quo(n.Rat(), d.Rat())
	f, _ := RatFloat64(q)
	if isNonFinite(f) || CmpFloatRat(f, q) <= 0 {
		return f
	}
	return NextDown(f)
}

// DyQuoFloatError is RationalFloatError(n/d, held) for d > 0: |n/d − held|
// rounded to nearest and stepped one ulp up when that rounding was inexact.
func DyQuoFloatError(n, d Dyadic, held float64) float64 {
	heldDy, ok := DyOf(held)
	if !ok || d.Sign() <= 0 {
		return math.Inf(1)
	}
	num := DyAbs(DySubScalar(n, DyMul(heldDy, d)))
	if f, exact, ok := dyQuoRound(num, d, false); ok {
		if !exact {
			f = NextUp(f)
		}
		return f
	}
	return RationalFloatError(new(big.Rat).Quo(n.Rat(), d.Rat()), held)
}
