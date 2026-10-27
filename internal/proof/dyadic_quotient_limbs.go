package proof

import (
	"math/bits"
	"slices"
)

// dyQuoMantissaWords divides the normalized leading numerator bits by a
// denominator of at most eight 64-bit words. Scaling leaves a quotient of at
// most 55 bits. Discarding low numerator bits cannot change its integer part;
// a right shift discards a nonzero bit because a canonical mantissa is odd.
func dyQuoMantissaWords(n, d Dyadic, exp int) (uint64, bool, bool) {
	denBits := d.bitLen()
	if denBits > dyQuoMaxWords*64 {
		return 0, false, false
	}
	count := (denBits + 63) / 64
	normalize := count*64 - denBits
	var denominator [dyQuoMaxWords]uint64
	for i := range count {
		denominator[i] = dyMagnitudeBits(d, i*64-normalize)
	}
	quotient, sticky := dyQuoMantissaNormalized(n, &denominator, count, 54-exp+normalize)
	return quotient, sticky, true
}

// dyQuoMaxWords is the widest denominator, in 64-bit words, that
// dyQuoMantissaNormalized divides by.
const dyQuoMaxWords = 8

// dyQuoMantissaProduct is dyQuoMantissaWords for the denominator whose odd
// magnitude is the four little-endian words product, of bit length denBits >
// 0: it fills the same normalized denominator words from product that
// dyMagnitudeBits reads from the canonical Dyadic of that magnitude.
func dyQuoMantissaProduct(n Dyadic, product *[4]uint64, denBits, exp int) (uint64, bool) {
	count := (denBits + 63) / 64
	normalize := uint(count*64 - denBits)
	var denominator [dyQuoMaxWords]uint64
	for i := range count {
		denominator[i] = product[i] << normalize
		if i > 0 && normalize != 0 {
			denominator[i] |= product[i-1] >> (64 - normalize)
		}
	}
	return dyQuoMantissaNormalized(n, &denominator, count, 54-exp+int(normalize))
}

// dyQuoMantissaNormalized divides n's magnitude, shifted left by shift bits,
// by the count normalized denominator words (leading bit set in word
// count-1), and reports the quotient and whether a remainder or a discarded
// numerator bit is nonzero.
func dyQuoMantissaNormalized(n Dyadic, denominator *[dyQuoMaxWords]uint64, count, shift int) (uint64, bool) {
	var numerator, product [dyQuoMaxWords + 1]uint64
	for i := range count + 1 {
		numerator[i] = dyMagnitudeBits(n, i*64-shift)
	}
	if count == 1 {
		quotient, remainder := bits.Div64(numerator[1], numerator[0], denominator[0])
		return quotient, remainder != 0 || shift < 0
	}
	quotient, _ := bits.Div64(numerator[count], numerator[count-1], denominator[count-1])
	carry := uint64(0)
	for i := range count {
		hi, lo := bits.Mul64(denominator[i], quotient)
		lo, c := bits.Add64(lo, carry, 0)
		product[i], carry = lo, hi+c
	}
	product[count] = carry
	comparison := cmpQuotientWords(product[:count+1], numerator[:count+1])
	if comparison > 0 {
		// The estimate divides by only the leading denominator word. It is
		// at most one too large: quotient < 2^56 and the normalized leading
		// word is >= 2^63, so (estimate-1)*d is already <= the numerator.
		quotient--
		borrow := uint64(0)
		for i := range count {
			product[i], borrow = bits.Sub64(product[i], denominator[i], borrow)
		}
		product[count], _ = bits.Sub64(product[count], 0, borrow)
		comparison = cmpQuotientWords(product[:count+1], numerator[:count+1])
	}
	return quotient, comparison != 0 || shift < 0
}

func cmpQuotientWords(a, b []uint64) int {
	for i, word := range slices.Backward(a) {
		if word > b[i] {
			return 1
		}
		if word < b[i] {
			return -1
		}
	}
	return 0
}
