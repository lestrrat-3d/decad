package proof

// Test-only copy of the FloatDeterminant implementation that preceded the
// allocation-trimmed one, kept as the differential reference.

import (
	"math/bits"

	"github.com/lestrrat-3d/r3"
)

type legacyFactor struct {
	mant uint64
	exp  int
	neg  bool
}

type legacyTerm struct {
	words [3]uint64
	exp   int
	neg   bool
}

func legacyTripleTerm(a, b, c legacyFactor, negate bool) legacyTerm {
	if a.mant == 0 || b.mant == 0 || c.mant == 0 {
		return legacyTerm{}
	}
	hi, lo := bits.Mul64(a.mant, b.mant)
	carry, p0 := bits.Mul64(lo, c.mant)
	p2, p1 := bits.Mul64(hi, c.mant)
	p1, carry = bits.Add64(p1, carry, 0)
	p2 += carry
	return legacyTerm{
		words: [3]uint64{p0, p1, p2}, exp: a.exp + b.exp + c.exp,
		neg: (a.neg != b.neg) != (c.neg != negate),
	}
}

// LegacyFloatDeterminant returns a dot (b cross c) exactly for finite coordinates.
// It sums the six signed triple products in native limbs, allocating only the
// final owned mantissa. Wide exponent spans use the general dyadic arithmetic.
// A non-finite coordinate panics, as in DyVec.
func LegacyFloatDeterminant(a, b, c r3.Vec) Dyadic {
	var factors [9]legacyFactor
	for i, coordinate := range [9]float64{a.X, a.Y, a.Z, b.X, b.Y, b.Z, c.X, c.Y, c.Z} {
		d := MustDyOf(coordinate)
		factors[i] = legacyFactor{mant: d.lo, exp: d.exp, neg: d.neg()}
	}
	terms := [6]legacyTerm{
		legacyTripleTerm(factors[0], factors[4], factors[8], false),
		legacyTripleTerm(factors[1], factors[5], factors[6], false),
		legacyTripleTerm(factors[2], factors[3], factors[7], false),
		legacyTripleTerm(factors[0], factors[5], factors[7], true),
		legacyTripleTerm(factors[1], factors[3], factors[8], true),
		legacyTripleTerm(factors[2], factors[4], factors[6], true),
	}
	minExp, seen := 0, false
	for _, term := range terms {
		if term.words[0] != 0 && (!seen || term.exp < minExp) {
			minExp, seen = term.exp, true
		}
	}
	if !seen {
		return Dyadic{}
	}
	var sum [8]uint64
	wordCount := 0
	for _, term := range terms {
		if term.words[0] == 0 {
			continue
		}
		width := 64 + bits.Len64(term.words[1])
		if term.words[2] != 0 {
			width = 128 + bits.Len64(term.words[2])
		} else if term.words[1] == 0 {
			width = bits.Len64(term.words[0])
		}
		// Six magnitudes require three carry bits; reserve a sign bit too.
		width += term.exp - minExp + 4
		if width > len(sum)*64 {
			da, db, dc := DyVec(a), DyVec(b), DyVec(c)
			cross := DvCross(db, dc)
			return DvDot(da, cross)
		}
		wordCount = max(wordCount, (width+63)/64)
	}
	for _, term := range terms {
		if term.words[0] == 0 {
			continue
		}
		shift := term.exp - minExp
		wordShift, bitShift := shift/64, uint(shift%64)
		carry := uint64(0)
		for i := wordShift; i < wordCount; i++ {
			j := i - wordShift
			word := uint64(0)
			if j < len(term.words) {
				word = term.words[j] << bitShift
			}
			if bitShift != 0 && j > 0 && j <= len(term.words) {
				word |= term.words[j-1] >> (64 - bitShift)
			}
			if term.neg {
				sum[i], carry = bits.Sub64(sum[i], word, carry)
			} else {
				sum[i], carry = bits.Add64(sum[i], word, carry)
			}
		}
	}
	negative := sum[wordCount-1]>>63 != 0
	if negative {
		borrow := uint64(0)
		for i := range wordCount {
			sum[i], borrow = bits.Sub64(0, sum[i], borrow)
		}
	}
	for wordCount > 0 && sum[wordCount-1] == 0 {
		wordCount--
	}
	if wordCount == 0 {
		return Dyadic{}
	}
	if wordCount <= 2 && sum[1]&dySign == 0 {
		return fromMag(sum[0], sum[1], negative, minExp)
	}
	return fromBig(bigOfWords(sum[:wordCount], negative), minExp)
}
