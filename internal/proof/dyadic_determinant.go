package proof

import (
	"math"
	"math/big"
	"math/bits"

	"github.com/lestrrat-3d/r3"
)

type floatDyadicFactor struct {
	mant uint64
	exp  int
	neg  bool
}

type dyadicTripleTerm struct {
	words [3]uint64
	exp   int
	neg   bool
}

func tripleTerm(a, b, c floatDyadicFactor, negate bool) dyadicTripleTerm {
	if a.mant == 0 || b.mant == 0 || c.mant == 0 {
		return dyadicTripleTerm{}
	}
	hi, lo := bits.Mul64(a.mant, b.mant)
	carry, p0 := bits.Mul64(lo, c.mant)
	p2, p1 := bits.Mul64(hi, c.mant)
	p1, carry = bits.Add64(p1, carry, 0)
	p2 += carry
	return dyadicTripleTerm{
		words: [3]uint64{p0, p1, p2}, exp: a.exp + b.exp + c.exp,
		neg: (a.neg != b.neg) != (c.neg != negate),
	}
}

// floatFactorOf splits a finite float into an odd mantissa and an exponent,
// exactly as MustDyOf does, without building a Dyadic.
func floatFactorOf(f float64) floatDyadicFactor {
	b := math.Float64bits(f)
	mag, exp := b&(1<<52-1), int(b>>52&0x7ff)
	switch {
	case exp == 0x7ff:
		panic("decad: exact Dyadic lift requires a finite float")
	case exp != 0:
		mag |= 1 << 52
		exp -= 1075
	case mag == 0:
		return floatDyadicFactor{}
	default:
		exp = -1074
	}
	tz := bits.TrailingZeros64(mag)
	return floatDyadicFactor{mant: mag >> tz, exp: exp + tz, neg: b>>63 != 0}
}

// ownedBig is a big.Int allocated together with the storage of a small
// mantissa, so a result of up to four words costs one allocation.
type ownedBig struct {
	z     big.Int
	words [4]big.Word
}

// bigOfOddWords returns the integer ±words (least significant word first,
// most significant word non-zero) as a new big.Int that owns its storage.
func bigOfOddWords(words []uint64, neg bool) *big.Int {
	if bits.UintSize != 64 || len(words) > len(ownedBig{}.words) {
		return bigOfWords(words, neg)
	}
	h := new(ownedBig)
	for i, w := range words {
		h.words[i] = big.Word(w)
	}
	h.z.SetBits(h.words[:len(words)])
	if neg {
		h.z.Neg(&h.z)
	}
	return &h.z
}

// FloatDeterminant returns a dot (b cross c) exactly for finite coordinates.
// It sums the six signed triple products in native limbs, allocating only the
// final owned mantissa, and only when that mantissa is wider than the inline
// representation. Wide exponent spans use the general dyadic arithmetic.
// A non-finite coordinate panics, as in DyVec.
func FloatDeterminant(a, b, c r3.Vec) Dyadic {
	fa := [3]floatDyadicFactor{floatFactorOf(a.X), floatFactorOf(a.Y), floatFactorOf(a.Z)}
	fb := [3]floatDyadicFactor{floatFactorOf(b.X), floatFactorOf(b.Y), floatFactorOf(b.Z)}
	fc := [3]floatDyadicFactor{floatFactorOf(c.X), floatFactorOf(c.Y), floatFactorOf(c.Z)}
	terms := [6]dyadicTripleTerm{
		tripleTerm(fa[0], fb[1], fc[2], false),
		tripleTerm(fa[1], fb[2], fc[0], false),
		tripleTerm(fa[2], fb[0], fc[1], false),
		tripleTerm(fa[0], fb[2], fc[1], true),
		tripleTerm(fa[1], fb[0], fc[2], true),
		tripleTerm(fa[2], fb[1], fc[0], true),
	}
	// One pass finds the lowest exponent and the highest bit any term reaches.
	minExp, top, seen := 0, 0, false
	for i := range terms {
		term := &terms[i]
		if term.words[0] == 0 {
			continue
		}
		width := 64 + bits.Len64(term.words[1])
		if term.words[2] != 0 {
			width = 128 + bits.Len64(term.words[2])
		} else if term.words[1] == 0 {
			width = bits.Len64(term.words[0])
		}
		if !seen {
			minExp, top, seen = term.exp, width+term.exp, true
			continue
		}
		minExp = min(minExp, term.exp)
		top = max(top, width+term.exp)
	}
	if !seen {
		return Dyadic{}
	}
	// Six magnitudes require three carry bits; reserve a sign bit too.
	width := top - minExp + 4
	if width > 8*64 {
		da, db, dc := DyVec(a), DyVec(b), DyVec(c)
		cross := DvCross(db, dc)
		return DvDot(da, cross)
	}
	wordCount := (width + 63) / 64
	var sum [8]uint64
	for i := range terms {
		term := &terms[i]
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
	exp := minExp
	// Reduce to an odd mantissa in place, as the canonical form requires.
	lowWord := 0
	for sum[lowWord] == 0 {
		lowWord++
	}
	if tz := bits.TrailingZeros64(sum[lowWord]); lowWord != 0 || tz != 0 {
		n := wordCount - lowWord
		for i := range n {
			w := sum[i+lowWord] >> uint(tz)
			if tz != 0 && i+lowWord+1 < wordCount {
				w |= sum[i+lowWord+1] << (64 - uint(tz))
			}
			sum[i] = w
		}
		clear(sum[n:wordCount])
		wordCount = n
		exp += lowWord*64 + tz
		for wordCount > 0 && sum[wordCount-1] == 0 {
			wordCount--
		}
	}
	if wordCount <= 1 || wordCount == 2 && sum[1]&dySign == 0 {
		return fromMag(sum[0], sum[1], negative, exp)
	}
	return Dyadic{big: bigOfOddWords(sum[:wordCount], negative), exp: exp}
}
