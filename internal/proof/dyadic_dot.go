package proof

import (
	"math"
	"math/bits"
)

// dvSquareInline sums three squares in machine words before creating the
// result's owned mantissa. This avoids allocating each wide square and each
// partial sum. Large mantissas or exponent gaps use the general dot product.
func dvSquareInline(v DyV3) (Dyadic, bool) {
	var sum [8]uint64
	n, exp, ok := dvSquareSum(v, &sum)
	if !ok || n == 0 {
		return Dyadic{}, ok
	}
	return fromBig(bigOfWords(sum[:n], false), exp), true
}

// dvSquareSum writes v·v into sum as the unreduced integer sum[:n] (least
// significant word first) times 2^exp. n is 0 for the zero vector. ok is
// false for a big component, or for exponent gaps or widths beyond the eight
// words; the caller then takes the general dot product.
func dvSquareSum(v DyV3, sum *[8]uint64) (int, int, bool) {
	minExp := 0
	seen := false
	for _, component := range v {
		if component.big != nil {
			return 0, 0, false
		}
		if component.IsZero() {
			continue
		}
		if !seen || component.exp < minExp {
			minExp = component.exp
			seen = true
		}
	}
	if !seen {
		return 0, 0, true
	}
	wordCount := 0
	for _, component := range v {
		if component.IsZero() {
			continue
		}
		delta := component.exp - minExp
		if delta < 0 || delta > 255 {
			return 0, 0, false
		}
		// Each square is below 2^(2*bitLen). Two extra bits accommodate
		// the sum of all three, including a carry across a word boundary.
		width := 2*(delta+component.bitLen()) + 2
		if width > len(sum)*64 {
			return 0, 0, false
		}
		wordCount = max(wordCount, (width+63)/64)
	}
	for _, component := range v {
		if component.IsZero() {
			continue
		}
		product := dySquareWords(component)
		shift := 2 * (component.exp - minExp)
		wordShift, bitShift := shift/64, uint(shift%64)
		carry := uint64(0)
		for i := wordShift; i < wordCount; i++ {
			j := i - wordShift
			word := uint64(0)
			if j < len(product) {
				word = product[j] << bitShift
			}
			if bitShift != 0 && j > 0 && j <= len(product) {
				word |= product[j-1] >> (64 - bitShift)
			}
			sum[i], carry = bits.Add64(sum[i], word, carry)
		}
	}
	return wordCount, 2 * minExp, true
}

// dySquareWords returns the full square of an inline mantissa's magnitude.
func dySquareWords(d Dyadic) [4]uint64 {
	lo, hi := d.lo, d.hi&^dySign
	h0, p0 := bits.Mul64(lo, lo)
	h1, l1 := bits.Mul64(lo, hi)
	h3, l3 := bits.Mul64(hi, hi)
	p1, c := bits.Add64(h0, l1, 0)
	p2, c2 := bits.Add64(h1, l3, c)
	p3 := h3 + c2
	p1, c = bits.Add64(p1, l1, 0)
	p2, c2 = bits.Add64(p2, h1, c)
	p3 += c2
	return [4]uint64{p0, p1, p2, p3}
}

// DvNormUp returns DySqrtUp(DyShift(DvDot(v, v), shift)), bit for bit: a
// proven upper bound on |v|·2^(shift/2).
//
// Over inline components whose square sum fits dvSquareSum's eight words, it
// reads that sum in place. A sum whose reduced mantissa fits inline becomes
// the same canonical Dyadic DvDot returns and is rooted by DySqrtUp itself. A
// wider sum, which DvDot would hold in a new big.Int, is rooted by
// dvWordsSqrtUp over the words, which repeats DySqrtUp's walk for a big
// mantissa: the same seed and the same exact comparisons. Anything else — a
// big component, an out-of-range sum, a seed DySqrtSeed would form through
// big.Float — takes the general expression.
func DvNormUp(v DyV3, shift int) float64 {
	var sum [8]uint64
	n, exp, ok := dvSquareSum(v, &sum)
	if !ok {
		return DySqrtUp(DyShift(DvDot(v, v), shift))
	}
	for n > 0 && sum[n-1] == 0 {
		n--
	}
	if n == 0 {
		return 0
	}
	exp += shift
	w := sum[:n]
	width := (n-1)*64 + bits.Len64(w[n-1])
	tz := 0
	for w[tz/64] == 0 {
		tz += 64
	}
	tz += bits.TrailingZeros64(w[tz/64])
	if width-tz <= dyBits {
		lo, hi := wordsField128(w, tz)
		return DySqrtUp(fromMag(lo, hi, false, exp+tz))
	}
	if f, ok := dvWordsSqrtUp(w, width, exp); ok {
		return f
	}
	return DySqrtUp(DyShift(DvDot(v, v), shift))
}

// DvNormBounds returns DySqrtDown(DvDot(v, v)) and DySqrtUp(DvDot(v, v)),
// bit for bit. Inline components share one exact square sum. A wide sum is
// rooted directly over its words, sharing the seed and its first comparison.
// Inputs outside that word path use the general dot product and root walks.
func DvNormBounds(v DyV3) (float64, float64) {
	var sum [8]uint64
	n, exp, ok := dvSquareSum(v, &sum)
	if ok {
		for n > 0 && sum[n-1] == 0 {
			n--
		}
		if n == 0 {
			return 0, 0
		}
		w := sum[:n]
		width := (n-1)*64 + bits.Len64(w[n-1])
		tz := 0
		for w[tz/64] == 0 {
			tz += 64
		}
		tz += bits.TrailingZeros64(w[tz/64])
		if width-tz <= dyBits {
			lo, hi := wordsField128(w, tz)
			d := fromMag(lo, hi, false, exp+tz)
			return DySqrtDown(d), DySqrtUp(d)
		}
		if low, high, ok := dvWordsSqrtBounds(w, width, exp); ok {
			return low, high
		}
	}
	d := DvDot(v, v)
	return DySqrtDown(d), DySqrtUp(d)
}

// dvWordsSqrtBounds repeats the two directed walks from the same seed. The
// first exact comparison settles one bound; only the other needs to step.
func dvWordsSqrtBounds(w []uint64, width, exp int) (float64, float64, bool) {
	f, ok := ratioSqrtSeedLeading(wordsLeading64(w, width), 1<<63, width+exp)
	if !ok {
		return 0, 0, false
	}
	if isNonFinite(f) {
		f = math.MaxFloat64
	}
	c, ok := cmpSquareWords(f, w, width, exp)
	if !ok {
		return 0, 0, false
	}
	if c == 0 {
		return f, f, true
	}
	if c < 0 {
		low := f
		for range SqrtAdjustLimit - 1 {
			f = NextUp(f)
			if isNonFinite(f) {
				return low, f, true
			}
			c, ok = cmpSquareWords(f, w, width, exp)
			if !ok {
				return 0, 0, false
			}
			if c >= 0 {
				return low, f, true
			}
		}
		return low, math.Inf(1), true
	}
	high := f
	for range SqrtAdjustLimit - 1 {
		f = NextDown(f)
		c, ok = cmpSquareWords(f, w, width, exp)
		if !ok {
			return 0, 0, false
		}
		if c <= 0 {
			return f, high, true
		}
	}
	return 0, high, true
}

// wordsField128 returns bits [at, at+128) of the integer w as two words.
func wordsField128(w []uint64, at int) (uint64, uint64) {
	word := func(i int) uint64 {
		if i < len(w) {
			return w[i]
		}
		return 0
	}
	i, off := at/64, uint(at%64)
	if off == 0 {
		return word(i), word(i + 1)
	}
	lo := word(i)>>off | word(i+1)<<(64-off)
	hi := word(i+1)>>off | word(i+2)<<(64-off)
	return lo, hi
}

// dvWordsSqrtUp is DySqrtUp for d = w·2^exp, a positive value whose reduced
// mantissa is wider than 127 bits — one a Dyadic holds in a big.Int — with
// width w's bit length. DySqrtSeed seeds such a value through RatioSqrtSeed
// (n = the reduced mantissa, d = 1, denomExp = −its exponent), which reads
// only n's leading 64 bits and n.BitLen() + its exponent; both are properties
// of the value, so w's own leading 64 bits and width + exp give the same seed.
// false is returned exactly where RatioSqrtSeed declines, which DySqrtSeed
// answers through big.Float, and where the comparison cannot align the words.
func dvWordsSqrtUp(w []uint64, width, exp int) (float64, bool) {
	f, ok := ratioSqrtSeedLeading(wordsLeading64(w, width), 1<<63, width+exp)
	if !ok {
		return 0, false
	}
	if isNonFinite(f) {
		f = math.MaxFloat64
	}
	for range SqrtAdjustLimit {
		if isNonFinite(f) {
			return f, true
		}
		c, ok := cmpSquareWords(f, w, width, exp)
		if !ok {
			return 0, false
		}
		if c >= 0 {
			return f, true
		}
		f = NextUp(f)
	}
	return math.Inf(1), true
}

// wordsLeading64 is leading64 over machine words: the top 64 bits of the
// integer w of the given bit width, which is above 64.
func wordsLeading64(w []uint64, width int) uint64 {
	lo, _ := wordsField128(w, width-64)
	return lo
}

// cmpSquareWords compares f², for a finite f >= 0, against w·2^exp, a
// positive value of bit width width, returning -1, 0 or +1 the way
// DyCmp(DyMul(DyOf(f), DyOf(f)), d) does. Two values whose leading bits sit
// at different positions compare by position. Otherwise f's square is
// shifted onto w's exponent and the words compare from the top; false
// reports a square whose exponent lies below exp, which a value wider than
// f's 106-bit square cannot share a leading position with.
func cmpSquareWords(f float64, w []uint64, width, exp int) (int, bool) {
	if f == 0 {
		return -1, true
	}
	b := math.Float64bits(f)
	mag, fexp := b&(1<<52-1), int(b>>52&0x7ff)
	if fexp == 0 {
		fexp = -1074
	} else {
		mag |= 1 << 52
		fexp -= 1075
	}
	ph, pl := bits.Mul64(mag, mag)
	sexp := 2 * fexp
	if top, other := bitLen128(pl, ph)+sexp, width+exp; top != other {
		if top > other {
			return 1, true
		}
		return -1, true
	}
	s := sexp - exp
	if s < 0 {
		return 0, false
	}
	var sq [10]uint64
	ws, bs := s/64, uint(s%64)
	sq[ws] = pl << bs
	sq[ws+1] = ph << bs
	if bs != 0 {
		sq[ws+1] |= pl >> (64 - bs)
		sq[ws+2] = ph >> (64 - bs)
	}
	for i := (width - 1) / 64; i >= 0; i-- {
		x, y := sq[i], w[i]
		if x != y {
			if x > y {
				return 1, true
			}
			return -1, true
		}
	}
	return 0, true
}
