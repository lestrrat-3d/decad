package proof

import "math/bits"

// dvSquareInline sums three squares in machine words before creating the
// result's owned mantissa. This avoids allocating each wide square and each
// partial sum. Large mantissas or exponent gaps use the general dot product.
func dvSquareInline(v DyV3) (Dyadic, bool) {
	minExp := 0
	seen := false
	for _, component := range v {
		if component.big != nil {
			return Dyadic{}, false
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
		return Dyadic{}, true
	}
	var sum [8]uint64
	wordCount := 0
	for _, component := range v {
		if component.IsZero() {
			continue
		}
		delta := component.exp - minExp
		if delta < 0 || delta > 255 {
			return Dyadic{}, false
		}
		// Each square is below 2^(2*bitLen). Two extra bits accommodate
		// the sum of all three, including a carry across a word boundary.
		width := 2*(delta+component.bitLen()) + 2
		if width > len(sum)*64 {
			return Dyadic{}, false
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
	return fromBig(bigOfWords(sum[:wordCount], false), 2*minExp), true
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
