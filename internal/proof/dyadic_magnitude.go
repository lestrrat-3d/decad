package proof

import "math/bits"

// dyMagnitudeBits reads 64 bits of a mantissa's magnitude starting at offset.
// Positions below bit zero and beyond the mantissa are zero. Quotient and
// comparison read these immutable words without allocating a shifted copy.
func dyMagnitudeBits(d Dyadic, offset int) uint64 {
	if offset < 0 {
		if offset <= -64 {
			return 0
		}
		return dyMagnitudeLimb(d, 0) << uint(-offset)
	}
	index, shift := offset/64, uint(offset%64)
	word := dyMagnitudeLimb(d, index) >> shift
	if shift != 0 {
		word |= dyMagnitudeLimb(d, index+1) << (64 - shift)
	}
	return word
}

func dyMagnitudeLimb(d Dyadic, index int) uint64 {
	if d.big == nil {
		switch index {
		case 0:
			return d.lo
		case 1:
			return d.hi &^ dySign
		default:
			return 0
		}
	}
	words := d.big.Bits()
	if bits.UintSize == 64 {
		if index < len(words) {
			return uint64(words[index])
		}
		return 0
	}
	index *= 2
	if index >= len(words) {
		return 0
	}
	word := uint64(words[index])
	if index+1 < len(words) {
		word |= uint64(words[index+1]) << 32
	}
	return word
}
