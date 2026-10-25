package proof

import (
	"math"
	"math/big"
	"math/bits"
)

// RatSqrtSeed is RatioSqrtSeed for a rational. Numerator and denominator
// references stay inside proof, which owns the read-only integer access.
func RatSqrtSeed(q *big.Rat) (float64, bool) {
	return RatioSqrtSeed(q.Num(), q.Denom(), 0)
}

// RatioSqrtSeed returns the 64-bit big.Float quotient seed for
// sqrt(n / (d * 2^denomExp)) when leading integer words determine its rounded
// mantissa. n and d must be positive. False asks the caller to use its full
// big.Float division; no approximate mantissa is accepted on that path.
func RatioSqrtSeed(n, d *big.Int, denomExp int) (float64, bool) {
	if n.Sign() <= 0 || d.Sign() <= 0 {
		return 0, false
	}
	num, den := leading64(n), leading64(d)
	exp := n.BitLen() - d.BitLen() + 1 - denomExp
	hi, lo := num>>1, num<<63
	if num < den {
		hi, lo = num, 0
		exp--
	}
	q, _ := bits.Div64(hi, lo, den)
	// Both prefixes lie in [2^63, 2^64). Their discarded tails are below
	// one. At either quotient scale, replacing those tails changes the
	// scaled exact quotient by less than four. Integer truncation and the
	// original 64-bit quotient rounding add less than two more units.
	// Eight units therefore enclose the original rounded quotient, even
	// when its binary normalization changes at a power of two.
	if q > ^uint64(0)-8 {
		return 0, false
	}
	lower, upper := float64(q-8), float64(q+8)
	if lower != upper {
		return 0, false
	}
	// Every possible 64-bit quotient rounds to this same binary64 mantissa.
	// Keep its exponent separate until after sqrt, including at underflow
	// and overflow. This is the full-division seed's scaling order.
	m := lower * 0x1p-64
	if exp%2 != 0 {
		exp--
		m *= 2
	}
	return math.Ldexp(math.Sqrt(m), exp/2), true
}

// leading64 is the top 64 bits of x, padded with low zeros for shorter
// positive integers. It reads at most three words on either Go word size.
func leading64(x *big.Int) uint64 {
	width := x.BitLen()
	if width <= 64 {
		return x.Uint64() << uint(64-width)
	}
	shift := uint(width - 64)
	i, off := shift/bits.UintSize, shift%bits.UintSize
	words := x.Bits()
	out := uint64(words[i]) >> off
	if bits.UintSize == 64 {
		if off != 0 {
			out |= uint64(words[i+1]) << (64 - off)
		}
		return out
	}
	out |= uint64(words[i+1]) << (32 - off)
	if off != 0 {
		out |= uint64(words[i+2]) << (64 - off)
	}
	return out
}
