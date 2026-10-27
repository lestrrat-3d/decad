// Package proof contains exact arithmetic used to bound modeling results.
package proof

import (
	"math"
	"math/big"
	"math/bits"
)

// FloatRat lifts a held float64 to the exact rational it denotes, or answers
// nil for a NaN or an infinity, which denote no rational. It is
// big.Rat.SetFloat64's value in SetFloat64's reduced form, built without the
// GCD SetFloat64 runs to normalise: a float is an integer mantissa times a
// power of two, so stripping the mantissa's trailing zero bits leaves an odd
// numerator over a power-of-two denominator, already in lowest terms. The
// shifts write through Rat.Num and Rat.Denom, which are documented references
// into the receiver; after SetInt64 the denominator is an initialised 1, so
// Denom never hands back the detached value it returns for a zero-value Rat.
func FloatRat(value float64) *big.Rat {
	if math.IsNaN(value) || math.IsInf(value, 0) {
		return nil
	}
	if value == 0 {
		return new(big.Rat)
	}
	mant, exp := floatOddMantissa(value)
	r := new(big.Rat).SetInt64(mant)
	if exp >= 0 {
		r.Num().Lsh(r.Num(), uint(exp))
	} else {
		r.Denom().Lsh(r.Denom(), uint(-exp))
	}
	return r
}

// floatOddMantissa splits a nonzero finite float into mant × 2^exp exactly,
// with mant odd and carrying value's sign. It reads the bit pattern: a
// subnormal has no implicit leading bit and the exponent of the smallest
// normal.
func floatOddMantissa(value float64) (int64, int) {
	raw := math.Float64bits(value)
	mant := int64(raw & (1<<52 - 1))
	exp := int(raw>>52) & 0x7ff
	if exp == 0 {
		exp = -1074
	} else {
		mant |= 1 << 52
		exp -= 1075
	}
	if raw>>63 != 0 {
		mant = -mant
	}
	// A negative mantissa has the same trailing zeros as its magnitude, so the
	// arithmetic shift leaves it odd with its sign intact.
	tz := bits.TrailingZeros64(uint64(mant))
	return mant >> tz, exp + tz
}

// CmpFloatRat returns -1, 0 or +1 as the finite float f is less than, equal to
// or greater than q. It decides the same sign as FloatRat(f).Cmp(q) from one
// integer product, one shift and one comparison, with no GCD: for f = m·2^e
// and q = n/d (d > 0), the sign of f − q is the sign of m·d·2^e − n. It shifts
// whichever side keeps both integers, so no fraction is formed. q is not
// modified. f must be finite and q non-nil.
func CmpFloatRat(f float64, q *big.Rat) int {
	qs := q.Sign()
	if f == 0 {
		return -qs
	}
	fs := 1
	if f < 0 {
		fs = -1
	}
	if fs != qs {
		// A nonzero f against a zero or opposite-signed q: the signs decide.
		return fs
	}
	mant, exp := floatOddMantissa(f)
	lhs := new(big.Int).Mul(big.NewInt(mant), q.Denom())
	rhs := q.Num()
	if exp >= 0 {
		return lhs.Lsh(lhs, uint(exp)).Cmp(rhs)
	}
	return lhs.Cmp(new(big.Int).Lsh(rhs, uint(-exp)))
}

// cmpFloatFrac is CmpFloatRat(f, q) for q = n/d, d > 0, a fraction that need
// not be in lowest terms: the sign of f − n/d is the sign of m·d·2^e − n for
// f = m·2^e, since d is positive. n and d are only read.
func cmpFloatFrac(f float64, n, d *big.Int) int {
	qs := n.Sign()
	if f == 0 {
		return -qs
	}
	fs := 1
	if f < 0 {
		fs = -1
	}
	if fs != qs {
		return fs
	}
	mant, exp := floatOddMantissa(f)
	lhs := new(big.Int).Mul(big.NewInt(mant), d)
	if exp >= 0 {
		return lhs.Lsh(lhs, uint(exp)).Cmp(n)
	}
	return lhs.Cmp(new(big.Int).Lsh(n, uint(-exp)))
}

// FracFloatDown is proofbound.RatFloatDown(n/d) for d > 0, n/d in any terms:
// the largest float at or below n/d, or the non-finite nearest float when n/d
// overflows. RatFloatDown reads q.Float64(), which quotientFloat64 returns bit
// for bit from any numerator and denominator of the same value, and keeps it
// when it is at or below q (CmpFloatRat, whose sign cmpFloatFrac decides from
// n and d), else steps one float down. Each step depends on the value alone,
// so no GCD normalises n/d first. n and d are only read.
func FracFloatDown(n, d *big.Int) float64 {
	f, _ := quotientFloat64(n, d)
	if math.IsNaN(f) || math.IsInf(f, 0) || cmpFloatFrac(f, n, d) <= 0 {
		return f
	}
	return NextDown(f)
}

// FracFloat64 is the nearest float64 to n/d, ties to even, for d > 0 and n/d
// in any terms: the bits big.Rat.Float64 returns for the same value, which
// quotientFloat64 returns from any numerator and denominator of that value
// (FracFloatDown). n and d are only read.
func FracFloat64(n, d *big.Int) float64 {
	f, _ := quotientFloat64(n, d)
	return f
}

// FracFloatUp is proofbound.RatFloatUp(n/d), FracFloatDown's mirror.
func FracFloatUp(n, d *big.Int) float64 {
	f, _ := quotientFloat64(n, d)
	if math.IsNaN(f) || math.IsInf(f, 0) || cmpFloatFrac(f, n, d) >= 0 {
		return f
	}
	return NextUp(f)
}

// RatFloat64 is q.Float64(): the nearest float64 (ties to even) and
// big.Rat.Float64's exactness flag, the same bits and the same flag for every
// q. A power-of-two denominator, which every FloatRat difference has, rounds by
// shifting the numerator; any other denominator calls q.Float64. q is only
// read and must not be nil.
func RatFloat64(q *big.Rat) (float64, bool) {
	den := q.Denom()
	if !isPositivePowerOfTwo(den) {
		return q.Float64()
	}
	return quotientFloat64(q.Num(), den)
}

// RationalFloatError returns |exact-held| rounded upward.
func RationalFloatError(exact *big.Rat, held float64) float64 {
	if exact == nil || !finite(held) {
		return math.Inf(1)
	}
	// Form an integer difference over a positive denominator and round that
	// quotient without reducing it. Dyadic inputs align their exponents;
	// other denominators cross-multiply the held float's odd mantissa.
	num, den := exact.Num(), exact.Denom()
	var difference, term, denominator big.Int
	if held != 0 {
		mantissa, exponent := floatOddMantissa(held)
		term.SetInt64(mantissa)
		if k, dyadic := denomExp(exact); dyadic {
			// n/2^k and m*2^e share the larger denominator. The sign
			// is held-exact; only its magnitude is published below.
			if shift := exponent + int(k); shift >= 0 {
				difference.Lsh(&term, uint(shift))
				difference.Sub(&difference, num)
			} else {
				difference.Lsh(num, uint(-shift))
				difference.Sub(&term, &difference)
				den = denominator.SetBit(&denominator, -exponent, 1)
			}
		} else {
			// For n/d and m*2^e, use denominator d when e>=0,
			// and d*2^-e otherwise. No normalization is needed.
			difference.Mul(&term, den)
			if exponent >= 0 {
				difference.Lsh(&difference, uint(exponent))
				difference.Sub(&difference, num)
			} else {
				term.Lsh(num, uint(-exponent))
				difference.Sub(&difference, &term)
				den = denominator.Lsh(den, uint(-exponent))
			}
		}
		if difference.Sign() == 0 {
			return 0
		}
		num = &difference
	}
	out, exactFloat := quotientFloat64(num, den)
	out = math.Abs(out)
	// Float64 can report exact even when a positive subnormal fraction flushed
	// to zero. The rational sign is the proof that this bound cannot be zero.
	if num.Sign() != 0 && out == 0 {
		return math.SmallestNonzeroFloat64
	}
	if !exactFloat {
		out = NextUp(out)
	}
	return out
}

// AddRoundError returns the exact rounding error of the held sum.
func AddRoundError(a, b, held float64) float64 {
	if finite(a) && finite(b) && finite(held) && held == a+b {
		// TwoSum recovers the exact residual of a finite rounded addition.
		// Keep the rational path when held came from a different expression.
		bPart := held - a
		aPart := held - bPart
		return math.Abs((a - aPart) + (b - bPart))
	}
	ra, rb := FloatRat(a), FloatRat(b)
	if ra == nil || rb == nil {
		return math.Inf(1)
	}
	return RationalFloatError(new(big.Rat).Add(ra, rb), held)
}

// MulRoundError returns the exact rounding error of the held product.
func MulRoundError(a, b, held float64) float64 {
	if finite(a) && finite(b) && finite(held) && held == a*b {
		if a == 0 || b == 0 {
			return 0
		}
		// Each operand's least bit is at or above Ilogb(x)-52. Under
		// this gate the exact product has no bits below the subnormal floor,
		// so FMA's product residual is exactly representable.
		if math.Ilogb(a)+math.Ilogb(b) >= -970 {
			return math.Abs(math.FMA(a, b, -held))
		}
	}
	ra, rb := FloatRat(a), FloatRat(b)
	if ra == nil || rb == nil {
		return math.Inf(1)
	}
	return RationalFloatError(new(big.Rat).Mul(ra, rb), held)
}

// ExactFloatSquare reports whether root·root == value EXACTLY. A true answer
// is a proof; a false answer proves nothing and only sends the caller to its
// exact rational comparison.
//
// The proof is math.FMA(root, root, -value) == 0 under mulRoundError's own
// gate. FMA forms root² − value exactly and rounds once, so the only way it
// can answer 0 for a nonzero residual is by rounding a residual of at most
// half the smallest subnormal down to zero. The gate rules that out. root's
// least significand bit weighs at least 2^(Ilogb(root)−52), so the exact root²
// is an integer multiple of 2^(2·Ilogb(root)−104), and 2·Ilogb(root) ≥ −970
// makes that at least 2^−1074. value is a finite float, which is always a
// multiple of 2^−1074. The residual is then an integer multiple of 2^−1074:
// either exactly zero, or at least 2^−1074 in magnitude, which is itself a
// float and which round-to-nearest therefore never sends to zero.
//
// Outside the gate the function answers false. A subnormal root has
// Ilogb ≤ −1023, and any value below about 2^−970 has a root under the gate,
// so neither is ever decided here. Without the gate, root = (1+2^−52)·2^−500
// against value = root*root has the exact residual 2^−1104, FMA flushes it to
// zero, and a wrong root would read exact. At the top of the range the product
// root*root may overflow; the cheap rounded check then fails and the answer is
// false, and FMA's exact residual is finite in any case. That rounded check
// only rejects: every exact square passes it, and FMA alone decides the rest.
// math.FMA is correctly rounded on every platform, hardware or software.
func ExactFloatSquare(root, value float64) bool {
	if !finite(root) || !finite(value) || value < 0 {
		return false
	}
	if root == 0 {
		return value == 0
	}
	if root*root != value || 2*math.Ilogb(root) < -970 {
		return false
	}
	return math.FMA(root, root, -value) == 0
}

// DivRoundError returns the exact rounding error of the held quotient.
func DivRoundError(a, b, held float64) float64 {
	ra, rb := FloatRat(a), FloatRat(b)
	if ra == nil || rb == nil || rb.Sign() == 0 {
		return math.Inf(1)
	}
	return RationalFloatError(new(big.Rat).Quo(ra, rb), held)
}

// ProvenUpRound rounds a proven positive bound outward. A positive value that
// flushed to zero must still publish a positive bound.
func ProvenUpRound(value float64) float64 {
	if value > 0 {
		return NextUp(value)
	}
	if value == 0 {
		return math.SmallestNonzeroFloat64
	}
	return value
}

// Radius3D turns a per-coordinate error into a bound on three-dimensional
// distance. The constant is sqrt(3) rounded upward.
func Radius3D(perCoord float64) float64 {
	if perCoord <= 0 {
		return 0
	}
	const sqrt3Up = 1.7320508075688774
	product := perCoord * sqrt3Up
	if product > 0 {
		return math.Nextafter(product, math.Inf(1))
	}
	return product
}

func finite(v float64) bool { return !math.IsNaN(v) && !math.IsInf(v, 0) }
