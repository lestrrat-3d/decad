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
	// value = frac × 2^exp exactly, frac in [0.5, 1) with at most 53
	// significant bits, so frac × 2^53 is an exact signed integer. Frexp is
	// exact for a subnormal too.
	frac, exp := math.Frexp(value)
	mant := int64(frac * (1 << 53))
	exp -= 53
	// A negative mantissa has the same trailing zeros as its magnitude, so the
	// arithmetic shift leaves it odd with its sign intact.
	tz := bits.TrailingZeros64(uint64(mant))
	mant >>= tz
	exp += tz
	r := new(big.Rat).SetInt64(mant)
	if exp >= 0 {
		r.Num().Lsh(r.Num(), uint(exp))
	} else {
		r.Denom().Lsh(r.Denom(), uint(-exp))
	}
	return r
}

// RationalFloatError returns |exact-held| rounded upward.
func RationalFloatError(exact *big.Rat, held float64) float64 {
	heldRat := FloatRat(held)
	if exact == nil || heldRat == nil {
		return math.Inf(1)
	}
	d := new(big.Rat).Sub(exact, heldRat)
	d.Abs(d)
	out, exactFloat := d.Float64()
	// Float64 can report exact even when a positive subnormal fraction flushed
	// to zero. The rational sign is the proof that this bound cannot be zero.
	if d.Sign() > 0 && out == 0 {
		return math.SmallestNonzeroFloat64
	}
	if !exactFloat {
		out = math.Nextafter(out, math.Inf(1))
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
		return math.Nextafter(value, math.Inf(1))
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
