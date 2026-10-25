package proofbound

import (
	"math"
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
)

// RatAdd is the exact sum of values, zero for an empty list, in lowest terms.
// It folds with proofarith.AddRat, which returns big.Rat.Add's value without
// a normalising GCD when the operands' denominators are powers of two. The
// result is a fresh Rat that shares no memory with any operand.
func RatAdd(values ...*big.Rat) *big.Rat {
	out := new(big.Rat)
	for _, value := range values {
		proofarith.AddRat(out, out, value)
	}
	return storeIntDenominator(out, len(values))
}

// RatMul is the exact product of values, one for an empty list, in lowest
// terms. The fold starts from a copy of the first value and uses
// proofarith.MulRat, which returns big.Rat.Mul's value without a normalising
// GCD when the operands' denominators are powers of two. The result is a
// fresh Rat that shares no memory with any operand.
func RatMul(values ...*big.Rat) *big.Rat {
	if len(values) == 0 {
		return new(big.Rat).SetInt64(1)
	}
	out := new(big.Rat).Set(values[0])
	for _, value := range values[1:] {
		proofarith.MulRat(out, out, value)
	}
	return storeIntDenominator(out, len(values))
}

// storeIntDenominator gives an integer result a stored denominator of 1, as
// big.Rat's own operations leave it. AddRat, MulRat and RatFrac may leave a zero
// denominator unstored, so that Denom would return a fresh detached 1 on every
// call; big.Rat.Add and Mul always store it. n is the number of folded values:
// an empty RatAdd returns the zero Rat unchanged, as the legacy fold did.
func storeIntDenominator(out *big.Rat, n int) *big.Rat {
	if n > 0 && out.IsInt() {
		out.SetInt(out.Num())
	}
	return out
}

func RatMin(a, b *big.Rat) *big.Rat {
	if proofarith.CmpRat(a, b) <= 0 {
		return a
	}
	return b
}

func RatMax(a, b *big.Rat) *big.Rat {
	if proofarith.CmpRat(a, b) >= 0 {
		return a
	}
	return b
}

// RatAbsDiff is |r − f| rounded up to float64.
func RatAbsDiff(r *big.Rat, f float64) float64 {
	return proofarith.RationalFloatError(r, f)
}

// RatOf is the exact rational f denotes, in lowest terms, and false for a NaN
// or an infinity. A nonzero finite f lifts through proofarith.FloatRat, which
// returns SetFloat64's value, numerator, denominator and stored denominator
// without SetFloat64's normalising GCD. Zero keeps SetFloat64, whose zero
// stores its denominator where FloatRat's zero leaves it unstored.
func RatOf(f float64) (*big.Rat, bool) {
	if f != 0 && !IsNonFinite(f) {
		return proofarith.FloatRat(f), true
	}
	r := new(big.Rat)
	if r.SetFloat64(f) == nil {
		return nil, false
	}
	return r, true
}

// RatFloatDown and RatFloatUp are proven outward roundings of an exact
// rational to float64: the largest float64 at or below q, and the smallest
// at or above it. big.Rat.Float64() already rounds to nearest, which can
// land on EITHER side of q, so each checks its own direction against the
// float, compared against q as exact integers (proofarith.CmpFloatRat), and
// only steps once via Nextafter when that check fails — an exactly representable q comes back UNCHANGED, which
// is what keeps a candidate whose true value is exact from picking up a
// spurious width it never earned.
//
// A q past the float64 range has no such rounding: Float64 saturates to an
// infinity, which each returns unchanged rather than stepping to a finite
// neighbour it cannot prove. That infinity is a REFUSAL, never a bound, so no
// caller may fold it — freeform.FreeformExtremeFloats is the one conversion the fold
// runs, and it is what turns the saturation into freeform.ErrFreeformExtremeUnrepresentable.
func RatFloatDown(q *big.Rat) float64 {
	f, _ := proofarith.RatFloat64(q)
	if IsNonFinite(f) {
		return f
	}
	if proofarith.CmpFloatRat(f, q) <= 0 {
		return f
	}
	return proofarith.NextDown(f)
}

func RatFloatUp(q *big.Rat) float64 {
	f, _ := proofarith.RatFloat64(q)
	if IsNonFinite(f) {
		return f
	}
	if proofarith.CmpFloatRat(f, q) >= 0 {
		return f
	}
	return proofarith.NextUp(f)
}

// RatSqrtSeed approximates sqrt(q) for a positive rational at EVERY scale a
// recorded coordinate can reach. It is only a seed: the exact rational
// comparisons below decide each bound, so a better seed can only reduce false
// refusals and can never widen or invert the interval.
//
// Rounding q to a float64 first is what a seed must not do. A leg distance
// small enough to make q subnormal loses most of q's significand, and one large
// enough to make q overflow loses q entirely — either way the seed lands far
// more than SqrtAdjustLimit ulps from the true root, the walk exhausts, and a
// perfectly valid curve is refused. Scaling instead is exact: split q into
// mantissa and binary exponent, force the exponent EVEN so half of it is an
// integer, take the square root of a mantissa that always sits in [0.5, 2), and
// scale the result back by a power of two, which moves no significand bit.
func RatSqrtSeed(q *big.Rat) float64 {
	if seed, ok := proofarith.RatSqrtSeed(q); ok {
		return seed
	}
	mant := new(big.Float).SetPrec(64)
	exp := new(big.Float).SetPrec(64).SetRat(q).MantExp(mant)
	if exp%2 != 0 {
		// Halving an odd exponent is not an integer, so shift one power of two
		// into the mantissa: [0.5, 1) becomes [1, 2), which still roots cleanly.
		exp--
		mant.SetMantExp(mant, 1)
	}
	m, _ := mant.Float64()
	return math.Ldexp(math.Sqrt(m), exp/2)
}

// RatSqrtDown returns a float f with f*f <= q, proven by exact comparison. The
// float sqrt seeds the answer; the exact test decides it, so no platform's
// sqrt accuracy can widen or invert the bracket.
func RatSqrtDown(q *big.Rat) float64 {
	if q.Sign() <= 0 {
		return 0
	}
	f := RatSqrtSeed(q)
	if IsNonFinite(f) {
		// sqrt(q) is at or beyond the top of the range, so the largest float
		// there is starts the walk; the exact test still decides it.
		f = math.MaxFloat64
	}
	for range SqrtAdjustLimit {
		if RatSquareAtMost(f, q) {
			return f
		}
		f = math.Nextafter(f, 0)
	}
	return 0
}

// RatSqrtUp returns a float f with f*f >= q, proven by exact comparison. It
// returns +Inf only where sqrt(q) genuinely exceeds MaxFloat64, which
// freeform.FreeformArcLength refuses as R15 rather than report.
func RatSqrtUp(q *big.Rat) float64 {
	if q.Sign() <= 0 {
		return 0
	}
	f := RatSqrtSeed(q)
	if IsNonFinite(f) {
		f = math.MaxFloat64
	}
	for range SqrtAdjustLimit {
		// Stop at the first float that is not finite or whose square is not
		// below q. One exact comparison decides it.
		if c, ok := ratSquareCmp(f, q); !ok || c >= 0 {
			return f
		}
		f = proofarith.NextUp(f)
	}
	return math.Inf(1)
}

// The exact arithmetic package owns the shared directed-rounding walk limit.
const SqrtAdjustLimit = proofarith.SqrtAdjustLimit

// RatSquareAtMost reports whether f*f <= q, exactly, and is false for a
// non-finite f.
func RatSquareAtMost(f float64, q *big.Rat) bool {
	c, ok := ratSquareCmp(f, q)
	return ok && c <= 0
}

// RatSquareEquals reports whether f*f == q, exactly, and is false for a
// non-finite f.
func RatSquareEquals(f float64, q *big.Rat) bool {
	c, ok := ratSquareCmp(f, q)
	return ok && c == 0
}

// ratSquareCmp returns the sign of f*f - q, exactly, and false for a
// non-finite f.
//
// A finite f is m·2^e with m an integer below 2^53. math.Frexp gives
// |f| = frac·2^x with frac in [0.5, 1) (subnormals included), and
// m = frac·2^53 is an integer because frac has a 53-bit significand. So
// e = x-53 and f*f = m²·2^(2e); the sign of f does not matter. Write q = n/d
// with d > 0, which big.Rat guarantees. Multiplying f*f - q by d > 0 keeps its
// sign, so sign(f*f - q) = sign(m²·d·2^(2e) - n). When 2e >= 0 the power of
// two shifts the left side. When 2e < 0, multiplying through by 2^(-2e) > 0
// moves it to n instead. Either way both sides are integers, and one big.Int
// comparison gives the sign that comparing the big.Rat square of f with q
// gave, with no big.Rat product and no GCD. A zero f gives m = 0, so the
// comparison is 0 against n·2^106, the sign of -q, as the square of the zero
// Rat gave.
func ratSquareCmp(f float64, q *big.Rat) (int, bool) {
	if math.IsNaN(f) || math.IsInf(f, 0) {
		return 0, false
	}
	frac, exp := math.Frexp(math.Abs(f))
	m := new(big.Int).SetInt64(int64(frac * (1 << 53)))
	e2 := 2 * (exp - 53)
	lhs := m.Mul(m, m)
	lhs.Mul(lhs, q.Denom())
	if e2 >= 0 {
		lhs.Lsh(lhs, uint(e2))
		return lhs.Cmp(q.Num()), true
	}
	return lhs.Cmp(new(big.Int).Lsh(q.Num(), uint(-e2))), true
}
