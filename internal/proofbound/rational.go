package proofbound

import (
	"math"
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
)

func RatAdd(values ...*big.Rat) *big.Rat {
	out := new(big.Rat)
	for _, value := range values {
		out.Add(out, value)
	}
	return out
}

func RatMul(values ...*big.Rat) *big.Rat {
	out := big.NewRat(1, 1)
	for _, value := range values {
		out.Mul(out, value)
	}
	return out
}

func RatMin(a, b *big.Rat) *big.Rat {
	if a.Cmp(b) <= 0 {
		return a
	}
	return b
}

func RatMax(a, b *big.Rat) *big.Rat {
	if a.Cmp(b) >= 0 {
		return a
	}
	return b
}

// RatAbsDiff is |r − f| rounded up to float64.
func RatAbsDiff(r *big.Rat, f float64) float64 {
	return proofarith.RationalFloatError(r, f)
}

func RatOf(f float64) (*big.Rat, bool) {
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
// float converted back to a rational and only steps once via Nextafter when
// that check fails — an exactly representable q comes back UNCHANGED, which
// is what keeps a candidate whose true value is exact from picking up a
// spurious width it never earned.
//
// A q past the float64 range has no such rounding: Float64 saturates to an
// infinity, which each returns unchanged rather than stepping to a finite
// neighbour it cannot prove. That infinity is a REFUSAL, never a bound, so no
// caller may fold it — freeformExtremeFloats is the one conversion the fold
// runs, and it is what turns the saturation into errFreeformExtremeUnrepresentable.
func RatFloatDown(q *big.Rat) float64 {
	f, _ := q.Float64()
	if IsNonFinite(f) {
		return f
	}
	if fr, ok := RatOf(f); ok && fr.Cmp(q) <= 0 {
		return f
	}
	return math.Nextafter(f, math.Inf(-1))
}

func RatFloatUp(q *big.Rat) float64 {
	f, _ := q.Float64()
	if IsNonFinite(f) {
		return f
	}
	if fr, ok := RatOf(f); ok && fr.Cmp(q) >= 0 {
		return f
	}
	return math.Nextafter(f, math.Inf(1))
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
// freeformArcLength refuses as R15 rather than report.
func RatSqrtUp(q *big.Rat) float64 {
	if q.Sign() <= 0 {
		return 0
	}
	f := RatSqrtSeed(q)
	if IsNonFinite(f) {
		f = math.MaxFloat64
	}
	for range SqrtAdjustLimit {
		if !RatSquareAtMost(f, q) || RatSquareEquals(f, q) {
			return f
		}
		f = math.Nextafter(f, math.Inf(1))
	}
	return math.Inf(1)
}

// The exact arithmetic package owns the shared directed-rounding walk limit.
const SqrtAdjustLimit = proofarith.SqrtAdjustLimit

func RatSquareAtMost(f float64, q *big.Rat) bool {
	square := proofarith.FloatRat(f)
	if square == nil {
		return false
	}
	square.Mul(square, square)
	return square.Cmp(q) <= 0
}

func RatSquareEquals(f float64, q *big.Rat) bool {
	square := proofarith.FloatRat(f)
	if square == nil {
		return false
	}
	square.Mul(square, square)
	return square.Cmp(q) == 0
}
