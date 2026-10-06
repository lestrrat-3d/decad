package proofbound

import (
	"math"
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
)

// Moved from the root package's spline_length.go.

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
