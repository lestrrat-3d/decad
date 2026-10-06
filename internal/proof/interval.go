package proof

import (
	"math"
	"math/big"
)

// RatInterval encloses a rational value between two owned endpoints.
type RatInterval struct {
	Lo *big.Rat
	Hi *big.Rat
}

func Interval(lo, hi *big.Rat) RatInterval {
	return RatInterval{Lo: new(big.Rat).Set(lo), Hi: new(big.Rat).Set(hi)}
}

// OwnedInterval takes ownership of two freshly allocated endpoints. Callers
// must not pass endpoints that alias an input Interval or a cached value.
func OwnedInterval(lo, hi *big.Rat) RatInterval {
	return RatInterval{Lo: lo, Hi: hi}
}

func AddInterval(a, b RatInterval) RatInterval {
	return OwnedInterval(new(big.Rat).Add(a.Lo, b.Lo), new(big.Rat).Add(a.Hi, b.Hi))
}

func NegInterval(a RatInterval) RatInterval {
	return OwnedInterval(new(big.Rat).Neg(a.Hi), new(big.Rat).Neg(a.Lo))
}

func SubInterval(a, b RatInterval) RatInterval {
	return OwnedInterval(new(big.Rat).Sub(a.Lo, b.Hi), new(big.Rat).Sub(a.Hi, b.Lo))
}

func ScaleInterval(a RatInterval, scale *big.Rat) RatInterval {
	if scale.Sign() < 0 {
		return OwnedInterval(
			new(big.Rat).Mul(a.Hi, scale),
			new(big.Rat).Mul(a.Lo, scale),
		)
	}
	return OwnedInterval(
		new(big.Rat).Mul(a.Lo, scale),
		new(big.Rat).Mul(a.Hi, scale),
	)
}

func PointInterval(value *big.Rat) RatInterval {
	return Interval(value, value)
}

// MulInterval multiplies two rational-bounded intervals. Endpoint signs
// identify the two extreme products unless both intervals cross zero; that
// case compares four products. Both result endpoints are owned independently.
func MulInterval(a, b RatInterval) RatInterval {
	if a.Lo.Sign() >= 0 {
		switch {
		case b.Lo.Sign() >= 0:
			return OwnedInterval(new(big.Rat).Mul(a.Lo, b.Lo), new(big.Rat).Mul(a.Hi, b.Hi))
		case b.Hi.Sign() <= 0:
			return OwnedInterval(new(big.Rat).Mul(a.Hi, b.Lo), new(big.Rat).Mul(a.Lo, b.Hi))
		default:
			return OwnedInterval(new(big.Rat).Mul(a.Hi, b.Lo), new(big.Rat).Mul(a.Hi, b.Hi))
		}
	}
	if a.Hi.Sign() <= 0 {
		switch {
		case b.Lo.Sign() >= 0:
			return OwnedInterval(new(big.Rat).Mul(a.Lo, b.Hi), new(big.Rat).Mul(a.Hi, b.Lo))
		case b.Hi.Sign() <= 0:
			return OwnedInterval(new(big.Rat).Mul(a.Hi, b.Hi), new(big.Rat).Mul(a.Lo, b.Lo))
		default:
			return OwnedInterval(new(big.Rat).Mul(a.Lo, b.Hi), new(big.Rat).Mul(a.Lo, b.Lo))
		}
	}
	if b.Lo.Sign() >= 0 {
		return OwnedInterval(new(big.Rat).Mul(a.Lo, b.Hi), new(big.Rat).Mul(a.Hi, b.Hi))
	}
	if b.Hi.Sign() <= 0 {
		return OwnedInterval(new(big.Rat).Mul(a.Hi, b.Lo), new(big.Rat).Mul(a.Lo, b.Lo))
	}

	loA := new(big.Rat).Mul(a.Lo, b.Hi)
	loB := new(big.Rat).Mul(a.Hi, b.Lo)
	hiA := new(big.Rat).Mul(a.Lo, b.Lo)
	hiB := new(big.Rat).Mul(a.Hi, b.Hi)
	if loB.Cmp(loA) < 0 {
		loA = loB
	}
	if hiB.Cmp(hiA) > 0 {
		hiA = hiB
	}
	return OwnedInterval(loA, hiA)
}

// DotInterval3 encloses the dot product of two three-component interval
// vectors: the sum of the three componentwise interval products. Each
// component is enclosed separately, so a value shared by both operands is
// treated as two independent intervals.
func DotInterval3(a, b [3]RatInterval) RatInterval {
	sum := MulInterval(a[0], b[0])
	for axis := 1; axis < 3; axis++ {
		sum = AddInterval(sum, MulInterval(a[axis], b[axis]))
	}
	return sum
}

// CrossInterval3 encloses the cross product a×b of two three-component
// interval vectors, each component as the interval difference of two
// interval products.
func CrossInterval3(a, b [3]RatInterval) [3]RatInterval {
	var out [3]RatInterval
	for axis := range out {
		j, k := (axis+1)%3, (axis+2)%3
		out[axis] = SubInterval(MulInterval(a[j], b[k]), MulInterval(a[k], b[j]))
	}
	return out
}

func IntervalFloatError(a RatInterval, held float64) float64 {
	return math.Max(
		RationalFloatError(a.Lo, held),
		RationalFloatError(a.Hi, held),
	)
}

// The common-denominator form writes several rationals as integer numerators
// over one shared positive denominator. big.Rat reduces to lowest terms after
// every operation, a Lehmer GCD over the full numerator and denominator; an
// expression evaluated over numerators that share a denominator needs only
// integer multiply-adds, and a result converted back (SetFrac) reduces to the
// same lowest terms, so it is the exact rational, bit for bit, the big.Rat
// evaluation of the same expression produces.

// LcmInt is the least common multiple of two positive integers. It returns a
// itself when b is 1 or equal to a, and otherwise a fresh integer, never b,
// which may be a reference into a big.Rat that its owner later mutates.
// Callers never mutate the result.
func LcmInt(a, b *big.Int) *big.Int {
	if a.Cmp(b) == 0 || b.IsInt64() && b.Int64() == 1 {
		return a
	}
	if a.IsInt64() && a.Int64() == 1 {
		return new(big.Int).Set(b)
	}
	g := new(big.Int).GCD(nil, nil, a, b)
	out := new(big.Int).Quo(a, g)
	return out.Mul(out, b)
}

// CommonDenom is the least common multiple of the values' denominators.
func CommonDenom(values ...*big.Rat) *big.Int {
	den := big.NewInt(1)
	for _, value := range values {
		den = LcmInt(den, value.Denom())
	}
	return den
}

// ScaledNum is r·den, an exact integer for a positive den that r's
// denominator divides.
func ScaledNum(r *big.Rat, den *big.Int) *big.Int {
	out := new(big.Int).Quo(den, r.Denom())
	return out.Mul(out, r.Num())
}
