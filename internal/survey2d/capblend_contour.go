package survey2d

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/proofbound"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
)

// IntervalQuo divides two intervals. ok is false where the divisor straddles
// zero, since the quotient is then unbounded and no box encloses it.
func IntervalQuo(a, b proofbound.RatInterval) (proofbound.RatInterval, bool) {
	if b.Lo.Sign() <= 0 && b.Hi.Sign() >= 0 {
		return proofbound.RatInterval{}, false
	}
	corners := [4]*big.Rat{
		new(big.Rat).Quo(a.Lo, b.Lo),
		new(big.Rat).Quo(a.Lo, b.Hi),
		new(big.Rat).Quo(a.Hi, b.Lo),
		new(big.Rat).Quo(a.Hi, b.Hi),
	}
	lo, hi := corners[0], corners[0]
	for _, c := range corners[1:] {
		if c.Cmp(lo) < 0 {
			lo = c
		}
		if c.Cmp(hi) > 0 {
			hi = c
		}
	}
	return proofbound.Interval(lo, hi), true
}

// IntervalSquare is x² over an interval. It is not proofbound.IntervalMul(a, a): the
// corner products of an interval straddling zero put a NEGATIVE value at the
// low end, and a square never takes one.
func IntervalSquare(a proofbound.RatInterval) proofbound.RatInterval {
	lo2 := new(big.Rat).Mul(a.Lo, a.Lo)
	hi2 := new(big.Rat).Mul(a.Hi, a.Hi)
	if a.Lo.Sign() >= 0 {
		return proofbound.Interval(lo2, hi2)
	}
	if a.Hi.Sign() <= 0 {
		return proofbound.Interval(hi2, lo2)
	}
	hi := lo2
	if hi2.Cmp(hi) > 0 {
		hi = hi2
	}
	return proofbound.Interval(new(big.Rat), hi)
}

// IntervalSqrt encloses the square root over a non-negative interval, each end
// rounded OUTWARD through internal/freeform/spline_length.go's exact-comparison bracket, so no
// platform's sqrt can narrow it. A lower end below zero is clamped to zero:
// the enclosure then covers the tangency the float discriminant reached for.
func IntervalSqrt(a proofbound.RatInterval) (proofbound.RatInterval, bool) {
	if a.Hi.Sign() < 0 {
		return proofbound.RatInterval{}, false
	}
	lo := 0.0
	if a.Lo.Sign() > 0 {
		lo = proofbound.RatSqrtDown(a.Lo)
	}
	hi := proofbound.RatSqrtUp(a.Hi)
	rlo, rhi := proofarith.FloatRat(lo), proofarith.FloatRat(hi)
	if rlo == nil || rhi == nil {
		return proofbound.RatInterval{}, false
	}
	return proofbound.Interval(rlo, rhi), true
}
