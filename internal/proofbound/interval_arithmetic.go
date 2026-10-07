package proofbound

import (
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
)

// IntervalQuo divides two intervals. ok is false where the divisor straddles
// zero, since the quotient is then unbounded and no box encloses it.
func IntervalQuo(a, b RatInterval) (RatInterval, bool) {
	if b.Lo.Sign() <= 0 && b.Hi.Sign() >= 0 {
		return RatInterval{}, false
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
	return Interval(lo, hi), true
}

// IntervalSquare is x² over an interval. It is not IntervalMul(a, a): the
// corner products of an interval straddling zero put a NEGATIVE value at the
// low end, and a square never takes one.
func IntervalSquare(a RatInterval) RatInterval {
	lo2 := new(big.Rat).Mul(a.Lo, a.Lo)
	hi2 := new(big.Rat).Mul(a.Hi, a.Hi)
	if a.Lo.Sign() >= 0 {
		return Interval(lo2, hi2)
	}
	if a.Hi.Sign() <= 0 {
		return Interval(hi2, lo2)
	}
	hi := lo2
	if hi2.Cmp(hi) > 0 {
		hi = hi2
	}
	return Interval(new(big.Rat), hi)
}

// IntervalSqrt encloses the square root over a non-negative interval, each end
// rounded OUTWARD through internal/proofbound/rational.go's
// exact-comparison bracket, so no
// platform's sqrt can narrow it. A lower end below zero is clamped to zero:
// the enclosure then covers the tangency the float discriminant reached for.
func IntervalSqrt(a RatInterval) (RatInterval, bool) {
	if a.Hi.Sign() < 0 {
		return RatInterval{}, false
	}
	lo := 0.0
	if a.Lo.Sign() > 0 {
		lo = RatSqrtDown(a.Lo)
	}
	hi := RatSqrtUp(a.Hi)
	rlo, rhi := proofarith.FloatRat(lo), proofarith.FloatRat(hi)
	if rlo == nil || rhi == nil {
		return RatInterval{}, false
	}
	return Interval(rlo, rhi), true
}

// IntervalWiden grows an enclosure by a non-negative rational on both ends.
func IntervalWiden(a RatInterval, w *big.Rat) RatInterval {
	return Interval(new(big.Rat).Sub(a.Lo, w), new(big.Rat).Add(a.Hi, w))
}

// RatFloorGrid rounds a rational DOWN onto the 2⁻ˢʰⁱᶠᵗ grid, so the result is
// never above the input and sits within 2⁻ˢʰⁱᶠᵗ of it. Rounding down in one
// direction only is what lets the caller charge the whole gap from one end.
func RatFloorGrid(x *big.Rat, shift uint) *big.Rat {
	scale := new(big.Int).Lsh(big.NewInt(1), shift)
	num := new(big.Int).Mul(x.Num(), scale)
	// Denom is positive for every big.Rat, so Div is the floor.
	q := new(big.Int).Div(num, x.Denom())
	return new(big.Rat).SetFrac(q, scale)
}
