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
	// Division is monotone in each endpoint once the signs are known. Pick
	// the two extreme corners directly instead of dividing all four pairs.
	var loNum, loDen, hiNum, hiDen *big.Rat
	if b.Lo.Sign() > 0 {
		switch {
		case a.Lo.Sign() >= 0:
			loNum, loDen, hiNum, hiDen = a.Lo, b.Hi, a.Hi, b.Lo
		case a.Hi.Sign() <= 0:
			loNum, loDen, hiNum, hiDen = a.Lo, b.Lo, a.Hi, b.Hi
		default:
			loNum, loDen, hiNum, hiDen = a.Lo, b.Lo, a.Hi, b.Lo
		}
	} else {
		switch {
		case a.Lo.Sign() >= 0:
			loNum, loDen, hiNum, hiDen = a.Hi, b.Hi, a.Lo, b.Lo
		case a.Hi.Sign() <= 0:
			loNum, loDen, hiNum, hiDen = a.Hi, b.Lo, a.Lo, b.Hi
		default:
			loNum, loDen, hiNum, hiDen = a.Hi, b.Hi, a.Lo, b.Hi
		}
	}
	return IntervalOwned(proofarith.QuoRat(new(big.Rat), loNum, loDen), proofarith.QuoRat(new(big.Rat), hiNum, hiDen)), true
}

// IntervalSquare is x² over an interval. It is not IntervalMul(a, a): the
// corner products of an interval straddling zero put a NEGATIVE value at the
// low end, and a square never takes one.
func IntervalSquare(a RatInterval) RatInterval {
	lo2 := proofarith.MulRat(new(big.Rat), a.Lo, a.Lo)
	hi2 := proofarith.MulRat(new(big.Rat), a.Hi, a.Hi)
	if a.Lo.Sign() >= 0 {
		return Interval(lo2, hi2)
	}
	if a.Hi.Sign() <= 0 {
		return Interval(hi2, lo2)
	}
	hi := lo2
	if proofarith.CmpRat(hi2, hi) > 0 {
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
	return IntervalOwned(proofarith.SubRat(new(big.Rat), a.Lo, w), proofarith.AddRat(new(big.Rat), a.Hi, w))
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
