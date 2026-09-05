package decad

import (
	"math"
	"math/big"
)

// This file is the exact rational interval arithmetic the certified readings
// are proven in: an enclosure [lo, hi] over big.Rat, its four operations, and
// the transcendental enclosures (atan, atan2, and the pi constants) that
// cannot be stated as a single rational.
//
// Every constant here is a PROVEN enclosure rather than a rounded literal, so
// a value computed through these operations encloses the true answer no
// matter how the float arithmetic beside it rounded. moments_trig.go owns the
// sine/cosine enclosure of an exact turn, which is proven without ever
// comparing against pi and so does not belong here.

type ratInterval struct {
	lo *big.Rat
	hi *big.Rat
}

var (
	piLower = mustRatDecimal("3.141592653589793238462643383279502884197169399375105820974944592307816406286")
	piUpper = mustRatDecimal("3.141592653589793238462643383279502884197169399375105820974944592307816406287")
)

// quarterPiIv, halfPiIv and twoPiIv cache the pi-interval scalings every
// atan2Interval/atanPositiveInterval call and several circular brackets
// rebuild, each of which otherwise pays a GCD normalization on piLower/
// piUpper's ~250-bit operands per call. They are declared from intervalScale
// directly, never from their own accessor — that would be an initialization
// cycle.
var (
	quarterPiIv = intervalScale(interval(piLower, piUpper), big.NewRat(1, 4))
	halfPiIv    = intervalScale(interval(piLower, piUpper), big.NewRat(1, 2))
	twoPiIv     = intervalScale(interval(piLower, piUpper), big.NewRat(2, 1))
)

// quarterPiInterval, halfPiInterval and twoPiInterval hand out copies of the
// cached pi-multiple intervals above. They MUST go through interval, which
// copies both endpoints: ratInterval holds pointers, so returning the cached
// value directly would let a caller mutate a package-level constant.
func quarterPiInterval() ratInterval { return interval(quarterPiIv.lo, quarterPiIv.hi) }
func halfPiInterval() ratInterval    { return interval(halfPiIv.lo, halfPiIv.hi) }
func twoPiInterval() ratInterval     { return interval(twoPiIv.lo, twoPiIv.hi) }

func mustRatDecimal(value string) *big.Rat {
	out, ok := new(big.Rat).SetString(value)
	if !ok {
		panic("decad: invalid in-tree rational constant")
	}
	return out
}

func interval(lo, hi *big.Rat) ratInterval {
	return ratInterval{lo: new(big.Rat).Set(lo), hi: new(big.Rat).Set(hi)}
}

func intervalAdd(a, b ratInterval) ratInterval {
	return interval(new(big.Rat).Add(a.lo, b.lo), new(big.Rat).Add(a.hi, b.hi))
}

func intervalNeg(a ratInterval) ratInterval {
	return interval(new(big.Rat).Neg(a.hi), new(big.Rat).Neg(a.lo))
}

func intervalSub(a, b ratInterval) ratInterval {
	return intervalAdd(a, intervalNeg(b))
}

func intervalScale(a ratInterval, scale *big.Rat) ratInterval {
	if scale.Sign() < 0 {
		return interval(
			new(big.Rat).Mul(a.hi, scale),
			new(big.Rat).Mul(a.lo, scale),
		)
	}
	return interval(
		new(big.Rat).Mul(a.lo, scale),
		new(big.Rat).Mul(a.hi, scale),
	)
}

func pointInterval(value *big.Rat) ratInterval {
	return interval(value, value)
}

// intervalMul multiplies two rational-bounded intervals by their four corner
// products, the general bound that assumes nothing about either interval's
// sign — what a radius bracket times a swept-angle bracket needs, since
// neither is known to be sign-fixed at the call site.
func intervalMul(a, b ratInterval) ratInterval {
	corners := [4]*big.Rat{
		new(big.Rat).Mul(a.lo, b.lo),
		new(big.Rat).Mul(a.lo, b.hi),
		new(big.Rat).Mul(a.hi, b.lo),
		new(big.Rat).Mul(a.hi, b.hi),
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
	return interval(lo, hi)
}

func intervalFloatError(a ratInterval, held float64) float64 {
	return math.Max(
		rationalFloatError(a.lo, held),
		rationalFloatError(a.hi, held),
	)
}

// fixedMulDown multiplies two non-negative fixed-point values and rounds the
// 2*trigFixedBits-wide product down to trigFixedBits, an inward (floor)
// rescale.
func fixedMulDown(a, b *big.Int) *big.Int {
	p := new(big.Int).Mul(a, b)
	return p.Rsh(p, trigFixedBits)
}

// fixedMulUp multiplies two non-negative fixed-point values and rounds the
// product up, fixedMulDown's outward (ceiling) mirror.
func fixedMulUp(a, b *big.Int) *big.Int {
	p := new(big.Int).Mul(a, b)
	p.Add(p, new(big.Int).Sub(trigFixedOne, big.NewInt(1)))
	return p.Rsh(p, trigFixedBits)
}

// fixedDivDown divides a non-negative fixed-point value by a positive
// integer, rounding down (Quo truncates toward zero, which is floor for a
// non-negative dividend).
func fixedDivDown(a *big.Int, d int64) *big.Int { return new(big.Int).Quo(a, big.NewInt(d)) }

// fixedDivUp is fixedDivDown's outward (ceiling) mirror.
func fixedDivUp(a *big.Int, d int64) *big.Int {
	q := new(big.Int).Add(a, big.NewInt(d-1))
	return q.Quo(q, big.NewInt(d))
}

// atanSmallInterval bounds atan(x) for |x| <= 1/2, evaluating the same
// 64-term alternating Maclaurin series moments_trig.go's trigFixedSeries
// uses for sin/cos, on the same fixed-point 2^-trigFixedBits grid, rather
// than over big.Rat: a big.Rat pays a GCD normalization per operation, which
// dominates cost at this series' 10^4-bit numerators.
//
// The negative case folds to the positive one first (intervalNeg), so every
// operand below is non-negative and Rsh/Quo's toward-zero rounding is floor.
// x is enclosed by xLo <= x*2^P <= xHi (fixedFloor/fixedCeil). By induction,
// carrying a lower chain (powerLo, rounded down at every step) and an upper
// chain (powerHi, rounded up at every step): powerLo_n <= x^(2n+1)*2^P <=
// powerHi_n for every n, so termLo_n <= (x^(2n+1)/(2n+1))*2^P <= termHi_n.
// The lower accumulator adds termLo on even n and subtracts termHi on odd n;
// the upper accumulator mirrors it (adds termHi, subtracts termLo) — each
// choice is the one that can only push its own side outward, never inward —
// so after all 64 terms, lo <= S_64(x)*2^P <= hi where S_64 is the 64-term
// partial sum.
//
// On [0, 1/2] the terms strictly decrease in magnitude, and the last summed
// term (n = 63) is subtracted, so the alternating-series remainder theorem
// gives S_64(x) <= atan(x) <= S_64(x) + x^129/129. The loop's final powerHi
// is the outward enclosure of x^129*2^P, and fixedDivUp(powerHi, 129) charges
// that remainder without narrowing it.
//
// Each power step's outward rescale (fixedMulUp against x2Hi, whose own
// factor has magnitude <=1) and each term's outward division together add at
// most 3 grid units of extra width per step (1 from the multiply's ceiling,
// 1 from the shared x2Hi rounding carried through a <=1-magnitude factor, 1
// from the divide's ceiling) — the same accounting trigFixedSeries's own
// comment uses. Over 64 terms that is under 64*3 = 192 < 2^8 units, i.e.
// under 2^-(trigFixedBits-8) = 2^-192, well inside the 2^-187 budget this
// function publishes and far below the 2^-136 series remainder already
// dominating the bound at x = 1/2.
func atanSmallInterval(x *big.Rat) ratInterval {
	if x.Sign() < 0 {
		return intervalNeg(atanSmallInterval(new(big.Rat).Neg(x)))
	}
	xLo, xHi := fixedFloor(x), fixedCeil(x)
	x2Lo := fixedMulDown(xLo, xLo)
	x2Hi := fixedMulUp(xHi, xHi)
	powerLo, powerHi := new(big.Int).Set(xLo), new(big.Int).Set(xHi)
	lo, hi := new(big.Int), new(big.Int)
	for n := range 64 {
		d := int64(2*n + 1)
		tLo := fixedDivDown(powerLo, d)
		tHi := fixedDivUp(powerHi, d)
		if n%2 == 0 {
			lo.Add(lo, tLo)
			hi.Add(hi, tHi)
		} else {
			lo.Sub(lo, tHi)
			hi.Sub(hi, tLo)
		}
		powerLo = fixedMulDown(powerLo, x2Lo)
		powerHi = fixedMulUp(powerHi, x2Hi)
	}
	hi.Add(hi, fixedDivUp(powerHi, 129))
	return interval(fixedToRat(lo), fixedToRat(hi))
}

func atanPositiveInterval(x *big.Rat) ratInterval {
	if x.Cmp(big.NewRat(1, 2)) <= 0 {
		return atanSmallInterval(x)
	}
	// atan(x) = π/4 + atan((x-1)/(x+1)); for x in (1/2,1], the transformed
	// argument lies in [-1/3,0), inside the fast alternating-series range.
	q := new(big.Rat).Quo(
		new(big.Rat).Sub(x, big.NewRat(1, 1)),
		new(big.Rat).Add(x, big.NewRat(1, 1)),
	)
	return intervalAdd(quarterPiInterval(), atanSmallInterval(q))
}

func atan2Interval(y, x *big.Rat, negativeZeroY bool) ratInterval {
	zero := new(big.Rat)
	if x.Sign() == 0 {
		halfPi := halfPiInterval()
		if y.Sign() < 0 {
			return intervalNeg(halfPi)
		}
		return halfPi
	}
	if y.Sign() == 0 {
		if x.Sign() < 0 {
			if negativeZeroY {
				return intervalNeg(interval(piLower, piUpper))
			}
			return interval(piLower, piUpper)
		}
		return pointInterval(zero)
	}
	ax, ay := new(big.Rat).Abs(x), new(big.Rat).Abs(y)
	var base ratInterval
	if ay.Cmp(ax) <= 0 {
		base = atanPositiveInterval(new(big.Rat).Quo(ay, ax))
	} else {
		base = intervalSub(halfPiInterval(), atanPositiveInterval(new(big.Rat).Quo(ax, ay)))
	}
	switch {
	case x.Sign() > 0 && y.Sign() > 0:
		return base
	case x.Sign() > 0 && y.Sign() < 0:
		return intervalNeg(base)
	case x.Sign() < 0 && y.Sign() > 0:
		return intervalSub(interval(piLower, piUpper), base)
	default:
		return intervalAdd(intervalNeg(interval(piLower, piUpper)), base)
	}
}
