package proofbound

import (
	"math/big"
	"sync"
)

// This file encloses the square root, the natural logarithm and the inverse
// hyperbolic sine of exact rationals on the package's fixed-point grid
// (turn_trig.go's TrigFixedBits), for the coil's wall-area closed form
// (docs/helix-design.md §7.1). Every enclosure is built from exact integer
// operations with an explicit, charged truncation, so no platform's libm or
// FMA contraction can narrow it.

// lnSeriesTerms is the number of artanh series terms summed per evaluation.
// The argument w never exceeds 1/3, so the first omitted term is below
// (1/3)^(2·64+1) ≈ 2^-204, under the grid's own 2^-200 step.
const lnSeriesTerms = 64

// SqrtFixed encloses sqrt(q) for a rational q ≥ 0 on the 2^-TrigFixedBits
// grid. With N = floor(q·4^P) and s = isqrt(N), s² ≤ N ≤ q·4^P < N + 1 ≤
// (s + 1)², so s/2^P ≤ sqrt(q) < (s + 1)/2^P. When q·4^P is itself the
// perfect square s², the enclosure is the point s/2^P: a rational with an
// exact square root answers exactly. ok is false for a negative q.
func SqrtFixed(q *big.Rat) (RatInterval, bool) {
	if q.Sign() < 0 {
		return RatInterval{}, false
	}
	scaled := new(big.Int).Lsh(q.Num(), 2*TrigFixedBits)
	rem := new(big.Int)
	n := new(big.Int)
	n.QuoRem(scaled, q.Denom(), rem)
	s := new(big.Int).Sqrt(n)
	lo := FixedToRat(s)
	if rem.Sign() == 0 && new(big.Int).Mul(s, s).Cmp(n) == 0 {
		return Interval(lo, lo), true
	}
	return Interval(lo, FixedToRat(new(big.Int).Add(s, big.NewInt(1)))), true
}

// SqrtInterval encloses the square root over an interval whose upper end is
// non-negative: sqrt is increasing, so the two ends decide the enclosure. A
// lower end below zero is clamped to zero. ok is false for an interval
// wholly below zero.
func SqrtInterval(a RatInterval) (RatInterval, bool) {
	if a.Hi.Sign() < 0 {
		return RatInterval{}, false
	}
	lo := new(big.Rat)
	if a.Lo.Sign() > 0 {
		l, _ := SqrtFixed(a.Lo)
		lo = l.Lo
	}
	h, _ := SqrtFixed(a.Hi)
	return Interval(lo, h.Hi), true
}

// artanhFixed brackets 2·artanh(w) = 2·Σ w^(2n+1)/(2n+1) for a rational
// w in [0, 1/3] on the fixed-point grid. Every term is positive, so the lower
// chain rounds each power and quotient down and the upper chain rounds them
// up. The tail past lnSeriesTerms terms is at most
// w^(2K+1)/((2K+1)(1 − w²)), and w² ≤ 1/9 makes 1/(1 − w²) at most 9/8, so
// the upper chain adds powerHi·9/(8·(2K+1)) rounded up.
func artanhFixed(w *big.Rat) (*big.Int, *big.Int) {
	wLo, wHi := FixedFloor(w), FixedCeil(w)
	w2Lo := FixedMulDown(wLo, wLo)
	w2Hi := FixedMulUp(wHi, wHi)
	powerLo, powerHi := new(big.Int).Set(wLo), new(big.Int).Set(wHi)
	lo, hi := new(big.Int), new(big.Int)
	for n := range lnSeriesTerms {
		d := int64(2*n + 1)
		lo.Add(lo, FixedDivDown(powerLo, d))
		hi.Add(hi, FixedDivUp(powerHi, d))
		powerLo = FixedMulDown(powerLo, w2Lo)
		powerHi = FixedMulUp(powerHi, w2Hi)
	}
	tail := new(big.Int).Mul(powerHi, big.NewInt(9))
	hi.Add(hi, FixedDivUp(tail, 8*(2*lnSeriesTerms+1)))
	lo.Lsh(lo, 1)
	hi.Lsh(hi, 1)
	return lo, hi
}

var (
	ln2Once sync.Once
	ln2Iv   RatInterval
)

// Ln2Interval encloses ln 2 = 2·artanh(1/3).
func Ln2Interval() RatInterval {
	ln2Once.Do(func() {
		lo, hi := artanhFixed(big.NewRat(1, 3))
		ln2Iv = Interval(FixedToRat(lo), FixedToRat(hi))
	})
	return Interval(ln2Iv.Lo, ln2Iv.Hi)
}

// lnPoint encloses ln y for a rational y > 0. y = 2^e·m with m in [1, 2) is
// an exact decomposition, and ln y = e·ln 2 + 2·artanh((m − 1)/(m + 1)) with
// the artanh argument in [0, 1/3).
func lnPoint(y *big.Rat) RatInterval {
	e := y.Num().BitLen() - y.Denom().BitLen()
	m := new(big.Rat).Set(y)
	if e > 0 {
		m.Quo(m, new(big.Rat).SetInt(new(big.Int).Lsh(big.NewInt(1), uint(e))))
	} else if e < 0 {
		m.Mul(m, new(big.Rat).SetInt(new(big.Int).Lsh(big.NewInt(1), uint(-e))))
	}
	one := big.NewRat(1, 1)
	if m.Cmp(one) < 0 {
		m.Mul(m, big.NewRat(2, 1))
		e--
	}
	w := new(big.Rat).Quo(new(big.Rat).Sub(m, one), new(big.Rat).Add(m, one))
	lo, hi := artanhFixed(w)
	series := Interval(FixedToRat(lo), FixedToRat(hi))
	if e == 0 {
		return series
	}
	return IntervalAdd(IntervalScale(Ln2Interval(), big.NewRat(int64(e), 1)), series)
}

// LnInterval encloses the natural logarithm over an interval of positive
// rationals. ln is increasing, so the two ends decide the enclosure. ok is
// false when the lower end is not positive.
func LnInterval(x RatInterval) (RatInterval, bool) {
	if x.Lo.Sign() <= 0 {
		return RatInterval{}, false
	}
	return Interval(lnPoint(x.Lo).Lo, lnPoint(x.Hi).Hi), true
}

// asinhPoint encloses asinh x = ln(x + sqrt(x² + 1)) for a rational x. It is
// exactly zero at zero and odd, so a negative x reads its magnitude's
// enclosure negated.
func asinhPoint(x *big.Rat) RatInterval {
	switch x.Sign() {
	case 0:
		return PointInterval(new(big.Rat))
	case -1:
		return IntervalNeg(asinhPoint(new(big.Rat).Neg(x)))
	}
	sq := new(big.Rat).Mul(x, x)
	sq.Add(sq, big.NewRat(1, 1))
	root, _ := SqrtFixed(sq)
	lo := lnPoint(new(big.Rat).Add(x, root.Lo))
	hi := lnPoint(new(big.Rat).Add(x, root.Hi))
	return Interval(lo.Lo, hi.Hi)
}

// AsinhInterval encloses the inverse hyperbolic sine over an interval.
// asinh is increasing, so the two ends decide the enclosure.
func AsinhInterval(x RatInterval) RatInterval {
	return Interval(asinhPoint(x.Lo).Lo, asinhPoint(x.Hi).Hi)
}
