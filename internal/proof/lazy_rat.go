package proof

import (
	"math"
	"math/big"
)

// LazyRat is an exact rational n/d with d > 0, NOT kept in lowest terms:
// each operation cancels its result's common power of two and nothing else.
// Its value is exact. Rat returns the lowest-terms big.Rat. Values are
// immutable: operations never write an operand's integers, so results may
// share them.
//
// LazyRat serves exact computations whose intermediate values are read only
// at a few points: the cross-bore cell proof, and proofbound.RatDistanceUp's
// one subtraction. Every value that leaves either goes through Rat, or
// through a rounding equal to the lowest-terms one (proofbound.LazyRatFloatUp),
// so a callee sees what big.Rat's own operations would have produced.
type LazyRat struct{ n, d *big.Int }

var lazyOne = big.NewInt(1)

// LazyOf views r as a LazyRat. r must not change afterwards.
func LazyOf(r *big.Rat) LazyRat { return LazyRat{n: r.Num(), d: r.Denom()} }

// lazyFrac takes ownership of n and d (d > 0), cancels their common power of
// two and returns n/d.
func lazyFrac(n, d *big.Int) LazyRat {
	if n.Sign() == 0 {
		return LazyRat{n: n, d: lazyOne}
	}
	if s := min(n.TrailingZeroBits(), d.TrailingZeroBits()); s > 0 {
		n.Rsh(n, s)
		d.Rsh(d, s)
	}
	return LazyRat{n: n, d: d}
}

func lazyAddSub(a, b LazyRat, sub bool) LazyRat {
	var n, d *big.Int
	if a.d.Cmp(b.d) == 0 {
		if sub {
			n = new(big.Int).Sub(a.n, b.n)
		} else {
			n = new(big.Int).Add(a.n, b.n)
		}
		d = new(big.Int).Set(a.d)
		return lazyFrac(n, d)
	}
	n = new(big.Int).Mul(a.n, b.d)
	m := new(big.Int).Mul(b.n, a.d)
	if sub {
		n.Sub(n, m)
	} else {
		n.Add(n, m)
	}
	return lazyFrac(n, m.Mul(a.d, b.d))
}

// LazyAdd is a+b.
func LazyAdd(a, b LazyRat) LazyRat { return lazyAddSub(a, b, false) }

// LazySub is a-b.
func LazySub(a, b LazyRat) LazyRat { return lazyAddSub(a, b, true) }

// LazyMul is a*b.
func LazyMul(a, b LazyRat) LazyRat {
	return lazyFrac(new(big.Int).Mul(a.n, b.n), new(big.Int).Mul(a.d, b.d))
}

// LazyQuo is a/b. b must be nonzero.
func LazyQuo(a, b LazyRat) LazyRat {
	n := new(big.Int).Mul(a.n, b.d)
	d := new(big.Int).Mul(a.d, b.n)
	if d.Sign() < 0 {
		n.Neg(n)
		d.Neg(d)
	}
	return lazyFrac(n, d)
}

// LazyNeg is -a.
func LazyNeg(a LazyRat) LazyRat { return LazyRat{n: new(big.Int).Neg(a.n), d: a.d} }

// LazyCmp compares a and b by value: -1, 0 or +1.
func LazyCmp(a, b LazyRat) int {
	if a.d.Cmp(b.d) == 0 {
		return a.n.Cmp(b.n)
	}
	sa, sb := a.n.Sign(), b.n.Sign()
	if sa != sb {
		if sa < sb {
			return -1
		}
		return 1
	}
	return new(big.Int).Mul(a.n, b.d).Cmp(new(big.Int).Mul(b.n, a.d))
}

// Sign is -1, 0 or +1 for the sign of x.
func (x LazyRat) Sign() int { return x.n.Sign() }

// Rat returns x in lowest terms as a new big.Rat.
func (x LazyRat) Rat() *big.Rat { return new(big.Rat).SetFrac(x.n, x.d) }

// Parts returns x's numerator and denominator, not in lowest terms. The
// caller must not write them.
func (x LazyRat) Parts() (*big.Int, *big.Int) { return x.n, x.d }

// Float64 is x's nearest float64 (ties to even) and whether it equals x,
// without normalising x. ok is false unless the rounded |x| lies in
// [2^-1021, 2^1023) (MantExp exponent in [-1020, 1023]), or x is zero;
// callers then use x.Rat() and the lowest-terms function.
func (x LazyRat) Float64() (float64, bool, bool) {
	if x.n.Sign() == 0 {
		return 0, true, true
	}
	var nf, df, q big.Float
	nf.SetInt(x.n)
	df.SetInt(x.d)
	q.SetPrec(53).SetMode(big.ToNearestEven).Quo(&nf, &df)
	if exp := q.MantExp(nil); exp < -1020 || exp > 1023 {
		return 0, false, false
	}
	f, _ := q.Float64()
	return f, q.Acc() == big.Exact, true
}

// LazyFloatError is RationalFloatError(x.Rat(), held): the difference is
// formed lazily and rounded by Float64; out of range it calls
// RationalFloatError(x.Rat(), held).
func LazyFloatError(x LazyRat, held float64) float64 {
	heldRat := FloatRat(held)
	if heldRat == nil {
		return math.Inf(1)
	}
	d := LazySub(x, LazyOf(heldRat))
	if d.Sign() < 0 {
		d = LazyNeg(d)
	}
	out, exact, ok := d.Float64()
	if !ok {
		return RationalFloatError(x.Rat(), held)
	}
	if d.Sign() > 0 && out == 0 {
		return math.SmallestNonzeroFloat64
	}
	if !exact {
		out = NextUp(out)
	}
	return out
}

// LazyInterval is RatInterval over LazyRat endpoints.
type LazyInterval struct{ Lo, Hi LazyRat }

// LazyIntervalOf views a's endpoints, which must not change afterwards.
func LazyIntervalOf(a RatInterval) LazyInterval { return LazyInterval{LazyOf(a.Lo), LazyOf(a.Hi)} }

// LazyPoint is the degenerate interval [r, r].
func LazyPoint(r *big.Rat) LazyInterval {
	v := LazyOf(r)
	return LazyInterval{v, v}
}

func lazyZeroInterval(a LazyInterval) bool { return a.Lo.Sign() == 0 && a.Hi.Sign() == 0 }

func lazyZero() LazyRat { return LazyRat{n: new(big.Int), d: lazyOne} }

// LazyIntervalAdd is AddInterval, zero shortcuts included.
func LazyIntervalAdd(a, b LazyInterval) LazyInterval {
	if lazyZeroInterval(a) {
		return b
	}
	if lazyZeroInterval(b) {
		return a
	}
	return LazyInterval{LazyAdd(a.Lo, b.Lo), LazyAdd(a.Hi, b.Hi)}
}

// LazyIntervalSub is SubInterval.
func LazyIntervalSub(a, b LazyInterval) LazyInterval {
	return LazyInterval{LazySub(a.Lo, b.Hi), LazySub(a.Hi, b.Lo)}
}

// LazyIntervalMul is MulInterval: the same sign branches, and LazyCmp where
// both intervals straddle zero.
func LazyIntervalMul(a, b LazyInterval) LazyInterval {
	if lazyZeroInterval(a) || lazyZeroInterval(b) {
		return LazyInterval{lazyZero(), lazyZero()}
	}
	m := LazyMul
	if a.Lo.Sign() >= 0 {
		switch {
		case b.Lo.Sign() >= 0:
			return LazyInterval{m(a.Lo, b.Lo), m(a.Hi, b.Hi)}
		case b.Hi.Sign() <= 0:
			return LazyInterval{m(a.Hi, b.Lo), m(a.Lo, b.Hi)}
		default:
			return LazyInterval{m(a.Hi, b.Lo), m(a.Hi, b.Hi)}
		}
	}
	if a.Hi.Sign() <= 0 {
		switch {
		case b.Lo.Sign() >= 0:
			return LazyInterval{m(a.Lo, b.Hi), m(a.Hi, b.Lo)}
		case b.Hi.Sign() <= 0:
			return LazyInterval{m(a.Hi, b.Hi), m(a.Lo, b.Lo)}
		default:
			return LazyInterval{m(a.Lo, b.Hi), m(a.Lo, b.Lo)}
		}
	}
	if b.Lo.Sign() >= 0 {
		return LazyInterval{m(a.Lo, b.Hi), m(a.Hi, b.Hi)}
	}
	if b.Hi.Sign() <= 0 {
		return LazyInterval{m(a.Hi, b.Lo), m(a.Lo, b.Lo)}
	}
	loA, loB := m(a.Lo, b.Hi), m(a.Hi, b.Lo)
	hiA, hiB := m(a.Lo, b.Lo), m(a.Hi, b.Hi)
	if LazyCmp(loB, loA) < 0 {
		loA = loB
	}
	if LazyCmp(hiB, hiA) > 0 {
		hiA = hiB
	}
	return LazyInterval{loA, hiA}
}

// LazyIntervalSquare is x² over an interval with proofbound.IntervalSquare's
// branches: a straddling interval has a zero low end.
func LazyIntervalSquare(a LazyInterval) LazyInterval {
	lo2, hi2 := LazyMul(a.Lo, a.Lo), LazyMul(a.Hi, a.Hi)
	if a.Lo.Sign() >= 0 {
		return LazyInterval{lo2, hi2}
	}
	if a.Hi.Sign() <= 0 {
		return LazyInterval{hi2, lo2}
	}
	hi := lo2
	if LazyCmp(hi2, hi) > 0 {
		hi = hi2
	}
	return LazyInterval{lazyZero(), hi}
}

// LazyIntervalQuo is proofbound.IntervalQuo: ok is false where the divisor
// reaches zero.
func LazyIntervalQuo(a, b LazyInterval) (LazyInterval, bool) {
	if b.Lo.Sign() <= 0 && b.Hi.Sign() >= 0 {
		return LazyInterval{}, false
	}
	var loNum, loDen, hiNum, hiDen LazyRat
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
	return LazyInterval{LazyQuo(loNum, loDen), LazyQuo(hiNum, hiDen)}, true
}

// LazyIntervalWiden is proofbound.IntervalWiden: Lo-w and Hi+w.
func LazyIntervalWiden(a LazyInterval, w LazyRat) LazyInterval {
	return LazyInterval{LazySub(a.Lo, w), LazyAdd(a.Hi, w)}
}
