package proof

import (
	"math/big"
	"math/bits"
)

// This file is the shared-denominator form of RatInterval's arithmetic, for a
// computation that reads all of its rational inputs before it computes: the
// island certificate (docs/multibody-dynamics-design.md §6.3).
//
// Every input is written once over one positive odd denominator D shared by
// the whole computation, as an integer numerator over a power of two (a
// Dyadic) and a power of D: the value n / D^k (SRat). A float64 input, the
// usual case, has an odd part of 1 in its denominator, so it is a Dyadic over
// D^0 and D is 1 unless some input's denominator has another odd factor (an
// exact driver velocity divides by a duration). A sum or a product of two
// such values is then Dyadic arithmetic on the numerators and an add of the
// powers of D, with no GCD: a numerator over D^j is moved to D^k, k > j, by
// a product with D^(k−j), which is the identity while D is 1. big.Rat
// reduces to lowest terms after every operation; here a value becomes a
// big.Rat (Rat) only where the computation publishes it, and that conversion
// reduces it to the same lowest terms. Every SInterval operation chooses its
// endpoints exactly as its RatInterval twin does, so a computation written
// over SInterval reaches the rationals and the decisions its RatInterval form
// reaches.
//
// A SharedDenom is built for one computation and is not safe for concurrent
// use: it caches the powers of D it has used.

// SharedDenom is the odd denominator D a computation's values share.
type SharedDenom struct {
	den     *big.Int // D; nil while D is 1
	dy      Dyadic   // D as a Dyadic
	powers  []Dyadic // powers[k] is D^k, filled as alignment needs them
	missing *big.Int // the lcm of the odd denominators Lift could not write over D
}

// SRat is the exact rational n / D^k over its computation's SharedDenom.
type SRat struct {
	n Dyadic
	k int
}

// SInterval encloses a value between two SRat endpoints.
type SInterval struct {
	Lo SRat
	Hi SRat
}

// SIVec3 is a three-component interval vector over a SharedDenom.
type SIVec3 [3]SInterval

// NewSharedDenom returns the shared denominator D = den, a positive odd
// integer; nil means 1.
func NewSharedDenom(den *big.Int) *SharedDenom {
	s := &SharedDenom{powers: []Dyadic{DyInt(1)}}
	if den == nil || den.IsInt64() && den.Int64() == 1 {
		return s
	}
	s.den = new(big.Int).Set(den)
	s.dy, _ = DyOfRat(new(big.Rat).SetInt(s.den))
	return s
}

// Lift writes the rational r over D: a power-of-two denominator over D^0, and
// any other one over D^1 when its odd part divides D. A denominator whose odd
// part does not divide D is recorded for Widen and lifts as zero, so a caller
// lifts every input first and computes only once Widen reports nothing
// missing.
func (s *SharedDenom) Lift(r *big.Rat) SRat {
	if d, ok := liftDyadic(r); ok {
		return SRat{n: d}
	}
	den := r.Denom()
	odd := new(big.Int).Rsh(den, den.TrailingZeroBits())
	if s.den != nil && new(big.Int).Rem(s.den, odd).Sign() == 0 {
		// r·D has the denominator 2^t that r's has after its odd part.
		d, _ := DyOfRat(new(big.Rat).Mul(r, new(big.Rat).SetInt(s.den)))
		return SRat{n: d, k: 1}
	}
	if s.missing == nil {
		s.missing = odd
	} else {
		s.missing = LcmInt(s.missing, odd)
	}
	return SRat{}
}

// liftDyadic is DyOfRat, reading a numerator that fits inline from its words
// rather than copying it into a big.Int first, and reading an integer's
// denominator as 1 without Rat.Denom, which allocates for a zero-value Rat.
func liftDyadic(r *big.Rat) (Dyadic, bool) {
	if r == nil {
		return Dyadic{}, false
	}
	var shift uint
	if !r.IsInt() {
		den := r.Denom()
		shift = den.TrailingZeroBits()
		if uint(den.BitLen()) != shift+1 {
			return Dyadic{}, false
		}
	}
	num := r.Num()
	if num.BitLen() > dyBits {
		return fromBig(new(big.Int).Set(num), -int(shift)), true
	}
	var lo, hi uint64
	words := num.Bits()
	if bits.UintSize == 64 {
		if len(words) > 0 {
			lo = uint64(words[0])
		}
		if len(words) > 1 {
			hi = uint64(words[1])
		}
	} else {
		var w [4]uint64
		for i, word := range words {
			w[i] = uint64(word)
		}
		lo, hi = w[0]|w[1]<<32, w[2]|w[3]<<32
	}
	return fromMag(lo, hi, num.Sign() < 0, -int(shift)), true
}

// Widen reports whether Lift met a denominator D does not cover and, when it
// did, returns a fresh SharedDenom over the lcm of D and every such
// denominator, over which those inputs lift.
func (s *SharedDenom) Widen() (*SharedDenom, bool) {
	if s.missing == nil {
		return s, false
	}
	den := s.missing
	if s.den != nil {
		den = LcmInt(s.den, s.missing)
	}
	return NewSharedDenom(den), true
}

// SFloat lifts a finite float64 over D^0, reporting false for a NaN or an
// infinity.
func SFloat(f float64) (SRat, bool) {
	d, ok := DyOf(f)
	return SRat{n: d}, ok
}

// SInt lifts an integer over D^0.
func SInt(v int64) SRat { return SRat{n: DyInt(v)} }

// Sign reports the value's sign, its numerator's.
func (x SRat) Sign() int { return x.n.Sign() }

// SNeg returns −x.
func SNeg(x SRat) SRat { return SRat{n: DyNeg(x.n), k: x.k} }

// SAbs returns |x|.
func SAbs(x SRat) SRat { return SRat{n: DyAbs(x.n), k: x.k} }

// SShift returns x·2^e.
func SShift(x SRat, e int) SRat { return SRat{n: DyShift(x.n, e), k: x.k} }

// power returns D^k.
func (s *SharedDenom) power(k int) Dyadic {
	for len(s.powers) <= k {
		s.powers = append(s.powers, DyMul(s.powers[len(s.powers)-1], s.dy))
	}
	return s.powers[k]
}

// at returns x's numerator over D^k, k at least x.k.
func (s *SharedDenom) at(x SRat, k int) Dyadic {
	if x.k == k {
		return x.n
	}
	return DyMul(x.n, s.power(k-x.k))
}

// Add returns a + b.
func (s *SharedDenom) Add(a, b SRat) SRat {
	switch {
	case a.k == b.k:
		return SRat{n: DyAdd(a.n, b.n), k: a.k}
	case b.n.IsZero():
		return a
	case a.n.IsZero():
		return b
	}
	k := max(a.k, b.k)
	return SRat{n: DyAdd(s.at(a, k), s.at(b, k)), k: k}
}

// Sub returns a − b.
func (s *SharedDenom) Sub(a, b SRat) SRat {
	switch {
	case a.k == b.k:
		return SRat{n: DySubScalar(a.n, b.n), k: a.k}
	case b.n.IsZero():
		return a
	case a.n.IsZero():
		return SNeg(b)
	}
	k := max(a.k, b.k)
	return SRat{n: DySubScalar(s.at(a, k), s.at(b, k)), k: k}
}

// Mul returns a·b: numerators multiply and powers of D add.
func (s *SharedDenom) Mul(a, b SRat) SRat {
	if a.n.IsZero() || b.n.IsZero() {
		return SRat{}
	}
	return SRat{n: DyMul(a.n, b.n), k: a.k + b.k}
}

// Cmp compares a against b the way big.Rat.Cmp does.
func (s *SharedDenom) Cmp(a, b SRat) int {
	if a.k == b.k {
		return DyCmp(a.n, b.n)
	}
	if sa, sb := a.n.Sign(), b.n.Sign(); sa != sb || sa == 0 {
		switch {
		case sa > sb:
			return 1
		case sa < sb:
			return -1
		}
		return 0
	}
	k := max(a.k, b.k)
	return DyCmp(s.at(a, k), s.at(b, k))
}

// Rat returns x as a fresh big.Rat in lowest terms.
func (s *SharedDenom) Rat(x SRat) *big.Rat {
	r := x.n.Rat()
	if x.k == 0 || r.Sign() == 0 {
		return r
	}
	return r.Quo(r, s.power(x.k).Rat())
}

// Float64 returns big.Rat.Float64 of x: the nearest float64 and whether it
// is exact.
func (s *SharedDenom) Float64(x SRat) (float64, bool) {
	if x.k == 0 {
		return x.n.Float64()
	}
	return s.Rat(x).Float64()
}

// LiftInterval writes a RatInterval over D.
func (s *SharedDenom) LiftInterval(iv RatInterval) SInterval {
	return SInterval{Lo: s.Lift(iv.Lo), Hi: s.Lift(iv.Hi)}
}

// LiftIVec3 writes a RatInterval vector over D.
func (s *SharedDenom) LiftIVec3(v [3]RatInterval) SIVec3 {
	return SIVec3{s.LiftInterval(v[0]), s.LiftInterval(v[1]), s.LiftInterval(v[2])}
}

// SPoint is the degenerate interval [x, x].
func SPoint(x SRat) SInterval { return SInterval{Lo: x, Hi: x} }

// SPoint3 is the degenerate interval vector at x.
func SPoint3(x [3]SRat) SIVec3 {
	x0, x1, x2 := x[0], x[1], x[2]
	return SIVec3{SPoint(x0), SPoint(x1), SPoint(x2)}
}

// AddI is AddInterval.
func (s *SharedDenom) AddI(a, b SInterval) SInterval {
	return SInterval{Lo: s.Add(a.Lo, b.Lo), Hi: s.Add(a.Hi, b.Hi)}
}

// SubI is SubInterval.
func (s *SharedDenom) SubI(a, b SInterval) SInterval {
	return SInterval{Lo: s.Sub(a.Lo, b.Hi), Hi: s.Sub(a.Hi, b.Lo)}
}

// SNegI is NegInterval.
func SNegI(a SInterval) SInterval { return SInterval{Lo: SNeg(a.Hi), Hi: SNeg(a.Lo)} }

// ScaleI is ScaleInterval.
func (s *SharedDenom) ScaleI(a SInterval, scale SRat) SInterval {
	if scale.Sign() < 0 {
		return SInterval{Lo: s.Mul(a.Hi, scale), Hi: s.Mul(a.Lo, scale)}
	}
	return SInterval{Lo: s.Mul(a.Lo, scale), Hi: s.Mul(a.Hi, scale)}
}

// MulI is MulInterval, choosing the same endpoint products by the same
// endpoint signs.
func (s *SharedDenom) MulI(a, b SInterval) SInterval {
	if a.Lo.Sign() >= 0 {
		switch {
		case b.Lo.Sign() >= 0:
			return SInterval{Lo: s.Mul(a.Lo, b.Lo), Hi: s.Mul(a.Hi, b.Hi)}
		case b.Hi.Sign() <= 0:
			return SInterval{Lo: s.Mul(a.Hi, b.Lo), Hi: s.Mul(a.Lo, b.Hi)}
		default:
			return SInterval{Lo: s.Mul(a.Hi, b.Lo), Hi: s.Mul(a.Hi, b.Hi)}
		}
	}
	if a.Hi.Sign() <= 0 {
		switch {
		case b.Lo.Sign() >= 0:
			return SInterval{Lo: s.Mul(a.Lo, b.Hi), Hi: s.Mul(a.Hi, b.Lo)}
		case b.Hi.Sign() <= 0:
			return SInterval{Lo: s.Mul(a.Hi, b.Hi), Hi: s.Mul(a.Lo, b.Lo)}
		default:
			return SInterval{Lo: s.Mul(a.Lo, b.Hi), Hi: s.Mul(a.Lo, b.Lo)}
		}
	}
	if b.Lo.Sign() >= 0 {
		return SInterval{Lo: s.Mul(a.Lo, b.Hi), Hi: s.Mul(a.Hi, b.Hi)}
	}
	if b.Hi.Sign() <= 0 {
		return SInterval{Lo: s.Mul(a.Hi, b.Lo), Hi: s.Mul(a.Lo, b.Lo)}
	}
	lo, loB := s.Mul(a.Lo, b.Hi), s.Mul(a.Hi, b.Lo)
	hi, hiB := s.Mul(a.Lo, b.Lo), s.Mul(a.Hi, b.Hi)
	if s.Cmp(loB, lo) < 0 {
		lo = loB
	}
	if s.Cmp(hiB, hi) > 0 {
		hi = hiB
	}
	return SInterval{Lo: lo, Hi: hi}
}

// Magnitude is the largest absolute value an interval admits.
func (s *SharedDenom) Magnitude(iv SInterval) SRat {
	lo, hi := SAbs(iv.Lo), SAbs(iv.Hi)
	if s.Cmp(lo, hi) > 0 {
		return lo
	}
	return hi
}

// The vector forms below copy each component into a named local before any
// call and index only by constants (see dyadic.go on stack arrays of Dyadic).

// AddI3 is the componentwise AddI.
func (s *SharedDenom) AddI3(a, b SIVec3) SIVec3 {
	a0, a1, a2 := a[0], a[1], a[2]
	b0, b1, b2 := b[0], b[1], b[2]
	return SIVec3{s.AddI(a0, b0), s.AddI(a1, b1), s.AddI(a2, b2)}
}

// SubI3 is the componentwise SubI.
func (s *SharedDenom) SubI3(a, b SIVec3) SIVec3 {
	a0, a1, a2 := a[0], a[1], a[2]
	b0, b1, b2 := b[0], b[1], b[2]
	return SIVec3{s.SubI(a0, b0), s.SubI(a1, b1), s.SubI(a2, b2)}
}

// ScaleI3 is the componentwise ScaleI.
func (s *SharedDenom) ScaleI3(a SIVec3, scale SRat) SIVec3 {
	a0, a1, a2 := a[0], a[1], a[2]
	return SIVec3{s.ScaleI(a0, scale), s.ScaleI(a1, scale), s.ScaleI(a2, scale)}
}

// DotI3 is DotInterval3: the three componentwise MulI products summed in
// component order.
func (s *SharedDenom) DotI3(a, b SIVec3) SInterval {
	a0, a1, a2 := a[0], a[1], a[2]
	b0, b1, b2 := b[0], b[1], b[2]
	sum := s.MulI(a0, b0)
	sum = s.AddI(sum, s.MulI(a1, b1))
	return s.AddI(sum, s.MulI(a2, b2))
}

// CrossI3 is CrossInterval3: each component the SubI of two MulI products.
func (s *SharedDenom) CrossI3(a, b SIVec3) SIVec3 {
	a0, a1, a2 := a[0], a[1], a[2]
	b0, b1, b2 := b[0], b[1], b[2]
	return SIVec3{
		s.SubI(s.MulI(a1, b2), s.MulI(a2, b1)),
		s.SubI(s.MulI(a2, b0), s.MulI(a0, b2)),
		s.SubI(s.MulI(a0, b1), s.MulI(a1, b0)),
	}
}

// PointCrossI3 is CrossInterval3 of the degenerate vector at a with b. The
// product of a degenerate interval [x, x] with an interval is its ScaleI by
// x, endpoint for endpoint, so it skips MulI's sign cases.
func (s *SharedDenom) PointCrossI3(a [3]SRat, b SIVec3) SIVec3 {
	a0, a1, a2 := a[0], a[1], a[2]
	b0, b1, b2 := b[0], b[1], b[2]
	return SIVec3{
		s.SubI(s.ScaleI(b2, a1), s.ScaleI(b1, a2)),
		s.SubI(s.ScaleI(b0, a2), s.ScaleI(b2, a0)),
		s.SubI(s.ScaleI(b1, a0), s.ScaleI(b0, a1)),
	}
}
