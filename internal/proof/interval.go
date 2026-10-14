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
	if a.Lo.Sign() == 0 && a.Hi.Sign() == 0 {
		return Interval(b.Lo, b.Hi)
	}
	if b.Lo.Sign() == 0 && b.Hi.Sign() == 0 {
		return Interval(a.Lo, a.Hi)
	}
	return OwnedInterval(AddRat(new(big.Rat), a.Lo, b.Lo), AddRat(new(big.Rat), a.Hi, b.Hi))
}

func NegInterval(a RatInterval) RatInterval {
	return OwnedInterval(new(big.Rat).Neg(a.Hi), new(big.Rat).Neg(a.Lo))
}

func SubInterval(a, b RatInterval) RatInterval {
	return OwnedInterval(SubRat(new(big.Rat), a.Lo, b.Hi), SubRat(new(big.Rat), a.Hi, b.Lo))
}

func ScaleInterval(a RatInterval, scale *big.Rat) RatInterval {
	if scale.Sign() < 0 {
		return OwnedInterval(
			MulRat(new(big.Rat), a.Hi, scale),
			MulRat(new(big.Rat), a.Lo, scale),
		)
	}
	return OwnedInterval(
		MulRat(new(big.Rat), a.Lo, scale),
		MulRat(new(big.Rat), a.Hi, scale),
	)
}

func PointInterval(value *big.Rat) RatInterval {
	return Interval(value, value)
}

// MulInterval multiplies two rational-bounded intervals. Endpoint signs
// identify the two extreme products unless both intervals cross zero; that
// case compares four products. Both result endpoints are owned independently.
func MulInterval(a, b RatInterval) RatInterval {
	if a.Lo.Sign() == 0 && a.Hi.Sign() == 0 || b.Lo.Sign() == 0 && b.Hi.Sign() == 0 {
		return OwnedInterval(new(big.Rat), new(big.Rat))
	}
	if a.Lo.Sign() >= 0 {
		switch {
		case b.Lo.Sign() >= 0:
			return OwnedInterval(MulRat(new(big.Rat), a.Lo, b.Lo), MulRat(new(big.Rat), a.Hi, b.Hi))
		case b.Hi.Sign() <= 0:
			return OwnedInterval(MulRat(new(big.Rat), a.Hi, b.Lo), MulRat(new(big.Rat), a.Lo, b.Hi))
		default:
			return OwnedInterval(MulRat(new(big.Rat), a.Hi, b.Lo), MulRat(new(big.Rat), a.Hi, b.Hi))
		}
	}
	if a.Hi.Sign() <= 0 {
		switch {
		case b.Lo.Sign() >= 0:
			return OwnedInterval(MulRat(new(big.Rat), a.Lo, b.Hi), MulRat(new(big.Rat), a.Hi, b.Lo))
		case b.Hi.Sign() <= 0:
			return OwnedInterval(MulRat(new(big.Rat), a.Hi, b.Hi), MulRat(new(big.Rat), a.Lo, b.Lo))
		default:
			return OwnedInterval(MulRat(new(big.Rat), a.Lo, b.Hi), MulRat(new(big.Rat), a.Lo, b.Lo))
		}
	}
	if b.Lo.Sign() >= 0 {
		return OwnedInterval(MulRat(new(big.Rat), a.Lo, b.Hi), MulRat(new(big.Rat), a.Hi, b.Hi))
	}
	if b.Hi.Sign() <= 0 {
		return OwnedInterval(MulRat(new(big.Rat), a.Hi, b.Lo), MulRat(new(big.Rat), a.Lo, b.Lo))
	}

	loA := MulRat(new(big.Rat), a.Lo, b.Hi)
	loB := MulRat(new(big.Rat), a.Hi, b.Lo)
	hiA := MulRat(new(big.Rat), a.Lo, b.Lo)
	hiB := MulRat(new(big.Rat), a.Hi, b.Hi)
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

// AddRat, SubRat and MulRat are big.Rat's Add, Sub and Mul in the
// common-denominator form for the operands every held float64 produces: a
// rational whose denominator is a power of two. Two such operands share the
// larger denominator 2^m, so a sum is one integer add over 2^m and a product
// one integer multiply over 2^(i+j), with the numerators shifted rather than
// multiplied by a denominator. The result's lowest terms then need no GCD:
// every common factor of a numerator and a power of two is a power of two,
// so stripping the numerator's trailing zero bits, at most m of them, reaches
// them (setPowerOfTwoFrac). Each returns the rational big.Rat's own operation
// returns, in the same lowest terms. An operand with any other denominator
// takes the split-GCD form (rat_split.go), which returns the same lowest
// terms as big.Rat's operation. Each sets z, which may alias either
// operand or both, and returns it; the result is written into z's own
// numerator and denominator.
//
// Operands must be in lowest terms, as every math/big constructor and
// arithmetic operation except Rat.GobDecode returns them, and as FloatRat,
// RatFrac, Dyadic.Rat and these helpers return them. A non-reduced operand
// (3/9, made by writing through Num or Denom) gives the exact value, but not
// always in lowest terms: AddRat(3/9, 0) returns 3/9 where big.Rat returns
// 1/3. Two readers see the form:
//
//   - Key2 (internal/meshbool) interns a vertex by its value text, so a
//     non-reduced coordinate would split one vertex into two (AddVert). The
//     memo keys motionbound AppendRatKey and proofbound radSinCosKey would
//     only miss their cache.
//   - EdgeCross2Fracs keeps a Cross2x result's Num and Denom in a ClipFrac,
//     which compares by cross multiplication and tolerates any form.
//
// The precondition rests on three things together, none of which is a proof
// alone. The producer audit: every producer the repository uses reduces; the
// one standard-library path that does not is Rat.GobDecode, which installs the
// encoded numerator and denominator as they are, and the repository does not
// call it; each write through Num and Denom in this package builds an odd
// numerator over a power of two, an integer, or a lowest-terms result. A
// runtime detector that checked every operand and result of the five helpers
// and found no violation: more than 100M checks over the packages under
// internal/, more than 222M over apitest, 241,365,639 over the root package,
// and more than 21M over internal/meshbool. And TestNoRatWritesThroughNumDenom,
// a tripwire for the write shapes it knows: it refuses most uses of Num and
// Denom outside this package, GobDecode calls and any import of encoding/gob.
// It does not prove that no other path produces a non-reduced Rat; reflection,
// unsafe and user-defined methods named like math/big methods are out of its
// scope. CmpRat has no such precondition.
func AddRat(z, a, b *big.Rat) *big.Rat {
	i, okA := denomExp(a)
	j, okB := denomExp(b)
	if okA != okB {
		return addMixedDyadic(z, a, b, i, j, okA, false)
	}
	if !okA {
		return ratSplitAdd(z, a, b, false)
	}
	return setPowerOfTwoFrac(z, alignedNum(z, a, b, i, j, false))
}

func SubRat(z, a, b *big.Rat) *big.Rat {
	i, okA := denomExp(a)
	j, okB := denomExp(b)
	if okA != okB {
		return addMixedDyadic(z, a, b, i, j, okA, true)
	}
	if !okA {
		return ratSplitAdd(z, a, b, true)
	}
	return setPowerOfTwoFrac(z, alignedNum(z, a, b, i, j, true))
}

func MulRat(z, a, b *big.Rat) *big.Rat {
	i, okA := denomExp(a)
	j, okB := denomExp(b)
	if !okA || !okB {
		if a == b {
			// big.Rat squares without a GCD; keep that path.
			return z.Mul(a, b)
		}
		return ratSplitMul(z, a, b)
	}
	z.Num().Mul(a.Num(), b.Num())
	return setPowerOfTwoFrac(z, i+j)
}

// addMixedDyadic sets z to a+b (or a-b with sub) when exactly one operand has
// a power-of-two denominator, without a GCD. aDyadic says which one. With
// r = p/q the other operand and w = m/2^k the dyadic one, the sum is
// N/D = (p·2^k ± m·q)/(q·2^k). gcd(N, u) = gcd(p, u) = 1 for the odd part u
// of q, so the only common factor is a power of two, 2^min(tz(N), tz(D)).
// N and D are built before z is written, so z may alias either operand.
func addMixedDyadic(z, a, b *big.Rat, i, j uint, aDyadic, sub bool) *big.Rat {
	r, w, k := b, a, i
	if !aDyadic {
		r, w, k = a, b, j
	}
	// The result is r+w, r-w or w-r; negate flips which term carries the sign.
	rTerm := new(big.Int).Lsh(r.Num(), k)
	wTerm := new(big.Int).Mul(w.Num(), r.Denom())
	n := rTerm
	switch {
	case sub && aDyadic:
		n.Sub(wTerm, rTerm)
	case sub:
		n.Sub(rTerm, wTerm)
	default:
		n.Add(rTerm, wTerm)
	}
	d := new(big.Int).Lsh(r.Denom(), k)
	if n.Sign() == 0 {
		return z.SetInt64(0)
	}
	if s := min(n.TrailingZeroBits(), d.TrailingZeroBits()); s > 0 {
		n.Rsh(n, s)
		d.Rsh(d, s)
	}
	z.SetInt(n)
	z.Denom().Set(d)
	return z
}

// denomExp is the exponent k of a denominator 2^k, and false for a
// denominator that is not a power of two.
func denomExp(r *big.Rat) (uint, bool) {
	if r.IsInt() {
		return 0, true
	}
	den := r.Denom()
	k := den.TrailingZeroBits()
	return k, uint(den.BitLen()) == k+1
}

// alignedNum sets z's numerator to a's numerator plus (or, with sub, minus)
// b's, both written over the larger denominator 2^m, m = max(i, j), for a
// over 2^i and b over 2^j, and returns m. z's denominator is left for
// setPowerOfTwoFrac. An operand z aliases is shifted in place once nothing
// else reads it; every other shifted numerator is a fresh integer.
func alignedNum(z, a, b *big.Rat, i, j uint, sub bool) uint {
	m := max(i, j)
	num := z.Num()
	if z == b && z != a {
		num.Lsh(num, m-j)
		x := a.Num()
		if m > i {
			x = new(big.Int).Lsh(x, m-i)
		}
		if sub {
			num.Sub(x, num)
		} else {
			num.Add(x, num)
		}
		return m
	}
	// z is a, both or neither operand; b's numerator is read, or copied by
	// its shift, before num changes. When z is both, i = j = m and y is num.
	y := b.Num()
	if m > j {
		y = new(big.Int).Lsh(y, m-j)
	}
	num.Lsh(a.Num(), m-i)
	if sub {
		num.Sub(num, y)
	} else {
		num.Add(num, y)
	}
	return m
}

// setPowerOfTwoFrac reduces z, whose numerator holds the value's numerator
// over 2^m, to lowest terms and returns it. Once the numerator's common
// trailing zero bits are shifted out it is odd or the denominator is 1,
// which is lowest terms. An integer result, zero among them, keeps z's
// denominator when that is already 1 and otherwise resets it in place
// through Rat.Denom's documented reference, so it allocates nothing; a
// fraction's denominator is written through the same reference once SetInt
// of z's own numerator, which copies nothing, has initialised it to 1.
func setPowerOfTwoFrac(z *big.Rat, m uint) *big.Rat {
	num := z.Num()
	if num.Sign() == 0 {
		m = 0
	} else {
		shift := min(num.TrailingZeroBits(), m)
		num.Rsh(num, shift)
		m -= shift
	}
	if m == 0 {
		if !z.IsInt() {
			z.Denom().SetInt64(1)
		}
		return z
	}
	z.SetInt(num)
	z.Denom().Lsh(z.Denom(), m)
	return z
}
