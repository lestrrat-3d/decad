package proof

import (
	"math"
	"math/big"
	"math/bits"

	"github.com/lestrrat-3d/r3"
)

// Xpt is an exact 3D point in homogeneous integer form. It has the same
// representation as Xhp, so conversions between them require no arithmetic.
// The zero value is unusable; construct through XptOf or the arithmetic
// helpers, which strip common powers of two before returning. A point has
// multiple homogeneous spellings, so Key compares canonical coordinates.
type Xpt struct{ X, Y, Z, W *big.Int }

// XptOf lifts a finite float vertex into exact homogeneous coordinates. A
// float64 is an exact rational, so no information is lost.
func XptOf(v r3.Vec) Xpt { return Xpt(XhpStripTwosOwned(XhpOf(v))) }

// Vec rounds the exact point to the nearest float64 coordinates.
func (p Xpt) Vec() r3.Vec { return XhpVec(Xhp(p)) }

// Xsub is a − b, exact, with the common power of two stripped on return (the
// growth control every construction pays — see XhpStripTwosOwned).
//
// Two power-of-two weights are aligned by a shift (xhpSubAligned); the
// strip then leaves the same four integers XhpSub's product form leaves,
// because a point with a power-of-two weight has exactly one stripped spelling.
func Xsub(a, b Xpt) Xpt {
	if d, ok := xhpSubAligned(Xhp(a), Xhp(b)); ok {
		return Xpt(XhpStripTwosOwned(d))
	}
	return Xpt(XhpStripTwosOwned(XhpSub(Xhp(a), Xhp(b))))
}

// XdotNum is the raw numerator of a·b over the positive denominator a.w·b.w —
// the value a sign-only consumer reads directly, and the value a
// cross-multiplied comparison reads without ever dividing.
func XdotNum(a, b Xpt) *big.Int { return XhpDotNum(Xhp(a), Xhp(b)) }

// XdotRat materialises a·b as a big.Rat — the one place this dot product pays
// a normalisation, and only when a caller genuinely needs the rational VALUE
// rather than a sign.
func XdotRat(a, b Xpt) *big.Rat {
	den := new(big.Int).Mul(a.W, b.W)
	return RatFrac(new(big.Rat), XdotNum(a, b), den)
}

// Xcross is a × b, exact, stripped the same way as Xsub.
func Xcross(a, b Xpt) Xpt {
	return Xpt(XhpStripTwosOwned(XhpCross(Xhp(a), Xhp(b))))
}

// XdotSign is the sign of a·b, decided as a plain integer sign: the shared
// denominator a.w·b.w is always positive, so the numerator's sign IS the
// dot product's sign.
func XdotSign(a, b Xpt) int { return XdotNum(a, b).Sign() }

// XdotDiffSign is the sign of n·(p−q). All weights are positive, so the
// subtraction can stay in integer numerators without constructing a point.
func XdotDiffSign(n, p, q Xpt) int {
	if p.W.Cmp(q.W) == 0 {
		var diff, sum, term big.Int
		diff.Sub(p.X, q.X)
		sum.Mul(n.X, &diff)
		diff.Sub(p.Y, q.Y)
		term.Mul(n.Y, &diff)
		sum.Add(&sum, &term)
		diff.Sub(p.Z, q.Z)
		term.Mul(n.Z, &diff)
		return sum.Add(&sum, &term).Sign()
	}
	pNum, qNum := XdotNum(n, p), XdotNum(n, q)
	// Float lifts carry power-of-two weights. Shift their numerators instead
	// of using general multiplication; other exact points retain that path.
	qShift := q.W.TrailingZeroBits()
	if q.W.BitLen() == int(qShift)+1 {
		pNum.Lsh(pNum, qShift)
	} else {
		pNum.Mul(pNum, q.W)
	}
	pShift := p.W.TrailingZeroBits()
	if p.W.BitLen() == int(pShift)+1 {
		qNum.Lsh(qNum, pShift)
	} else {
		qNum.Mul(qNum, p.W)
	}
	return pNum.Sub(pNum, qNum).Sign()
}

// Xlerp is a + t·(b − a) for t = tn/td, exact, with the common power of two
// stripped on return — the growth control that keeps a chain of lerps from
// growing its denominator multiplicatively at every link (measured: 14113
// bits unreduced at lerp depth 6, 462 bits stripped after every step).
func Xlerp(a, b Xpt, tn, td *big.Int) Xpt {
	return Xpt(XhpStripTwosOwned(XhpLerp(Xhp(a), Xhp(b), tn, td)))
}

// Xhp is an exact 3D point in homogeneous integer form: (x, y, z) is an
// integer numerator triple over one shared positive denominator w — the point
// it denotes is (x/w, y/w, z/w). big.Int carries no normalisation step of its
// own, unlike big.Rat, which runs a full Lehmer GCD on every construction
// whose denominator is not exactly 1 (rat.go's norm) — dyadic denominators
// included, since norm skips the GCD only when the denominator is 1. Every
// predicate built from these four integers is therefore a plain integer sign
// test.
//
// The invariant w > 0 is load-bearing: every helper below reduces a
// division's sign to the sign of a product of denominators, which only holds
// when every denominator involved is positive. XhpSub and XhpCross each
// combine two operands whose own w is positive into a result whose w is their
// PRODUCT — again positive by construction, with no branch needed — so the
// invariant propagates through every point/vector this file builds except
// one: XhpLerp's lerp-parameter denominator can arrive negative, and it
// renormalises explicitly before folding it in.
//
// A homogeneous point has many spellings — (x, y, z, w) and (2x, 2y, 2z, 2w)
// denote the same coordinate — so exact identity (welding) never compares raw
// fields; it goes through the canonical form (XhpCanon).
type Xhp struct{ X, Y, Z, W *big.Int }

// XhpOf lifts a finite float vertex into homogeneous integer coordinates. A
// float64 is an exact dyadic rational. Aligning its three binary exponents
// directly gives one shared power-of-two denominator without constructing
// three big.Rat values or multiplying their denominators.
func XhpOf(v r3.Vec) Xhp {
	dx, dy, dz := MustDyOf(v.X), MustDyOf(v.Y), MustDyOf(v.Z)
	base := 0
	lower := func(d Dyadic) {
		if !d.IsZero() && d.Exp() < base {
			base = d.Exp()
		}
	}
	lower(dx)
	lower(dy)
	lower(dz)
	coord := func(d Dyadic) *big.Int {
		if d.IsZero() {
			return new(big.Int)
		}
		mant := d.MantInto(new(big.Int))
		return mant.Lsh(mant, uint(d.Exp()-base))
	}
	return Xhp{
		X: coord(dx),
		Y: coord(dy),
		Z: coord(dz),
		W: new(big.Int).Lsh(big.NewInt(1), uint(-base)),
	}
}

// XhpSub is p − q, exact: a homogeneous vector over the positive denominator
// p.w·q.w, or over their shared denominator when the weights match.
func XhpSub(p, q Xhp) Xhp {
	if p.W.Cmp(q.W) == 0 {
		return Xhp{
			X: new(big.Int).Sub(p.X, q.X),
			Y: new(big.Int).Sub(p.Y, q.Y),
			Z: new(big.Int).Sub(p.Z, q.Z),
			W: new(big.Int).Set(p.W),
		}
	}
	var term big.Int
	axis := func(pn, qn *big.Int) *big.Int {
		out := new(big.Int).Mul(pn, q.W)
		return out.Sub(out, term.Mul(qn, p.W))
	}
	return Xhp{
		X: axis(p.X, q.X),
		Y: axis(p.Y, q.Y),
		Z: axis(p.Z, q.Z),
		W: new(big.Int).Mul(p.W, q.W),
	}
}

// XhpDotNum is the NUMERATOR of a·b over the positive denominator a.w·b.w.
// The denominator is never formed: every consumer either reads only this
// numerator's sign, or divides it back out at the one point a value is
// actually published (XhpRat).
func XhpDotNum(a, b Xhp) *big.Int {
	s := new(big.Int).Mul(a.X, b.X)
	var term big.Int
	s.Add(s, term.Mul(a.Y, b.Y))
	s.Add(s, term.Mul(a.Z, b.Z))
	return s
}

// XhpVec rounds p to the nearest float64 coordinates.
func XhpVec(p Xhp) r3.Vec {
	return r3.Vec{
		X: xhpFractionFloat64(p.X, p.W),
		Y: xhpFractionFloat64(p.Y, p.W),
		Z: xhpFractionFloat64(p.Z, p.W),
	}
}

// xhpFractionFloat64 rounds an integer quotient directly, avoiding the full
// GCD that big.Rat.SetFrac performs before its Float64 conversion.
func xhpFractionFloat64(num, den *big.Int) float64 {
	f, _ := quotientFloat64(num, den)
	return f
}

// quotientFloat64 is big.Rat.Float64 of num/den without forming the Rat: the
// nearest float64 (ties to even) and big.Rat.Float64's exactness flag, for a
// den that need not be in lowest terms. A positive power-of-two den takes the
// shift path of powerOfTwoQuotient; any other den takes the integer division.
func quotientFloat64(num, den *big.Int) (float64, bool) {
	if num.Sign() == 0 {
		return 0, true
	}
	exp := num.BitLen() - den.BitLen()
	if isPositivePowerOfTwo(den) {
		mantissa, haveRem := powerOfTwoQuotient(num)
		return roundQuotient(num.Sign() < 0, exp, mantissa, haveRem)
	}
	var a, b, quotient, remainder big.Int
	a.Abs(num)
	b.Set(den)
	if shift := 54 - exp; shift > 0 {
		a.Lsh(&a, uint(shift))
	} else if shift < 0 {
		b.Lsh(&b, uint(-shift))
	}
	quotient.QuoRem(&a, &b, &remainder)
	return roundQuotient(num.Sign() < 0, exp, quotient.Uint64(), remainder.Sign() != 0)
}

// isPositivePowerOfTwo reports whether x is 2^k for some k >= 0.
func isPositivePowerOfTwo(x *big.Int) bool {
	return x.Sign() > 0 && x.TrailingZeroBits() == uint(x.BitLen()-1)
}

// powerOfTwoQuotient returns the mantissa and remainder flag the integer
// division of quotientFloat64 produces when den = 2^k and num is nonzero. The
// division scales |num|/den by 2^(54-exp), exp = BitLen(num) - (k+1), which is
// |num| · 2^(55-BitLen(num)) whatever k is. That is the top 55 bits of |num|,
// zero-padded below when |num| is shorter, and it leaves a remainder exactly
// when a nonzero bit of |num| lies below those 55.
func powerOfTwoQuotient(num *big.Int) (uint64, bool) {
	alen := num.BitLen()
	words := num.Bits()
	if alen <= 55 {
		return absShiftedLow64(words, 0) << uint(55-alen), false
	}
	shift := uint(alen - 55)
	return absShiftedLow64(words, shift), num.TrailingZeroBits() < shift
}

// absShiftedLow64 returns the low 64 bits of |x| >> shift for the little-endian
// words of |x|. The bits above 64 are dropped.
func absShiftedLow64(words []big.Word, shift uint) uint64 {
	const wordBits = bits.UintSize
	idx, off := int(shift/wordBits), shift%wordBits
	var out uint64
	for i := 0; idx+i < len(words) && uint(i)*wordBits < 64+off; i++ {
		chunk := uint64(words[idx+i])
		if i == 0 {
			out = chunk >> off
			continue
		}
		out |= chunk << (uint(i)*wordBits - off)
	}
	return out
}

// roundQuotient finishes big.Rat.Float64's rounding from the scaled quotient:
// mantissa holds 54 or 55 bits, exp is BitLen(num) - BitLen(den) and haveRem
// records a nonzero division remainder. It returns the float and Float64's
// exactness flag, which stays true for a positive subnormal quotient that
// flushes to zero and turns false on overflow to infinity.
func roundQuotient(negative bool, exp int, mantissa uint64, haveRem bool) (float64, bool) {
	if mantissa>>54 == 1 {
		if mantissa&1 != 0 {
			haveRem = true
		}
		mantissa >>= 1
		exp++
	}
	if -1074 <= exp && exp <= -1022 {
		shift := uint(-1022 - (exp - 1))
		haveRem = haveRem || mantissa&(1<<shift-1) != 0
		mantissa >>= shift
		exp = -1021
	}
	exact := !haveRem
	if mantissa&1 != 0 {
		exact = false
		if haveRem || mantissa&2 != 0 {
			mantissa++
			if mantissa >= 1<<54 {
				mantissa >>= 1
				exp++
			}
		}
	}
	mantissa >>= 1
	value := math.Ldexp(float64(mantissa), exp-53)
	if math.IsInf(value, 0) {
		exact = false
	}
	if negative {
		return -value, exact
	}
	return value, exact
}

// XhpStripTwosOwned takes ownership of p and divides all four integers by
// their largest common power of two — the growth control. It costs no GCD
// (TrailingZeroBits and Rsh only), which is what makes it cheap enough to run
// on every fresh construction; the full GCD XhpCanon runs is not (its own doc
// comment). Callers must pass only a freshly constructed xhp whose limbs do
// not belong to another point.
func XhpStripTwosOwned(p Xhp) Xhp {
	tz := p.W.TrailingZeroBits()
	for _, v := range [3]*big.Int{p.X, p.Y, p.Z} {
		if v.Sign() == 0 {
			continue
		}
		if t := v.TrailingZeroBits(); t < tz {
			tz = t
		}
	}
	if tz == 0 {
		return p
	}
	p.X.Rsh(p.X, tz)
	p.Y.Rsh(p.Y, tz)
	p.Z.Rsh(p.Z, tz)
	p.W.Rsh(p.W, tz)
	return p
}

// XhpCanon reduces p to its unique canonical spelling: every one of the four
// integers divided by gcd(|x|, |y|, |z|, w). This is the FULL GCD the
// representation otherwise exists to avoid, so it is reserved for the one
// place a homogeneous point's many spellings must collapse to one —
// vertex-emission identity (key, welding) — and must NEVER be called per
// arithmetic operation: measured, a depth-3 XhpLerp chain drops from
// big.Rat's 31.4us to 9.6us with xhpStripTwos alone, but only to 23.4us with
// a full XhpCanon on every construction — nearly the whole win, given back.
//
// A power-of-two W makes the divisor 2^min(tz(W), tz(x), tz(y), tz(z)) over the
// nonzero coordinates, so that case is four right shifts and no GCD.
func XhpCanon(p Xhp) Xhp {
	if k, ok := pow2Exp(p.W); ok {
		for _, v := range [3]*big.Int{p.X, p.Y, p.Z} {
			if v.Sign() == 0 {
				continue
			}
			if t := v.TrailingZeroBits(); t < k {
				k = t
			}
		}
		return Xhp{
			X: new(big.Int).Rsh(p.X, k),
			Y: new(big.Int).Rsh(p.Y, k),
			Z: new(big.Int).Rsh(p.Z, k),
			W: new(big.Int).Rsh(p.W, k),
		}
	}
	g := new(big.Int).Set(p.W)
	for _, v := range [3]*big.Int{p.X, p.Y, p.Z} {
		if v.Sign() == 0 {
			continue
		}
		g.GCD(nil, nil, g, v)
	}
	return Xhp{
		X: new(big.Int).Quo(p.X, g),
		Y: new(big.Int).Quo(p.Y, g),
		Z: new(big.Int).Quo(p.Z, g),
		W: new(big.Int).Quo(p.W, g),
	}
}

// XhpCross is a × b, exact: its numerators over the positive denominator
// a.w·b.w, which two power-of-two weights 2^i and 2^j form as 2^(i+j) by
// setting one bit.
func XhpCross(a, b Xhp) Xhp {
	var term big.Int
	w := new(big.Int)
	i, aok := pow2Exp(a.W)
	j, bok := pow2Exp(b.W)
	if aok && bok {
		w.SetBit(w, int(i+j), 1)
	} else {
		w.Mul(a.W, b.W)
	}
	axis := func(a0, b0, a1, b1 *big.Int) *big.Int {
		out := new(big.Int).Mul(a0, b0)
		return out.Sub(out, term.Mul(a1, b1))
	}
	return Xhp{
		X: axis(a.Y, b.Z, a.Z, b.Y),
		Y: axis(a.Z, b.X, a.X, b.Z),
		Z: axis(a.X, b.Y, a.Y, b.X),
		W: w,
	}
}

// XhpLerp is a + t·(b − a) for t = tn/td, exact. td may arrive negative; a.w
// and b.w are already positive by invariant, so the result's own w — their
// product with td — is renormalised by flipping td's (and tn's) sign first,
// which leaves the value t = tn/td unchanged.
func XhpLerp(a, b Xhp, tn, td *big.Int) Xhp {
	n, d := tn, td
	if d.Sign() < 0 {
		n = new(big.Int).Neg(n)
		d = new(big.Int).Neg(d)
	}
	// a + t·(b−a) = a·(d−n)/d + b·n/d = [a·(d−n)·b.w + b·n·a.w] / (a.w·b.w·d).
	diff := new(big.Int).Sub(d, n)
	axis := func(av, bv *big.Int) *big.Int {
		term := new(big.Int).Mul(av, diff)
		term.Mul(term, b.W)
		other := new(big.Int).Mul(bv, n)
		other.Mul(other, a.W)
		return term.Add(term, other)
	}
	w := new(big.Int).Mul(a.W, b.W)
	w.Mul(w, d)
	return Xhp{X: axis(a.X, b.X), Y: axis(a.Y, b.Y), Z: axis(a.Z, b.Z), W: w}
}

// XhpOrientSign is the exact sign of det[b−a, c−a, d−a], decided as a plain
// integer sign with no big.Rat and no normalisation anywhere in the chain:
// every intermediate xhp carries a positive denominator by construction, so
// the final numerator's sign IS the determinant's sign.
func XhpOrientSign(a, b, c, d Xhp) int {
	ba, ca, da := xhpSubSign(b, a), xhpSubSign(c, a), xhpSubSign(d, a)
	return XhpDotNum(XhpCross(ba, ca), da).Sign()
}

// xhpSubSign is p − q up to a positive factor of the weight: the aligned form
// when both weights are powers of two, XhpSub otherwise. The factor is
// positive, so a sign read off the numerators is unchanged.
func xhpSubSign(p, q Xhp) Xhp {
	if d, ok := xhpSubAligned(p, q); ok {
		return d
	}
	return XhpSub(p, q)
}

// xhpSubAligned is p − q for weights 2^i and 2^j, over the larger of the two
// weights: the numerators of the operand with the smaller weight are shifted
// left by |i−j|, and the difference over that weight is XhpSub's result divided
// by 2^min(i, j). It reports false, and builds nothing, when either weight is
// not a positive power of two. Every integer of the result is freshly
// allocated, W included, because XhpStripTwosOwned shifts them in place.
func xhpSubAligned(p, q Xhp) (Xhp, bool) {
	i, ok := pow2Exp(p.W)
	if !ok {
		return Xhp{}, false
	}
	j, ok := pow2Exp(q.W)
	if !ok {
		return Xhp{}, false
	}
	axis := func(pn, qn *big.Int) *big.Int {
		switch {
		case i < j:
			out := new(big.Int).Lsh(pn, j-i)
			return out.Sub(out, qn)
		case i > j:
			var term big.Int
			return new(big.Int).Sub(pn, term.Lsh(qn, i-j))
		}
		return new(big.Int).Sub(pn, qn)
	}
	w := p.W
	if j > i {
		w = q.W
	}
	return Xhp{X: axis(p.X, q.X), Y: axis(p.Y, q.Y), Z: axis(p.Z, q.Z), W: new(big.Int).Set(w)}, true
}

// pow2Exp reports k when w is exactly 2^k.
func pow2Exp(w *big.Int) (uint, bool) {
	if w.Sign() <= 0 {
		return 0, false
	}
	k := w.TrailingZeroBits()
	return k, w.BitLen() == int(k)+1
}

// XhpRat materialises p's three coordinates as big.Rat — the one place a
// homogeneous point pays a normalisation, and only when a caller genuinely
// needs a rational VALUE rather than a sign. Each coordinate is in lowest
// terms, as SetFrac returns it; a power-of-two W reduces by a shift.
func XhpRat(p Xhp) (x, y, z *big.Rat) {
	return xhpCoordRat(p.X, p.W), xhpCoordRat(p.Y, p.W), xhpCoordRat(p.Z, p.W)
}

// xhpCoordRat is n/w in lowest terms through RatFrac. An integer value, zero
// included, gets a stored denominator of 1, as SetFrac leaves it: RatFrac may
// leave a zero denominator unstored, and Denom would then return a fresh
// detached 1 on every call.
func xhpCoordRat(n, w *big.Int) *big.Rat {
	r := RatFrac(new(big.Rat), n, w)
	if r.IsInt() {
		r.SetInt(r.Num())
	}
	return r
}
