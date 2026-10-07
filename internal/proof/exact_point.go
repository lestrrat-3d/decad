package proof

import (
	"encoding/binary"
	"math/big"

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

// Key is the exact identity of the point: two points weld exactly when their
// CANONICAL homogeneous coordinates are identical — stitching by shared exact
// vertices, never by distance (docs/evaluator-design.md §9). A homogeneous
// point has many spellings, so the raw fields are never compared directly;
// XhpCanon collapses every spelling of one coordinate to one four-tuple
// before the key is built.
func (p Xpt) Key() string {
	c := XhpCanon(Xhp(p))
	return ExactIntsKey(c.X, c.Y, c.Z, c.W)
}

// ExactIntsKey encodes signed integers without decimal conversion. Each value
// carries its sign and byte length, so adjacent magnitudes cannot collide.
func ExactIntsKey(values ...*big.Int) string {
	size := 9 * len(values)
	for _, v := range values {
		size += (v.BitLen() + 7) / 8
	}
	buf := make([]byte, 0, size)
	for _, v := range values {
		buf = append(buf, byte(v.Sign()+1))
		n := (v.BitLen() + 7) / 8
		buf = binary.LittleEndian.AppendUint64(buf, uint64(n))
		start := len(buf)
		buf = buf[:start+n]
		v.FillBytes(buf[start:])
	}
	return string(buf)
}

// Xsub is a − b, exact, with the common power of two stripped on return (the
// growth control every construction pays — see XhpStripTwosOwned).
func Xsub(a, b Xpt) Xpt { return Xpt(XhpStripTwosOwned(XhpSub(Xhp(a), Xhp(b)))) }

// XdotNum is the raw numerator of a·b over the positive denominator a.w·b.w —
// the value a sign-only consumer reads directly, and the value a
// cross-multiplied comparison reads without ever dividing.
func XdotNum(a, b Xpt) *big.Int { return XhpDotNum(Xhp(a), Xhp(b)) }

// XdotRat materialises a·b as a big.Rat — the one place this dot product pays
// a normalisation, and only when a caller genuinely needs the rational VALUE
// rather than a sign.
func XdotRat(a, b Xpt) *big.Rat {
	den := new(big.Int).Mul(a.W, b.W)
	return new(big.Rat).SetFrac(XdotNum(a, b), den)
}

// Xcross is a × b, exact, stripped the same way as Xsub.
func Xcross(a, b Xpt) Xpt {
	return Xpt(XhpStripTwosOwned(XhpCross(Xhp(a), Xhp(b))))
}

// XdotSign is the sign of a·b, decided as a plain integer sign: the shared
// denominator a.w·b.w is always positive, so the numerator's sign IS the
// dot product's sign.
func XdotSign(a, b Xpt) int { return XdotNum(a, b).Sign() }

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
	var r big.Rat
	fx, _ := r.SetFrac(p.X, p.W).Float64()
	fy, _ := r.SetFrac(p.Y, p.W).Float64()
	fz, _ := r.SetFrac(p.Z, p.W).Float64()
	return r3.Vec{X: fx, Y: fy, Z: fz}
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
// arithmetic operation: measured, a depth-3 xhpLerp chain drops from
// big.Rat's 31.4us to 9.6us with xhpStripTwos alone, but only to 23.4us with
// a full XhpCanon on every construction — nearly the whole win, given back.
func XhpCanon(p Xhp) Xhp {
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
// a.w·b.w.
func XhpCross(a, b Xhp) Xhp {
	var term big.Int
	axis := func(a0, b0, a1, b1 *big.Int) *big.Int {
		out := new(big.Int).Mul(a0, b0)
		return out.Sub(out, term.Mul(a1, b1))
	}
	return Xhp{
		X: axis(a.Y, b.Z, a.Z, b.Y),
		Y: axis(a.Z, b.X, a.X, b.Z),
		Z: axis(a.X, b.Y, a.Y, b.X),
		W: new(big.Int).Mul(a.W, b.W),
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
	ba, ca, da := XhpSub(b, a), XhpSub(c, a), XhpSub(d, a)
	return XhpDotNum(XhpCross(ba, ca), da).Sign()
}

// XhpRat materialises p's three coordinates as big.Rat — the one place a
// homogeneous point pays a normalisation, and only when a caller genuinely
// needs a rational VALUE rather than a sign.
func XhpRat(p Xhp) (x, y, z *big.Rat) {
	return new(big.Rat).SetFrac(p.X, p.W), new(big.Rat).SetFrac(p.Y, p.W), new(big.Rat).SetFrac(p.Z, p.W)
}
