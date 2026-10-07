package proof

import (
	"math"
	"math/big"

	"github.com/lestrrat-3d/r3"
)

// OrientNum is the exact value of det[b−a, c−a, d−a] as an integer numerator
// over a positive denominator, formed without ever materialising a big.Rat:
// positive when d lies on the side the counter-clockwise normal of (a, b, c)
// points to.
func OrientNum(a, b, c, d Xpt) (num, den *big.Int) {
	ha, hb, hc, hd := Xhp(a), Xhp(b), Xhp(c), Xhp(d)
	ba, ca, da := XhpSub(hb, ha), XhpSub(hc, ha), XhpSub(hd, ha)
	cr := XhpCross(ba, ca)
	return XhpDotNum(cr, da), new(big.Int).Mul(cr.W, da.W)
}

// OrientSignExact is the exact sign of det[b−a, c−a, d−a], decided as a plain
// integer sign with no big.Rat and no normalisation anywhere in the chain —
// XhpOrientSign's own guarantee, carried through xpt.
func OrientSignExact(a, b, c, d Xpt) int {
	return XhpOrientSign(Xhp(a), Xhp(b), Xhp(c), Xhp(d))
}

// OrientRat materialises det[b−a, c−a, d−a] as a big.Rat — the one place this
// value pays a normalisation, for the rare caller that needs the value rather
// than the sign.
func OrientRat(a, b, c, d Xpt) *big.Rat {
	num, den := OrientNum(a, b, c, d)
	return new(big.Rat).SetFrac(num, den)
}

// OrientSign is the adaptive-precision sign of det[b−a, c−a, d−a] for float
// inputs: a float evaluation whose forward error provably cannot cross zero
// decides the generic case; anything inside the error bound falls back to the
// exact value — docs/evaluator-design.md §9's discipline, so a sign is never wrong.
func OrientSign(a, b, c, d r3.Vec) int {
	if sign, certain := OrientSignFloat(a, b, c, d); certain {
		return sign
	}
	return OrientSignExact(XptOf(a), XptOf(b), XptOf(c), XptOf(d))
}

// OrientSignPrepared uses an already lifted triangle and its exact normal on
// the uncertain path. xa and xd are the exact lifts of a and d, and n is the
// exact oriented cross product of (b-a) and (c-a), with positive denominator.
func OrientSignPrepared(a, b, c, d r3.Vec, xa, xd, n Xpt) int {
	if sign, certain := OrientSignFloat(a, b, c, d); certain {
		return sign
	}
	return XdotSign(n, Xsub(xd, xa))
}

// OrientSignFloat gives the same adaptive float decision to both plane-side
// callers. An uncertain sign must be decided by exact integer arithmetic.
func OrientSignFloat(a, b, c, d r3.Vec) (int, bool) {
	bax, bay, baz := b.X-a.X, b.Y-a.Y, b.Z-a.Z
	cax, cay, caz := c.X-a.X, c.Y-a.Y, c.Z-a.Z
	dax, day, daz := d.X-a.X, d.Y-a.Y, d.Z-a.Z
	det := bax*(cay*daz-caz*day) + bay*(caz*dax-cax*daz) + baz*(cax*day-cay*dax)
	perm := math.Abs(bax)*(math.Abs(cay)*math.Abs(daz)+math.Abs(caz)*math.Abs(day)) +
		math.Abs(bay)*(math.Abs(caz)*math.Abs(dax)+math.Abs(cax)*math.Abs(daz)) +
		math.Abs(baz)*(math.Abs(cax)*math.Abs(day)+math.Abs(cay)*math.Abs(dax))
	// The true forward error is bounded by a few ulps of the permanent; 1e-12
	// leaves three decades of margin, so a sign the filter accepts is proven.
	if err := 1e-12 * perm; det > err || det < -err {
		if det > 0 {
			return 1, true
		}
		return -1, true
	}
	return 0, false
}

// OrientSignMixed is the exact plane-side sign of a homogeneous probe against
// a float triangle: positive on the triangle's counter-clockwise-normal side.
func OrientSignMixed(a, b, c r3.Vec, d Xpt) int {
	return OrientSignExact(XptOf(a), XptOf(b), XptOf(c), d)
}
