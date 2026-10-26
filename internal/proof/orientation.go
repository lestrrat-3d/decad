package proof

import (
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
// exact value — agent-docs/evaluator-design.md §9's discipline, so a sign is never wrong.
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
	return XdotDiffSign(n, xd, xa)
}

// OrientSignFloat gives the same adaptive float decision to both plane-side
// callers. All input coordinates must be finite. Outside the exact-zero
// coordinate-plane shortcut, any nonzero difference with absolute value outside
// [2^-300, 2^250] makes the result uncertain, so callers use the exact path. An
// uncertain sign must be decided by exact integer arithmetic.
func OrientSignFloat(a, b, c, d r3.Vec) (int, bool) {
	bax, bay, baz := b.X-a.X, b.Y-a.Y, b.Z-a.Z
	cax, cay, caz := c.X-a.X, c.Y-a.Y, c.Z-a.Z
	dax, day, daz := d.X-a.X, d.Y-a.Y, d.Z-a.Z
	// A finite float difference is zero exactly when its operands are equal,
	// so three zero differences on one axis put all four points in one
	// coordinate plane: the exact determinant has a zero column and is zero.
	if bax == 0 && cax == 0 && dax == 0 || bay == 0 && cay == 0 && day == 0 ||
		baz == 0 && caz == 0 && daz == 0 {
		return 0, true
	}
	for _, v := range [...]float64{bax, bay, baz, cax, cay, caz, dax, day, daz} {
		x := Abs(v)
		if !(x == 0 || (x >= 0x1p-300 && x <= 0x1p250)) {
			return 0, false
		}
	}
	det := bax*(cay*daz-caz*day) + bay*(caz*dax-cax*daz) + baz*(cax*day-cay*dax)
	perm := Abs(bax)*(Abs(cay)*Abs(daz)+Abs(caz)*Abs(day)) +
		Abs(bay)*(Abs(caz)*Abs(dax)+Abs(cax)*Abs(daz)) +
		Abs(baz)*(Abs(cax)*Abs(day)+Abs(cay)*Abs(dax))
	// Admitted nonzero differences are normal, and rounded zero differences
	// are exact. Every rounded difference is a multiple of 2^-352, so quadratic
	// and cubic intermediates lie on grids of 2^-704 and 2^-1056. Subnormal
	// results on those grids are exact; magnitudes below 6*2^750 cannot overflow.
	// Each signed triple product has at most eight roundings, giving
	// |Dhat-D| < 9e-16*Phat. When Phat is nonzero, it is at least 2^-900,
	// so 2e-15*Phat stays normal and exceeds this error bound after rounding.
	if err := 2e-15 * perm; det > err || det < -err {
		if det > 0 {
			return 1, true
		}
		return -1, true
	}
	return OrientSignExpansion(a, b, c, d)
}

// OrientSignMixed is the exact plane-side sign of a homogeneous probe against
// a float triangle: positive on the triangle's counter-clockwise-normal side.
func OrientSignMixed(a, b, c r3.Vec, d Xpt) int {
	return OrientSignExact(XptOf(a), XptOf(b), XptOf(c), d)
}
