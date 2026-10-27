package pointquery

import "math/big"

// surdPoint is a plane point whose coordinates are u0 + u1·s and v0 + v1·s
// for one square root s = √r, r ≥ 0: a full Revolve's query point carried
// into its meridian plane (agent-docs/point-containment-design.md §10.3).
// Every region test it answers is a polynomial of degree at most two in its
// coordinates, which s² = r reduces to a + b·s over rationals; surdSign
// decides that sign exactly without forming s.
type surdPoint struct {
	u0, u1, v0, v1, r *big.Rat
}

// mirrored is the point with s replaced by −s.
func (p surdPoint) mirrored() surdPoint {
	return surdPoint{u0: p.u0, u1: new(big.Rat).Neg(p.u1), v0: p.v0, v1: new(big.Rat).Neg(p.v1), r: p.r}
}

// offset is p − a as the two linear forms (du0 + du1·s, dv0 + dv1·s).
func (p surdPoint) offset(a planePoint) (*big.Rat, *big.Rat) {
	return new(big.Rat).Sub(p.u0, a.u), new(big.Rat).Sub(p.v0, a.v)
}

// sideOf is the sign of (b − a) × (p − a)
// = [du·(v0 − a.v) − dv·(u0 − a.u)] + [du·v1 − dv·u1]·s.
func (p surdPoint) sideOf(a, b planePoint) int {
	d := b.sub(a)
	ou, ov := p.offset(a)
	constant := new(big.Rat).Sub(new(big.Rat).Mul(d.u, ov), new(big.Rat).Mul(d.v, ou))
	linear := new(big.Rat).Sub(new(big.Rat).Mul(d.u, p.v1), new(big.Rat).Mul(d.v, p.u1))
	return surdSign(constant, linear, p.r)
}

// spanOf is the sign of (p − a)·(p − b). Per coordinate,
// (α + c·s)(β + c·s) = αβ + c²·r + (α + β)·c·s.
func (p surdPoint) spanOf(a, b planePoint) int {
	au, av := p.offset(a)
	bu, bv := p.offset(b)
	constant, linear := new(big.Rat), new(big.Rat)
	for _, term := range [...][3]*big.Rat{{au, bu, p.u1}, {av, bv, p.v1}} {
		alpha, beta, c := term[0], term[1], term[2]
		constant.Add(constant, new(big.Rat).Mul(alpha, beta))
		constant.Add(constant, new(big.Rat).Mul(new(big.Rat).Mul(c, c), p.r))
		linear.Add(linear, new(big.Rat).Mul(new(big.Rat).Add(alpha, beta), c))
	}
	return surdSign(constant, linear, p.r)
}

// heightOf is the sign of (v0 − v) + v1·s.
func (p surdPoint) heightOf(v *big.Rat) int {
	return surdSign(new(big.Rat).Sub(p.v0, v), p.v1, p.r)
}

// radialOf is the sign of |p − c|² − r2. Per coordinate,
// (α + k·s)² = α² + k²·r + 2·α·k·s.
func (p surdPoint) radialOf(c planePoint, r2 *big.Rat) int {
	ou, ov := p.offset(c)
	constant := new(big.Rat).Neg(r2)
	linear := new(big.Rat)
	for _, term := range [...][2]*big.Rat{{ou, p.u1}, {ov, p.v1}} {
		alpha, k := term[0], term[1]
		constant.Add(constant, new(big.Rat).Mul(alpha, alpha))
		constant.Add(constant, new(big.Rat).Mul(new(big.Rat).Mul(k, k), p.r))
		linear.Add(linear, new(big.Rat).Mul(big.NewRat(2, 1), new(big.Rat).Mul(alpha, k)))
	}
	return surdSign(constant, linear, p.r)
}

// surdSign is the exact sign of a + b·√r for r ≥ 0. Two terms of one sign
// take that sign. Otherwise the larger in magnitude decides, compared through
// a² against b²·r, which a vanishing term loses; equal magnitudes cancel to
// zero.
func surdSign(a, b, r *big.Rat) int {
	sa := a.Sign()
	sb := b.Sign() * r.Sign()
	if sa == sb {
		return sa
	}
	square := new(big.Rat).Mul(b, b)
	switch new(big.Rat).Mul(a, a).Cmp(square.Mul(square, r)) {
	case 1:
		return sa
	case -1:
		return sb
	default:
		return 0
	}
}
