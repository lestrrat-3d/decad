package brepmodify

import (
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// ivl is an exact rational interval, the arithmetic every carrier-route
// reading is enclosed in (agent-docs/prism-boolean-design.md Part K §7.5). Only
// square roots round, each outward, and the result of a reading converts to
// float64 once, rounded up.
type ivl struct{ lo, hi *big.Rat }

func ivPoint(q *big.Rat) ivl { return ivl{lo: q, hi: q} }

func ivFloat(f float64) (ivl, bool) {
	q, ok := proofbound.RatOf(f)
	if !ok {
		return ivl{}, false
	}
	return ivPoint(q), true
}

func (a ivl) add(b ivl) ivl {
	return ivl{lo: new(big.Rat).Add(a.lo, b.lo), hi: new(big.Rat).Add(a.hi, b.hi)}
}

func (a ivl) sub(b ivl) ivl {
	return ivl{lo: new(big.Rat).Sub(a.lo, b.hi), hi: new(big.Rat).Sub(a.hi, b.lo)}
}

func (a ivl) mul(b ivl) ivl {
	ps := []*big.Rat{
		new(big.Rat).Mul(a.lo, b.lo), new(big.Rat).Mul(a.lo, b.hi),
		new(big.Rat).Mul(a.hi, b.lo), new(big.Rat).Mul(a.hi, b.hi),
	}
	lo, hi := ps[0], ps[0]
	for _, p := range ps[1:] {
		if p.Cmp(lo) < 0 {
			lo = p
		}
		if p.Cmp(hi) > 0 {
			hi = p
		}
	}
	return ivl{lo: lo, hi: hi}
}

// quo divides by an interval that excludes zero; ok=false otherwise.
func (a ivl) quo(b ivl) (ivl, bool) {
	if b.lo.Sign() <= 0 && b.hi.Sign() >= 0 {
		return ivl{}, false
	}
	inv := ivl{lo: new(big.Rat).Inv(b.hi), hi: new(big.Rat).Inv(b.lo)}
	return a.mul(inv), true
}

func (a ivl) neg() ivl {
	return ivl{lo: new(big.Rat).Neg(a.hi), hi: new(big.Rat).Neg(a.lo)}
}

// maxAbs is the largest magnitude the interval permits, rounded up.
func (a ivl) maxAbs() float64 {
	lo, hi := new(big.Rat).Abs(a.lo), new(big.Rat).Abs(a.hi)
	if lo.Cmp(hi) > 0 {
		return proofbound.RatFloatUp(lo)
	}
	return proofbound.RatFloatUp(hi)
}

func (a ivl) loDown() float64 { return proofbound.RatFloatDown(a.lo) }
func (a ivl) hiUp() float64   { return proofbound.RatFloatUp(a.hi) }

// sqrtIv is the square root of a non-negative interval, enclosed outward.
func sqrtIv(a ivl) (ivl, bool) {
	if a.lo.Sign() < 0 {
		a = ivl{lo: new(big.Rat), hi: a.hi}
	}
	lo, hi := proofbound.RatSqrtDown(a.lo), proofbound.RatSqrtUp(a.hi)
	l, ok1 := proofbound.RatOf(lo)
	h, ok2 := proofbound.RatOf(hi)
	if !ok1 || !ok2 || math.IsInf(hi, 0) {
		return ivl{}, false
	}
	return ivl{lo: l, hi: h}, true
}

// sqr is the square of an interval.
func (a ivl) sqr() ivl {
	m := a.mul(a)
	if a.lo.Sign() <= 0 && a.hi.Sign() >= 0 {
		m.lo = new(big.Rat)
	}
	return m
}

// norm is the Euclidean length of the interval vector (u, v).
func norm(u, v ivl) (ivl, bool) { return sqrtIv(u.sqr().add(v.sqr())) }

// pointDistUp bounds the distance between two exact points, rounded up.
func pointDistUp(au, av, bu, bv *big.Rat) float64 {
	du, dv := new(big.Rat).Sub(au, bu), new(big.Rat).Sub(av, bv)
	return proofbound.RatSqrtUp(proofbound.RatAdd(new(big.Rat).Mul(du, du), new(big.Rat).Mul(dv, dv)))
}

// boxDistUp bounds the distance from the exact point p to any point of the
// box u × v, rounded up.
func boxDistUp(pu, pv *big.Rat, u, v ivl) float64 {
	far := func(p *big.Rat, a ivl) *big.Rat {
		d1 := new(big.Rat).Abs(new(big.Rat).Sub(p, a.lo))
		d2 := new(big.Rat).Abs(new(big.Rat).Sub(p, a.hi))
		if d1.Cmp(d2) > 0 {
			return d1
		}
		return d2
	}
	fu, fv := far(pu, u), far(pv, v)
	return proofbound.RatSqrtUp(proofbound.RatAdd(new(big.Rat).Mul(fu, fu), new(big.Rat).Mul(fv, fv)))
}

func ratOf2(a, b float64) (*big.Rat, *big.Rat, bool) {
	u, ok1 := proofbound.RatOf(a)
	v, ok2 := proofbound.RatOf(b)
	return u, v, ok1 && ok2
}
