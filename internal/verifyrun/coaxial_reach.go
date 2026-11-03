package verifyrun

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolveplan"
)

// This file is the coaxial separator's Revolve reach
// (agent-docs/interference-design.md §3.2, "The coaxial separator"): a
// Revolve whose lifted axis is a round cylinder's axis misses the cylinder's
// open interior when every one of its points lies at least the cylinder's
// radius from that axis. Its point at radius ρ and angle φ sits ρ·√Q(φ) from
// the axis, with Q the quadratic form of its lifted radial directions across
// the axis: 1 under an exactly orthonormal lift, and 1 within rounding under a
// lift that is a rotation by rounding only. Q is read exactly, in rationals,
// and bounded over the Revolve's sweep, so a partial Revolve whose rounding
// stretches it outward along its own angles is decided though a whole turn of
// the same lift would dip inside.

// across returns the part of v across the direction d.
func across(v, d [3]*big.Rat) [3]*big.Rat {
	k := new(big.Rat).Quo(dotVec(v, d), dotVec(d, d))
	return subVec(v, scaleVec(d, k))
}

// revolveClearsCylinder reads a Revolve the meridian separator reads
// (revolveMeridian) whose lifted axis is c's axis line. Its point at axial z,
// radius ρ and angle φ is A + z·D + ρ·(cos φ·a₁ + sin φ·a₂) in the world, A
// and D the lifted anchor and axis and a₁, a₂ the lifted radial directions,
// so it sits ρ·√Q(φ) from c's axis, with Q(φ) = |cos φ·P a₁ + sin φ·P a₂|²
// and P the part across the axis. It misses c's open interior when Q is at
// least κ = R²/ρ_min² over its sweep, ρ_min the least ρ of its region.
func revolveClearsCylinder(p revolveplan.Payload, c coaxialCylinder) bool {
	if !c.iso {
		return false
	}
	m, ok := revolveMeridian(p)
	if !ok {
		return false
	}
	var rhoMin *big.Rat
	if m.disc != nil {
		rhoMin = new(big.Rat).Sub(m.disc[0], m.disc[2])
	} else {
		for _, q := range m.region {
			if rhoMin == nil || q[0].Cmp(rhoMin) < 0 {
				rhoMin = q[0]
			}
		}
	}
	if rhoMin == nil || rhoMin.Sign() <= 0 {
		return false
	}
	// The denoted point is Xform(A3 + W·z + (E0·cos φ + E1·sin φ)·ρ), with
	// A3 = O + U·au + V·av, W = U·du + V·dv, E0 = −U·dv + V·du and E1 = W×E0,
	// every term exact (revolvemesh.RevolveLift.ExactPointRound).
	uR, vR, o := ratVec(p.Frame.U()), ratVec(p.Frame.V()), ratVec(p.Frame.Origin())
	au, av, du, dv := m.anchor[0], m.anchor[1], m.dir[0], m.dir[1]
	plane := func(x, y *big.Rat) [3]*big.Rat { return addVec(scaleVec(uR, x), scaleVec(vR, y)) }
	w := plane(du, dv)
	e0 := plane(new(big.Rat).Neg(dv), du)
	e1 := crossVec(w, e0)
	basis := p.Xform.Basis()
	ex, ey, ez := ratVec(basis.EX), ratVec(basis.EY), ratVec(basis.EZ)
	apply := func(q [3]*big.Rat) [3]*big.Rat {
		return addVec(addVec(scaleVec(ex, q[0]), scaleVec(ey, q[1])), scaleVec(ez, q[2]))
	}
	anchor := addVec(apply(addVec(o, plane(au, av))), ratVec(p.Xform.Translation()))
	dir := apply(w)
	cd := c.l[2]
	if !isZero(crossVec(dir, cd)) || !isZero(crossVec(subVec(anchor, c.anchor()), cd)) {
		return false
	}
	a1, a2 := across(apply(e0), cd), across(apply(e1), cd)
	kappa := new(big.Rat).Quo(new(big.Rat).Mul(c.radius, c.radius), new(big.Rat).Mul(rhoMin, rhoMin))
	qa := new(big.Rat).Sub(dotVec(a1, a1), kappa)
	qb := new(big.Rat).Sub(dotVec(a2, a2), kappa)
	qc := dotVec(a1, a2)
	if p.Full {
		// The least eigenvalue of [[qa, qc], [qc, qb]] is non-negative.
		return qa.Sign() >= 0 && qb.Sign() >= 0 &&
			new(big.Rat).Mul(qa, qb).Cmp(new(big.Rat).Mul(qc, qc)) >= 0
	}
	return sweepFormNonNegative(p, qa, qb, qc)
}

// sweepFormNonNegative reports whether the quadratic form
// qa·c² + qb·s² + 2·qc·c·s is non-negative at every direction (cos φ, sin φ)
// of p's partial sweep. The sweep is narrower than a half turn, so each of
// its directions is a non-negative combination α·e₀ + β·e₁ of its two end
// directions, where the form is α²·q₀ + β²·q₁ + 2αβ·b; that is non-negative
// when q₀ and q₁ are, and b is too or b² ≤ q₀·q₁. The ends' sines and
// cosines are read as rational enclosures, so every term is a bound.
func sweepFormNonNegative(p revolveplan.Payload, qa, qb, qc *big.Rat) bool {
	if !(p.Phi0 <= p.Phi1) {
		return false
	}
	enc0, ok0 := p.Den.Phi0.EnclosureFor(p.Phi0)
	enc1, ok1 := p.Den.Phi1.EnclosureFor(p.Phi1)
	if !ok0 || !ok1 {
		return false
	}
	width := proofbound.IntervalSub(enc1, enc0)
	if width.Lo.Sign() < 0 || width.Hi.Cmp(proofbound.PiLower) >= 0 {
		return false
	}
	s0, c0, ok0 := p.Den.Phi0.SinCosFor(p.Phi0)
	s1, c1, ok1 := p.Den.Phi1.SinCosFor(p.Phi1)
	if !ok0 || !ok1 {
		return false
	}
	form := func(ca, sa, cb, sb proofbound.RatInterval) proofbound.RatInterval {
		cross := proofbound.IntervalAdd(proofbound.IntervalMul(ca, sb), proofbound.IntervalMul(sa, cb))
		return proofbound.IntervalAdd(proofbound.IntervalAdd(
			proofbound.IntervalScale(proofbound.IntervalMul(ca, cb), qa),
			proofbound.IntervalScale(proofbound.IntervalMul(sa, sb), qb)),
			proofbound.IntervalScale(cross, qc))
	}
	q0, q1 := form(c0, s0, c0, s0), form(c1, s1, c1, s1)
	b := form(c0, s0, c1, s1)
	if q0.Lo.Sign() < 0 || q1.Lo.Sign() < 0 {
		return false
	}
	if b.Lo.Sign() >= 0 {
		return true
	}
	worst := new(big.Rat).Mul(b.Lo, b.Lo)
	if hi := new(big.Rat).Mul(b.Hi, b.Hi); hi.Cmp(worst) > 0 {
		worst = hi
	}
	return new(big.Rat).Mul(q0.Lo, q1.Lo).Cmp(worst) >= 0
}
