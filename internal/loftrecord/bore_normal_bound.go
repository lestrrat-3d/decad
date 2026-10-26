package loftrecord

import (
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/massmoment"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// BoreCylinderNormalBound bounds how far the true bore wall's unit normal
// can turn away from the Cylinder tag the shared cylinder builder publishes
// for it (topo.Face.NormalBound, agent-docs/loft-design.md §18). The record
// denotes the right circular cylinder of the given radius about the plane-local
// center (u, v), for local heights 0 through level, carried through the exact
// affine map A(x) = L·x + T. L = B·[U V N] is the held frame and placement
// read as exact rationals (massmoment.PrismRotation). The tag is the cylinder
// about the rounded center c_h = pp.Point(u, v, 0) along the rounded axis
// a_h = pp.Dir(0, 0, 1).
//
// Let q be a true wall point's radial offset in the plane, so |q| = ρ and q ⟂ e3, and
// let z be its height. Its normal is the direction of L⁻ᵀq. The tag's normal
// at the same point is the direction of P(Lq + z·Le3 − δ), where P removes
// the component along â = a_h/|a_h| and δ = c_h − A(u, v, 0). With every
// eigenvalue of LᵀL in [1 − s, 1 + s] (s the map's Stretch):
//
//   - |L⁻ᵀq − Lq| ≤ √(1+s)·s/(1−s)·ρ, since L⁻ᵀ = L(LᵀL)⁻¹;
//   - |â·Lq| ≤ (s + √(1+s)·σ)·ρ/√(1−s), where σ = |a_h × Le3|/|a_h| is
//     the component of Le3 across the tag axis;
//   - |P·Le3| = σ and |P·δ| ≤ |δ|, the center's exact lift rounding.
//
// Two vectors a, b with |a − b| ≤ ε have unit directions at most 2ε/|b|
// apart, and |L⁻ᵀq| ≥ ρ/√(1+s). With k = 1 + s ≥ √(1+s) and
// 1/(1−s) ≥ 1/√(1−s), the departure is at most
//
//	2k·[ s·(1+k)/(1−s) + σ·(k/(1−s) + level/ρ) + |δ|/ρ ]
//
// which is zero exactly when the center lift, the map and the axis are all
// exact. The bound holds at every point of the true wall between the two
// recorded section planes. It answers false for a non-finite reading or a
// bound that reaches 1, where the tag no longer names the wall's side.
func BoreCylinderNormalBound(frame r3.Frame, xform r3.Transform, u, v, level, radius float64) (float64, bool) {
	pp := prismrecord.Payload{Frame: frame, Xform: xform}
	round := pp.PointRound(u, v, 0, pp.Point(u, v, 0))
	linear, err := massmoment.PrismRotation(frame, xform)
	if err != nil {
		return 0, false
	}
	charge, err := massmoment.MapChargeOf(linear)
	if err != nil {
		return 0, false
	}
	tilt, ok := boreAxisTilt(pp.Dir(0, 0, 1), linear)
	if !ok {
		return 0, false
	}
	s := proofarith.FloatRat(charge.Stretch)
	center := proofarith.FloatRat(round)
	sigma := proofarith.FloatRat(tilt)
	rho := proofarith.FloatRat(radius)
	height := proofarith.FloatRat(math.Abs(level))
	if s == nil || center == nil || sigma == nil || rho == nil || height == nil || rho.Sign() <= 0 {
		return 0, false
	}
	one := big.NewRat(1, 1)
	k := new(big.Rat).Add(one, s)
	inv := new(big.Rat).Inv(new(big.Rat).Sub(one, s))
	stretch := new(big.Rat).Mul(new(big.Rat).Mul(s, inv), new(big.Rat).Add(one, k))
	tiltLeg := new(big.Rat).Mul(sigma, new(big.Rat).Add(new(big.Rat).Mul(k, inv), new(big.Rat).Quo(height, rho)))
	centerLeg := new(big.Rat).Quo(center, rho)
	sum := new(big.Rat).Add(new(big.Rat).Add(stretch, tiltLeg), centerLeg)
	bound := proofbound.RatFloatUp(new(big.Rat).Mul(new(big.Rat).Mul(big.NewRat(2, 1), k), sum))
	if proofbound.IsNonFinite(bound) || !(bound < 1) {
		return 0, false
	}
	return bound, true
}

// boreAxisTilt bounds σ = |a × e|/|a| from above: the length of the part of
// the denoted ruling direction e = L·e3 that lies across the held axis a.
// Both are exact dyadics, so the quotient is exact before its outward root.
func boreAxisTilt(axis r3.Vec, linear [3][3]*big.Rat) (float64, bool) {
	if !proofbound.FiniteVec(axis) {
		return 0, false
	}
	a := proofarith.DyVec(axis)
	var e proofarith.DyV3
	for i := range e {
		d, ok := proofarith.DyOfRat(linear[i][2])
		if !ok {
			return 0, false
		}
		e[i] = d
	}
	norm := proofarith.DvDot(a, a)
	if norm.Sign() <= 0 {
		return 0, false
	}
	cross := proofarith.DvCross(a, e)
	squared := new(big.Rat).Quo(proofarith.DvDot(cross, cross).Rat(), norm.Rat())
	tilt := proofbound.RatSqrtUp(squared)
	return tilt, !proofbound.IsNonFinite(tilt)
}
