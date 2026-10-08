package revolvemesh

import (
	"math"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// RevolveBasis is the unplaced world anchor of the sweep: a3 the axis
// origin, w the unit axis direction, e0 the in-plane radial direction at
// sweep angle zero, and e1 = w × e0 the sweep-velocity direction at zero —
// so a rotation by +φ about w carries e0 toward e1, the right-handed sense
// Along means.
type RevolveBasis struct {
	A3, W, E0, E1 r3.Vec
}

// RevolveLift is the record a revolve's sweep basis is built from: the sketch
// plane's frame and the resolved axis in that plane, its plane-local anchor
// (AU, AV) and direction (DU, DV).
type RevolveLift struct {
	Frame          r3.Frame
	AU, AV, DU, DV float64
}

// Basis builds the unplaced float basis: A3 = Frame.ToWorldUV(AU, AV),
// W = U·DU + V·DV, E0 = −U·DV + V·DU and E1 = W × E0.
func (l RevolveLift) Basis() RevolveBasis {
	u, v := l.Frame.U(), l.Frame.V()
	a3 := l.Frame.ToWorldUV(l.AU, l.AV)
	w := u.Scale(l.DU).Add(v.Scale(l.DV))
	e0 := u.Scale(-l.DV).Add(v.Scale(l.DU))
	return RevolveBasis{A3: a3, W: w, E0: e0, E1: w.Cross(e0)}
}

// Point evaluates the swept point at axial z, radius rho and angle phi in
// float64: xform.Apply(A3 + W·z + (E0·cos φ + E1·sin φ)·rho) over b, which
// must be l.Basis().
func (l RevolveLift) Point(b RevolveBasis, xform r3.Transform, z, rho, phi float64) r3.Vec {
	sin, cos := math.Sincos(phi)
	radial := b.E0.Scale(cos).Add(b.E1.Scale(sin))
	return xform.Apply(b.A3.Add(b.W.Scale(z)).Add(radial.Scale(rho)))
}

// ExactPointRound proves, exactly, how far held sits from the point the same
// construction denotes over dyadic rationals: Basis's frame lift and products,
// the axial and radial terms and xform.Apply, with the frame's origin, U and V,
// the axis's AU, AV, DU and DV, z, rho, cos, sin and the placement's basis and
// translation all read as exact leaves (E1 is the exact cross product of the
// exact W and E0). It answers the Radius3D bound of the per-coordinate gap,
// rounded upward, and +Inf for a non-finite leaf.
//
// cos and sin are the held values the caller's own evaluation multiplied by —
// math.Sincos(phi) for Point — so the comparison charges the frame lift and
// placement rounding alone, never the trigonometric evaluation, which is its
// own term. Every lift that evaluates the axis and the radial term in a
// different order (the clearance kernel's xform.Apply(A3) + xform.ApplyDir(W)·z)
// denotes the same exact point, so the same helper measures it. The answer is
// zero exactly where that evaluation is exact for the input at hand.
func (l RevolveLift) ExactPointRound(xform r3.Transform, z, rho, cos, sin float64, held r3.Vec) float64 {
	basis := xform.Basis()
	tr := xform.Translation()
	origin, fu, fv := l.Frame.Origin(), l.Frame.U(), l.Frame.V()
	for _, w := range [...]r3.Vec{origin, fu, fv, basis.EX, basis.EY, basis.EZ, tr} {
		if !proofbound.FiniteVec(w) {
			return math.Inf(1)
		}
	}
	for _, f := range [...]float64{l.AU, l.AV, l.DU, l.DV, z, rho, cos, sin} {
		if proofbound.IsNonFinite(f) {
			return math.Inf(1)
		}
	}
	dy := proofarith.MustDyOf
	o, du, dv := proofarith.DyVec(origin), proofarith.DyVec(fu), proofarith.DyVec(fv)
	aU, aV, dU, dV := dy(l.AU), dy(l.AV), dy(l.DU), dy(l.DV)
	var a3, w, e0 proofarith.DyV3
	for i := range 3 {
		a3[i] = proofarith.DyAdd(o[i], proofarith.DyAdd(proofarith.DyMul(du[i], aU), proofarith.DyMul(dv[i], aV)))
		w[i] = proofarith.DyAdd(proofarith.DyMul(du[i], dU), proofarith.DyMul(dv[i], dV))
		e0[i] = proofarith.DySubScalar(proofarith.DyMul(dv[i], dU), proofarith.DyMul(du[i], dV))
	}
	e1 := proofarith.DvCross(w, e0)
	rz, rr, rc, rs := dy(z), dy(rho), dy(cos), dy(sin)
	var local proofarith.DyV3
	for i := range 3 {
		radial := proofarith.DyAdd(proofarith.DyMul(e0[i], rc), proofarith.DyMul(e1[i], rs))
		local[i] = proofarith.DyAdd(
			proofarith.DyAdd(a3[i], proofarith.DyMul(w[i], rz)),
			proofarith.DyMul(radial, rr),
		)
	}
	return proofbound.ExactRigidRound(basis, tr, local, held)
}
