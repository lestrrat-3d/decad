package revolvemesh

import (
	"math"
	"math/big"

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

// BasisRound proves, exactly, how far b, which must be l.Basis(), sits from
// the basis its construction denotes over the frame's origin, U and V and
// the axis's AU, AV, DU and DV read as exact leaves, with E1 the exact cross
// product of the exact W and E0: the L1 norm of each vector's per-component
// gap, rounded up, in the order A3, W, E0, E1. A reading that treats b's
// floats as its leaves owes these. Every entry is zero where Basis is exact
// for the inputs at hand, and +Inf for a non-finite leaf.
func (l RevolveLift) BasisRound(b RevolveBasis) [4]float64 {
	origin, fu, fv := l.Frame.Origin(), l.Frame.U(), l.Frame.V()
	inf := [4]float64{math.Inf(1), math.Inf(1), math.Inf(1), math.Inf(1)}
	for _, w := range [...]r3.Vec{origin, fu, fv, b.A3, b.W, b.E0, b.E1} {
		if !proofbound.FiniteVec(w) {
			return inf
		}
	}
	for _, f := range [...]float64{l.AU, l.AV, l.DU, l.DV} {
		if proofbound.IsNonFinite(f) {
			return inf
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
	var out [4]float64
	for k, pair := range [4]struct {
		exact proofarith.DyV3
		held  r3.Vec
	}{{a3, b.A3}, {w, b.W}, {e0, b.E0}, {e1, b.E1}} {
		gap := 0.0
		for i := range 3 {
			gap = proofbound.AbsSumUpper(gap, proofarith.DyadicFloatError(pair.exact[i], VecComponent(pair.held, i)))
		}
		out[k] = gap
	}
	return out
}

// AxisBound is the resolved axis's own proven displacement: how far each of
// RevolveLift's AU, AV, DU and DV sits from the anchor and the unit direction
// the record names (revolveaxis.Line2's four bounds).
type AxisBound struct {
	AU, AV, DU, DV float64
}

// SweptPointGap proves, exactly, how far held sits from the point a swept
// vertex denotes: the recorded plane-local point (u, v), known to within uv,
// rotated about the axis the record names by the angle whose sine and cosine
// sin and cos enclose, lifted through the frame and placed by xform.
//
// The construction is ExactPointRound's, evaluated over rational intervals:
// the frame's origin, U and V and the placement's basis and translation are
// exact leaves, the axis anchor and direction are widened by ab, and the axial
// and radial coordinates are re-expressed from (u, v) inside the comparison
// rather than read from the float (z, ρ) the build placed the vertex from.
// The answer therefore covers the rounding of that re-expression, a radius
// the build snapped onto the axis, and the axis's own anchor and direction
// error at any angle, beside the frame lift and placement rounding
// ExactPointRound measures. Every interval collapses to a point for an exact
// axis and an exactly stated point and angle, so the answer is zero exactly
// where the held evaluation is exact for them. It is the Radius3D bound of
// the worst per-coordinate gap, rounded upward, and +Inf for a non-finite
// leaf.
func (l RevolveLift) SweptPointGap(ab AxisBound, xform r3.Transform, u, v float64, uv proofbound.WalkEndBound, sin, cos proofbound.RatInterval, held r3.Vec) float64 {
	basis := xform.Basis()
	vecs := [...]r3.Vec{l.Frame.Origin(), l.Frame.U(), l.Frame.V(), basis.EX, basis.EY, basis.EZ, xform.Translation()}
	var iv [len(vecs)]proofbound.IvVec3
	for i, w := range vecs {
		enc, ok := proofbound.IvVec3Of(w)
		if !ok {
			return math.Inf(1)
		}
		iv[i] = enc
	}
	o, fu, fv, ex, ey, ez, tr := iv[0], iv[1], iv[2], iv[3], iv[4], iv[5], iv[6]
	widen := func(x, w float64) (proofbound.RatInterval, bool) {
		r, b := proofarith.FloatRat(x), proofarith.FloatRat(math.Abs(w))
		if r == nil || b == nil {
			return proofbound.RatInterval{}, false
		}
		return proofbound.IntervalWiden(proofbound.PointInterval(r), b), true
	}
	var leaves [6]proofbound.RatInterval
	for i, pair := range [...][2]float64{{l.AU, ab.AU}, {l.AV, ab.AV}, {l.DU, ab.DU}, {l.DV, ab.DV}, {u, uv.U}, {v, uv.V}} {
		enc, ok := widen(pair[0], pair[1])
		if !ok {
			return math.Inf(1)
		}
		leaves[i] = enc
	}
	aU, aV, dU, dV, pu, pv := leaves[0], leaves[1], leaves[2], leaves[3], leaves[4], leaves[5]

	du, dv := proofbound.IntervalSub(pu, aU), proofbound.IntervalSub(pv, aV)
	z := proofbound.IntervalAdd(proofbound.IntervalMul(du, dU), proofbound.IntervalMul(dv, dV))
	rho := proofbound.IntervalSub(proofbound.IntervalMul(dv, dU), proofbound.IntervalMul(du, dV))

	a3 := proofbound.IvVec3Add(o, proofbound.IvVec3Add(proofbound.IvVec3Mul(fu, aU), proofbound.IvVec3Mul(fv, aV)))
	w := proofbound.IvVec3Add(proofbound.IvVec3Mul(fu, dU), proofbound.IvVec3Mul(fv, dV))
	e0 := proofbound.IvVec3Add(proofbound.IvVec3Mul(fu, proofbound.IntervalNeg(dV)), proofbound.IvVec3Mul(fv, dU))
	e1 := proofbound.IvVec3Cross(w, e0)
	radial := proofbound.IvVec3Add(proofbound.IvVec3Mul(e0, cos), proofbound.IvVec3Mul(e1, sin))
	local := proofbound.IvVec3Add(a3, proofbound.IvVec3Add(proofbound.IvVec3Mul(w, z), proofbound.IvVec3Mul(radial, rho)))

	perCoord := 0.0
	for i := range 3 {
		placed := proofbound.IntervalAdd(
			proofbound.IntervalAdd(proofbound.IntervalMul(ex[i], local[0]), proofbound.IntervalMul(ey[i], local[1])),
			proofbound.IntervalAdd(proofbound.IntervalMul(ez[i], local[2]), tr[i]),
		)
		perCoord = math.Max(perCoord, proofbound.IntervalFloatError(placed, VecComponent(held, i)))
	}
	return proofbound.Radius3D(perCoord)
}

// CircleGap bounds how far every point of a denoted circle lies from a held
// circle about center, normal to axis, whose radius is radius. The denoted circle
// is every point C + r·(cos t·X + sin t·Y) for C, X and Y anywhere in their
// interval vectors and r in its interval, which covers an image of a circle
// under any linear map the intervals enclose. A point c′ + R with
// |c′ − center| ≤ g lies within g + |h| + |ρ − radius| of the held circle,
// h = R·â its height off the held plane and ρ ≥ |R| − |h| its in-plane
// radius, so within g + 2|R·â| + ||R| − radius|. Here
// |R·â| ≤ r·(|X·â| + |Y·â|) and, since cos²t·|X|² + sin²t·|Y|² +
// 2·sin t·cos t·X·Y lies within |X·Y| of [min, max] of |X|² and |Y|², |R|
// lies in [r_lo·√m_lo, r_hi·√m_hi]. It answers +Inf for a non-finite held
// value, an axis with no length, or a span whose lower end of |R| is not
// positive.
func CircleGap(c, x, y proofbound.IvVec3, r proofbound.RatInterval, center, axis r3.Vec, radius float64) float64 {
	if !proofbound.FiniteVec(center) || !proofbound.FiniteVec(axis) || proofbound.IsNonFinite(radius) {
		return math.Inf(1)
	}
	gap := 0.0
	for i := range 3 {
		gap = math.Max(gap, proofbound.IntervalFloatError(c[i], VecComponent(center, i)))
	}
	gap = proofbound.Radius3D(gap)
	a, ok := proofbound.IvVec3Of(axis)
	if !ok {
		return math.Inf(1)
	}
	norm, ok := proofbound.IntervalSqrt(proofbound.IvVec3NormSq(a))
	if !ok || norm.Lo.Sign() <= 0 {
		return math.Inf(1)
	}
	tiltNum := new(big.Rat).Add(proofbound.IntervalAbsUpper(proofbound.IvVec3Dot(x, a)), proofbound.IntervalAbsUpper(proofbound.IvVec3Dot(y, a)))
	rHi := proofbound.IntervalAbsUpper(r)
	tilt := new(big.Rat).Mul(rHi, tiltNum)
	tilt.Quo(tilt, norm.Lo)
	xx, yy := proofbound.IvVec3NormSq(x), proofbound.IvVec3NormSq(y)
	xy := proofbound.IntervalAbsUpper(proofbound.IvVec3Dot(x, y))
	mLo := new(big.Rat).Sub(proofbound.RatMin(xx.Lo, yy.Lo), xy)
	mHi := new(big.Rat).Add(proofbound.RatMax(xx.Hi, yy.Hi), xy)
	if mLo.Sign() <= 0 || r.Lo.Sign() <= 0 {
		return math.Inf(1)
	}
	m, ok := proofbound.IntervalSqrt(proofbound.Interval(mLo, mHi))
	if !ok {
		return math.Inf(1)
	}
	reachLo := new(big.Rat).Mul(r.Lo, m.Lo)
	reachHi := new(big.Rat).Mul(r.Hi, m.Hi)
	held := new(big.Rat).SetFloat64(radius)
	radial := proofbound.RatMax(new(big.Rat).Sub(reachHi, held), new(big.Rat).Sub(held, reachLo))
	if radial.Sign() < 0 {
		radial = new(big.Rat)
	}
	return proofbound.AbsSumUpper(gap, proofbound.RatFloatUp(new(big.Rat).Add(new(big.Rat).Mul(big.NewRat(2, 1), tilt), radial)))
}

// placedLeaves reads the exact leaves SweptPointGap's construction lifts
// through: the frame's origin, U and V, the axis anchor and direction
// widened by ab, and the placement's basis and translation.
type placedLeaves struct {
	o, fu, fv, ex, ey, ez, tr proofbound.IvVec3
	aU, aV, dU, dV            proofbound.RatInterval
}

func (l RevolveLift) placedLeaves(ab AxisBound, xform r3.Transform) (placedLeaves, bool) {
	basis := xform.Basis()
	vecs := [...]r3.Vec{l.Frame.Origin(), l.Frame.U(), l.Frame.V(), basis.EX, basis.EY, basis.EZ, xform.Translation()}
	var iv [len(vecs)]proofbound.IvVec3
	for i, w := range vecs {
		enc, ok := proofbound.IvVec3Of(w)
		if !ok {
			return placedLeaves{}, false
		}
		iv[i] = enc
	}
	var leaves [4]proofbound.RatInterval
	for i, pair := range [...][2]float64{{l.AU, ab.AU}, {l.AV, ab.AV}, {l.DU, ab.DU}, {l.DV, ab.DV}} {
		enc, ok := widenLeaf(pair[0], pair[1])
		if !ok {
			return placedLeaves{}, false
		}
		leaves[i] = enc
	}
	return placedLeaves{o: iv[0], fu: iv[1], fv: iv[2], ex: iv[3], ey: iv[4], ez: iv[5], tr: iv[6],
		aU: leaves[0], aV: leaves[1], dU: leaves[2], dV: leaves[3]}, true
}

// basis is the axis basis (A3, W, E0, E1) over the leaves, E1 the cross
// product of W and E0.
func (p placedLeaves) basis() (a3, w, e0, e1 proofbound.IvVec3) {
	a3 = proofbound.IvVec3Add(p.o, proofbound.IvVec3Add(proofbound.IvVec3Mul(p.fu, p.aU), proofbound.IvVec3Mul(p.fv, p.aV)))
	w = proofbound.IvVec3Add(proofbound.IvVec3Mul(p.fu, p.dU), proofbound.IvVec3Mul(p.fv, p.dV))
	e0 = proofbound.IvVec3Add(proofbound.IvVec3Mul(p.fu, proofbound.IntervalNeg(p.dV)), proofbound.IvVec3Mul(p.fv, p.dU))
	return a3, w, e0, proofbound.IvVec3Cross(w, e0)
}

// dir carries a direction through the placement's basis.
func (p placedLeaves) dir(v proofbound.IvVec3) proofbound.IvVec3 {
	return proofbound.IvVec3Add(proofbound.IvVec3Add(proofbound.IvVec3Mul(p.ex, v[0]), proofbound.IvVec3Mul(p.ey, v[1])), proofbound.IvVec3Mul(p.ez, v[2]))
}

// point carries a point through the placement.
func (p placedLeaves) point(v proofbound.IvVec3) proofbound.IvVec3 {
	return proofbound.IvVec3Add(p.dir(v), p.tr)
}

// LatitudeGap is an Edge's curve bound for a junction's latitude circle or
// arc: the recorded plane-local point (u, v), within uv, swept about the
// axis the record names (its anchor and direction widened by ab) and placed
// by xform. Every point it reaches at any angle is C + ρ·(cos φ·B·E0 +
// sin φ·B·E1) with C = B·(A3 + W·z) + t, so CircleGap bounds the whole
// circle, and with it any arc of it, against the held circle.
func (l RevolveLift) LatitudeGap(ab AxisBound, xform r3.Transform, u, v float64, uv proofbound.WalkEndBound, center, axis r3.Vec, radius float64) float64 {
	p, ok := l.placedLeaves(ab, xform)
	if !ok {
		return math.Inf(1)
	}
	pu, okU := widenLeaf(u, uv.U)
	pv, okV := widenLeaf(v, uv.V)
	if !okU || !okV {
		return math.Inf(1)
	}
	du, dv := proofbound.IntervalSub(pu, p.aU), proofbound.IntervalSub(pv, p.aV)
	z := proofbound.IntervalAdd(proofbound.IntervalMul(du, p.dU), proofbound.IntervalMul(dv, p.dV))
	rho := proofbound.IntervalSub(proofbound.IntervalMul(dv, p.dU), proofbound.IntervalMul(du, p.dV))
	if rho.Hi.Sign() < 0 {
		rho = proofbound.IntervalNeg(rho)
	}
	a3, w, e0, e1 := p.basis()
	c := p.point(proofbound.IvVec3Add(a3, proofbound.IvVec3Mul(w, z)))
	return CircleGap(c, p.dir(e0), p.dir(e1), rho, center, axis, radius)
}

// CapArcGap is an Edge's curve bound for a partial sweep's cap copy of a
// recorded circular segment: the circle of radius r about the plane-local
// centre (cu, cv), the centre within cuv and the radius within rBound,
// re-expressed about the axis the record names (widened by ab), rotated to
// the end angle whose sine and cosine sin and cos enclose, and placed by
// xform. A plane point p maps to B·(A3 + W·z(p) + ρ(p)·R) + t with
// R = E0·cos + E1·sin, and z and ρ are linear in p, so the copy is
// C + r·(cos t·X + sin t·Y) with X = B·(W·dU − R·dV), Y = B·(W·dV + R·dU)
// and C the image of the centre; CircleGap bounds it.
func (l RevolveLift) CapArcGap(ab AxisBound, xform r3.Transform, cu, cv float64, cuv proofbound.WalkEndBound, r, rBound float64, sin, cos proofbound.RatInterval, center, axis r3.Vec, radius float64) float64 {
	p, ok := l.placedLeaves(ab, xform)
	if !ok {
		return math.Inf(1)
	}
	pu, okU := widenLeaf(cu, cuv.U)
	pv, okV := widenLeaf(cv, cuv.V)
	rr, okR := widenLeaf(r, rBound)
	if !okU || !okV || !okR {
		return math.Inf(1)
	}
	du, dv := proofbound.IntervalSub(pu, p.aU), proofbound.IntervalSub(pv, p.aV)
	z := proofbound.IntervalAdd(proofbound.IntervalMul(du, p.dU), proofbound.IntervalMul(dv, p.dV))
	rho := proofbound.IntervalSub(proofbound.IntervalMul(dv, p.dU), proofbound.IntervalMul(du, p.dV))
	a3, w, e0, e1 := p.basis()
	rot := proofbound.IvVec3Add(proofbound.IvVec3Mul(e0, cos), proofbound.IvVec3Mul(e1, sin))
	c := p.point(proofbound.IvVec3Add(a3, proofbound.IvVec3Add(proofbound.IvVec3Mul(w, z), proofbound.IvVec3Mul(rot, rho))))
	x := proofbound.IvVec3Sub(proofbound.IvVec3Mul(w, p.dU), proofbound.IvVec3Mul(rot, p.dV))
	y := proofbound.IvVec3Add(proofbound.IvVec3Mul(w, p.dV), proofbound.IvVec3Mul(rot, p.dU))
	return CircleGap(c, p.dir(x), p.dir(y), rr, center, axis, radius)
}
