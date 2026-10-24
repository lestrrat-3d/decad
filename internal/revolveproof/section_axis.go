package revolveproof

import (
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
)

// This file holds the section-certificate arithmetic about a general axis A
// (agent-docs/revolve-axis-design.md §5.2, §5.3). A certificate whose General
// is nil takes the +X code beside each caller unchanged. Every implicit form
// here is Sketch's own: F = |(p−O)×A|² − s²((p−O)·A)², homogeneous of degree
// two in A, so a Sketch margin is |A|² times its millimetre reading.

// implicitMarginExceeds reports whether an exact Sketch margin on |F| exceeds
// the implicit cost, a bound in mm² from RevolveImplicitDisplacement. A
// general certificate's margin is in |A|²·mm², so the cost is multiplied by the
// exact A·A first (§5.3).
func implicitMarginExceeds(margin *big.Rat, cost float64, section facetedrecord.RevolveSectionCertificate) bool {
	if section.General == nil {
		return margin.Cmp(proofarith.FloatRat(cost)) > 0
	}
	scaled := new(big.Rat).Mul(proofarith.FloatRat(cost), section.Axis.NormSq().Rat())
	return margin.Cmp(scaled) > 0
}

// implicitDisplacementAbout is RevolveImplicitDisplacement for a general
// certificate. Every fan vertex needs p·A > 0 exactly (§4.4). Its axial
// distance from P*, (p·A − Z_a)/|A| converted per §3.4 and widened by the
// mesh bound, must exceed δ_a, so the fan lies past both the true and the
// reported apex. The displacement d is the mesh bound plus the §5.2 cone gap
// over the fan's reach. The cost (1+s²)·d·(2r+d) bounds the change of
// ρ² − s²h² when a point moves by d, where r bounds both the radial ρ and
// the axial h: the apex may lie behind the origin, so r takes the larger of
// the fan's distance from the origin and its axial reach from P*.
func implicitDisplacementAbout(section facetedrecord.RevolveSectionCertificate,
	base facetedrecord.Payload) (float64, bool) {
	general, axis := section.General, section.Axis
	if !proofbound.FiniteVec(r3.Vec{X: base.MeshBound}) || base.MeshBound < 0 || len(base.Verts) < 4 {
		return 0, false
	}
	var axialLo, axialHi *big.Rat
	reach := 0.0
	for _, world := range base.Verts[1:] {
		if !proofbound.FiniteVec(world) {
			return 0, false
		}
		axial := axis.Axial(world)
		if axial.Sign() <= 0 {
			return 0, false
		}
		value := axial.Rat()
		if axialLo == nil || value.Cmp(axialLo) < 0 {
			axialLo = value
		}
		if axialHi == nil || value.Cmp(axialHi) > 0 {
			axialHi = value
		}
		reach = max(reach, proofbound.DvLenUpper(proofbound.HeldDelta(world, r3.Vec{})))
	}
	zLo, loOK := axis.ScaledDown(new(big.Rat).Sub(axialLo, general.Apex))
	zHi, hiOK := axis.ScaledUp(new(big.Rat).Sub(axialHi, general.Apex))
	if !loOK || !hiOK {
		return 0, false
	}
	zLo = math.Nextafter(zLo-base.MeshBound, math.Inf(-1))
	zHi = proofbound.AbsSumUpper(zHi, base.MeshBound)
	if !(zLo > general.ApexGap) {
		return 0, false
	}
	slope := general.Cone.Slope
	d := proofbound.AbsSumUpper(base.MeshBound, general.ConeGap(zHi))
	r := proofbound.AbsSumUpper(max(reach, zHi), d)
	cost := proofbound.ProductUpper(proofbound.AbsSumUpper(1,
		proofbound.ProductUpper(slope, slope)),
		proofbound.ProductUpper(d, proofbound.AbsSumUpper(proofbound.ProductUpper(2, r), d)))
	return cost, !proofbound.IsNonFinite(cost)
}

// axisRats holds the exact axis vector, the reported apex, the reported
// slope and the root meridian intercept B = −k*·Z_a of a general certificate.
type axisRats struct {
	a, apex     [3]*big.Rat
	slope, k, b *big.Rat
}

func axisRatsOf(section facetedrecord.RevolveSectionCertificate) axisRats {
	general := section.General
	v, o := section.Axis.Vec(), general.Cone.Apex
	return axisRats{
		a:     [3]*big.Rat{proofarith.FloatRat(v.X), proofarith.FloatRat(v.Y), proofarith.FloatRat(v.Z)},
		apex:  [3]*big.Rat{proofarith.FloatRat(o.X), proofarith.FloatRat(o.Y), proofarith.FloatRat(o.Z)},
		slope: proofarith.FloatRat(general.Cone.Slope),
		k:     general.Slope,
		b:     new(big.Rat).Neg(new(big.Rat).Mul(general.Slope, general.Apex)),
	}
}

func ratDot(p, q [3]*big.Rat) *big.Rat {
	out := new(big.Rat)
	for i := range p {
		out.Add(out, new(big.Rat).Mul(p[i], q[i]))
	}
	return out
}

func ratCross(p, q [3]*big.Rat) [3]*big.Rat {
	component := func(i, j int) *big.Rat {
		return new(big.Rat).Sub(new(big.Rat).Mul(p[i], q[j]), new(big.Rat).Mul(p[j], q[i]))
	}
	return [3]*big.Rat{component(1, 2), component(2, 0), component(0, 1)}
}

func ratSub(p, q [3]*big.Rat) [3]*big.Rat {
	var out [3]*big.Rat
	for i := range p {
		out[i] = new(big.Rat).Sub(p[i], q[i])
	}
	return out
}

// coneForm is Sketch's F of the reported cone at p, and coneDerivative its
// derivative along the direction d.
func (c axisRats) coneForm(p [3]*big.Rat) *big.Rat {
	w := ratSub(p, c.apex)
	cross := ratCross(w, c.a)
	h := ratDot(w, c.a)
	return new(big.Rat).Sub(ratDot(cross, cross),
		new(big.Rat).Mul(new(big.Rat).Mul(c.slope, c.slope), new(big.Rat).Mul(h, h)))
}

func (c axisRats) coneDerivative(p, d [3]*big.Rat) *big.Rat {
	w := ratSub(p, c.apex)
	value := new(big.Rat).Sub(ratDot(ratCross(w, c.a), ratCross(d, c.a)),
		new(big.Rat).Mul(new(big.Rat).Mul(c.slope, c.slope),
			new(big.Rat).Mul(ratDot(w, c.a), ratDot(d, c.a))))
	return value.Mul(value, big.NewRat(2, 1))
}

// sourceRadius is the true root meridian's A-scaled radius k*·(p·A) + B at
// p's axial coordinate, and sourceForm the true root cone's
// |p×A|² − sourceRadius².
func (c axisRats) sourceRadius(p [3]*big.Rat) *big.Rat {
	return new(big.Rat).Add(new(big.Rat).Mul(c.k, ratDot(p, c.a)), c.b)
}

func (c axisRats) sourceForm(p [3]*big.Rat) *big.Rat {
	cross := ratCross(p, c.a)
	radius := c.sourceRadius(p)
	return new(big.Rat).Sub(ratDot(cross, cross), new(big.Rat).Mul(radius, radius))
}

// seatShiftAbout is revolveSeatPoint's bracket for a general certificate
// (§5.3). On the exact connector q + t·d, t in [0, 1], the reported cone's F
// is quadratic in t, its derivative is linear, and both derivative ends share
// a nonzero sign, so |F'| ≥ minD over the whole line. The difference
// D = F − F_source of the reported and the true root cone is quadratic in t
// too, and its exact maximum over [0, 1] comes from the two ends and the
// vertex. Both endpoint values of F must exceed that maximum, so the true
// contact is the one root of F_source and lies within maxD/minD of the
// certified root in t. It returns that parameter shift, rounded upward.
func seatShiftAbout(section facetedrecord.RevolveSectionCertificate, q, endQ, d [3]*big.Rat) (float64, bool) {
	c := axisRatsOf(section)
	f0, f1 := c.coneForm(q), c.coneForm(endQ)
	d0, d1 := c.coneDerivative(q, d), c.coneDerivative(endQ, d)
	if d0.Sign() == 0 || d0.Sign() != d1.Sign() || f0.Sign() == f1.Sign() {
		return 0, false
	}
	if c.sourceRadius(q).Sign() <= 0 || c.sourceRadius(endQ).Sign() <= 0 ||
		c.sourceForm(q).Sign() == c.sourceForm(endQ).Sign() {
		return 0, false
	}
	var mid [3]*big.Rat
	for i := range mid {
		mid[i] = new(big.Rat).Add(q[i], new(big.Rat).Quo(d[i], big.NewRat(2, 1)))
	}
	difference := func(p [3]*big.Rat, f *big.Rat) *big.Rat {
		return new(big.Rat).Sub(f, c.sourceForm(p))
	}
	at0, at1 := difference(q, f0), difference(endQ, f1)
	atMid := difference(mid, c.coneForm(mid))
	// D(t) = α t² + β t + D(0), with D(1/2) fixing α.
	alpha := new(big.Rat).Sub(new(big.Rat).Mul(big.NewRat(2, 1), new(big.Rat).Add(at0, at1)),
		new(big.Rat).Mul(big.NewRat(4, 1), atMid))
	beta := new(big.Rat).Sub(new(big.Rat).Sub(at1, at0), alpha)
	maxD := proofbound.RatMax(new(big.Rat).Abs(at0), new(big.Rat).Abs(at1))
	if alpha.Sign() != 0 {
		vertex := new(big.Rat).Quo(new(big.Rat).Neg(beta), new(big.Rat).Mul(big.NewRat(2, 1), alpha))
		if vertex.Sign() > 0 && vertex.Cmp(big.NewRat(1, 1)) < 0 {
			value := new(big.Rat).Add(new(big.Rat).Mul(new(big.Rat).Add(
				new(big.Rat).Mul(alpha, vertex), beta), vertex), at0)
			maxD = proofbound.RatMax(maxD, value.Abs(value))
		}
	}
	if new(big.Rat).Abs(f0).Cmp(maxD) <= 0 || new(big.Rat).Abs(f1).Cmp(maxD) <= 0 {
		return 0, false
	}
	minD := proofbound.RatMin(new(big.Rat).Abs(d0), new(big.Rat).Abs(d1))
	shift := proofbound.RatFloatUp(new(big.Rat).Quo(maxD, minD))
	return shift, !proofbound.IsNonFinite(shift)
}

// fitMeridianDifferenceAbout is revolveFitMeridianDifference for a general
// certificate. Over the world box of a fitted-spline range it bounds
// F_blank − F_cone, where F_blank = |p×A|² − (k*·(p·A) + B)² is the true root
// cone, and its parameter derivative. With u = (p−O)·A and c_O = O·A the
// difference splits into two parts, each read without cancelling a large
// product:
//
//   - |p×A|² − |(p−O)×A|² = 2·p·W − |O×A|², with W = (A·A)·O − c_O·A exact;
//   - s²u² − (k*(u + c_O) + B)² = gap·sum, with gap = (s−k*)·u − c and
//     sum = (s+k*)·u + c, where c = k*·c_O + B.
//
// The derivative is 2·W·p' + 2·(s(s−k*)·u + k*·gap)·u', with u' = p'·A.
func fitMeridianDifferenceAbout(section facetedrecord.RevolveSectionCertificate, cone sketch.ConeSection,
	frame r3.Frame, pointLo, pointHi, tangentLo, tangentHi [2]float64,
) (*big.Rat, *big.Rat, [3]proofbound.RatInterval, bool) {
	var empty [3]proofbound.RatInterval
	if cone != section.ConeSection() || !(cone.Slope > 0) || math.IsInf(cone.Slope, 0) {
		return nil, nil, empty, false
	}
	point, ok := proofbound.FrameBoxWorld(frame, frame.Origin(), pointLo, pointHi)
	if !ok {
		return nil, nil, empty, false
	}
	tangent, ok := proofbound.FrameBoxWorld(frame, r3.Vec{}, tangentLo, tangentHi)
	if !ok {
		return nil, nil, empty, false
	}
	c := axisRatsOf(section)
	dotBox := func(box [3]proofbound.RatInterval, v [3]*big.Rat) proofbound.RatInterval {
		out := proofbound.PointInterval(new(big.Rat))
		for i := range box {
			out = proofbound.IntervalAdd(out, proofbound.IntervalScale(box[i], v[i]))
		}
		return out
	}
	apexAxial := ratDot(c.apex, c.a)
	u := proofbound.IntervalSub(dotBox(point, c.a), proofbound.PointInterval(apexAxial))
	rootRadius := proofbound.IntervalAdd(proofbound.IntervalScale(dotBox(point, c.a), c.k),
		proofbound.PointInterval(c.b))
	if rootRadius.Lo.Sign() <= 0 || u.Lo.Sign() <= 0 {
		return nil, nil, empty, false
	}
	normSq := ratDot(c.a, c.a)
	var w [3]*big.Rat
	for i := range w {
		w[i] = new(big.Rat).Sub(new(big.Rat).Mul(normSq, c.apex[i]), new(big.Rat).Mul(apexAxial, c.a[i]))
	}
	apexCross := ratCross(c.apex, c.a)
	two := big.NewRat(2, 1)
	first := proofbound.IntervalSub(proofbound.IntervalScale(dotBox(point, w), two),
		proofbound.PointInterval(ratDot(apexCross, apexCross)))
	offset := new(big.Rat).Add(new(big.Rat).Mul(c.k, apexAxial), c.b)
	deltaSlope := new(big.Rat).Sub(c.slope, c.k)
	gap := proofbound.IntervalSub(proofbound.IntervalScale(u, deltaSlope), proofbound.PointInterval(offset))
	sum := proofbound.IntervalAdd(proofbound.IntervalScale(u, new(big.Rat).Add(c.slope, c.k)),
		proofbound.PointInterval(offset))
	difference := proofbound.IntervalAdd(first, proofbound.IntervalMul(gap, sum))
	coefficient := proofbound.IntervalAdd(
		proofbound.IntervalScale(proofbound.IntervalScale(u, c.slope), deltaSlope),
		proofbound.IntervalScale(gap, c.k))
	derivative := proofbound.IntervalScale(proofbound.IntervalAdd(dotBox(tangent, w),
		proofbound.IntervalMul(coefficient, dotBox(tangent, c.a))), two)
	return proofbound.IntervalAbsUpper(difference), proofbound.IntervalAbsUpper(derivative), point, true
}

// coneResidualAbout is CircleFragmentSide's residual |q×A|² − s²(q·A)² over a
// world box q of offsets from the cone apex, for a cone about any axis A. It
// reports false unless q·A is strictly positive over the box, the half-cone
// Sketch's positive section names.
func coneResidualAbout(q [3]proofbound.RatInterval, cone sketch.ConeSection) (proofbound.RatInterval, bool) {
	a := [3]*big.Rat{proofarith.FloatRat(cone.Axis.X), proofarith.FloatRat(cone.Axis.Y),
		proofarith.FloatRat(cone.Axis.Z)}
	h := proofbound.PointInterval(new(big.Rat))
	for i := range q {
		h = proofbound.IntervalAdd(h, proofbound.IntervalScale(q[i], a[i]))
	}
	if h.Lo.Sign() <= 0 {
		return proofbound.RatInterval{}, false
	}
	radial := proofbound.PointInterval(new(big.Rat))
	for _, ij := range [3][2]int{{1, 2}, {2, 0}, {0, 1}} {
		i, j := ij[0], ij[1]
		component := proofbound.IntervalSub(proofbound.IntervalScale(q[i], a[j]),
			proofbound.IntervalScale(q[j], a[i]))
		radial = proofbound.IntervalAdd(radial, proofbound.IntervalSquare(component))
	}
	slope2 := new(big.Rat).Mul(proofarith.FloatRat(cone.Slope), proofarith.FloatRat(cone.Slope))
	return proofbound.IntervalSub(radial, proofbound.IntervalScale(proofbound.IntervalSquare(h), slope2)), true
}

// rootAngle returns the angle of a root-boundary node about the section's
// axis: atan2(Z, Y) for a +X certificate, and atan2(p·E1, p·E0) in the
// blank's own basis for a general one (agent-docs/revolve-axis-design.md §3.7).
func rootAngle(section facetedrecord.RevolveSectionCertificate, p r3.Vec) float64 {
	if section.General == nil {
		return math.Atan2(p.Z, p.Y)
	}
	return math.Atan2(p.Dot(section.General.E1), p.Dot(section.General.E0))
}
