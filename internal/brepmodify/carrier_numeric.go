package brepmodify

import (
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/offset2d"
	"github.com/lestrrat-3d/decad/internal/prismcells"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionaudit"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/units"
)

// This file reads one recorded carrier at a selected corner as exact data and
// computes the quantities of agent-docs/carrier-record-design.md §7.5 from it.

// cbCarrier is one recorded segment's carrier at a corner: a line through its
// recorded Start with the End − Start direction in the walk's travel sense,
// or a circle of its recorded centre and radius (|Start − Center| for an
// ArcSeg, an enclosure).
type cbCarrier struct {
	seg    sectionrecord.CurveSegment
	line   bool
	ov     offset2d.CornerCarrier
	eps    float64
	inside float64
	flip   bool // line: the oriented direction runs End → Start

	su, sv, du, dv *big.Rat // line: exact Start and oriented End − Start
	length         ivl      // line: |End − Start|
	cu, cv         *big.Rat // circle: exact centre
	rad            ivl      // circle: radius enclosure
}

// newCBCarrier reads seg as the carrier of walk w with entry eps. ok=false is
// CB0: the carrier is not a finite line or circle, or a displaced circle has
// R < 2ε.
func newCBCarrier(seg sectionrecord.CurveSegment, w survey2d.SideWalk, eps float64) (cbCarrier, bool) {
	c := cbCarrier{seg: seg, eps: eps, inside: 1}
	if w.Th1 < w.Th0 {
		c.inside = -1
	}
	switch s := seg.(type) {
	case sectionrecord.LineSeg:
		su, sv, ok1 := ratOf2(s.Start.U, s.Start.V)
		eu, ev, ok2 := ratOf2(s.End.U, s.End.V)
		if !ok1 || !ok2 {
			return cbCarrier{}, false
		}
		c.line = true
		c.ov = offset2d.LineCarrier(s.Start, s.End, w)
		c.su, c.sv = su, sv
		c.du, c.dv = new(big.Rat).Sub(eu, su), new(big.Rat).Sub(ev, sv)
		if c.ov.DU*(s.End.U-s.Start.U)+c.ov.DV*(s.End.V-s.Start.V) < 0 {
			c.du, c.dv = new(big.Rat).Neg(c.du), new(big.Rat).Neg(c.dv)
			c.flip = true
		}
		l, ok := sqrtIv(ivPoint(proofbound.RatAdd(new(big.Rat).Mul(c.du, c.du), new(big.Rat).Mul(c.dv, c.dv))))
		if !ok || l.lo.Sign() <= 0 {
			return cbCarrier{}, false
		}
		c.length = l
		return c, true
	case sectionrecord.CircleSeg:
		r, err := s.Radius.In(units.Millimeter)
		rr, ok := ivFloat(r)
		cu, cv, okc := ratOf2(s.Center.U, s.Center.V)
		if err != nil || !ok || !okc || !(r > 0) {
			return cbCarrier{}, false
		}
		c.ov = offset2d.CircleCarrier(s.Center, r)
		c.cu, c.cv, c.rad = cu, cv, rr
	case sectionrecord.ArcSeg:
		cu, cv, okc := ratOf2(s.Center.U, s.Center.V)
		xu, xv, oks := ratOf2(s.Start.U, s.Start.V)
		if !okc || !oks {
			return cbCarrier{}, false
		}
		du, dv := new(big.Rat).Sub(xu, cu), new(big.Rat).Sub(xv, cv)
		rr, ok := sqrtIv(ivPoint(proofbound.RatAdd(new(big.Rat).Mul(du, du), new(big.Rat).Mul(dv, dv))))
		if !ok || rr.lo.Sign() <= 0 {
			return cbCarrier{}, false
		}
		c.ov = offset2d.CircleCarrier(s.Center, math.Hypot(s.Start.U-s.Center.U, s.Start.V-s.Center.V))
		c.cu, c.cv, c.rad = cu, cv, rr
	default:
		return cbCarrier{}, false
	}
	// A displaced circle needs R ≥ 2ε (CB0).
	if eps > 0 && !(proofbound.ProductUpper(2, eps) <= c.rLo()) {
		return cbCarrier{}, false
	}
	return c, true
}

func (c cbCarrier) rLo() float64 { return c.rad.loDown() }
func (c cbCarrier) rHi() float64 { return c.rad.hiUp() }
func (c cbCarrier) lenLo() float64 {
	return c.length.loDown()
}

// theta1 is θ of Table CB: how far the carrier's unit tangent at the corner can
// turn between the recorded and the denoted section, 4ε/ℓ for a line and
// 2(δ_J + a_J + ε)/(R − ε) for a circle.
func (c cbCarrier) theta1(deltaJ, aJ float64) float64 {
	if c.line {
		if c.eps == 0 {
			return 0
		}
		return proofbound.DivUpper(proofbound.ProductUpper(4, c.eps), c.lenLo())
	}
	den := subDownF(c.rLo(), c.eps)
	if !(den > 0) {
		return math.Inf(1)
	}
	return proofbound.DivUpper(proofbound.ProductUpper(2, proofbound.AbsSumUpper(deltaJ, aJ, c.eps)), den)
}

// theta2 is §6.1's tangent allowance of a denoted carrier: 4ε/ℓ for a line,
// zero for a circle read over a ball widened by ε.
func (c cbCarrier) theta2() float64 {
	if !c.line || c.eps == 0 {
		return 0
	}
	return proofbound.DivUpper(proofbound.ProductUpper(4, c.eps), c.lenLo())
}

// transport is T(x), how far a displacement x at a corner moves a position
// along the walk: x on a line, R_lo·x/(R_lo − x) on a circle with
// R_lo = R − ε. +Inf where R_lo ≤ x.
func (c cbCarrier) transport(x float64) float64 {
	if c.line || x == 0 {
		return x
	}
	rl := subDownF(c.rLo(), c.eps)
	if !(rl > x) {
		return math.Inf(1)
	}
	return ratQuotientUp(x, rl, true)
}

func subDownF(a, b float64) float64 { return math.Nextafter(a-b, math.Inf(-1)) }

// productDownF is a float at or below the exact product a·b of two finite
// floats, for a denominator that a quotient rounded up must not overstate.
// It is 0, which DivUpper reads as +Inf, when an input is not finite.
func productDownF(a, b float64) float64 {
	ra, okA := proofbound.RatOf(a)
	rb, okB := proofbound.RatOf(b)
	if !okA || !okB {
		return 0
	}
	f := proofbound.RatFloatDown(new(big.Rat).Mul(ra, rb))
	if proofbound.IsNonFinite(f) {
		return 0
	}
	return f
}

// offsetRadius is the circle's offset radius R − sign·inside·r as an
// enclosure (the radius the construction offsets to).
func (c cbCarrier) offsetRadius(sign, r float64) (ivl, bool) {
	rr, ok := ivFloat(sign * c.inside * r)
	if !ok {
		return ivl{}, false
	}
	off := c.rad.sub(rr)
	if off.lo.Sign() <= 0 {
		return ivl{}, false
	}
	return off, true
}

// cb6Margin is CB6's two sides for a circular carrier: the offset radius
// R − r (rounded down) and tol + ε + D (rounded up). ok=false when no positive
// offset radius exists (S5).
func (c cbCarrier) cb6Margin(sign, r, d float64) (off, need float64, ok bool) {
	iv, ok := c.offsetRadius(sign, r)
	if !ok {
		return 0, 0, false
	}
	return iv.loDown(), proofbound.AbsSumUpper(sectionaudit.Tolerance, c.eps, d), true
}

// offsetResidual is q, the exact distance from p to the recorded offset
// carrier on the construction's side, rounded up. For a circle it also
// returns the offset radius enclosure.
func (c cbCarrier) offsetResidual(p [2]*big.Rat, sign, r float64) (float64, ivl, bool) {
	if c.line {
		t, ok := ivFloat(sign * r)
		if !ok {
			return 0, ivl{}, false
		}
		au, av := new(big.Rat).Sub(p[0], c.su), new(big.Rat).Sub(p[1], c.sv)
		num := new(big.Rat).Sub(new(big.Rat).Mul(c.du, av), new(big.Rat).Mul(c.dv, au)) // left normal · a
		s, ok := ivPoint(num).quo(c.length)
		if !ok {
			return 0, ivl{}, false
		}
		return s.sub(t).maxAbs(), ivl{}, true
	}
	off, ok := c.offsetRadius(sign, r)
	if !ok {
		return 0, ivl{}, false
	}
	dist, ok := c.centreDist(p)
	if !ok {
		return 0, ivl{}, false
	}
	return dist.sub(off).maxAbs(), off, true
}

// centreDist is |p − C| as an enclosure.
func (c cbCarrier) centreDist(p [2]*big.Rat) (ivl, bool) {
	return norm(ivPoint(new(big.Rat).Sub(p[0], c.cu)), ivPoint(new(big.Rat).Sub(p[1], c.cv)))
}

// bound states the offset carrier through p for prismcells.SinLowerBounds.
func (c cbCarrier) bound(off ivl) prismcells.BoundCarrier {
	if c.line {
		seg := sectionrecord.LineSeg{
			Start: sectionrecord.Point2{U: c.ov.SU, V: c.ov.SV},
			End:   sectionrecord.Point2{U: c.ov.SU + c.ov.DU, V: c.ov.SV + c.ov.DV}, TStart: 0, TEnd: 1}
		return prismcells.BoundCarrier{Seg: seg}
	}
	lo, hi := off.loDown(), off.hiUp()
	return prismcells.BoundCarrier{
		Seg: sectionrecord.CircleSeg{Center: sectionrecord.Point2{U: c.ov.CU, V: c.ov.CV},
			Radius: units.Millimeters(hi), CCW: true, TStart: 0, TEnd: 1},
		RadiusLo: lo, RadiusHi: hi,
	}
}

// offsetDisplacement is e of §7.5: how far the carrier's r-offset moves over a
// ball of radius rho about p, whose projection falls tau outside the recorded
// segment.
func (c cbCarrier) offsetDisplacement(rho, tau, r float64) float64 {
	if c.eps == 0 {
		return 0
	}
	if !c.line {
		return c.eps
	}
	l := c.lenLo()
	slope := proofbound.DivUpper(proofbound.ProductUpper(2, proofbound.AbsSumUpper(rho, tau)), l)
	return proofbound.AbsSumUpper(proofbound.ProductUpper(c.eps, proofbound.AbsSumUpper(1, slope)),
		proofbound.DivUpper(proofbound.ProductUpper(4, proofbound.ProductUpper(r, c.eps)), l))
}

// projectionOutside is τ: the distance the projection of p onto the carrier's
// line falls outside the recorded [Start, End], rounded up; zero for a circle.
func (c cbCarrier) projectionOutside(p [2]*big.Rat) float64 {
	if !c.line {
		return 0
	}
	au, av := new(big.Rat).Sub(p[0], c.su), new(big.Rat).Sub(p[1], c.sv)
	dot := new(big.Rat).Add(new(big.Rat).Mul(c.du, au), new(big.Rat).Mul(c.dv, av))
	s, ok := ivPoint(dot).quo(c.length)
	if !ok {
		return math.Inf(1)
	}
	// Start is at s = 0 and End at s = ±ℓ along the oriented direction.
	l := c.length
	var span ivl
	if c.flip {
		span = ivl{lo: new(big.Rat).Neg(l.hi), hi: new(big.Rat)}
	} else {
		span = ivl{lo: new(big.Rat), hi: l.hi}
	}
	under := proofbound.RatFloatUp(new(big.Rat).Sub(span.lo, s.hi)) // s below the span
	over := proofbound.RatFloatUp(new(big.Rat).Sub(s.lo, span.hi))  // s above the span
	return math.Max(0, math.Max(under, over))
}

// lineFoot is the exact foot of p on the recorded line.
func (c cbCarrier) lineFoot(p [2]*big.Rat) (*big.Rat, *big.Rat) {
	au, av := new(big.Rat).Sub(p[0], c.su), new(big.Rat).Sub(p[1], c.sv)
	dd := proofbound.RatAdd(new(big.Rat).Mul(c.du, c.du), new(big.Rat).Mul(c.dv, c.dv))
	t := new(big.Rat).Quo(new(big.Rat).Add(new(big.Rat).Mul(au, c.du), new(big.Rat).Mul(av, c.dv)), dd)
	return new(big.Rat).Add(c.su, new(big.Rat).Mul(t, c.du)), new(big.Rat).Add(c.sv, new(big.Rat).Mul(t, c.dv))
}

// footResidual is ρ_f: the exact distance from the float foot f of centre p
// to the exact foot of p on the recorded carrier, rounded up. ok=false when
// p lies on a circle's centre.
func (c cbCarrier) footResidual(f [2]float64, p [2]*big.Rat) (float64, bool) {
	fu, fv, ok := ratOf2(f[0], f[1])
	if !ok {
		return 0, false
	}
	if c.line {
		xu, xv := c.lineFoot(p)
		return pointDistUp(fu, fv, xu, xv), true
	}
	dist, ok := c.centreDist(p)
	if !ok || dist.lo.Sign() <= 0 {
		return 0, false
	}
	bu, bv := ivPoint(new(big.Rat).Sub(p[0], c.cu)), ivPoint(new(big.Rat).Sub(p[1], c.cv))
	xu, ok1 := c.rad.mul(bu).quo(dist)
	xv, ok2 := c.rad.mul(bv).quo(dist)
	if !ok1 || !ok2 {
		return 0, false
	}
	xu, xv = xu.add(ivPoint(c.cu)), xv.add(ivPoint(c.cv))
	return boxDistUp(fu, fv, xu, xv), true
}

// filletFootDelta is Δf of §7.5 for a fillet foot: ρ_f plus the centre's
// displacement D and the offset displacement e (line), or ε and the radial
// transfer (circle, with rc a lower bound on |ĉ − C|).
func (c cbCarrier) filletFootDelta(rhoF, d, e, r, rc float64) float64 {
	if c.line {
		return proofbound.AbsSumUpper(rhoF, d, e,
			proofbound.DivUpper(proofbound.ProductUpper(4, proofbound.ProductUpper(r, c.eps)), c.lenLo()))
	}
	if c.eps == 0 && d == 0 {
		return rhoF
	}
	if !(rc > 0) {
		return math.Inf(1)
	}
	factor := proofbound.DivUpper(proofbound.ProductUpper(2, proofbound.AbsSumUpper(d, c.eps)), rc)
	return proofbound.AbsSumUpper(rhoF, c.eps, proofbound.ProductUpper(proofbound.AbsSumUpper(c.rHi(), c.eps), factor))
}

// chamferFootResidual is ρ_f for a chamfer foot: the exact distance from the
// float foot f, set back dist along the walk from its corner, to the exact
// point set back dist from the read corner along the recorded carrier.
// arriving selects the walk that ends at the corner. a is the walked corner's
// proven bound (its distance from the read corner).
func (c cbCarrier) chamferFootResidual(f [2]float64, corner [2]float64, a, dist float64, arriving bool) (float64, bool) {
	fu, fv, ok := ratOf2(f[0], f[1])
	if !ok {
		return 0, false
	}
	if c.line {
		line, isLine := c.seg.(sectionrecord.LineSeg)
		if !isLine {
			return 0, false
		}
		start, end := momentinput.ExactLineEnds(line)
		p := start
		if arriving {
			p = end
		}
		d, okd := ivFloat(dist)
		if p == nil || p.U == nil || p.V == nil || !okd {
			return 0, false
		}
		ux, ok1 := ivPoint(c.du).quo(c.length)
		uy, ok2 := ivPoint(c.dv).quo(c.length)
		if !ok1 || !ok2 {
			return 0, false
		}
		step := d
		if arriving {
			step = d.neg()
		}
		xu := ivPoint(p.U).add(step.mul(ux))
		xv := ivPoint(p.V).add(step.mul(uy))
		return boxDistUp(fu, fv, xu, xv), true
	}
	// A circle: the exact foot lies d/R of angle from the read corner, whose
	// bearing is the walked corner's within 2a/(R − a).
	wu, wv, ok := ratOf2(corner[0], corner[1])
	if !ok {
		return 0, false
	}
	rlo := c.rLo()
	if !(rlo > proofbound.ProductUpper(2, a)) {
		return 0, false
	}
	bu, bv := new(big.Rat).Sub(wu, c.cu), new(big.Rat).Sub(wv, c.cv)
	if bu.Sign() == 0 && bv.Sign() == 0 {
		return 0, false
	}
	bearing := proofbound.Atan2Interval(bv, bu, false)
	d, okd := ivFloat(dist)
	if !okd {
		return 0, false
	}
	arc, ok := d.quo(c.rad)
	if !ok {
		return 0, false
	}
	sign := 1.0 // a CCW walk increases the bearing along travel
	if c.inside < 0 {
		sign = -1
	}
	if arriving {
		sign = -sign
	}
	sg, _ := ivFloat(sign)
	theta := proofbound.IntervalAdd(bearing, proofbound.RatInterval{Lo: arc.mul(sg).lo, Hi: arc.mul(sg).hi})
	slack := new(big.Rat).SetFloat64(proofbound.DivUpper(proofbound.ProductUpper(2, a), subDownF(rlo, a)))
	if slack == nil {
		return 0, false
	}
	theta = proofbound.IntervalAdd(theta, proofbound.RatInterval{Lo: new(big.Rat).Neg(slack), Hi: slack})
	mid := new(big.Rat).Quo(new(big.Rat).Add(theta.Lo, theta.Hi), big.NewRat(2, 1))
	sin, cos, ok := proofbound.RadSinCosInterval(mid)
	if !ok {
		return 0, false
	}
	xu := ivPoint(c.cu).add(c.rad.mul(ivl{lo: cos.Lo, hi: cos.Hi}))
	xv := ivPoint(c.cv).add(c.rad.mul(ivl{lo: sin.Lo, hi: sin.Hi}))
	half := new(big.Rat).Quo(new(big.Rat).Sub(theta.Hi, theta.Lo), big.NewRat(2, 1))
	spread := proofbound.RatFloatUp(new(big.Rat).Mul(half, c.rad.hi))
	return proofbound.AbsSumUpper(boxDistUp(fu, fv, xu, xv), spread), true
}

// chamferFootDelta is Δf of §7.5 for a chamfer foot.
func (c cbCarrier) chamferFootDelta(rhoF, deltaJ, setback float64) float64 {
	if c.line {
		return proofbound.AbsSumUpper(rhoF, deltaJ,
			proofbound.DivUpper(proofbound.ProductUpper(4, proofbound.ProductUpper(setback, c.eps)), c.lenLo()))
	}
	if c.eps == 0 {
		return proofbound.AbsSumUpper(rhoF, ratQuotientUp(deltaJ, c.rLo(), true))
	}
	rl := c.rLo()
	den := productDownF(rl, subDownF(rl, c.eps))
	step := proofbound.DivUpper(proofbound.ProductUpper(setback, c.eps), den)
	angle := proofbound.DivUpper(proofbound.ProductUpper(math.Nextafter(math.Pi, 4), proofbound.AbsSumUpper(deltaJ, c.eps)), rl)
	return proofbound.AbsSumUpper(rhoF, c.eps,
		proofbound.ProductUpper(proofbound.AbsSumUpper(c.rHi(), c.eps), proofbound.AbsSumUpper(angle, step)))
}

// centre is one Lemma T application at a fillet centre (CB2).
type centre struct {
	d      float64 // D: |ĉ − c*| ≤ D
	eA, eB float64 // the offset displacements at the accepted radius
}

// centreDisplacement is §7.5's Centre (CB2): Lemma T applied to the two
// denoted offsets about the float root. ok=false means no acceptance in four
// tries.
func centreDisplacement(a, b cbCarrier, sign float64, root [2]float64, r float64) (centre, bool) {
	pu, pv, ok := ratOf2(root[0], root[1])
	if !ok {
		return centre{}, false
	}
	p := [2]*big.Rat{pu, pv}
	qa, offA, okA := a.offsetResidual(p, sign, r)
	qb, offB, okB := b.offsetResidual(p, sign, r)
	if !okA || !okB {
		return centre{}, false
	}
	tauA, tauB := a.projectionOutside(p), b.projectionOutside(p)
	maxEps := math.Max(a.eps, b.eps)
	rho := math.Max(proofbound.ProductUpper(4, proofbound.AbsSumUpper(qa, qb)), 0x1p-60)
	for range 4 {
		eA, eB := a.offsetDisplacement(rho, tauA, r), b.offsetDisplacement(rho, tauB, r)
		s, err := prismcells.SinLowerBounds(a.bound(offA), b.bound(offB), p, proofbound.AbsSumUpper(rho, maxEps))
		if err != nil {
			return centre{}, false
		}
		sStar := subDownF(subDownF(s, a.theta2()), b.theta2())
		la, lb := proofbound.AbsSumUpper(qa, eA), proofbound.AbsSumUpper(qb, eB)
		if sStar > sectionaudit.ContactEps {
			l := proofbound.AbsSumUpper(proofbound.DivUpper(proofbound.AbsSumUpper(la, lb), sStar), math.Min(la, lb))
			if 2*l <= rho {
				return centre{d: l, eA: eA, eB: eB}, true
			}
			rho = proofbound.ProductUpper(4, l)
			continue
		}
		return centre{}, false
	}
	return centre{}, false
}
