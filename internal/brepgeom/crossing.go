package brepgeom

import (
	"math"
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/units"
)

// exactCarrier is a segment's carrier over exact rationals: a line
// n·x = c through the recorded endpoints, or a circle about its recorded
// centre with the squared radius it denotes.
type exactCarrier struct {
	line       bool
	nu, nv, c  *big.Rat
	cu, cv, r2 *big.Rat
}

// carrierOf states seg's carrier exactly: a line's support through its
// recorded Start and End, a circle's recorded radius squared, an arc's
// squared distance from its centre to its Start (the radius it denotes). It
// reports false for any other kind, a line whose endpoints coincide, or a
// field that is not finite.
func carrierOf(seg sectionrecord.CurveSegment) (exactCarrier, bool) {
	switch s := seg.(type) {
	case sectionrecord.LineSeg:
		su, sv, eu, ev := rat(s.Start.U), rat(s.Start.V), rat(s.End.U), rat(s.End.V)
		if su == nil || sv == nil || eu == nil || ev == nil {
			return exactCarrier{}, false
		}
		du, dv := new(big.Rat).Sub(eu, su), new(big.Rat).Sub(ev, sv)
		if du.Sign() == 0 && dv.Sign() == 0 {
			return exactCarrier{}, false
		}
		nu, nv := new(big.Rat).Neg(dv), du
		c := new(big.Rat).Add(new(big.Rat).Mul(nu, su), new(big.Rat).Mul(nv, sv))
		return exactCarrier{line: true, nu: nu, nv: nv, c: c}, true
	case sectionrecord.CircleSeg:
		r, err := s.Radius.In(units.Millimeter)
		if err != nil {
			return exactCarrier{}, false
		}
		cu, cv, rr := rat(s.Center.U), rat(s.Center.V), rat(r)
		if cu == nil || cv == nil || rr == nil {
			return exactCarrier{}, false
		}
		return exactCarrier{cu: cu, cv: cv, r2: rr.Mul(rr, rr)}, true
	case sectionrecord.ArcSeg:
		cu, cv, su, sv := rat(s.Center.U), rat(s.Center.V), rat(s.Start.U), rat(s.Start.V)
		if cu == nil || cv == nil || su == nil || sv == nil {
			return exactCarrier{}, false
		}
		du, dv := su.Sub(su, cu), sv.Sub(sv, cv)
		return exactCarrier{cu: cu, cv: cv, r2: du.Add(du.Mul(du, du), dv.Mul(dv, dv))}, true
	}
	return exactCarrier{}, false
}

func rat(x float64) *big.Rat { return proofarith.FloatRat(x) }

// SameCarrier reports whether a and b lie on one carrier, stated exactly: two
// lines on one support, or two circular segments about one centre with one
// squared radius. Such a junction names no crossing.
func SameCarrier(a, b sectionrecord.CurveSegment) bool {
	ca, okA := carrierOf(a)
	cb, okB := carrierOf(b)
	if !okA || !okB || ca.line != cb.line {
		return false
	}
	if ca.line {
		// n_a ∥ n_b and the supports share a point: n_a × n_b = 0 and
		// c_a·n_b = c_b·n_a componentwise.
		cross := new(big.Rat).Sub(new(big.Rat).Mul(ca.nu, cb.nv), new(big.Rat).Mul(ca.nv, cb.nu))
		if cross.Sign() != 0 {
			return false
		}
		return new(big.Rat).Mul(ca.c, cb.nu).Cmp(new(big.Rat).Mul(cb.c, ca.nu)) == 0 &&
			new(big.Rat).Mul(ca.c, cb.nv).Cmp(new(big.Rat).Mul(cb.c, ca.nv)) == 0
	}
	return ca.cu.Cmp(cb.cu) == 0 && ca.cv.Cmp(cb.cv) == 0 && ca.r2.Cmp(cb.r2) == 0
}

// CrossingOffsetUpper is a proven upper bound on the distance from p to the
// crossing of a's and b's carriers that p names (docs/general-boolean-design.md
// §5): the point where two fragments of a boolean's arrangement meet is that
// crossing, and a walked or keyed float placed there sits off it by the cut
// parameter's own error, which far from the plane origin is about the
// coordinates' rounding rather than the cut allowance δ_cut. Every quantity is
// an exact rational over the recorded floats, and only square roots are
// rounded, each in the direction that enlarges the bound.
//
// Two lines cross at one exact rational point, and the bound is p's distance
// from it. A line crosses a circle where the line's points lie at
// a = ±√b along it from the centre's foot, b = R² − h² with h the centre's
// distance from the line. p's own foot on the line lies at a, so p is
//
//	|e|/|n|  +  |a² − b| / (a + √b)
//
// from the crossing on its side, e = n·p − c its offset from the line n·x = c.
// Two circles cross on their radical line,
// 2(C₂ − C₁)·x = R₁² − R₂² + |C₂|² − |C₁|², which is exact, so they take the
// line arm against the first circle. Where p's foot lies within twice that
// bound of the centre's foot, the side it names is not decided, and the bound
// covers the far crossing too: a + √b.
//
// A pair with no crossing to measure against — two parallel lines, one circle
// twice, two concentric circles, a line that misses its circle (b < 0) — and a
// bound that cannot be stated answer +Inf, which the caller refuses on. A p
// exactly at the crossing answers 0.
func CrossingOffsetUpper(a, b sectionrecord.CurveSegment, p sectionrecord.Point2) float64 {
	ca, okA := carrierOf(a)
	cb, okB := carrierOf(b)
	pu, pv := rat(p.U), rat(p.V)
	if !okA || !okB || pu == nil || pv == nil {
		return math.Inf(1)
	}
	switch {
	case ca.line && cb.line:
		return lineLineOffset(ca, cb, pu, pv)
	case ca.line:
		return lineCircleOffset(ca, cb, pu, pv)
	case cb.line:
		return lineCircleOffset(cb, ca, pu, pv)
	}
	// The radical line of two circles: n = 2(C₂ − C₁),
	// c = R₁² − R₂² + |C₂|² − |C₁|².
	nu := new(big.Rat).Sub(cb.cu, ca.cu)
	nv := new(big.Rat).Sub(cb.cv, ca.cv)
	if nu.Sign() == 0 && nv.Sign() == 0 {
		return math.Inf(1)
	}
	two := big.NewRat(2, 1)
	nu.Mul(nu, two)
	nv.Mul(nv, two)
	c := new(big.Rat).Sub(ca.r2, cb.r2)
	c.Add(c, new(big.Rat).Add(new(big.Rat).Mul(cb.cu, cb.cu), new(big.Rat).Mul(cb.cv, cb.cv)))
	c.Sub(c, new(big.Rat).Add(new(big.Rat).Mul(ca.cu, ca.cu), new(big.Rat).Mul(ca.cv, ca.cv)))
	return lineCircleOffset(exactCarrier{line: true, nu: nu, nv: nv, c: c}, ca, pu, pv)
}

// lineLineOffset is p's distance from the crossing of two lines, rounded up.
func lineLineOffset(a, b exactCarrier, pu, pv *big.Rat) float64 {
	det := new(big.Rat).Sub(new(big.Rat).Mul(a.nu, b.nv), new(big.Rat).Mul(a.nv, b.nu))
	if det.Sign() == 0 {
		return math.Inf(1)
	}
	// Cramer's rule on n_a·x = c_a, n_b·x = c_b.
	xu := new(big.Rat).Sub(new(big.Rat).Mul(a.c, b.nv), new(big.Rat).Mul(b.c, a.nv))
	xu.Quo(xu, det)
	xv := new(big.Rat).Sub(new(big.Rat).Mul(a.nu, b.c), new(big.Rat).Mul(b.nu, a.c))
	xv.Quo(xv, det)
	du, dv := xu.Sub(xu, pu), xv.Sub(xv, pv)
	return finiteOrInf(proofbound.RatSqrtUp(du.Add(du.Mul(du, du), dv.Mul(dv, dv))))
}

// lineCircleOffset is p's distance from the crossing of line l with circle k
// on the side of k's centre that p names, rounded up (CrossingOffsetUpper).
func lineCircleOffset(l, k exactCarrier, pu, pv *big.Rat) float64 {
	n2 := new(big.Rat).Add(new(big.Rat).Mul(l.nu, l.nu), new(big.Rat).Mul(l.nv, l.nv))
	if n2.Sign() == 0 {
		return math.Inf(1)
	}
	dot := func(u, v *big.Rat) *big.Rat {
		return new(big.Rat).Add(new(big.Rat).Mul(l.nu, u), new(big.Rat).Mul(l.nv, v))
	}
	// e = n·p − c, p's offset from the line, times |n|.
	e := new(big.Rat).Sub(dot(pu, pv), l.c)
	across := proofbound.RatSqrtUp(new(big.Rat).Quo(new(big.Rat).Mul(e, e), n2))
	// h·|n| = n·C − c; b = R² − h².
	h := new(big.Rat).Sub(dot(k.cu, k.cv), l.c)
	b := new(big.Rat).Sub(k.r2, new(big.Rat).Quo(new(big.Rat).Mul(h, h), n2))
	if b.Sign() < 0 {
		return math.Inf(1)
	}
	// a·|n| = t·(p − C) along the line's direction t = (−n_v, n_u).
	s := new(big.Rat).Sub(
		new(big.Rat).Mul(l.nu, new(big.Rat).Sub(pv, k.cv)),
		new(big.Rat).Mul(l.nv, new(big.Rat).Sub(pu, k.cu)),
	)
	a2 := new(big.Rat).Quo(new(big.Rat).Mul(s, s), n2)
	num := new(big.Rat).Sub(a2, b)
	num.Abs(num)
	along := 0.0
	if num.Sign() != 0 {
		aDown, bDown := rat(proofbound.RatSqrtDown(a2)), rat(proofbound.RatSqrtDown(b))
		if aDown == nil || bDown == nil {
			return math.Inf(1)
		}
		den := new(big.Rat).Add(aDown, bDown)
		if den.Sign() <= 0 {
			return math.Inf(1)
		}
		along = proofbound.RatFloatUp(num.Quo(num, den))
		if proofbound.IsNonFinite(along) {
			return math.Inf(1)
		}
		if twice := rat(along); aDown.Cmp(twice.Add(twice, twice)) <= 0 {
			// Too close to the centre's foot to name a side: cover both.
			along = proofbound.AbsSumUpper(proofbound.RatSqrtUp(a2), proofbound.RatSqrtUp(b))
		}
	}
	return finiteOrInf(proofbound.AbsSumUpper(across, along))
}

func finiteOrInf(x float64) float64 {
	if proofbound.IsNonFinite(x) {
		return math.Inf(1)
	}
	return x
}
