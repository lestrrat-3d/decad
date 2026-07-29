package decad

import (
	"math/big"
)

// This file is docs/spline-design.md §5.1's exact integration: the
// Green's-theorem boundary forms over the Bézier spans spline_bezier.go
// produced. Every integrand is a POLYNOMIAL in the span parameter, so every
// integral is an exact rational — which is what earns a Tier A kind its zero
// bound, and why §5.2 forbids a quadrature fallback here.
//
// The polynomial machinery is clearance_poly.go's ratPoly, reused rather than
// forked (spline design §6.2): that file already owns dense rational
// polynomials with the products and derivatives these forms need.

// rpIntegral01 is the exact ∫₀¹ of a rational polynomial: Σ cᵢ/(i+1).
func rpIntegral01(p ratPoly) *big.Rat {
	out := new(big.Rat)
	for i, coefficient := range p {
		out.Add(out, new(big.Rat).Quo(coefficient, big.NewRat(int64(i)+1, 1)))
	}
	return out
}

// rpFromBernstein converts one coordinate's Bézier control values to the
// monomial form of the same polynomial: Σ bᵢ·C(n,i)·tⁱ·(1−t)ⁿ⁻ⁱ, built as that
// literal sum of products so the conversion is transparently the identity it
// claims rather than a difference table a reader must re-derive.
func rpFromBernstein(values []*big.Rat) ratPoly {
	degree := len(values) - 1
	out := ratPoly{}
	for i, value := range values {
		if value.Sign() == 0 {
			continue
		}
		// tⁱ
		term := make(ratPoly, i+1)
		for k := range term {
			term[k] = new(big.Rat)
		}
		term[i] = big.NewRat(1, 1)
		// ·(1−t)ⁿ⁻ⁱ
		oneMinusT := ratPoly{big.NewRat(1, 1), big.NewRat(-1, 1)}
		for range degree - i {
			term = rpMul(term, oneMinusT)
		}
		// ·bᵢ·C(n,i)
		out = rpAdd(out, rpScale(term, new(big.Rat).Mul(value, binomialRat(degree, i))))
	}
	return rpTrim(out)
}

// binomialRat is C(n, k) as an exact rational. n is a Bézier degree, so it is
// small and the multiplicative form cannot overflow the rationals it builds.
func binomialRat(n, k int) *big.Rat {
	out := big.NewRat(1, 1)
	for i := 1; i <= k; i++ {
		out.Mul(out, big.NewRat(int64(n-k+i), int64(i)))
	}
	return out
}

// spanCoordinatePolys returns one span's u(t) and v(t) in monomial form.
func spanCoordinatePolys(span bezierSpan) (ratPoly, ratPoly) {
	us := make([]*big.Rat, len(span))
	vs := make([]*big.Rat, len(span))
	for i, point := range span {
		us[i], vs[i] = point.u, point.v
	}
	return rpFromBernstein(us), rpFromBernstein(vs)
}

// exactFreeformMoments integrates the region moments of one converted
// free-form curve exactly. reversed negates every signed result, which is how
// the recorded range order carries the walk direction (spline design §2).
//
// The boundary forms are the same ones the line path integrates, so the two
// implementations are checkable against each other on a degree-1 span:
//
//	A     = ½∮(u dv − v du)
//	∫u dA = ½∮u² dv
//	∫v dA = −½∮v² du
//	∫u² dA = ⅓∮u³ dv
//	∫v² dA = −⅓∮v³ du
//	∫uv dA = ½∮u²v dv
func exactFreeformMoments(spans []bezierSpan, reversed bool, work *freeformWork) (exactMoments, error) {
	half := big.NewRat(1, 2)
	third := big.NewRat(1, 3)
	out := exactMoments{
		area: new(big.Rat),
		mu:   new(big.Rat),
		mv:   new(big.Rat),
		muu:  new(big.Rat),
		muv:  new(big.Rat),
		mvv:  new(big.Rat),
	}
	for _, span := range spans {
		if err := work.step(uint64(len(span)) * 8); err != nil {
			return exactMoments{}, err
		}
		u, v := spanCoordinatePolys(span)
		du, dv := rpDeriv(u), rpDeriv(v)
		uu := rpMul(u, u)
		vv := rpMul(v, v)

		out.area.Add(out.area, new(big.Rat).Mul(half, rpIntegral01(rpSub(rpMul(u, dv), rpMul(v, du)))))
		out.mu.Add(out.mu, new(big.Rat).Mul(half, rpIntegral01(rpMul(uu, dv))))
		out.mv.Sub(out.mv, new(big.Rat).Mul(half, rpIntegral01(rpMul(vv, du))))
		out.muu.Add(out.muu, new(big.Rat).Mul(third, rpIntegral01(rpMul(rpMul(uu, u), dv))))
		out.mvv.Sub(out.mvv, new(big.Rat).Mul(third, rpIntegral01(rpMul(rpMul(vv, v), du))))
		out.muv.Add(out.muv, new(big.Rat).Mul(half, rpIntegral01(rpMul(rpMul(uu, v), dv))))
	}
	if reversed {
		for _, value := range []*big.Rat{out.area, out.mu, out.mv, out.muu, out.muv, out.mvv} {
			value.Neg(value)
		}
	}
	return out, nil
}

// addFreeform accumulates one converted free-form curve's contribution. The
// held float of each moment is the exact rational rounded ONCE, so the reported
// bound is that single rounding — zero whenever the exact value is
// representable, which is what lets a Tier A section report Exact.
func (ig *regionIntegrals) addFreeform(spans []bezierSpan, reversed bool, work *freeformWork) error {
	exact, err := exactFreeformMoments(spans, reversed, work)
	if err != nil {
		return err
	}
	if extent := freeformControlExtent(spans); extent > ig.coordUpper {
		ig.coordUpper = extent
	}
	for _, moment := range []struct {
		value *float64
		bound *float64
		exact *big.Rat
	}{
		{&ig.area, &ig.areaBound, exact.area},
		{&ig.mu, &ig.muBound, exact.mu},
		{&ig.mv, &ig.mvBound, exact.mv},
		{&ig.muu, &ig.muuBound, exact.muu},
		{&ig.muv, &ig.muvBound, exact.muv},
		{&ig.mvv, &ig.mvvBound, exact.mvv},
	} {
		held, _ := moment.exact.Float64()
		accumulateMoment(moment.value, moment.bound, held, rationalFloatError(moment.exact, held))
	}
	return nil
}
