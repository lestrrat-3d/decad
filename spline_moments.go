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

// freeformSpanCeiling is the span length, in control points, past which
// freeformSpanCost stops evaluating its own formula: a span this wide is
// hopeless at any budget, and cutting off here keeps the cubic term far inside
// uint64 rather than relying on saturation to catch an overflow.
const freeformSpanCeiling = 1 << 12

// freeformSpanCost is the conservative preflight of one span's exact
// integration, charged BEFORE any coefficient is allocated. Cost is CUBIC in
// the span degree p while the span itself is only p+1 control points long, so
// a charge read off the control count alone lets an arbitrarily wide span
// through — which is what the whole ceiling exists to stop.
//
// The dominant term is the Bernstein-to-monomial expansion (rpFromBernstein),
// run once per coordinate: term i starts at degree i and is multiplied by
// (1−t) until it reaches degree p, so it costs Σᵢ Σ_{d=i}^{p−1} 2(d+1) =
// p(p+1)(2p+1)/3 coefficient products. Everything else is QUADRATIC in p — the
// per-term add and scale inside that expansion, the six Green's-theorem
// products (none above degree 4p, so under 24(p+1)² together) and their ∫₀¹
// terms — and 64(p+1)² covers all of it at once.
func freeformSpanCost(controls int) uint64 {
	if controls <= 0 {
		return 0
	}
	if controls > freeformSpanCeiling {
		return freeformCostCeiling
	}
	p, n := uint64(controls-1), uint64(controls)
	bernstein := 2 * (p * n * (2*p + 1) / 3)
	return costAdd(bernstein, 64*n*n)
}

// chargeFreeformSpans preflights the exact integration of a WHOLE converted
// chain, before any of it runs. Charging it up front rather than span by span
// is what lets a caller levy the cost at a validation step that precedes the
// integration entirely (moments_validate.go): a record whose integration cannot
// fit the budget must refuse before anything downstream of the conversion
// samples or reconstructs the curve, since the ceiling exists precisely because
// the public ProfileRecord methods take no context and cannot be cancelled.
func chargeFreeformSpans(spans []bezierSpan, work *freeformWork) error {
	for _, span := range spans {
		if err := work.step(freeformSpanCost(len(span))); err != nil {
			return err
		}
	}
	return nil
}

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
	if err := chargeFreeformSpans(spans, work); err != nil {
		return exactMoments{}, err
	}
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
// exact rational goes into the REGION's own accumulator, which is what the
// published float is rounded from — once, after the complete signed
// outer-minus-holes sum (moments.go). Rounding here instead would make a
// multi-segment region's held float a sum of per-curve roundings, and the
// single-rounding bound docs/spline-design.md §3 requires would hold only for a
// region whose whole boundary is one segment. The float accumulation below is
// what the region publishes instead when some OTHER segment — a circular walk —
// leaves it with no exact rational at all.
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
	ig.addExact(exact)
	return nil
}
