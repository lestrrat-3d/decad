package freeform

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/polynomial"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// FreeformSpanCeiling is the span length, in control points, past which
// FreeformSpanCost stops evaluating its own formula: a span this wide is
// hopeless at any budget, and cutting off here keeps the cubic term far inside
// uint64 rather than relying on saturation to catch an overflow.
const FreeformSpanCeiling = 1 << 12

// FreeformSpanCost is the conservative preflight of one span's exact
// integration, charged BEFORE any coefficient is allocated. The charge keeps
// its original cubic ceiling even though RpFromBernstein now computes the
// coefficients with a quadratic difference table. Keeping the charge gives
// existing records the same work-budget acceptance and refusal.
//
// The cubic term bounds the former Bernstein expansion's
// p(p+1)(2p+1)/3 products per coordinate. The difference table performs fewer
// products, while the six Green's-theorem products (none above degree 4p, so
// under 24(p+1)² together) and their ∫₀¹ terms remain quadratic in p. The
// unchanged 64(p+1)² term covers those quadratic operations, and it still
// covers them when MomentThirdOrder adds FreeformThirdMoments: its second
// coefficient conversion and its four boundary forms (none above degree 5p)
// bring the total to under 63(p+1)² coefficient products.
func FreeformSpanCost(controls int) uint64 {
	if controls <= 0 {
		return 0
	}
	if controls > FreeformSpanCeiling {
		return FreeformCostCeiling
	}
	p, n := uint64(controls-1), uint64(controls)
	bernstein := 2 * (p * n * (2*p + 1) / 3)
	return CostAdd(bernstein, 64*n*n)
}

// ChargeFreeformSpans preflights the exact integration of a WHOLE converted
// chain, before any of it runs. It is the SINGLE owner of that charge: the
// record-level preflight levies it (moments_validate.go) and the moments pass
// then integrates the chain the preflight already paid for. A record whose
// integration cannot fit the budget must refuse before anything downstream of
// the conversion samples or reconstructs the curve, since the ceiling exists
// precisely because the public ProfileRecord methods take no context and cannot
// be cancelled.
func ChargeFreeformSpans(spans []survey2d.BezierSpan, work *FreeformWork) error {
	for _, span := range spans {
		if err := work.Step(FreeformSpanCost(len(span))); err != nil {
			return err
		}
	}
	return nil
}

// RpIntegral01 is the exact ∫₀¹ of a rational polynomial: Σ cᵢ/(i+1).
func RpIntegral01(p polynomial.RatPoly) *big.Rat {
	out := new(big.Rat)
	for i, coefficient := range p {
		out.Add(out, new(big.Rat).Quo(coefficient, big.NewRat(int64(i)+1, 1)))
	}
	return out
}

// RpFromBernstein converts one coordinate's Bézier control values to the
// monomial form of the same polynomial. Its coefficient of tᵏ is
// C(n,k)·Δᵏb₀, where Δ is the forward difference on the control values.
// This is the binomial expansion of Σ bᵢ·C(n,i)·tⁱ·(1−t)ⁿ⁻ⁱ; the difference
// table computes all coefficients with quadratic rather than cubic rational
// work. The record's existing conservative integration charge is unchanged.
func RpFromBernstein(values []*big.Rat) polynomial.RatPoly {
	degree := len(values) - 1
	if degree < 0 {
		return nil
	}
	differences := append([]*big.Rat(nil), values...)
	out := make(polynomial.RatPoly, len(values))
	choose := big.NewRat(1, 1)
	for k := range out {
		out[k] = new(big.Rat).Mul(choose, differences[0])
		for i := range len(differences) - 1 {
			differences[i] = new(big.Rat).Sub(differences[i+1], differences[i])
		}
		differences = differences[:len(differences)-1]
		if k < degree {
			choose.Mul(choose, big.NewRat(int64(degree-k), int64(k+1)))
		}
	}
	return polynomial.RpTrim(out)
}

// BinomialRat is C(n, k) as an exact rational. n is a Bézier degree, so it is
// small and the multiplicative form cannot overflow the rationals it builds.
func BinomialRat(n, k int) *big.Rat {
	out := big.NewRat(1, 1)
	for i := 1; i <= k; i++ {
		out.Mul(out, big.NewRat(int64(n-k+i), int64(i)))
	}
	return out
}

// SpanCoordinatePolys returns one span's u(t) and v(t) in monomial form.
func SpanCoordinatePolys(span survey2d.BezierSpan) (polynomial.RatPoly, polynomial.RatPoly) {
	us := make([]*big.Rat, len(span))
	vs := make([]*big.Rat, len(span))
	for i, point := range span {
		us[i], vs[i] = point.U, point.V
	}
	return RpFromBernstein(us), RpFromBernstein(vs)
}

// ExactFreeformMoments integrates the region moments of one converted
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
func ExactFreeformMoments(spans []survey2d.BezierSpan, reversed bool, order MomentIntegralOrder) ExactMoments {
	half := big.NewRat(1, 2)
	var third *big.Rat
	if order >= MomentSecondOrder {
		third = big.NewRat(1, 3)
	}
	out := ExactMoments{
		Area: new(big.Rat),
		Mu:   new(big.Rat),
		Mv:   new(big.Rat),
		Muu:  new(big.Rat),
		Muv:  new(big.Rat),
		Mvv:  new(big.Rat),
	}
	for _, span := range spans {
		u, v := SpanCoordinatePolys(span)
		du, dv := polynomial.RpDeriv(u), polynomial.RpDeriv(v)
		uu := polynomial.RpMul(u, u)
		vv := polynomial.RpMul(v, v)

		out.Area.Add(out.Area, new(big.Rat).Mul(half, RpIntegral01(polynomial.RpSub(polynomial.RpMul(u, dv), polynomial.RpMul(v, du)))))
		out.Mu.Add(out.Mu, new(big.Rat).Mul(half, RpIntegral01(polynomial.RpMul(uu, dv))))
		out.Mv.Sub(out.Mv, new(big.Rat).Mul(half, RpIntegral01(polynomial.RpMul(vv, du))))
		if order >= MomentSecondOrder {
			out.Muu.Add(out.Muu, new(big.Rat).Mul(third, RpIntegral01(polynomial.RpMul(polynomial.RpMul(uu, u), dv))))
			out.Mvv.Sub(out.Mvv, new(big.Rat).Mul(third, RpIntegral01(polynomial.RpMul(polynomial.RpMul(vv, v), du))))
			out.Muv.Add(out.Muv, new(big.Rat).Mul(half, RpIntegral01(polynomial.RpMul(polynomial.RpMul(uu, v), dv))))
		}
	}
	if reversed {
		for _, value := range []*big.Rat{out.Area, out.Mu, out.Mv, out.Muu, out.Muv, out.Mvv} {
			value.Neg(value)
		}
	}
	return out
}

// PolyThirdMoments integrates one polynomial boundary path's third-order
// contributions exactly, through the dv form every third-order contribution
// takes (moments.go's MomentThirdOrder):
//
//	∫u³ dA  = ¼∮u⁴ dv
//	∫u²v dA = ⅓∮u³v dv
//	∫uv² dA = ½∮u²v² dv
//	∫v³ dA  = ∮uv³ dv
//
// A line is the degree-1 path, so moments.go's line arm and this file's span
// arm share it.
func PolyThirdMoments(u, v polynomial.RatPoly) [4]*big.Rat {
	dv := polynomial.RpDeriv(v)
	uu, vv := polynomial.RpMul(u, u), polynomial.RpMul(v, v)
	uuu := polynomial.RpMul(uu, u)
	return [4]*big.Rat{
		new(big.Rat).Mul(big.NewRat(1, 4), RpIntegral01(polynomial.RpMul(polynomial.RpMul(uu, uu), dv))),
		new(big.Rat).Mul(big.NewRat(1, 3), RpIntegral01(polynomial.RpMul(polynomial.RpMul(uuu, v), dv))),
		new(big.Rat).Mul(big.NewRat(1, 2), RpIntegral01(polynomial.RpMul(polynomial.RpMul(uu, vv), dv))),
		RpIntegral01(polynomial.RpMul(polynomial.RpMul(u, polynomial.RpMul(vv, v)), dv)),
	}
}

// FreeformThirdMoments integrates one converted free-form curve's third-order
// contributions about the plane origin. The spans must be the RECORDED
// control points, before regionIntegrals.add shifts them to the walk anchor,
// because the third-order sum is kept about the origin. reversed negates
// every term, as it does in ExactFreeformMoments.
func FreeformThirdMoments(spans []survey2d.BezierSpan, reversed bool) [4]*big.Rat {
	out := [4]*big.Rat{new(big.Rat), new(big.Rat), new(big.Rat), new(big.Rat)}
	for _, span := range spans {
		u, v := SpanCoordinatePolys(span)
		for i, term := range PolyThirdMoments(u, v) {
			out[i].Add(out[i], term)
		}
	}
	if reversed {
		for _, value := range out {
			value.Neg(value)
		}
	}
	return out
}
