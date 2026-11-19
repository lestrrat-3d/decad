package smoothloft

import (
	"math/big"
	"slices"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/polynomial"
)

// rationalCompose evaluates a monomial polynomial at another polynomial.
func rationalCompose(p, q polynomial.RatPoly) polynomial.RatPoly {
	out := polynomial.RatPoly{rint(0)}
	for _, coefficient := range slices.Backward(p) {
		out = polynomial.RpAdd(polynomial.RpMul(out, q), polynomial.RatPoly{coefficient})
	}
	return out
}

// rationalReparameterize preserves the image of c. The positive endpoint
// slopes do not exceed one, so the cubic's Bernstein controls strictly
// increase and the map covers [0,1] exactly once.
func rationalReparameterize(c rationalCurve, start, end *big.Rat) rationalCurve {
	controls := []*big.Rat{rint(0), div(start, rint(3)), sub(rint(1), div(end, rint(3))), rint(1)}
	q := freeform.RpFromBernstein(controls)
	return rationalCurve{x: rationalCompose(c.x, q), y: rationalCompose(c.y, q), w: rationalCompose(c.w, q)}
}

func rationalEndpointDerivative(c rationalCurve, end int64) freeform.RatPoint {
	u := rint(end)
	w := polynomial.RpEval(c.w, u)
	dw := polynomial.RpEval(polynomial.RpDeriv(c.w), u)
	coordinate := func(n polynomial.RatPoly) *big.Rat {
		return div(sub(mul(polynomial.RpEval(polynomial.RpDeriv(n), u), w),
			mul(polynomial.RpEval(n, u), dw)), mul(w, w))
	}
	return freeform.RatPoint{U: coordinate(c.x), V: coordinate(c.y)}
}

// rationalJoinSlopes returns endpoint slopes that make matching geometric
// tangents equal. Nonparallel, opposite and zero tangents do not form a G1 join.
func rationalJoinSlopes(a, b rationalCurve) (*big.Rat, *big.Rat, bool) {
	aw, bw := polynomial.RpEval(a.w, rint(1)), polynomial.RpEval(b.w, rint(0))
	if aw.Sign() <= 0 || bw.Sign() <= 0 ||
		sub(mul(polynomial.RpEval(a.x, rint(1)), bw), mul(polynomial.RpEval(b.x, rint(0)), aw)).Sign() != 0 ||
		sub(mul(polynomial.RpEval(a.y, rint(1)), bw), mul(polynomial.RpEval(b.y, rint(0)), aw)).Sign() != 0 {
		return nil, nil, false
	}
	l, r := rationalEndpointDerivative(a, 1), rationalEndpointDerivative(b, 0)
	cross := sub(mul(l.U, r.V), mul(l.V, r.U))
	dot := add(mul(l.U, r.U), mul(l.V, r.V))
	if cross.Sign() != 0 || dot.Sign() <= 0 {
		return nil, nil, false
	}
	ls, rs := add(abs(l.U), abs(l.V)), add(abs(r.U), abs(r.V))
	left, right := rint(1), rint(1)
	if ls.Cmp(rs) > 0 {
		left = div(rs, ls)
	} else {
		right = div(ls, rs)
	}
	return left, right, true
}
