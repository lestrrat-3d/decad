package smoothloft

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/polynomial"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// rationalCurve holds a planar curve (x/w,y/w) over a positive denominator.
// Coordinates and denominator use exact monomial coefficients on [0,1].
type rationalCurve struct {
	x, y, w polynomial.RatPoly
}

// rationalPolyAbsBound bounds |p| on [0,1] by its Bernstein hull.
func rationalPolyAbsBound(p polynomial.RatPoly) *big.Rat {
	coeff := freeform.RpToBernstein(p, max(0, polynomial.RpDeg(p)))
	bound := new(big.Rat)
	for _, c := range coeff {
		bound = maxRat(bound, abs(c))
	}
	return bound
}

// rationalCurveBounds bounds the L1 norms of a curve and its first two
// derivatives. The caller supplies an exact positive lower bound on w.
func rationalCurveBounds(c rationalCurve, wMin *big.Rat) (radius, speed, curvature *big.Rat) {
	boundPair := func(x, y polynomial.RatPoly, den *big.Rat) *big.Rat {
		return div(add(rationalPolyAbsBound(x), rationalPolyAbsBound(y)), den)
	}
	w1 := polynomial.RpDeriv(c.w)
	derivativeNumerator := func(n polynomial.RatPoly) polynomial.RatPoly {
		return polynomial.RpSub(polynomial.RpMul(polynomial.RpDeriv(n), c.w), polynomial.RpMul(n, w1))
	}
	dx, dy := derivativeNumerator(c.x), derivativeNumerator(c.y)
	secondNumerator := func(d polynomial.RatPoly) polynomial.RatPoly {
		return polynomial.RpSub(polynomial.RpMul(polynomial.RpDeriv(d), c.w),
			polynomial.RpScale(polynomial.RpMul(d, w1), rint(2)))
	}
	w2 := mul(wMin, wMin)
	return boundPair(c.x, c.y, wMin), boundPair(dx, dy, w2),
		boundPair(secondNumerator(dx), secondNumerator(dy), mul(w2, wMin))
}

// rationalTransition blends curves with the same denominator using smoothstep.
type rationalTransition struct {
	a, b   rationalCurve
	linear bool
}

// bounds supplies §14's derivative bounds for the existing mesh proof.
func (p rationalTransition) bounds() (twistBounds, error) {
	coeff := freeform.RpToBernstein(p.a.w, max(0, polynomial.RpDeg(p.a.w)))
	wMin := coeff[0]
	for _, c := range coeff[1:] {
		if c.Cmp(wMin) < 0 {
			wMin = c
		}
	}
	if wMin.Sign() <= 0 {
		return twistBounds{}, twistUnsupported("circular fitted loft denominator is not proved positive")
	}
	ar, au, auu := rationalCurveBounds(p.a, wMin)
	br, bu, buu := rationalCurveBounds(p.b, wMin)
	delta := rationalCurve{x: polynomial.RpSub(p.b.x, p.a.x), y: polynomial.RpSub(p.b.y, p.a.y), w: p.a.w}
	d, du, _ := rationalCurveBounds(delta, wMin)
	if p.linear {
		return twistBounds{bu: maxRat(au, bu), bt: d, muu: maxRat(auu, buu), mtt: rint(0), mut: du,
			radius: maxRat(ar, br)}, nil
	}
	return twistBounds{
		bu: maxRat(au, bu), bt: mul(big.NewRat(3, 2), d),
		muu: maxRat(auu, buu), mtt: mul(rint(6), d), mut: mul(big.NewRat(3, 2), du),
		radius: maxRat(ar, br),
	}, nil
}

func (p rationalTransition) at(u, t *big.Rat) freeform.RatPoint {
	s := smoothstep(t)
	if p.linear {
		s = t
	}
	blend := func(a, b polynomial.RatPoly) *big.Rat {
		x, y := polynomial.RpEval(a, u), polynomial.RpEval(b, u)
		return div(add(mul(sub(rint(1), s), x), mul(s, y)), polynomial.RpEval(p.a.w, u))
	}
	return freeform.RatPoint{U: blend(p.a.x, p.b.x), V: blend(p.a.y, p.b.y)}
}

// smoothstep is the blend weight s(t) = t²(3-2t) of a non-linear transition.
func smoothstep(t *big.Rat) *big.Rat {
	return mul(mul(t, t), sub(rint(3), mul(rint(2), t)))
}

// rationalStation is a transition at one station u, shifted by a plane
// origin. Its point at blend weight s is (x0 + s·dx, y0 + s·dy), which
// equals at(u, t) + origin exactly for s = s(t): every term is an exact
// rational, and big.Rat keeps each value in lowest terms.
type rationalStation struct {
	x0, y0, dx, dy *big.Rat
}

// station evaluates the five station polynomials of p at u once, so the
// rings of a span share them.
func (p rationalTransition) station(u, originX, originY *big.Rat) rationalStation {
	w := polynomial.RpEval(p.a.w, u)
	ax, ay := div(polynomial.RpEval(p.a.x, u), w), div(polynomial.RpEval(p.a.y, u), w)
	bx, by := div(polynomial.RpEval(p.b.x, u), w), div(polynomial.RpEval(p.b.y, u), w)
	return rationalStation{x0: add(ax, originX), y0: add(ay, originY), dx: sub(bx, ax), dy: sub(by, ay)}
}

// at returns the shifted point at blend weight s. It reads st and s only.
func (st rationalStation) at(s *big.Rat) (*big.Rat, *big.Rat) {
	return add(st.x0, mul(s, st.dx)), add(st.y0, mul(s, st.dy))
}

func rationalCross(ax, ay, bx, by polynomial.RatPoly) polynomial.RatPoly {
	return polynomial.RpSub(polynomial.RpMul(ax, by), polynomial.RpMul(ay, bx))
}

// rationalPolarRows gives the three Bernstein rows in blend weight s of the
// numerator of cross(C,C_u), where C=(1-s)a+s b and the denominators agree.
func rationalPolarRows(a, b rationalCurve) [3][]*big.Rat {
	ax, ay := polynomial.RpDeriv(a.x), polynomial.RpDeriv(a.y)
	bx, by := polynomial.RpDeriv(b.x), polynomial.RpDeriv(b.y)
	p := [3]polynomial.RatPoly{
		rationalCross(a.x, a.y, ax, ay),
		polynomial.RpScale(polynomial.RpAdd(rationalCross(a.x, a.y, bx, by),
			rationalCross(b.x, b.y, ax, ay)), big.NewRat(1, 2)),
		rationalCross(b.x, b.y, bx, by),
	}
	degree := max(polynomial.RpDeg(p[0]), polynomial.RpDeg(p[1]), polynomial.RpDeg(p[2]), 0)
	var rows [3][]*big.Rat
	for i := range rows {
		rows[i] = freeform.RpToBernstein(p[i], degree)
	}
	return rows
}

// rationalPolarPositive proves all coefficients positive after exact splits.
// A nonpositive corner disproves strict positivity; an unresolved cell refuses.
func rationalPolarPositive(ctx context.Context, rows [3][]*big.Rat, budget *proofbound.WorkBudget) error {
	var visit func([3][]*big.Rat, int) error
	visit = func(cell [3][]*big.Rat, depth int) error {
		if err := ctx.Err(); err != nil {
			return err
		}
		if err := budget.Step(); err != nil {
			return err
		}
		last := len(cell[0]) - 1
		for _, corner := range [...][2]int{{0, 0}, {0, last}, {2, 0}, {2, last}} {
			if cell[corner[0]][corner[1]].Sign() <= 0 {
				return twistUnsupported("circular fitted loft has a nonpositive polar-angle corner")
			}
		}
		positive := true
		for _, row := range cell {
			for _, c := range row {
				if c.Sign() <= 0 {
					positive = false
				}
			}
		}
		if positive {
			return nil
		}
		if depth == 12 {
			return twistUnsupported("circular fitted loft exceeds the polar proof depth")
		}
		var left, right [3][]*big.Rat
		if depth%2 == 0 {
			for i := range cell {
				left[i], right[i] = polynomial.BernsteinSplit(cell[i], big.NewRat(1, 2))
			}
		} else {
			for i := range left {
				left[i], right[i] = make([]*big.Rat, last+1), make([]*big.Rat, last+1)
			}
			for j := 0; j <= last; j++ {
				l, r := polynomial.BernsteinSplit([]*big.Rat{cell[0][j], cell[1][j], cell[2][j]}, big.NewRat(1, 2))
				for i := range left {
					left[i][j], right[i][j] = l[i], r[i]
				}
			}
		}
		if err := visit(left, depth+1); err != nil {
			return err
		}
		return visit(right, depth+1)
	}
	return visit(rows, 0)
}
