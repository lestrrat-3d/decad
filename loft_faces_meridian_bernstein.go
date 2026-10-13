package decad

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/freeform"
)

// loftFacesCapBernsteinRows gives the rational denominator and coordinate
// numerators of one exact LoftFaces cap map over a source Bezier span.
func loftFacesCapBernsteinRows(m loftFaceMap, span freeform.BezierSpan) ([]*big.Rat, [3][]*big.Rat) {
	g := make([]*big.Rat, len(span))
	var h [3][]*big.Rat
	for axis := range h {
		h[axis] = make([]*big.Rat, len(span))
	}
	for i, p := range span {
		g[i] = lfAdd(lfAdd(m.g0, lfMul(m.gu, p.U)), lfMul(m.gv, p.V))
		for axis := range h {
			h[axis][i] = lfAdd(lfMul(m.constant[axis], g[i]),
				lfAdd(lfAdd(m.q0[axis], lfMul(m.qu[axis], p.U)), lfMul(m.qv[axis], p.V)))
		}
	}
	return g, h
}

// loftFacesCapIntegerRows holds the same map coefficients over shared integer
// denominators while products and derivatives are formed.
func loftFacesCapIntegerRows(m loftFaceMap, span freeform.BezierSpan) (bernsteinIntRow, [3]bernsteinIntRow) {
	g, h := loftFacesCapBernsteinRows(m, span)
	var rows [3]bernsteinIntRow
	for axis := range rows {
		rows[axis] = bernsteinIntRowOf(h[axis])
	}
	return bernsteinIntRowOf(g), rows
}

// loftFacesMeridianQRows returns Q=D²(y²+z²−(m*x+b)²) after exact cap
// interpolation, where D=g0*g1. For a source span of degree n, each row has
// Bernstein degree at most 4n in source parameter s. The three rows are the
// quadratic Bernstein coefficients in band time t. A caller proves D>0
// before using Q's sign as the geometric side of the meridian.
func loftFacesMeridianQRows(first, second loftFaceMap, span freeform.BezierSpan,
	m, b *big.Rat) [3][]*big.Rat {
	g0, h0 := loftFacesCapIntegerRows(first, span)
	g1, h1 := loftFacesCapIntegerRows(second, span)
	d := g0.mul(g1)
	var a, z [3]bernsteinIntRow
	for axis := range a {
		a[axis] = h0[axis].mul(g1)
		z[axis] = h1[axis].mul(g0)
	}
	al := a[0].scale(m).add(d.scale(b))
	zl := z[0].scale(m).add(d.scale(b))
	dy, dz, dl := z[1].sub(a[1]), z[2].sub(a[2]), zl.sub(al)
	q0 := a[1].mul(a[1]).add(a[2].mul(a[2])).sub(al.mul(al))
	q1 := a[1].mul(dy).add(a[2].mul(dz)).sub(al.mul(dl)).scale(big.NewRat(2, 1))
	q2 := dy.mul(dy).add(dz.mul(dz)).sub(dl.mul(dl))
	return [3][]*big.Rat{q0.rats(), q0.add(q1.scale(big.NewRat(1, 2))).rats(),
		q0.add(q1).add(q2).rats()}
}

// loftFacesMeridianJRows returns the numerator of X_t*Q_s−X_s*Q_t along
// Q=0, after multiplying by D². With X=(A+t*B)/D, it is
// J=B*D*Q_s−(H_s*D−H*D_s)*Q_t. For a source span of degree n, each row has
// Bernstein degree at most 8n−1 in s and quadratic degree in t. A caller
// separately proves the sign of Q_s before using J to order a moving crossing.
func loftFacesMeridianJRows(first, second loftFaceMap, span freeform.BezierSpan,
	q [3][]*big.Rat) [3][]*big.Rat {
	g0, h0 := loftFacesCapIntegerRows(first, span)
	g1, h1 := loftFacesCapIntegerRows(second, span)
	d := g0.mul(g1)
	a, z := h0[0].mul(g1), h1[0].mul(g0)
	b := z.sub(a)
	bd := b.mul(d)
	u0 := a.deriv().mul(d).sub(a.mul(d.deriv()))
	u1 := b.deriv().mul(d).sub(b.mul(d.deriv()))
	rows := [3]bernsteinIntRow{bernsteinIntRowOf(q[0]),
		bernsteinIntRowOf(q[1]), bernsteinIntRowOf(q[2])}
	two := big.NewRat(2, 1)
	q0 := rows[0]
	q1 := rows[1].sub(rows[0]).scale(two)
	q2 := rows[2].sub(rows[1].scale(two)).add(rows[0])
	j0 := bd.mul(q0.deriv()).sub(u0.mul(q1))
	j1 := bd.mul(q1.deriv()).sub(u0.mul(q2).scale(two).add(u1.mul(q1)))
	j2 := bd.mul(q2.deriv()).sub(u1.mul(q2).scale(two))
	return [3][]*big.Rat{j0.rats(), j0.add(j1.scale(big.NewRat(1, 2))).rats(),
		j0.add(j1).add(j2).rats()}
}
