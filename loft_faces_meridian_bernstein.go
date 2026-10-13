package decad

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/polynomial"
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
		g[i] = lfAdd(lfAdd(m.G0, lfMul(m.GU, p.U)), lfMul(m.GV, p.V))
		for axis := range h {
			h[axis][i] = lfAdd(lfMul(m.Constant[axis], g[i]),
				lfAdd(lfAdd(m.Q0[axis], lfMul(m.QU[axis], p.U)), lfMul(m.QV[axis], p.V)))
		}
	}
	return g, h
}

// loftFacesCapIntegerRows holds the same map coefficients over shared integer
// denominators while products and derivatives are formed.
func loftFacesCapIntegerRows(m loftFaceMap, span freeform.BezierSpan) (polynomial.BernsteinIntRow, [3]polynomial.BernsteinIntRow) {
	g, h := loftFacesCapBernsteinRows(m, span)
	var rows [3]polynomial.BernsteinIntRow
	for axis := range rows {
		rows[axis] = polynomial.BernsteinIntRowOf(h[axis])
	}
	return polynomial.BernsteinIntRowOf(g), rows
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
	d := g0.Mul(g1)
	var a, z [3]polynomial.BernsteinIntRow
	for axis := range a {
		a[axis] = h0[axis].Mul(g1)
		z[axis] = h1[axis].Mul(g0)
	}
	al := a[0].Scale(m).Add(d.Scale(b))
	zl := z[0].Scale(m).Add(d.Scale(b))
	dy, dz, dl := z[1].Sub(a[1]), z[2].Sub(a[2]), zl.Sub(al)
	q0 := a[1].Mul(a[1]).Add(a[2].Mul(a[2])).Sub(al.Mul(al))
	q1 := a[1].Mul(dy).Add(a[2].Mul(dz)).Sub(al.Mul(dl)).Scale(big.NewRat(2, 1))
	q2 := dy.Mul(dy).Add(dz.Mul(dz)).Sub(dl.Mul(dl))
	return [3][]*big.Rat{q0.Rats(), q0.Add(q1.Scale(big.NewRat(1, 2))).Rats(),
		q0.Add(q1).Add(q2).Rats()}
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
	d := g0.Mul(g1)
	a, z := h0[0].Mul(g1), h1[0].Mul(g0)
	b := z.Sub(a)
	bd := b.Mul(d)
	u0 := a.Deriv().Mul(d).Sub(a.Mul(d.Deriv()))
	u1 := b.Deriv().Mul(d).Sub(b.Mul(d.Deriv()))
	rows := [3]polynomial.BernsteinIntRow{polynomial.BernsteinIntRowOf(q[0]),
		polynomial.BernsteinIntRowOf(q[1]), polynomial.BernsteinIntRowOf(q[2])}
	two := big.NewRat(2, 1)
	q0 := rows[0]
	q1 := rows[1].Sub(rows[0]).Scale(two)
	q2 := rows[2].Sub(rows[1].Scale(two)).Add(rows[0])
	j0 := bd.Mul(q0.Deriv()).Sub(u0.Mul(q1))
	j1 := bd.Mul(q1.Deriv()).Sub(u0.Mul(q2).Scale(two).Add(u1.Mul(q1)))
	j2 := bd.Mul(q2.Deriv()).Sub(u1.Mul(q2).Scale(two))
	return [3][]*big.Rat{j0.Rats(), j0.Add(j1.Scale(big.NewRat(1, 2))).Rats(),
		j0.Add(j1).Add(j2).Rats()}
}
