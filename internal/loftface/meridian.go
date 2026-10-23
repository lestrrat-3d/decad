package loftface

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/polynomial"
)

// CapBernsteinRows gives the rational denominator and coordinate
// numerators of one exact LoftFaces cap map over a source Bezier span.
func CapBernsteinRows(m Map, span freeform.BezierSpan) ([]*big.Rat, [3][]*big.Rat) {
	g := make([]*big.Rat, len(span))
	var h [3][]*big.Rat
	for axis := range h {
		h[axis] = make([]*big.Rat, len(span))
	}
	for i, p := range span {
		g[i] = ratAdd(ratAdd(m.G0, ratMul(m.GU, p.U)), ratMul(m.GV, p.V))
		for axis := range h {
			h[axis][i] = ratAdd(ratMul(m.Constant[axis], g[i]),
				ratAdd(ratAdd(m.Q0[axis], ratMul(m.QU[axis], p.U)), ratMul(m.QV[axis], p.V)))
		}
	}
	return g, h
}

// capIntegerRows holds the same map coefficients over shared integer
// denominators while products and derivatives are formed.
func capIntegerRows(m Map, span freeform.BezierSpan) (polynomial.BernsteinIntRow, [3]polynomial.BernsteinIntRow) {
	g, h := CapBernsteinRows(m, span)
	var rows [3]polynomial.BernsteinIntRow
	for axis := range rows {
		rows[axis] = polynomial.BernsteinIntRowOf(h[axis])
	}
	return polynomial.BernsteinIntRowOf(g), rows
}

// MeridianQRows returns Q=D²(y²+z²−(m*x+b)²) after exact cap
// interpolation, where D=g0*g1. For a source span of degree n, each row has
// Bernstein degree at most 4n in source parameter s. The three rows are the
// quadratic Bernstein coefficients in band time t. A caller proves D>0
// before using Q's sign as the geometric side of the meridian.
func MeridianQRows(first, second Map, span freeform.BezierSpan,
	m, b *big.Rat) [3][]*big.Rat {
	g0, h0 := capIntegerRows(first, span)
	g1, h1 := capIntegerRows(second, span)
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
