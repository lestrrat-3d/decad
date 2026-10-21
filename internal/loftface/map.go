// Package loftface holds the LoftFaces exact cap maps, rational vectors and
// intervals, and the band injectivity gate and meridian rows built on them.
// See agent-docs/loft-face-sections-design.md.
package loftface

import (
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
)

// The rational source maps are kept here rather than reconstructed from
// public cap edges. A rounded FacetedCurve does not contain a fitted flank.

// RatVec is an exact rational 3-vector.
type RatVec [3]*big.Rat

// RatMat is an exact rational 3x3 matrix, indexed [row][column].
type RatMat [3][3]*big.Rat

// Affine is the exact affine map x -> A*x + B.
type Affine struct {
	A RatMat
	B RatVec
}

// Map is one exact LoftFaces cap map F(u, v) = Constant + Q(u, v)/G(u, v),
// where Q = Q0 + QU*u + QV*v and G = G0 + GU*u + GV*v. Stretch is a float
// upper bound on the Lipschitz factor of the map's linear part.
type Map struct {
	Constant   RatVec
	Q0, QU, QV RatVec
	G0, GU, GV *big.Rat
	Stretch    float64
}

// Interval is a closed exact rational interval [Lo, Hi].
type Interval struct{ Lo, Hi *big.Rat }

func ratZero() *big.Rat             { return new(big.Rat) }
func ratOne() *big.Rat              { return new(big.Rat).SetInt64(1) }
func ratF(v float64) *big.Rat       { return proofarith.FloatRat(v) }
func ratAdd(a, b *big.Rat) *big.Rat { return proofarith.AddRat(new(big.Rat), a, b) }
func ratSub(a, b *big.Rat) *big.Rat { return proofarith.SubRat(new(big.Rat), a, b) }
func ratMul(a, b *big.Rat) *big.Rat { return proofarith.MulRat(new(big.Rat), a, b) }
func ratDiv(a, b *big.Rat) *big.Rat { return proofarith.QuoRat(new(big.Rat), a, b) }
func ratAbs(a *big.Rat) *big.Rat    { return new(big.Rat).Abs(a) }

// Denominator returns the denominator G(u, v) of the map.
func (m Map) Denominator(u, v *big.Rat) *big.Rat {
	return ratAdd(ratAdd(m.G0, ratMul(m.GU, u)), ratMul(m.GV, v))
}

// Numerator returns the numerator Q(u, v) of the map.
func (m Map) Numerator(u, v *big.Rat) RatVec {
	return VAdd(VAdd(m.Q0, VScale(m.QU, u)), VScale(m.QV, v))
}

// At returns the exact image F(u, v) of the map.
func (m Map) At(u, v *big.Rat) RatVec {
	g := m.Denominator(u, v)
	return VAdd(m.Constant, VScale(m.Numerator(u, v), ratDiv(ratOne(), g)))
}

// Deriv returns the exact partial derivative of the map along axis 0 (u) or
// 1 (v) at (u, v).
func (m Map) Deriv(u, v *big.Rat, axis int) RatVec {
	g := m.Denominator(u, v)
	q := m.Numerator(u, v)
	qAxis, gAxis := m.QU, m.GU
	if axis == 1 {
		qAxis, gAxis = m.QV, m.GV
	}
	return VScale(VSub(VScale(qAxis, g), VScale(q, gAxis)),
		ratDiv(ratOne(), ratMul(g, g)))
}
