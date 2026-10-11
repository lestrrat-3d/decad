package meshbool

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/polynomial"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
)

// ExactSimilarityImage evaluates anchor + factor*(p-anchor) over the exact
// rational values of the supplied floats. It is the point that a rounded
// similarity vertex must enclose, including when anchor*(1-factor) rounds.
func ExactSimilarityImage(anchor, p r3.Vec, factor float64) proof.Xpt {
	a := [3]float64{anchor.X, anchor.Y, anchor.Z}
	v := [3]float64{p.X, p.Y, p.Z}
	s := polynomial.MustRatOf(factor)
	var q [3]*big.Rat
	for i := range q {
		q[i] = new(big.Rat).Sub(polynomial.MustRatOf(v[i]), polynomial.MustRatOf(a[i]))
		q[i].Mul(q[i], s)
		q[i].Add(q[i], polynomial.MustRatOf(a[i]))
	}
	w := new(big.Int).Set(q[0].Denom())
	for _, coord := range q[1:] {
		g := new(big.Int).GCD(nil, nil, w, coord.Denom())
		w.Mul(w, new(big.Int).Quo(coord.Denom(), g))
	}
	num := func(coord *big.Rat) *big.Int {
		return new(big.Int).Mul(coord.Num(), new(big.Int).Quo(w, coord.Denom()))
	}
	return proof.Xpt(proof.XhpStripTwosOwned(proof.Xhp{
		X: num(q[0]), Y: num(q[1]), Z: num(q[2]), W: w,
	}))
}
