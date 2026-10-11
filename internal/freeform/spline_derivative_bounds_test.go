package freeform_test

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/stretchr/testify/require"
)

func TestBernsteinSquaredNormIntegralMatchesPairwiseBetaIntegral(t *testing.T) {
	t.Parallel()
	point := func(un, ud, vn, vd int64) freeform.RatPoint {
		return freeform.RatPoint{U: big.NewRat(un, ud), V: big.NewRat(vn, vd)}
	}
	cases := [][]freeform.RatPoint{
		nil,
		{point(-1, 3, 2, 5)},
		{point(1, 2, -2, 3), point(7, 11, 3, 13)},
		{point(2, 3, -1, 7), point(-5, 11, 4, 9), point(1, 17, 3, 5), point(-7, 19, -2, 13)},
	}
	for _, g := range cases {
		got, err := freeform.BernsteinSquaredNormIntegral(nil, g)
		require.NoError(t, err)
		if len(g) == 0 {
			require.Zero(t, got.Sign())
			continue
		}
		q := len(g) - 1
		factorials := make([]*big.Int, 2*q+2)
		factorials[0] = big.NewInt(1)
		for k := 1; k < len(factorials); k++ {
			factorials[k] = new(big.Int).Mul(factorials[k-1], big.NewInt(int64(k)))
		}
		want := new(big.Rat)
		for i, a := range g {
			for j, b := range g {
				dot := new(big.Rat).Add(
					new(big.Rat).Mul(a.U, b.U),
					new(big.Rat).Mul(a.V, b.V),
				)
				weight := new(big.Rat).SetFrac(
					new(big.Int).Mul(factorials[i+j], factorials[2*q-i-j]),
					factorials[2*q+1],
				)
				want.Add(want, dot.Mul(dot, weight))
			}
		}
		require.Zero(t, got.Cmp(want))
	}
}
