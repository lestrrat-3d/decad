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

func TestSpanHodographGapSquaredMatchesRationalHull(t *testing.T) {
	t.Parallel()
	point := func(un, ud, vn, vd int64) freeform.RatPoint {
		return freeform.RatPoint{U: big.NewRat(un, ud), V: big.NewRat(vn, vd)}
	}
	cases := []freeform.BezierSpan{
		nil,
		{point(3, 7, -2, 5)},
		{point(1, 2, -2, 3), point(7, 11, 3, 13)},
		{point(2, 3, -1, 7), point(-5, 11, 4, 9), point(1, 17, 3, 5), point(-7, 19, -2, 13)},
	}
	for _, span := range cases {
		got, err := freeform.SpanHodographGapSquared(nil, span)
		require.NoError(t, err)
		if len(span) < 2 {
			require.Zero(t, got.Sign())
			continue
		}
		last := span[len(span)-1]
		du := new(big.Rat).Sub(last.U, span[0].U)
		dv := new(big.Rat).Sub(last.V, span[0].V)
		degree := big.NewRat(int64(len(span)-1), 1)
		want := new(big.Rat)
		for i := 0; i+1 < len(span); i++ {
			hu := new(big.Rat).Sub(span[i+1].U, span[i].U)
			hu.Mul(hu, degree).Sub(hu, du)
			hv := new(big.Rat).Sub(span[i+1].V, span[i].V)
			hv.Mul(hv, degree).Sub(hv, dv)
			squared := new(big.Rat).Add(new(big.Rat).Mul(hu, hu), new(big.Rat).Mul(hv, hv))
			if squared.Cmp(want) > 0 {
				want.Set(squared)
			}
		}
		require.Zero(t, got.Cmp(want))
	}
}
