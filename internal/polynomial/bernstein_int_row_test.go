package polynomial_test

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/polynomial"
	"github.com/stretchr/testify/require"
)

func bernsteinRatValue(coeff []*big.Rat, at *big.Rat) *big.Rat {
	work := make([]*big.Rat, len(coeff))
	for i, value := range coeff {
		work[i] = new(big.Rat).Set(value)
	}
	other := new(big.Rat).Sub(big.NewRat(1, 1), at)
	for width := len(work); width > 1; width-- {
		for i := 0; i+1 < width; i++ {
			left := new(big.Rat).Mul(other, work[i])
			right := new(big.Rat).Mul(at, work[i+1])
			work[i] = left.Add(left, right)
		}
	}
	return work[0]
}

func TestBernsteinIntegerRowAlgebraMatchesRationalValues(t *testing.T) {
	a := []*big.Rat{big.NewRat(1, 2), big.NewRat(-2, 3), big.NewRat(5, 7)}
	b := []*big.Rat{big.NewRat(-3, 5), big.NewRat(4, 9), big.NewRat(2, 11)}
	ar, br := polynomial.BernsteinIntRowOf(a), polynomial.BernsteinIntRowOf(b)
	for i, got := range ar.Add(br).Rats() {
		want := new(big.Rat).Add(a[i], b[i])
		require.Zero(t, got.Cmp(want), "sum coefficient %d", i)
	}
	for i, got := range ar.Sub(br).Rats() {
		want := new(big.Rat).Sub(a[i], b[i])
		require.Zero(t, got.Cmp(want), "difference coefficient %d", i)
	}
	scale := big.NewRat(-7, 13)
	for i, got := range ar.Scale(scale).Rats() {
		want := new(big.Rat).Mul(a[i], scale)
		require.Zero(t, got.Cmp(want), "scaled coefficient %d", i)
	}
	for i, got := range ar.Deriv().Rats() {
		want := new(big.Rat).Sub(a[i+1], a[i])
		want.Mul(want, big.NewRat(int64(len(a)-1), 1))
		require.Zero(t, got.Cmp(want), "derivative coefficient %d", i)
	}
	product := ar.Mul(br).Rats()
	for i := range product {
		at := big.NewRat(int64(i), int64(len(product)-1))
		want := new(big.Rat).Mul(bernsteinRatValue(a, at), bernsteinRatValue(b, at))
		require.Zero(t, bernsteinRatValue(product, at).Cmp(want), "product at %s", at)
	}
	require.Equal(t, []*big.Rat{big.NewRat(1, 2), big.NewRat(-2, 3), big.NewRat(5, 7)}, a)
	require.Equal(t, []*big.Rat{big.NewRat(-3, 5), big.NewRat(4, 9), big.NewRat(2, 11)}, b)
}
