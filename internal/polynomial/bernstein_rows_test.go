package polynomial_test

import (
	"context"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/polynomial"
	"github.com/stretchr/testify/require"
)

func TestBernsteinRowsSignAt(t *testing.T) {
	row := func(a, b int64) []*big.Rat { return []*big.Rat{big.NewRat(a, 1), big.NewRat(b, 1)} }
	q := [3][]*big.Rat{row(-2, -2), row(-2, -2), row(2, 2)}
	half := big.NewRat(1, 2)
	quarter := big.NewRat(1, 4)

	// The curve at s=1/2 is the row {-2, -2, 2}; restricting it to [0, 1/4]
	// leaves {-2, -2, -7/4}, which is negative everywhere.
	restricted := polynomial.BernsteinRestrict(
		[]*big.Rat{big.NewRat(-2, 1), big.NewRat(-2, 1), big.NewRat(2, 1)}, big.NewRat(0, 1), quarter)
	want := []*big.Rat{big.NewRat(-2, 1), big.NewRat(-2, 1), big.NewRat(-7, 4)}
	require.Len(t, restricted, len(want))
	for i := range want {
		require.Zero(t, want[i].Cmp(restricted[i]), "restricted coefficient %d", i)
	}

	zero, one := big.NewRat(0, 1), big.NewRat(1, 1)
	require.True(t, polynomial.BernsteinRowsSignAt(q, half, zero, quarter, -1))
	require.False(t, polynomial.BernsteinRowsSignAt(q, half, zero, quarter, 1))
	require.False(t, polynomial.BernsteinRowsSignAt(q, half, zero, one, -1))
}

func TestBernsteinSignByHalving(t *testing.T) {
	coeff := []*big.Rat{big.NewRat(1, 1), big.NewRat(-2, 5), big.NewRat(1, 1)}

	ok, err := polynomial.BernsteinSignByHalving(t.Context(), coeff, 1, 0)
	require.NoError(t, err)
	require.False(t, ok)

	ok, err = polynomial.BernsteinSignByHalving(t.Context(), coeff, 1, 1)
	require.NoError(t, err)
	require.True(t, ok)

	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	ok, err = polynomial.BernsteinSignByHalving(ctx, coeff, 1, 1)
	require.False(t, ok)
	require.ErrorIs(t, err, context.Canceled)
}
