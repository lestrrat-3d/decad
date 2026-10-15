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

func TestBernsteinRectOpenSidePreservesEndpointVerdictsAndInput(t *testing.T) {
	zero, one := big.NewRat(0, 1), big.NewRat(1, 1)
	quarter, threeQuarters := big.NewRat(1, 4), big.NewRat(3, 4)
	cases := []struct {
		name           string
		controls       [3][]int64
		lo, hi         *big.Rat
		loOpen, hiOpen bool
		want           int
	}{
		{"positive closed", [3][]int64{{2, 3}, {2, 3}, {2, 3}}, zero, one, false, false, 1},
		{"negative closed", [3][]int64{{-2, -3}, {-2, -3}, {-2, -3}}, zero, one, false, false, -1},
		{"open lower zero", [3][]int64{{0, 0}, {2, 3}, {2, 3}}, zero, one, true, false, 1},
		{"closed lower zero", [3][]int64{{0, 0}, {2, 3}, {2, 3}}, zero, one, false, false, 0},
		{"open upper zero", [3][]int64{{-2, -3}, {-2, -3}, {0, 0}}, zero, one, false, true, -1},
		{"closed upper zero", [3][]int64{{-2, -3}, {-2, -3}, {0, 0}}, zero, one, false, false, 0},
		{"zero source column", [3][]int64{{2, 0}, {2, 0}, {2, 0}}, zero, one, true, true, 0},
		{"mixed source signs", [3][]int64{{2, -2}, {2, -2}, {2, -2}}, zero, one, true, true, 0},
		{"restricted time", [3][]int64{{0, 0}, {2, 3}, {0, 0}}, quarter, threeQuarters, false, false, 1},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			var q [3][]*big.Rat
			for row, controls := range tc.controls {
				for _, value := range controls {
					q[row] = append(q[row], big.NewRat(value, 1))
				}
			}
			require.Equal(t, tc.want, polynomial.BernsteinRectOpenSide(q, tc.lo, tc.hi, tc.loOpen, tc.hiOpen))
			for row, controls := range tc.controls {
				for column, value := range controls {
					require.Zero(t, q[row][column].Cmp(big.NewRat(value, 1)),
						"input coefficient at row %d, column %d", row, column)
				}
			}
		})
	}
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
