package decad

import (
	"context"
	"errors"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/stretchr/testify/require"
)

func loftHullIntegerPoints(coords [6][3]int64) [6][3]*big.Rat {
	var points [6][3]*big.Rat
	for i, p := range coords {
		for j, c := range p {
			points[i][j] = big.NewRat(c, 1)
		}
	}
	return points
}

func TestLoftVolumeHullBoundExactPolyhedra(t *testing.T) {
	cases := []struct {
		name   string
		points [6][3]int64
		want   string
	}{
		{"tetrahedron with repeated vertices", [6][3]int64{
			{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}, {0, 0, 0}, {0, 0, 0},
		}, "1/6"},
		{"triangular prism with quadrilateral sides", [6][3]int64{
			{0, 0, 0}, {2, 0, 0}, {0, 2, 0}, {0, 0, 3}, {2, 0, 3}, {0, 2, 3},
		}, "6"},
		{"octahedron", [6][3]int64{
			{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1},
		}, "4/3"},
		{"coplanar square", [6][3]int64{
			{0, 0, 0}, {2, 0, 0}, {2, 2, 0}, {0, 2, 0}, {1, 1, 0}, {0, 0, 0},
		}, "0"},
		{"collinear points", [6][3]int64{
			{0, 0, 0}, {1, 1, 1}, {2, 2, 2}, {3, 3, 3}, {1, 1, 1}, {0, 0, 0},
		}, "0"},
		{"interior vertices", [6][3]int64{
			{0, 0, 0}, {8, 0, 0}, {0, 8, 0}, {0, 0, 8}, {1, 1, 1}, {2, 1, 1},
		}, "256/3"},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			points := loftHullIntegerPoints(tc.points)
			got, ok, err := loftVolumeHullBound(proofbound.NewWorkBudget(t.Context()), points)
			require.NoError(t, err)
			require.True(t, ok)
			require.Equal(t, tc.want, got.RatString())
			require.GreaterOrEqual(t, got.Sign(), 0)
		})
	}
}

func TestLoftVolumeHullBoundRationalAndOwnership(t *testing.T) {
	points := loftHullIntegerPoints([6][3]int64{
		{0, 0, 0}, {1, 0, 0}, {0, 2, 0}, {0, 0, 3}, {0, 0, 0}, {0, 0, 0},
	})
	points[1][0] = big.NewRat(1, 2)
	got, ok, err := loftVolumeHullBound(proofbound.NewWorkBudget(t.Context()), points)
	require.NoError(t, err)
	require.True(t, ok)
	require.Equal(t, "1/2", got.RatString())
	got.SetInt64(99)
	require.Equal(t, "1/2", points[1][0].RatString())
}

func TestLoftVolumeHullBoundAffineAndPermutation(t *testing.T) {
	base := [6][3]int64{
		{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1},
	}
	// This integer affine map has determinant six. A translated and sheared
	// octahedron therefore has six times the base volume, independently of
	// the order in which its six endpoint positions are supplied.
	for shift := range 6 {
		var transformed [6][3]int64
		for i := range transformed {
			p := base[(i+shift)%6]
			transformed[i] = [3]int64{2*p[0] + p[1] + 3, p[1] - p[2] - 2, 3*p[2] + 5}
		}
		got, ok, err := loftVolumeHullBound(proofbound.NewWorkBudget(t.Context()), loftHullIntegerPoints(transformed))
		require.NoError(t, err)
		require.True(t, ok)
		require.Equal(t, "8", got.RatString())
	}
}

func TestLoftVolumeHullBoundRefusals(t *testing.T) {
	points := loftHullIntegerPoints([6][3]int64{
		{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}, {0, 0, 0}, {0, 0, 0},
	})
	got, ok, err := loftVolumeHullBound(nil, points)
	require.Nil(t, got)
	require.False(t, ok)
	require.NoError(t, err)

	missing := points
	missing[4][1] = nil
	got, ok, err = loftVolumeHullBound(proofbound.NewWorkBudget(t.Context()), missing)
	require.Nil(t, got)
	require.False(t, ok)
	require.NoError(t, err)

	oversized := points
	oversized[4][0] = new(big.Rat).SetInt(new(big.Int).Lsh(big.NewInt(1), loftHullRatBits))
	got, ok, err = loftVolumeHullBound(proofbound.NewWorkBudget(t.Context()), oversized)
	require.Nil(t, got)
	require.False(t, ok)
	require.NoError(t, err)

	// Each input fits, but the intermediate exact arithmetic does not.
	intermediate := points
	intermediate[1][0] = new(big.Rat).SetFrac(big.NewInt(1),
		new(big.Int).Lsh(big.NewInt(1), 5000))
	intermediate[2][1] = new(big.Rat).SetFrac(big.NewInt(1),
		new(big.Int).Lsh(big.NewInt(1), 5000))
	got, ok, err = loftVolumeHullBound(proofbound.NewWorkBudget(t.Context()), intermediate)
	require.Nil(t, got)
	require.False(t, ok)
	require.NoError(t, err)
}

func TestLoftVolumeHullBoundWorkAndCancellation(t *testing.T) {
	points := loftHullIntegerPoints([6][3]int64{
		{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1},
	})
	steps := 0
	budget := &proofbound.WorkBudget{
		StepFn: func() error { steps++; return nil },
		ErrFn:  t.Context().Err,
	}
	got, ok, err := loftVolumeHullBound(budget, points)
	require.NoError(t, err)
	require.True(t, ok)
	require.Equal(t, "4/3", got.RatString())
	require.Positive(t, steps)
	require.Less(t, steps, 65_536)

	stopped := errors.New("shared output work exhausted")
	steps = 0
	budget = &proofbound.WorkBudget{
		StepFn: func() error {
			steps++
			if steps == 40 {
				return stopped
			}
			return nil
		},
		ErrFn: t.Context().Err,
	}
	got, ok, err = loftVolumeHullBound(budget, points)
	require.Nil(t, got)
	require.False(t, ok)
	require.ErrorIs(t, err, stopped)
	require.Equal(t, 40, steps)

	ctx, cancel := context.WithCancel(t.Context())
	defer cancel()
	steps = 0
	budget = &proofbound.WorkBudget{
		StepFn: func() error {
			steps++
			if steps == 40 {
				cancel()
			}
			return nil
		},
		ErrFn: ctx.Err,
	}
	got, ok, err = loftVolumeHullBound(budget, points)
	require.Nil(t, got)
	require.False(t, ok)
	require.ErrorIs(t, err, context.Canceled)
	require.Equal(t, 40, steps)

	preCanceled, stop := context.WithCancel(t.Context())
	stop()
	got, ok, err = loftVolumeHullBound(proofbound.NewWorkBudget(preCanceled), points)
	require.Nil(t, got)
	require.False(t, ok)
	require.ErrorIs(t, err, context.Canceled)
}
