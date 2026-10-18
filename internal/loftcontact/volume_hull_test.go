package loftcontact

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
			got, ok, err := VolumeHullBound(proofbound.NewWorkBudget(t.Context()), points)
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
	got, ok, err := VolumeHullBound(proofbound.NewWorkBudget(t.Context()), points)
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
		got, ok, err := VolumeHullBound(proofbound.NewWorkBudget(t.Context()), loftHullIntegerPoints(transformed))
		require.NoError(t, err)
		require.True(t, ok)
		require.Equal(t, "8", got.RatString())
	}
}

func TestLoftVolumeHullBoundRefusals(t *testing.T) {
	points := loftHullIntegerPoints([6][3]int64{
		{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}, {0, 0, 0}, {0, 0, 0},
	})
	got, ok, err := VolumeHullBound(nil, points)
	require.Nil(t, got)
	require.False(t, ok)
	require.NoError(t, err)

	missing := points
	missing[4][1] = nil
	got, ok, err = VolumeHullBound(proofbound.NewWorkBudget(t.Context()), missing)
	require.Nil(t, got)
	require.False(t, ok)
	require.NoError(t, err)

	oversized := points
	oversized[4][0] = new(big.Rat).SetInt(new(big.Int).Lsh(big.NewInt(1), loftHullRatBits))
	got, ok, err = VolumeHullBound(proofbound.NewWorkBudget(t.Context()), oversized)
	require.Nil(t, got)
	require.False(t, ok)
	require.NoError(t, err)

	// Each input fits, but the intermediate exact arithmetic does not.
	intermediate := points
	intermediate[1][0] = new(big.Rat).SetFrac(big.NewInt(1),
		new(big.Int).Lsh(big.NewInt(1), 5000))
	intermediate[2][1] = new(big.Rat).SetFrac(big.NewInt(1),
		new(big.Int).Lsh(big.NewInt(1), 5000))
	got, ok, err = VolumeHullBound(proofbound.NewWorkBudget(t.Context()), intermediate)
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
	got, ok, err := VolumeHullBound(budget, points)
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
	got, ok, err = VolumeHullBound(budget, points)
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
	got, ok, err = VolumeHullBound(budget, points)
	require.Nil(t, got)
	require.False(t, ok)
	require.ErrorIs(t, err, context.Canceled)
	require.Equal(t, 40, steps)

	preCanceled, stop := context.WithCancel(t.Context())
	stop()
	got, ok, err = VolumeHullBound(proofbound.NewWorkBudget(preCanceled), points)
	require.Nil(t, got)
	require.False(t, ok)
	require.ErrorIs(t, err, context.Canceled)
}

func TestLoftVolumeHullBoundZeroMotion(t *testing.T) {
	cases := []struct {
		name   string
		points [6][3]int64
	}{
		{"triangle", [6][3]int64{
			{0, 0, 0}, {2, 0, 0}, {0, 3, 0}, {0, 0, 0}, {2, 0, 0}, {0, 3, 0},
		}},
		{"collinear triangle", [6][3]int64{
			{0, 0, 0}, {1, 1, 1}, {2, 2, 2}, {0, 0, 0}, {1, 1, 1}, {2, 2, 2},
		}},
		{"point triangle", [6][3]int64{
			{4, -2, 7}, {4, -2, 7}, {4, -2, 7}, {4, -2, 7}, {4, -2, 7}, {4, -2, 7},
		}},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			points := loftHullIntegerPoints(tc.points)
			for vertex := range 3 {
				for axis := range 3 {
					points[vertex+3][axis] = new(big.Rat).Set(points[vertex][axis])
				}
			}
			got, ok, err := VolumeHullBound(proofbound.NewWorkBudget(t.Context()), points)
			require.NoError(t, err)
			require.True(t, ok)
			require.Equal(t, "0", got.RatString())
			got.SetInt64(9)
			require.Equal(t, tc.points[0][0], points[0][0].Num().Int64())
		})
	}

	// One moved vertex gives a tetrahedron, so it must take the general path.
	moved := loftHullIntegerPoints([6][3]int64{
		{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}, {1, 0, 0}, {0, 1, 0},
	})
	got, ok, err := VolumeHullBound(proofbound.NewWorkBudget(t.Context()), moved)
	require.NoError(t, err)
	require.True(t, ok)
	require.Equal(t, "1/6", got.RatString())
}

func TestLoftVolumeHullBoundZeroMotionWorkAndRefusals(t *testing.T) {
	points := loftHullIntegerPoints([6][3]int64{
		{0, 0, 0}, {2, 0, 0}, {0, 3, 0}, {0, 0, 0}, {2, 0, 0}, {0, 3, 0},
	})
	steps := 0
	budget := &proofbound.WorkBudget{
		StepFn: func() error { steps++; return nil },
		ErrFn:  t.Context().Err,
	}
	got, ok, err := VolumeHullBound(budget, points)
	require.NoError(t, err)
	require.True(t, ok)
	require.Equal(t, "0", got.RatString())
	require.Equal(t, 27, steps) // 18 input checks and 9 guarded coordinate comparisons.
	stationarySteps := steps

	generalSteps := 0
	generalBudget := &proofbound.WorkBudget{
		StepFn: func() error { generalSteps++; return nil },
		ErrFn:  t.Context().Err,
	}
	general, err := (loftHullMath{budget: generalBudget}).volume(points)
	require.NoError(t, err)
	require.Equal(t, "0", general.RatString())
	require.Greater(t, generalSteps, steps)
	t.Logf("stationary steps=%d, previous general path steps=%d", stationarySteps, 18+generalSteps)

	stopped := errors.New("shared output work exhausted")
	steps = 0
	budget = &proofbound.WorkBudget{
		StepFn: func() error {
			steps++
			if steps == 20 {
				return stopped
			}
			return nil
		},
		ErrFn: t.Context().Err,
	}
	got, ok, err = VolumeHullBound(budget, points)
	require.Nil(t, got)
	require.False(t, ok)
	require.ErrorIs(t, err, stopped)
	require.Equal(t, 20, steps)

	ctx, cancel := context.WithCancel(t.Context())
	defer cancel()
	steps = 0
	budget = &proofbound.WorkBudget{
		StepFn: func() error {
			steps++
			if steps == 27 {
				cancel()
			}
			return nil
		},
		ErrFn: ctx.Err,
	}
	got, ok, err = VolumeHullBound(budget, points)
	require.Nil(t, got)
	require.False(t, ok)
	require.ErrorIs(t, err, context.Canceled)
	require.Equal(t, 27, steps)

	tooLarge := points
	tooLarge[0][0] = new(big.Rat).SetInt(new(big.Int).Lsh(big.NewInt(1), loftHullRatBits))
	got, ok, err = VolumeHullBound(proofbound.NewWorkBudget(t.Context()), tooLarge)
	require.Nil(t, got)
	require.False(t, ok)
	require.NoError(t, err)

	missing := points
	missing[0][0] = nil
	got, ok, err = VolumeHullBound(proofbound.NewWorkBudget(t.Context()), missing)
	require.Nil(t, got)
	require.False(t, ok)
	require.NoError(t, err)

	// A large exact rational still qualifies when each guarded cross product fits.
	safe := points
	safeNumerator := new(big.Int).Add(new(big.Int).Lsh(big.NewInt(1), 3900), big.NewInt(1))
	safeDenominator := new(big.Int).Lsh(big.NewInt(1), 3900)
	safe[0][0] = new(big.Rat).SetFrac(safeNumerator, safeDenominator)
	safe[3][0] = new(big.Rat).Set(safe[0][0])
	got, ok, err = VolumeHullBound(proofbound.NewWorkBudget(t.Context()), safe)
	require.NoError(t, err)
	require.True(t, ok)
	require.Equal(t, "0", got.RatString())

	// Both inputs fit, but comparing them would multiply two 5001-bit integers.
	// The shortcut must let the guarded general path decide, not compare them raw.
	large := points
	n := new(big.Int).Add(new(big.Int).Lsh(big.NewInt(1), 5000), big.NewInt(1))
	d := new(big.Int).Lsh(big.NewInt(1), 5000)
	large[0][0] = new(big.Rat).SetFrac(n, d)
	large[3][0] = new(big.Rat).Set(large[0][0])
	got, ok, err = VolumeHullBound(proofbound.NewWorkBudget(t.Context()), large)
	require.Nil(t, got)
	require.False(t, ok)
	require.NoError(t, err)
}
