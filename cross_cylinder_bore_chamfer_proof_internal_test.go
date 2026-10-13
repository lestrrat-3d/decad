package decad

import (
	"context"
	"errors"
	"math"
	"math/big"
	"sync/atomic"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/stretchr/testify/require"
)

func crossBoreSyntheticCell(i int) crossBoreCellResult {
	h := uint64(i)*0x9E3779B97F4A7C15 + 0x1234567
	next := func() float64 {
		h ^= h >> 31
		h *= 0xBF58476D1CE4E5B9
		h ^= h >> 29
		return float64(h>>11) / (1 << 53)
	}
	c := crossBoreCellResult{width: next(), sampleError: next(), speed: next(), bandLength: next()}
	switch i {
	case 5:
		c.width = math.NaN()
	case 6:
		c.speed = math.Inf(1)
	case 7:
		c.sampleError = math.Copysign(0, -1)
	case 8:
		c.bandLength = 0
	}
	return c
}

func requireCrossBoreBitsEqual(t *testing.T, a, b crossBoreCellResult) {
	t.Helper()
	require.Equal(t, math.Float64bits(a.width), math.Float64bits(b.width))
	require.Equal(t, math.Float64bits(a.sampleError), math.Float64bits(b.sampleError))
	require.Equal(t, math.Float64bits(a.speed), math.Float64bits(b.speed))
	require.Equal(t, math.Float64bits(a.bandLength), math.Float64bits(b.bandLength))
}

func TestCrossBoreCellMaximaMatchesSerial(t *testing.T) {
	const n = 4096
	ok := func(i int) (crossBoreCellResult, error) { return crossBoreSyntheticCell(i), nil }
	serial, err := crossBoreCellMaxima(t.Context(), n, 1, ok)
	require.NoError(t, err)
	parallel, err := crossBoreCellMaxima(t.Context(), n, 4, ok)
	require.NoError(t, err)
	requireCrossBoreBitsEqual(t, serial, parallel)

	err37, err3000 := errors.New("cell 37"), errors.New("cell 3000")
	for _, workers := range []int{1, 4} {
		var calls [n]atomic.Int32
		_, err := crossBoreCellMaxima(t.Context(), n, workers, func(i int) (crossBoreCellResult, error) {
			calls[i].Add(1)
			switch i {
			case 37:
				return crossBoreCellResult{}, err37
			case 3000:
				return crossBoreCellResult{}, err3000
			}
			return crossBoreSyntheticCell(i), nil
		})
		require.Same(t, err37, err)
		for i := range 38 {
			require.EqualValues(t, 1, calls[i].Load(), "cell %d, workers %d", i, workers)
		}
		if workers == 1 {
			for i := 38; i < n; i++ {
				require.Zero(t, calls[i].Load())
			}
		}
	}
}

func TestCrossBoreCellMaximaHonoursCancelledContext(t *testing.T) {
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	for _, workers := range []int{1, 4} {
		var calls atomic.Int32
		_, err := crossBoreCellMaxima(ctx, 4096, workers, func(int) (crossBoreCellResult, error) {
			calls.Add(1)
			return crossBoreCellResult{}, nil
		})
		require.ErrorIs(t, err, context.Canceled)
		require.Zero(t, calls.Load())
	}
}

func TestCrossBoreCellProofsParallelMatchSerial(t *testing.T) {
	const n = 64
	gap := new(big.Rat).Quo(new(big.Rat).Mul(big.NewRat(2, 1), proofbound.PiUpper), big.NewRat(2*crossBoreHalfStations, 1))
	for _, tc := range []struct {
		rec     crossCylinderBoreRecord
		setback float64
	}{
		{crossCylinderBoreRecord{radius: 5, bore: 1, height: 10, centerZ: 5, sourceBound: 1e-9}, 0.5},
		{crossCylinderBoreRecord{radius: 8, bore: 1.5, height: 16, centerZ: 8, sourceBound: 1e-9}, 0.1875},
	} {
		cell := func(i int) (crossBoreCellResult, error) {
			return crossBoreCellProof(&tc.rec, tc.setback, 64*i, gap)
		}
		var want crossBoreCellResult
		for i := range n {
			c, err := cell(i)
			require.NoError(t, err)
			want.width = max(want.width, c.width)
			want.sampleError = max(want.sampleError, c.sampleError)
			want.speed = max(want.speed, c.speed)
			want.bandLength = max(want.bandLength, c.bandLength)
		}
		got, err := crossBoreCellMaxima(t.Context(), n, 4, cell)
		require.NoError(t, err)
		requireCrossBoreBitsEqual(t, want, got)
	}
}
