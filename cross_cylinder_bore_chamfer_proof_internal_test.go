package decad

import (
	"context"
	"errors"
	"math"
	"math/big"
	"sync/atomic"
	"testing"

	"github.com/lestrrat-3d/decad/internal/orderedwork"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/stretchr/testify/require"
)

// crossBoreCellProof is one cell proved directly, with no mirror map: its
// exact part, compared with the samples crossBoreFeet builds at the cell's
// own angle, Z read relative to c in float, exactly as
// crossBoreCellProofReference and crossBoreCellProofFullEnds compare them.
// The reference tests and the mirrored route's tests compare against it.
func crossBoreCellProof(rec *crossCylinderBoreRecord, setback float64,
	i int, angularGap *big.Rat) (crossBoreCellResult, error) {
	exact, err := crossBoreCellExactProof(rec, setback, i, angularGap)
	if err != nil {
		return crossBoreCellResult{}, err
	}
	const n = 2 * crossBoreHalfStations
	theta := 2 * math.Pi * float64(i) / n
	outer, inner := crossBoreFeet(rec, setback, theta)
	co, sn := math.Cos(theta), math.Sin(theta)
	held := [9]float64{
		outer.X, outer.Y, outer.Z - rec.CenterZ,
		inner.X, inner.Y, inner.Z - rec.CenterZ,
		-math.Sqrt(rec.Radius*rec.Radius - rec.Bore*rec.Bore*co*co), rec.Bore * co, rec.Bore * sn,
	}
	sampleError := 0.0
	for c := range held {
		sampleError = max(sampleError, proofbound.LazyIntervalFloatError(exact.coords[c], held[c]))
	}
	return crossBoreCellResult{
		width: exact.width, sampleError: proofbound.Radius3D(sampleError),
		speed: exact.speed, bandLength: exact.bandLength,
	}, nil
}

// crossBoreCellMaxima is the direct route the mirrored one replaced: it folds
// cell(i) for i in [0, n) into the componentwise maximum in index order and returns the error of the lowest failing cell,
// whatever the worker count. With workers below 2 it polls ctx every 32
// cells; otherwise it polls ctx before the fan-out and the calling goroutine
// polls it while the workers run. cell runs on worker goroutines and must
// not touch ctx or any shared budget.
func crossBoreCellMaxima(ctx context.Context, n, workers int,
	cell func(i int) (crossBoreCellResult, error)) (crossBoreCellResult, error) {
	maximum := crossBoreCellResult{}
	fold := func(c crossBoreCellResult) {
		maximum.width = max(maximum.width, c.width)
		maximum.sampleError = max(maximum.sampleError, c.sampleError)
		maximum.speed = max(maximum.speed, c.speed)
		maximum.bandLength = max(maximum.bandLength, c.bandLength)
	}
	if workers < 2 {
		for i := range n {
			if i%32 == 0 {
				if err := ctx.Err(); err != nil {
					return crossBoreCellResult{}, err
				}
			}
			c, err := cell(i)
			if err != nil {
				return crossBoreCellResult{}, err
			}
			fold(c)
		}
		return maximum, nil
	}
	if err := ctx.Err(); err != nil {
		return crossBoreCellResult{}, err
	}
	cells := make([]crossBoreCellResult, n)
	if _, err := orderedwork.Each(n, workers, crossBoreCellGrain, ctx.Err, func(i int) error {
		c, err := cell(i)
		cells[i] = c
		return err
	}); err != nil {
		return crossBoreCellResult{}, err
	}
	for i := range n {
		fold(cells[i])
	}
	return maximum, nil
}

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
		{crossCylinderBoreRecord{Radius: 5, Bore: 1, Height: 10, CenterZ: 5, SourceBound: 1e-9}, 0.5},
		{crossCylinderBoreRecord{Radius: 8, Bore: 1.5, Height: 16, CenterZ: 8, SourceBound: 1e-9}, 0.1875},
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
