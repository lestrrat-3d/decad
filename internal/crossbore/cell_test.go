package crossbore

import (
	"context"
	"errors"
	"math"
	"math/big"
	"sync/atomic"
	"testing"

	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/orderedwork"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// crossBoreCellProof is one cell proved directly, with no mirror map: its
// exact part, compared with the samples Feet builds at the cell's
// own angle, Z read relative to c in float, exactly as
// crossBoreCellProofReference and crossBoreCellProofFullEnds compare them.
// The reference tests and the mirrored route's tests compare against it.
func crossBoreCellProof(rec *facetedrecord.CrossCylinderBoreRecord, setback float64,
	i int, angularGap *big.Rat) (CellResult, error) {
	exact, err := CellExactProof(rec, setback, i, angularGap)
	if err != nil {
		return CellResult{}, err
	}
	const n = 2 * HalfStations
	theta := 2 * math.Pi * float64(i) / n
	outer, inner := Feet(rec, setback, theta)
	co, sn := math.Cos(theta), math.Sin(theta)
	held := [9]float64{
		outer.X, outer.Y, outer.Z - rec.CenterZ,
		inner.X, inner.Y, inner.Z - rec.CenterZ,
		-math.Sqrt(rec.Radius*rec.Radius - rec.Bore*rec.Bore*co*co), rec.Bore * co, rec.Bore * sn,
	}
	sampleError := 0.0
	for c := range held {
		sampleError = max(sampleError, proofbound.LazyIntervalFloatError(exact.Coords[c], held[c]))
	}
	return CellResult{
		Width: exact.Width, SampleError: proofbound.Radius3D(sampleError),
		Speed: exact.Speed, BandLength: exact.BandLength,
	}, nil
}

// crossBoreCellMaxima is the direct route the mirrored one replaced: it folds
// cell(i) for i in [0, n) into the componentwise maximum in index order and returns the error of the lowest failing cell,
// whatever the worker count. With workers below 2 it polls ctx every 32
// cells; otherwise it polls ctx before the fan-out and the calling goroutine
// polls it while the workers run. cell runs on worker goroutines and must
// not touch ctx or any shared budget.
func crossBoreCellMaxima(ctx context.Context, n, workers int,
	cell func(i int) (CellResult, error)) (CellResult, error) {
	maximum := CellResult{}
	fold := func(c CellResult) {
		maximum.Width = max(maximum.Width, c.Width)
		maximum.SampleError = max(maximum.SampleError, c.SampleError)
		maximum.Speed = max(maximum.Speed, c.Speed)
		maximum.BandLength = max(maximum.BandLength, c.BandLength)
	}
	if workers < 2 {
		for i := range n {
			if i%32 == 0 {
				if err := ctx.Err(); err != nil {
					return CellResult{}, err
				}
			}
			c, err := cell(i)
			if err != nil {
				return CellResult{}, err
			}
			fold(c)
		}
		return maximum, nil
	}
	if err := ctx.Err(); err != nil {
		return CellResult{}, err
	}
	cells := make([]CellResult, n)
	if _, err := orderedwork.Each(n, workers, CellGrain, ctx.Err, func(i int) error {
		c, err := cell(i)
		cells[i] = c
		return err
	}); err != nil {
		return CellResult{}, err
	}
	for i := range n {
		fold(cells[i])
	}
	return maximum, nil
}

func crossBoreSyntheticCell(i int) CellResult {
	h := uint64(i)*0x9E3779B97F4A7C15 + 0x1234567
	next := func() float64 {
		h ^= h >> 31
		h *= 0xBF58476D1CE4E5B9
		h ^= h >> 29
		return float64(h>>11) / (1 << 53)
	}
	c := CellResult{Width: next(), SampleError: next(), Speed: next(), BandLength: next()}
	switch i {
	case 5:
		c.Width = math.NaN()
	case 6:
		c.Speed = math.Inf(1)
	case 7:
		c.SampleError = math.Copysign(0, -1)
	case 8:
		c.BandLength = 0
	}
	return c
}

func requireCrossBoreBitsEqual(t *testing.T, a, b CellResult) {
	t.Helper()
	require.Equal(t, math.Float64bits(a.Width), math.Float64bits(b.Width))
	require.Equal(t, math.Float64bits(a.SampleError), math.Float64bits(b.SampleError))
	require.Equal(t, math.Float64bits(a.Speed), math.Float64bits(b.Speed))
	require.Equal(t, math.Float64bits(a.BandLength), math.Float64bits(b.BandLength))
}

func TestCrossBoreCellMaximaMatchesSerial(t *testing.T) {
	const n = 4096
	ok := func(i int) (CellResult, error) { return crossBoreSyntheticCell(i), nil }
	serial, err := crossBoreCellMaxima(t.Context(), n, 1, ok)
	require.NoError(t, err)
	parallel, err := crossBoreCellMaxima(t.Context(), n, 4, ok)
	require.NoError(t, err)
	requireCrossBoreBitsEqual(t, serial, parallel)

	err37, err3000 := errors.New("cell 37"), errors.New("cell 3000")
	for _, workers := range []int{1, 4} {
		var calls [n]atomic.Int32
		_, err := crossBoreCellMaxima(t.Context(), n, workers, func(i int) (CellResult, error) {
			calls[i].Add(1)
			switch i {
			case 37:
				return CellResult{}, err37
			case 3000:
				return CellResult{}, err3000
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
		_, err := crossBoreCellMaxima(ctx, 4096, workers, func(int) (CellResult, error) {
			calls.Add(1)
			return CellResult{}, nil
		})
		require.ErrorIs(t, err, context.Canceled)
		require.Zero(t, calls.Load())
	}
}

func TestCrossBoreCellProofsParallelMatchSerial(t *testing.T) {
	const n = 64
	gap := new(big.Rat).Quo(new(big.Rat).Mul(big.NewRat(2, 1), proofbound.PiUpper), big.NewRat(2*HalfStations, 1))
	for _, tc := range []struct {
		rec     facetedrecord.CrossCylinderBoreRecord
		setback float64
	}{
		{facetedrecord.CrossCylinderBoreRecord{Radius: 5, Bore: 1, Height: 10, CenterZ: 5, SourceBound: 1e-9}, 0.5},
		{facetedrecord.CrossCylinderBoreRecord{Radius: 8, Bore: 1.5, Height: 16, CenterZ: 8, SourceBound: 1e-9}, 0.1875},
	} {
		cell := func(i int) (CellResult, error) {
			return crossBoreCellProof(&tc.rec, tc.setback, 64*i, gap)
		}
		var want CellResult
		for i := range n {
			c, err := cell(i)
			require.NoError(t, err)
			want.Width = max(want.Width, c.Width)
			want.SampleError = max(want.SampleError, c.SampleError)
			want.Speed = max(want.Speed, c.Speed)
			want.BandLength = max(want.BandLength, c.BandLength)
		}
		got, err := crossBoreCellMaxima(t.Context(), n, 4, cell)
		require.NoError(t, err)
		requireCrossBoreBitsEqual(t, want, got)
	}
}

// crossBoreMirrorCoords returns coords with the Y enclosures negated when
// flipY and the Z−c enclosures when flipZ.
func crossBoreMirrorCoords(coords [9]crossIV, flipY, flipZ bool) [9]crossIV {
	for c := range coords {
		if (flipY && c%3 == 1) || (flipZ && c%3 == 2) {
			coords[c] = crossNeg(coords[c])
		}
	}
	return coords
}

func TestCrossBoreCellMirrorMapsTurnsExactly(t *testing.T) {
	const n = 2 * HalfStations
	hit := make([]bool, n/4+1)
	half, one := big.NewRat(1, 2), big.NewRat(1, 1)
	for j := range n {
		k, flipY, flipZ := CellMirror(j)
		require.True(t, 0 <= k && k <= n/4, "cell %d maps to %d", j, k)
		hit[k] = true
		turnJ, turnK := big.NewRat(int64(j), n), big.NewRat(int64(k), n)
		var want *big.Rat
		switch {
		case !flipY && !flipZ:
			want = turnK
		case flipY && !flipZ:
			want = new(big.Rat).Sub(half, turnK)
		case flipY && flipZ:
			want = new(big.Rat).Add(half, turnK)
		default:
			want = new(big.Rat).Sub(one, turnK)
		}
		require.Zero(t, turnJ.Cmp(want), "cell %d: base %d flipY %v flipZ %v", j, k, flipY, flipZ)
	}
	for k, ok := range hit {
		require.True(t, ok, "base cell %d is never used", k)
	}
}

// TestCrossBoreMirroredCellsEncloseSamples is a falsifier: source points
// across a mapped cell's angular range must lie inside its base cell's
// mirrored enclosures. A point outside disproves the map; points inside
// prove nothing by themselves.
func TestCrossBoreMirroredCellsEncloseSamples(t *testing.T) {
	const n = 2 * HalfStations
	gap := crossBoreCellGap()
	cells := map[int]struct{}{}
	for j := 0; j < n; j += 64 {
		cells[j] = struct{}{}
	}
	for _, q := range []int{0, n / 4, n / 2, 3 * n / 4, n} {
		for d := -3; d <= 3; d++ {
			cells[(q+d+n)%n] = struct{}{}
		}
	}
	for _, tc := range []struct {
		name    string
		rec     facetedrecord.CrossCylinderBoreRecord
		setback float64
	}{
		{"five by one", facetedrecord.CrossCylinderBoreRecord{Radius: 5, Bore: 1, Height: 10, CenterZ: 5}, 0.5},
		{"eight by one and half", facetedrecord.CrossCylinderBoreRecord{Radius: 8, Bore: 1.5, Height: 16, CenterZ: 8}, 0.1875},
	} {
		t.Run(tc.name, func(t *testing.T) {
			bases := map[int]CellExact{}
			for j := range n {
				if _, ok := cells[j]; !ok {
					continue
				}
				k, flipY, flipZ := CellMirror(j)
				base, ok := bases[k]
				if !ok {
					var err error
					base, err = CellExactProof(&tc.rec, tc.setback, k, gap)
					require.NoError(t, err)
					bases[k] = base
				}
				coords := crossBoreMirrorCoords(base.Coords, flipY, flipZ)
				for _, u := range []float64{-1, -0.5, 0, 0.5, 1} {
					theta := 2 * math.Pi * (float64(j) + u) / n
					outer, inner := Feet(&tc.rec, tc.setback, theta)
					co, sn := math.Cos(theta), math.Sin(theta)
					points := [9]float64{
						outer.X, outer.Y, outer.Z - tc.rec.CenterZ,
						inner.X, inner.Y, inner.Z - tc.rec.CenterZ,
						-math.Sqrt(tc.rec.Radius*tc.rec.Radius - tc.rec.Bore*tc.rec.Bore*co*co),
						tc.rec.Bore * co, tc.rec.Bore * sn,
					}
					for c, p := range points {
						lo, _ := coords[c].Lo.Rat().Float64()
						hi, _ := coords[c].Hi.Rat().Float64()
						require.True(t, lo-1e-12 <= p && p <= hi+1e-12,
							"cell %d (base %d) u %g coordinate %d: %.17g outside [%.17g, %.17g]",
							j, k, u, c, p, lo, hi)
					}
				}
			}
		})
	}
}

// crossBoreSyntheticExact is a base cell with hash-derived readings whose
// first coordinate carries its index k.
func crossBoreSyntheticExact(k int) CellExact {
	c := crossBoreSyntheticCell(k + 1000)
	e := CellExact{Width: c.Width, Speed: c.Speed, BandLength: c.BandLength}
	e.Coords[0] = crossPoint(float64(k))
	return e
}

func crossBoreSyntheticSample(j, k int, flipY, flipZ bool) float64 {
	flags := 0
	if flipY {
		flags |= 1
	}
	if flipZ {
		flags |= 2
	}
	return crossBoreSyntheticCell(j*4096*4 + k*4 + flags).SampleError
}

func TestCrossBoreMirroredMaximaFold(t *testing.T) {
	const n = 4096
	var want CellResult
	for k := range n/4 + 1 {
		e := crossBoreSyntheticExact(k)
		want.Width = max(want.Width, e.Width)
		want.Speed = max(want.Speed, e.Speed)
		want.BandLength = max(want.BandLength, e.BandLength)
	}
	for j := range n {
		k, flipY, flipZ := CellMirror(j)
		want.SampleError = max(want.SampleError, crossBoreSyntheticSample(j, k, flipY, flipZ))
	}
	for _, workers := range []int{1, 4} {
		var calls [n]atomic.Int32
		got, err := MirroredCellMaxima(t.Context(), workers,
			func(k int) (CellExact, error) { return crossBoreSyntheticExact(k), nil },
			func(j int, base *CellExact, flipY, flipZ bool) float64 {
				calls[j].Add(1)
				k, wantY, wantZ := CellMirror(j)
				f, _ := base.Coords[0].Lo.Rat().Float64()
				if int(f) != k || flipY != wantY || flipZ != wantZ {
					return math.Inf(1)
				}
				return crossBoreSyntheticSample(j, k, flipY, flipZ)
			})
		require.NoError(t, err)
		requireCrossBoreBitsEqual(t, want, got)
		for j := range n {
			require.EqualValues(t, 1, calls[j].Load(), "cell %d, workers %d", j, workers)
		}
	}
}

func TestCrossBoreMirroredMaximaFirstFailureAndContext(t *testing.T) {
	const n = 4096
	err37, err900 := errors.New("cell 37"), errors.New("cell 900")
	for _, workers := range []int{1, 4} {
		var calls [n/4 + 1]atomic.Int32
		var samples atomic.Int32
		_, err := MirroredCellMaxima(t.Context(), workers,
			func(k int) (CellExact, error) {
				calls[k].Add(1)
				switch k {
				case 37:
					return CellExact{}, err37
				case 900:
					return CellExact{}, err900
				}
				return crossBoreSyntheticExact(k), nil
			},
			func(int, *CellExact, bool, bool) float64 {
				samples.Add(1)
				return 0
			})
		require.Same(t, err37, err)
		require.Zero(t, samples.Load())
		for k := range 38 {
			require.EqualValues(t, 1, calls[k].Load(), "cell %d, workers %d", k, workers)
		}
		if workers == 1 {
			for k := 38; k <= n/4; k++ {
				require.Zero(t, calls[k].Load())
			}
		}
	}

	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	for _, workers := range []int{1, 4} {
		var calls atomic.Int32
		_, err := MirroredCellMaxima(ctx, workers,
			func(k int) (CellExact, error) {
				calls.Add(1)
				return crossBoreSyntheticExact(k), nil
			},
			func(int, *CellExact, bool, bool) float64 {
				calls.Add(1)
				return 0
			})
		require.ErrorIs(t, err, context.Canceled)
		require.Zero(t, calls.Load())
	}

	// A context cancelled while the samples are compared fails the call,
	// even when every remaining sample still runs.
	for _, workers := range []int{1, 4} {
		ctx, cancel := context.WithCancel(t.Context())
		_, err := MirroredCellMaxima(ctx, workers,
			func(k int) (CellExact, error) { return crossBoreSyntheticExact(k), nil },
			func(j int, _ *CellExact, _, _ bool) float64 {
				if j == n-1 {
					cancel()
				}
				return 0
			})
		cancel()
		require.ErrorIs(t, err, context.Canceled)
	}
}

func TestCrossBoreMirroredRouteRefusesLikeDirect(t *testing.T) {
	const n = 2 * HalfStations
	gap := crossBoreCellGap()
	for _, tc := range []struct {
		name string
		rec  facetedrecord.CrossCylinderBoreRecord
	}{
		{"loses clearance or folds", facetedrecord.CrossCylinderBoreRecord{Radius: 5, Bore: 4.9, Height: 10, CenterZ: 5}},
		{"no real X", facetedrecord.CrossCylinderBoreRecord{Radius: 5, Bore: 5, Height: 10, CenterZ: 5}},
	} {
		t.Run(tc.name, func(t *testing.T) {
			var firstDirect error
			for j := range n {
				_, err := CellExactProof(&tc.rec, 0.5, j, gap)
				if err != nil {
					firstDirect = err
					break
				}
			}
			require.Error(t, firstDirect)
			_, err := MirroredCellMaxima(t.Context(), 4,
				func(k int) (CellExact, error) { return CellExactProof(&tc.rec, 0.5, k, gap) },
				func(int, *CellExact, bool, bool) float64 { return 0 })
			require.EqualError(t, err, firstDirect.Error())
		})
	}
}

// TestCrossBoreNegationIsExact confirms crossNeg is the exact negation the
// sample comparison uses: −[a, b] = [−b, −a] as rationals, with the same
// width and the mirrored float error.
func TestCrossBoreNegationIsExact(t *testing.T) {
	a := crossIV{Lo: proofarith.LazyOf(big.NewRat(-3, 7)), Hi: proofarith.LazyOf(big.NewRat(5, 11))}
	got := crossNeg(a)
	require.Zero(t, got.Lo.Rat().Cmp(big.NewRat(-5, 11)))
	require.Zero(t, got.Hi.Rat().Cmp(big.NewRat(3, 7)))
	require.Equal(t, proofbound.LazyIntervalWidthUp(a), proofbound.LazyIntervalWidthUp(got))
	require.Equal(t, proofbound.LazyIntervalFloatError(a, 0.25), proofbound.LazyIntervalFloatError(got, -0.25))
}

// TestCrossBoreCellSampleErrorReadsHeldZExactly compares a held Z with an
// enclosure whose shift by c is not a float: Z = 1e-20 against the exact
// point c + (−1) = 0 with c = 1. The held vertex is 1e-20 from the source,
// while fl(Z − c) = −1 would read no error at all.
func TestCrossBoreCellSampleErrorReadsHeldZExactly(t *testing.T) {
	var coords [9]crossIV
	for c := range coords {
		coords[c] = crossPoint(0)
	}
	coords[2] = crossPoint(-1)
	held := [3]r3.Vec{{Z: 1e-20}, {Z: 1}, {Z: 1}}
	got := CellSampleError(&held, 1, &coords, false, false)
	require.GreaterOrEqual(t, got, 1e-20)
	require.Less(t, got, 1e-19)
}

func crossBoreCellGap() *big.Rat {
	return new(big.Rat).Quo(new(big.Rat).Mul(big.NewRat(2, 1), proofbound.PiUpper),
		big.NewRat(2*HalfStations, 1))
}
