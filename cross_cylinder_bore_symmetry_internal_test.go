package decad

import (
	"context"
	"errors"
	"math"
	"math/big"
	"sync/atomic"
	"testing"

	"github.com/lestrrat-3d/decad/internal/orderedwork"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func crossBoreCellGap() *big.Rat {
	return new(big.Rat).Quo(new(big.Rat).Mul(big.NewRat(2, 1), proofbound.PiUpper),
		big.NewRat(2*crossBoreHalfStations, 1))
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

func crossBoreMeshHeld(mesh *crossBoreMesh, j int) [3]r3.Vec {
	cell := mesh.cellHeld[j]
	return [3]r3.Vec{mesh.verts[cell[0]], mesh.verts[cell[1]], mesh.verts[cell[2]]}
}

func TestCrossBoreCellMirrorMapsTurnsExactly(t *testing.T) {
	const n = 2 * crossBoreHalfStations
	hit := make([]bool, n/4+1)
	half, one := big.NewRat(1, 2), big.NewRat(1, 1)
	for j := range n {
		k, flipY, flipZ := crossBoreCellMirror(j)
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
	const n = 2 * crossBoreHalfStations
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
		rec     crossCylinderBoreRecord
		setback float64
	}{
		{"five by one", crossCylinderBoreRecord{radius: 5, bore: 1, height: 10, centerZ: 5}, 0.5},
		{"eight by one and half", crossCylinderBoreRecord{radius: 8, bore: 1.5, height: 16, centerZ: 8}, 0.1875},
	} {
		t.Run(tc.name, func(t *testing.T) {
			bases := map[int]crossBoreCellExact{}
			for j := range n {
				if _, ok := cells[j]; !ok {
					continue
				}
				k, flipY, flipZ := crossBoreCellMirror(j)
				base, ok := bases[k]
				if !ok {
					var err error
					base, err = crossBoreCellExactProof(&tc.rec, tc.setback, k, gap)
					require.NoError(t, err)
					bases[k] = base
				}
				coords := crossBoreMirrorCoords(base.coords, flipY, flipZ)
				for _, u := range []float64{-1, -0.5, 0, 0.5, 1} {
					theta := 2 * math.Pi * (float64(j) + u) / n
					outer, inner := crossBoreFeet(&tc.rec, tc.setback, theta)
					co, sn := math.Cos(theta), math.Sin(theta)
					points := [9]float64{
						outer.X, outer.Y, outer.Z - tc.rec.centerZ,
						inner.X, inner.Y, inner.Z - tc.rec.centerZ,
						-math.Sqrt(tc.rec.radius*tc.rec.radius - tc.rec.bore*tc.rec.bore*co*co),
						tc.rec.bore * co, tc.rec.bore * sn,
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
func crossBoreSyntheticExact(k int) crossBoreCellExact {
	c := crossBoreSyntheticCell(k + 1000)
	e := crossBoreCellExact{width: c.width, speed: c.speed, bandLength: c.bandLength}
	e.coords[0] = crossPoint(float64(k))
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
	return crossBoreSyntheticCell(j*4096*4 + k*4 + flags).sampleError
}

func TestCrossBoreMirroredMaximaFold(t *testing.T) {
	const n = 4096
	var want crossBoreCellResult
	for k := range n/4 + 1 {
		e := crossBoreSyntheticExact(k)
		want.width = max(want.width, e.width)
		want.speed = max(want.speed, e.speed)
		want.bandLength = max(want.bandLength, e.bandLength)
	}
	for j := range n {
		k, flipY, flipZ := crossBoreCellMirror(j)
		want.sampleError = max(want.sampleError, crossBoreSyntheticSample(j, k, flipY, flipZ))
	}
	for _, workers := range []int{1, 4} {
		var calls [n]atomic.Int32
		got, err := crossBoreMirroredCellMaxima(t.Context(), workers,
			func(k int) (crossBoreCellExact, error) { return crossBoreSyntheticExact(k), nil },
			func(j int, base *crossBoreCellExact, flipY, flipZ bool) float64 {
				calls[j].Add(1)
				k, wantY, wantZ := crossBoreCellMirror(j)
				f, _ := base.coords[0].Lo.Rat().Float64()
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
		_, err := crossBoreMirroredCellMaxima(t.Context(), workers,
			func(k int) (crossBoreCellExact, error) {
				calls[k].Add(1)
				switch k {
				case 37:
					return crossBoreCellExact{}, err37
				case 900:
					return crossBoreCellExact{}, err900
				}
				return crossBoreSyntheticExact(k), nil
			},
			func(int, *crossBoreCellExact, bool, bool) float64 {
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
		_, err := crossBoreMirroredCellMaxima(ctx, workers,
			func(k int) (crossBoreCellExact, error) {
				calls.Add(1)
				return crossBoreSyntheticExact(k), nil
			},
			func(int, *crossBoreCellExact, bool, bool) float64 {
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
		_, err := crossBoreMirroredCellMaxima(ctx, workers,
			func(k int) (crossBoreCellExact, error) { return crossBoreSyntheticExact(k), nil },
			func(j int, _ *crossBoreCellExact, _, _ bool) float64 {
				if j == n-1 {
					cancel()
				}
				return 0
			})
		cancel()
		require.ErrorIs(t, err, context.Canceled)
	}
}

// crossBoreSourceBodies builds the target and tool prisms of the public
// five-by-one cross bore (R = 5, r = 1, H = 10, c = 5) with its target circle
// centred at (0, targetY) and its tool circle centred at Y = toolY.
func crossBoreSourceBodies(t *testing.T, targetY, toolY float64) (*Body, *Body) {
	t.Helper()
	const radius, bore, height, centerZ = 5.0, 1.0, 10.0, 5.0
	doc := New()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	center := s.CreatePoint(0, targetY)
	s.Fix(center)
	s.CreateCircle(center, radius)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	target, err := Extrude(t.Context(), doc, s, s.Profiles()[0],
		Distance{D: units.Millimeters(height), Dir: Along})
	require.NoError(t, err)
	tw := sketch.NewWorld()
	ts, err := tw.CreateSketch(tw.YZ())
	require.NoError(t, err)
	toolCenter := ts.CreatePoint(toolY, centerZ)
	ts.Fix(toolCenter)
	ts.CreateCircle(toolCenter, bore)
	_, err = ts.Solve(t.Context())
	require.NoError(t, err)
	tool, err := Extrude(t.Context(), doc, ts, ts.Profiles()[0], Symmetric{D: units.Millimeters(2*radius + 2)})
	require.NoError(t, err)
	return target, tool
}

func crossBoreCircleCenter(t *testing.T, body *Body) planePoint {
	t.Helper()
	pp, ok := body.n.Payload.(prismPayload)
	require.True(t, ok)
	circle, ok := pp.Profile.Outer.Segments[0].(circleSeg)
	require.True(t, ok)
	return circle.Center
}

// TestCrossBoreCellsRefuseNearSymmetricPose breaks the pose's mirror
// symmetry by one rounding step and requires the cell proof to refuse
// before it proves any cell. The cell formulas read only R, r and d, so
// without the pose check every one of these records would prove, on the
// symmetric record's own mesh.
func TestCrossBoreCellsRefuseNearSymmetricPose(t *testing.T) {
	const d = 0.5
	ulp := math.Nextafter(0, 1)
	target, tool := crossBoreSourceBodies(t, 0, 0)
	exact := recognizeCrossCylinderBore(target, tool)
	require.NotNil(t, exact)
	require.True(t, crossBoreMirrorPose(exact))
	mesh, err := buildCrossBoreChamferMesh(t.Context(), exact, d)
	require.NoError(t, err)

	_, offsetTool := crossBoreSourceBodies(t, 0, ulp)
	require.Equal(t, ulp, crossBoreCircleCenter(t, offsetTool).U)
	offsetTarget, _ := crossBoreSourceBodies(t, ulp, 0)
	require.Equal(t, ulp, crossBoreCircleCenter(t, offsetTarget).V)
	// A full turn about the bore axis is the identity in the reals; in
	// floating point it leaves sin(2π) ≠ 0 off the diagonal.
	turn, err := r3.Rotation(r3.Vec{X: 1}, units.Radians(2*math.Pi))
	require.NoError(t, err)
	require.NotEqual(t, r3.Identity(), turn)
	_, turnSource := crossBoreSourceBodies(t, 0, 0)
	turnedTool, err := Transform(t.Context(), turnSource, turn)
	require.NoError(t, err)

	for _, tc := range []struct {
		name string
		rec  crossCylinderBoreRecord
	}{
		{"tool centre one ulp off the Y mirror", func() crossCylinderBoreRecord {
			rec := *exact
			rec.tool = offsetTool
			return rec
		}()},
		{"target centre one ulp off the Y mirror", func() crossCylinderBoreRecord {
			rec := *exact
			rec.target = offsetTarget
			return rec
		}()},
		{"tool turned by a rounded full turn", func() crossCylinderBoreRecord {
			rec := *exact
			rec.tool = turnedTool
			return rec
		}()},
		{"bore radius one ulp off its source", func() crossCylinderBoreRecord {
			rec := *exact
			rec.bore = math.Nextafter(rec.bore, 2)
			return rec
		}()},
	} {
		t.Run(tc.name, func(t *testing.T) {
			_, err := proveCrossBoreCells(t.Context(), &tc.rec, d, mesh)
			require.ErrorIs(t, err, ErrUnsupported)
			require.ErrorContains(t, err, "not the exact mirror-symmetric pose")
		})
	}
}

// TestCrossBoreMirroredRouteMatchesDirect proves every cell of the public
// five-by-one record directly, with no mirror map, and compares that route
// with the mirrored one on the same held mesh: every cell's verdict, the
// readings that must be bit-identical, and the readings that may only
// tighten. It then moves one held vertex of a mapped cell and requires the
// mirrored route to charge it.
func TestCrossBoreMirroredRouteMatchesDirect(t *testing.T) {
	const n = 2 * crossBoreHalfStations
	const setback = 0.5
	rec := crossBoreExactRecord(t, 5, 1, 10, 5)
	mesh, err := buildCrossBoreChamferMesh(t.Context(), rec, setback)
	require.NoError(t, err)
	gap := crossBoreCellGap()

	direct := make([]crossBoreCellExact, n)
	_, err = orderedwork.Each(n, orderedwork.EachWorkers(n, crossBoreCellParallelMin), crossBoreCellGrain, nil,
		func(j int) error {
			var err error
			direct[j], err = crossBoreCellExactProof(rec, setback, j, gap)
			return err
		})
	require.NoError(t, err, "every cell passes directly")
	var want crossBoreCellResult
	var directMax crossBoreCellResult
	for j := range n {
		k, flipY, flipZ := crossBoreCellMirror(j)
		held := crossBoreMeshHeld(mesh, j)
		mapped := crossBoreCellSampleError(&held, rec.centerZ, &direct[k].coords, flipY, flipZ)
		own := crossBoreCellSampleError(&held, rec.centerZ, &direct[j].coords, false, false)
		require.False(t, proofbound.IsNonFinite(mapped), "cell %d", j)
		want.sampleError = max(want.sampleError, mapped)
		directMax.sampleError = max(directMax.sampleError, own)
		directMax.width = max(directMax.width, direct[j].width)
		directMax.speed = max(directMax.speed, direct[j].speed)
		directMax.bandLength = max(directMax.bandLength, direct[j].bandLength)
		if j <= n/4 {
			want.width = max(want.width, direct[j].width)
			want.speed = max(want.speed, direct[j].speed)
			want.bandLength = max(want.bandLength, direct[j].bandLength)
		}
	}
	got, err := proveCrossBoreCells(t.Context(), rec, setback, mesh)
	require.NoError(t, err, "every base cell passes")
	requireCrossBoreBitsEqual(t, want, got)
	require.Equal(t, math.Float64bits(directMax.speed), math.Float64bits(got.speed))
	require.Equal(t, math.Float64bits(directMax.bandLength), math.Float64bits(got.bandLength))
	require.LessOrEqual(t, got.width, directMax.width)
	require.LessOrEqual(t, got.sampleError, directMax.sampleError)
	require.Less(t, got.sampleError, 0.01, "the held vertices lie within the cell enclosures' scale")

	// A held mesh that is not symmetric is still caught: cell 3000 maps onto
	// base 952 with Y and Z−c negated, and the route compares cell 3000's own
	// outer foot with that mirrored enclosure.
	moved := append([]r3.Vec(nil), mesh.verts...)
	moved[mesh.cellHeld[3000][0]].Y += 0.05
	charged, err := crossBoreMirroredCellMaxima(t.Context(), 4,
		func(k int) (crossBoreCellExact, error) { return direct[k], nil },
		func(j int, base *crossBoreCellExact, flipY, flipZ bool) float64 {
			cell := mesh.cellHeld[j]
			held := [3]r3.Vec{moved[cell[0]], moved[cell[1]], moved[cell[2]]}
			return crossBoreCellSampleError(&held, rec.centerZ, &base.coords, flipY, flipZ)
		})
	require.NoError(t, err)
	require.Greater(t, charged.sampleError, proofbound.Radius3D(0.04))
}

func TestCrossBoreMirroredRouteRefusesLikeDirect(t *testing.T) {
	const n = 2 * crossBoreHalfStations
	gap := crossBoreCellGap()
	for _, tc := range []struct {
		name string
		rec  crossCylinderBoreRecord
	}{
		{"loses clearance or folds", crossCylinderBoreRecord{radius: 5, bore: 4.9, height: 10, centerZ: 5}},
		{"no real X", crossCylinderBoreRecord{radius: 5, bore: 5, height: 10, centerZ: 5}},
	} {
		t.Run(tc.name, func(t *testing.T) {
			var firstDirect error
			for j := range n {
				_, err := crossBoreCellExactProof(&tc.rec, 0.5, j, gap)
				if err != nil {
					firstDirect = err
					break
				}
			}
			require.Error(t, firstDirect)
			_, err := crossBoreMirroredCellMaxima(t.Context(), 4,
				func(k int) (crossBoreCellExact, error) { return crossBoreCellExactProof(&tc.rec, 0.5, k, gap) },
				func(int, *crossBoreCellExact, bool, bool) float64 { return 0 })
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
	got := crossBoreCellSampleError(&held, 1, &coords, false, false)
	require.GreaterOrEqual(t, got, 1e-20)
	require.Less(t, got, 1e-19)
}
