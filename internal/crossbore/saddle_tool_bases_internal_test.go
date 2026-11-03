package crossbore

import (
	"context"
	"fmt"
	"math"
	"math/big"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/orderedwork"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// oldProveToolBand is ProveToolBand as it was before ProveToolBases split
// out the base cells: one body that proves its own bases.
func oldProveToolBand(ctx context.Context, R, r, d float64, pose Pose,
	held func(j, i int) r3.Vec) (ToolProof, error) {
	const n = ToolStations
	gap := new(big.Rat).Quo(new(big.Rat).Mul(big.NewRat(2, 1), proofbound.PiUpper), big.NewRat(n, 1))
	bases := make([]toolBase, n/4+1)
	workers := orderedwork.EachWorkers(len(bases), CellParallelMin)
	if err := cellEach(ctx, len(bases), workers, func(k int) error {
		b, err := proveToolBase(R, r, d, k, gap)
		bases[k] = b
		return err
	}); err != nil {
		return ToolProof{}, err
	}
	samples := make([]float64, n)
	if err := cellEach(ctx, n, workers, func(j int) error {
		k, flipY, flipZ := cellMirrorOf(j, n)
		samples[j] = stationSampleError(&bases[k].ends, func(i int) r3.Vec { return held(j, i) }, flipY, flipZ, pose)
		return nil
	}); err != nil {
		return ToolProof{}, err
	}
	if err := ctx.Err(); err != nil {
		return ToolProof{}, err
	}
	var p ToolProof
	spread := 0.0
	var areaLo, areaHi [ToolRows]float64
	for _, b := range bases[:n/4] {
		spread = max(spread, b.spread)
		for i := range ToolRows {
			areaLo[i], areaHi[i] = down(areaLo[i]+b.areaLo[i]), up(areaHi[i]+b.areaHi[i])
		}
	}
	for _, e := range samples {
		p.SampleError = max(p.SampleError, e)
	}
	cells := big.NewRat(n*ToolAreaCells, 1)
	stepLo := proofbound.RatFloatDown(new(big.Rat).Quo(new(big.Rat).Mul(big.NewRat(8, 1), proofbound.PiLower), cells))
	stepHi := proofbound.RatFloatUp(new(big.Rat).Quo(new(big.Rat).Mul(big.NewRat(8, 1), proofbound.PiUpper), cells))
	for i := range ToolRows {
		p.RowLo[i], p.RowHi[i] = down(areaLo[i]*stepLo), up(areaHi[i]*stepHi)
		p.AreaLo, p.AreaHi = down(p.AreaLo+p.RowLo[i]), up(p.AreaHi+p.RowHi[i])
	}
	step := proofbound.RatFloatUp(gap)
	p.SampleError = proofbound.Radius3D(p.SampleError)
	p.Chord = proofbound.AbsSumUpper(
		proofbound.Radius3D(proofbound.DivUpper(proofbound.ProductUpper(step, spread), 4)), p.SampleError)
	if proofbound.IsNonFinite(p.Chord) || proofbound.IsNonFinite(p.AreaHi) || !(p.AreaLo >= 0) {
		return ToolProof{}, fmt.Errorf("%w: the saddle chamfer's band has no finite bound", decaderr.ErrUnsupported)
	}
	return p, nil
}

// diffToolProof returns the names of the fields of a and b that differ by
// bits, never the values' graphs.
func diffToolProof(a, b ToolProof) []string {
	var bad []string
	same := func(name string, x, y float64) {
		if math.Float64bits(x) != math.Float64bits(y) {
			bad = append(bad, name)
		}
	}
	same("Chord", a.Chord, b.Chord)
	same("SampleError", a.SampleError, b.SampleError)
	same("AreaLo", a.AreaLo, b.AreaLo)
	same("AreaHi", a.AreaHi, b.AreaHi)
	for i := range ToolRows {
		same(fmt.Sprintf("RowLo[%d]", i), a.RowLo[i], b.RowLo[i])
		same(fmt.Sprintf("RowHi[%d]", i), a.RowHi[i], b.RowHi[i])
	}
	return bad
}

// TestProveToolBasesMatchesOldPath proves that two saddles of equal radii
// reading one ToolBases get the bits of the old per-saddle proofs and of
// ProveToolBand, with held rows moved off the exact rows so SampleError is
// nonzero.
func TestProveToolBasesMatchesOldPath(t *testing.T) {
	t.Parallel()
	type radii struct{ R, r, d float64 }
	cases := []radii{{10, 3, 1.5}}
	rng := rand.New(rand.NewPCG(7, 11))
	for range 2 {
		R := 6 + 8*rng.Float64()
		cases = append(cases, radii{R, R * (0.2 + 0.3*rng.Float64()), 0.5 + 1.5*rng.Float64()})
	}
	poses := []Pose{
		{Axis: [3]int{2, 1, 0}, Sign: [3]float64{-1, -1, 1}, Shift: r3.Vec{X: 40}},
		{Axis: [3]int{2, 1, 0}, Sign: [3]float64{1, 1, 1}, Shift: r3.Vec{X: -40}},
	}
	for ci, c := range cases {
		t.Run(fmt.Sprintf("case%d", ci), func(t *testing.T) {
			t.Parallel()
			tb, err := ProveToolBases(t.Context(), c.R, c.r, c.d)
			require.NoError(t, err)
			for pi, pose := range poses {
				held := func(j, i int) r3.Vec {
					o, b := BandEnds(c.R, c.r, c.d, j%ToolStations)
					v := pose.World(BandRow(o, b, i))
					v.X += 1e-9 * float64((j+i)%5)
					return v
				}
				want, err := oldProveToolBand(t.Context(), c.R, c.r, c.d, pose, held)
				require.NoError(t, err)
				require.Positive(t, want.SampleError, "case %d pose %d", ci, pi)
				got, err := ProveToolBandWith(t.Context(), tb, pose, held)
				require.NoError(t, err)
				require.Empty(t, diffToolProof(want, got), "shared bases, case %d pose %d", ci, pi)
				comb, err := ProveToolBand(t.Context(), c.R, c.r, c.d, pose, held)
				require.NoError(t, err)
				require.Empty(t, diffToolProof(want, comb), "combined, case %d pose %d", ci, pi)
			}
		})
	}
}

// TestProveToolBasesDiffer proves the comparison can fail: bases proved for
// other radii give a different proof.
func TestProveToolBasesDiffer(t *testing.T) {
	t.Parallel()
	const R, r, d = 10.0, 3.0, 1.5
	pose := Pose{Axis: [3]int{2, 1, 0}, Sign: [3]float64{-1, -1, 1}, Shift: r3.Vec{X: 40}}
	held := func(j, i int) r3.Vec {
		o, b := BandEnds(R, r, d, j%ToolStations)
		return pose.World(BandRow(o, b, i))
	}
	want, err := oldProveToolBand(t.Context(), R, r, d, pose, held)
	require.NoError(t, err)
	other, err := ProveToolBases(t.Context(), R, r+0.5, d)
	require.NoError(t, err)
	got, err := ProveToolBandWith(t.Context(), other, pose, held)
	require.NoError(t, err)
	require.NotEmpty(t, diffToolProof(want, got))
}
