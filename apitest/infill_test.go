package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is agent-docs/lattice-design.md §5's public-surface tests for Infill.

// gyroidAt is the gyroid field of lattice-design.md G1 at p for a cell of the given length.
func gyroidAt(p r3.Vec, cell float64) float64 {
	k := 2 * math.Pi / cell
	return math.Sin(k*p.X)*math.Cos(k*p.Y) + math.Sin(k*p.Y)*math.Cos(k*p.Z) + math.Sin(k*p.Z)*math.Cos(k*p.X)
}

func TestInfillFillsABlockWithTheGyroidOnlyInsideIt(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	block := boxBody(t, doc, 5, 5, 15, 15, 12) // x, y 5 to 15, z 0 to 12
	const cell, level = 10.0, 0.8
	out, err := decad.Infill(t.Context(), block, decad.InfillSpec{
		Kind: decad.GyroidInfill, Cell: units.Millimeters(cell), Level: units.Scalar(level), Resolution: 8})
	require.NoError(t, err)
	require.Equal(t, []*decad.Body{out}, doc.Bodies(), "the block and the lattice are consumed")
	box, err := out.Bounds()
	require.NoError(t, err)
	require.GreaterOrEqual(t, box.Min.X, 5-1e-6)
	require.LessOrEqual(t, box.Max.X, 15+1e-6)
	require.LessOrEqual(t, box.Max.Z, 12+1e-6)
	// The volume is the field's solid fraction, sampled on a grid of this test's own, over the block.
	const n = 40
	in, total := 0, 0
	for i := range n {
		for j := range n {
			for k := range n {
				p := r3.NewVec(5+10*(float64(i)+0.5)/n, 5+10*(float64(j)+0.5)/n, 12*(float64(k)+0.5)/n)
				total++
				if math.Abs(gyroidAt(p, cell)) <= level {
					in++
				}
			}
		}
	}
	want := 1200 * float64(in) / float64(total)
	volume, err := out.Volume()
	require.NoError(t, err)
	// The lattice mesh at 8 samples to a cell overshoots the field's fraction (lattice-design.md §2), so the
	// volume agrees to a tenth; the points below carry the exact claim.
	require.InEpsilon(t, want, volume.Value.Base(), 0.1)
	// Points well inside the wall are material, and points well clear of it are not.
	checked, wrong, undecided := 0, 0, 0
	for i := range 12 {
		for j := range 12 {
			for k := range 12 {
				p := r3.NewVec(5.4+9.2*float64(i)/11, 5.4+9.2*float64(j)/11, 0.4+11.2*float64(k)/11)
				g := math.Abs(gyroidAt(p, cell))
				var want decad.PointLocation
				switch {
				case g < 0.5*level:
					want = decad.PointInside
				case g > 1.5*level:
					want = decad.PointOutside
				default:
					continue
				}
				got, err := out.LocatePoint(t.Context(), p, units.Millimeters(0.01))
				require.NoError(t, err)
				switch got {
				case want:
				case decad.PointUndecided:
					undecided++
				default:
					wrong++
				}
				checked++
			}
		}
	}
	require.Greater(t, checked, 500)
	// The lattice mesh at 8 samples to a cell is the field to a grid's accuracy, and misplaces a handful of
	// points at its saddles (4 of 784 in the unfilled lattice), so a decided answer agrees 99% and not all.
	require.LessOrEqual(t, wrong, checked/100, "%d of %d points disagree with the field", wrong, checked)
	// A point near the result's boundary may read undecided, and how near depends on the platform: where the
	// compiler fuses multiply-adds (arm64, GOAMD64=v3) rounding the result needs a held-embedding repair that
	// widens the proven bound, and 248 of 784 points sit within the widened reach. Most still decide.
	require.LessOrEqual(t, undecided, checked/2, "%d of %d points are undecided", undecided, checked)
}

func TestInfillRefusals(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	block := boxBody(t, doc, 5, 5, 15, 15, 12)
	spec := decad.InfillSpec{Kind: decad.GyroidInfill, Cell: units.Millimeters(10), Level: units.Scalar(0.8), Resolution: 8}
	bad := spec
	bad.Kind = decad.LatticeKind(9)
	_, err := decad.Infill(t.Context(), block, bad)
	require.ErrorIs(t, err, decad.ErrDegenerate)
	bad = spec
	bad.Cell = units.Degrees(5)
	_, err = decad.Infill(t.Context(), block, bad)
	require.ErrorIs(t, err, decad.ErrUnitKind)
	bad = spec
	bad.Cell = units.Millimeters(0)
	_, err = decad.Infill(t.Context(), block, bad)
	require.ErrorIs(t, err, decad.ErrDegenerate)
	bad = spec
	bad.Level = units.Scalar(2)
	_, err = decad.Infill(t.Context(), block, bad)
	require.ErrorIs(t, err, decad.ErrDegenerate, "a level past the gyroid's range")
	_, err = decad.Infill(nil, block, spec) //nolint:staticcheck // a nil context is the case under test
	require.ErrorIs(t, err, decad.ErrDegenerate)
	_, err = decad.Infill(t.Context(), nil, spec)
	require.ErrorIs(t, err, decad.ErrDegenerate)
	require.Len(t, doc.Bodies(), 1, "every refusal leaves the block alone")
	require.Equal(t, "SchwarzPInfill", decad.SchwarzPInfill.String())
}

func TestInfillTakesEachLatticeKind(t *testing.T) {
	t.Parallel()
	for kind, level := range map[decad.LatticeKind]float64{decad.SchwarzPInfill: 1, decad.DiamondInfill: 0.5} {
		doc := decad.New()
		block := boxBody(t, doc, 5, 5, 15, 15, 12)
		out, err := decad.Infill(t.Context(), block, decad.InfillSpec{
			Kind: kind, Cell: units.Millimeters(10), Level: units.Scalar(level), Resolution: 8})
		require.NoError(t, err, kind.String())
		volume, err := out.Volume()
		require.NoError(t, err)
		require.Greater(t, volume.Value.Base(), 100.0, kind.String())
		require.Less(t, volume.Value.Base(), 1200.0, kind.String())
	}
}
