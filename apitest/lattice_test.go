package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is agent-docs/lattice-design.md's public-surface tests for
// GyroidLattice, on a box two cells on a side.

// gyroidFraction is the share of one cell where |g| ≤ level, sampled on a grid so fine
// that the estimate stands apart from the lattice's own mesh: the lattice's solid
// fraction, read straight from the field.
func gyroidFraction(level float64) float64 {
	const n = 160
	k := 2 * math.Pi
	inside := 0
	for i := range n {
		for j := range n {
			for l := range n {
				x, y, z := (float64(i)+0.5)/n*k, (float64(j)+0.5)/n*k, (float64(l)+0.5)/n*k
				g := math.Sin(x)*math.Cos(y) + math.Sin(y)*math.Cos(z) + math.Sin(z)*math.Cos(x)
				if math.Abs(g) <= level {
					inside++
				}
			}
		}
	}
	return float64(inside) / (n * n * n)
}

func latticeSpec(level float64, res int) decad.LatticeSpec {
	return decad.LatticeSpec{
		Min: r3.Vec{X: 1, Y: 2, Z: 3}, Max: r3.Vec{X: 21, Y: 22, Z: 23},
		Cell: units.Millimeters(10), Level: units.Scalar(level), Resolution: res,
	}
}

func TestGyroidLatticeIsAClosedSolidOfTheExpectedFraction(t *testing.T) {
	t.Parallel()
	// The wall at level 0.6 is a fifth of a cell thick, so 24 samples to a cell follow it
	// within a percent and 16 follow the heavier wall at 0.8 within two.
	for _, c := range []struct {
		level float64
		res   int
		tol   float64
	}{{0.6, 24, 0.01}, {0.8, 16, 0.02}} {
		level := c.level
		m, err := decad.GyroidLattice(t.Context(), latticeSpec(level, c.res))
		require.NoError(t, err)
		report := m.Report()
		require.True(t, report.Closed, "level %g: every edge has two triangles", level)
		require.False(t, report.Inverted, "wound outward")
		require.Zero(t, report.NonManifoldEdges)
		volume, err := m.Volume()
		require.NoError(t, err)
		want := gyroidFraction(level) * 20 * 20 * 20
		require.InDelta(t, want, volume.Value.Base(), c.tol*want, "level %g", level)
		box := m.Bounds()
		require.True(t, box.Min.Equal(r3.Vec{X: 1, Y: 2, Z: 3}, 1e-9))
		require.True(t, box.Max.Equal(r3.Vec{X: 21, Y: 22, Z: 23}, 1e-9), "the box's faces cap the lattice")
	}
}

func TestGyroidLatticeConvergesAsTheGridRefines(t *testing.T) {
	t.Parallel()
	want := gyroidFraction(0.6) * 20 * 20 * 20
	var errs []float64
	for _, res := range []int{8, 24} {
		m, err := decad.GyroidLattice(t.Context(), latticeSpec(0.6, res))
		require.NoError(t, err)
		v, err := m.Volume()
		require.NoError(t, err)
		errs = append(errs, math.Abs(v.Value.Base()-want))
	}
	require.Less(t, errs[1], errs[0]/3, "a finer grid follows the lattice more closely")
}

func TestGyroidLatticeThickensWithItsLevel(t *testing.T) {
	t.Parallel()
	volumes := make([]float64, 3)
	for i, level := range []float64{0.3, 0.6, 0.9} {
		m, err := decad.GyroidLattice(t.Context(), latticeSpec(level, 12))
		require.NoError(t, err)
		v, err := m.Volume()
		require.NoError(t, err)
		volumes[i] = v.Value.Base()
	}
	require.Less(t, volumes[0], volumes[1])
	require.Less(t, volumes[1], volumes[2])
}

func TestGyroidLatticeRepeatsEveryCell(t *testing.T) {
	t.Parallel()
	// A box of one cell at the lattice's own phase holds half the volume a box of two cells does
	// per cell count, so its volume is an eighth of the two-cell box's, to within the cap's shrink.
	big, err := decad.GyroidLattice(t.Context(), latticeSpec(0.6, 12))
	require.NoError(t, err)
	spec := latticeSpec(0.6, 12)
	spec.Min, spec.Max = r3.Vec{}, r3.Vec{X: 10, Y: 10, Z: 10}
	small, err := decad.GyroidLattice(t.Context(), spec)
	require.NoError(t, err)
	vb, err := big.Volume()
	require.NoError(t, err)
	vs, err := small.Volume()
	require.NoError(t, err)
	require.InDelta(t, vb.Value.Base()/8, vs.Value.Base(), 0.04*vb.Value.Base()/8)
}

func TestGyroidLatticeRefusals(t *testing.T) {
	t.Parallel()
	good := latticeSpec(0.6, 8)
	with := func(edit func(*decad.LatticeSpec)) error {
		s := good
		edit(&s)
		_, err := decad.GyroidLattice(t.Context(), s)
		return err
	}
	require.ErrorIs(t, with(func(s *decad.LatticeSpec) { s.Level = units.Scalar(0) }), decad.ErrDegenerate)
	require.ErrorIs(t, with(func(s *decad.LatticeSpec) { s.Level = units.Scalar(1.5) }), decad.ErrDegenerate)
	require.ErrorIs(t, with(func(s *decad.LatticeSpec) { s.Level = units.Millimeters(1) }), decad.ErrUnitKind)
	require.ErrorIs(t, with(func(s *decad.LatticeSpec) { s.Cell = units.Millimeters(-1) }), decad.ErrNegativeMagnitude)
	require.ErrorIs(t, with(func(s *decad.LatticeSpec) { s.Cell = units.Scalar(10) }), decad.ErrUnitKind)
	require.ErrorIs(t, with(func(s *decad.LatticeSpec) { s.Cell = units.Millimeters(0) }), decad.ErrDegenerate)
	require.ErrorIs(t, with(func(s *decad.LatticeSpec) { s.Resolution = 3 }), decad.ErrDegenerate)
	require.ErrorIs(t, with(func(s *decad.LatticeSpec) { s.Max = s.Min }), decad.ErrDegenerate)
	require.ErrorIs(t, with(func(s *decad.LatticeSpec) { s.Max.X = math.NaN() }), decad.ErrNotFinite)
	require.ErrorIs(t, with(func(s *decad.LatticeSpec) {
		s.Max = r3.Vec{X: 1e4, Y: 1e4, Z: 1e4}
		s.Cell = units.Millimeters(1)
	}), decad.ErrDegenerate, "too many samples")
	_, err := decad.GyroidLattice(nil, good) //nolint:staticcheck // a nil context is the case under test
	require.ErrorIs(t, err, decad.ErrDegenerate)
}
