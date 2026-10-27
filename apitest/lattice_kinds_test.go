package apitest_test

import (
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is agent-docs/lattice-design.md §4's public-surface tests for
// SchwarzPLattice, DiamondLattice, NeoviusLattice and IWPLattice, on the box of lattice_test.go.

// latticeKind is a lattice entry point with the field it cuts, written out here
// from its definition and not taken from the library.
type latticeKind struct {
	name     string
	build    func(context.Context, decad.LatticeSpec) (*decad.MeshBody, error)
	field    func(x, y, z float64) float64
	maxLevel float64
}

var latticeKinds = []latticeKind{
	{
		"Schwarz P", decad.SchwarzPLattice,
		func(x, y, z float64) float64 { return math.Cos(x) + math.Cos(y) + math.Cos(z) },
		3,
	},
	{
		"diamond", decad.DiamondLattice,
		func(x, y, z float64) float64 {
			return math.Sin(x)*math.Sin(y)*math.Sin(z) + math.Sin(x)*math.Cos(y)*math.Cos(z) +
				math.Cos(x)*math.Sin(y)*math.Cos(z) + math.Cos(x)*math.Cos(y)*math.Sin(z)
		},
		math.Sqrt2,
	},
	{
		"Neovius", decad.NeoviusLattice,
		func(x, y, z float64) float64 {
			return 3*(math.Cos(x)+math.Cos(y)+math.Cos(z)) + 4*math.Cos(x)*math.Cos(y)*math.Cos(z)
		},
		13,
	},
	{
		"IWP", decad.IWPLattice,
		func(x, y, z float64) float64 {
			return 2*(math.Cos(x)*math.Cos(y)+math.Cos(y)*math.Cos(z)+math.Cos(z)*math.Cos(x)) -
				(math.Cos(2*x) + math.Cos(2*y) + math.Cos(2*z))
		},
		5,
	},
	{
		"Fischer-Koch S", decad.FischerKochSLattice,
		func(x, y, z float64) float64 {
			return math.Cos(2*x)*math.Sin(y)*math.Cos(z) + math.Cos(x)*math.Cos(2*y)*math.Sin(z) +
				math.Sin(x)*math.Cos(y)*math.Cos(2*z)
		},
		math.Sqrt2,
	},
	{
		"Lidinoid", decad.LidinoidLattice,
		func(x, y, z float64) float64 {
			return 0.5*(math.Sin(2*x)*math.Cos(y)*math.Sin(z)+math.Sin(2*y)*math.Cos(z)*math.Sin(x)+
				math.Sin(2*z)*math.Cos(x)*math.Sin(y)) -
				0.5*(math.Cos(2*x)*math.Cos(2*y)+math.Cos(2*y)*math.Cos(2*z)+math.Cos(2*z)*math.Cos(2*x)) + 0.15
		},
		1.35,
	},
}

// fieldFraction is the share of one cell where |g| ≤ level, sampled on a grid so fine
// that the estimate stands apart from the lattice's own mesh.
func fieldFraction(field func(x, y, z float64) float64, level float64) float64 {
	const n = 160
	k := 2 * math.Pi
	inside := 0
	for i := range n {
		for j := range n {
			for l := range n {
				if math.Abs(field((float64(i)+0.5)/n*k, (float64(j)+0.5)/n*k, (float64(l)+0.5)/n*k)) <= level {
					inside++
				}
			}
		}
	}
	return float64(inside) / (n * n * n)
}

func TestLatticeKindsAreClosedSolidsOfTheExpectedFraction(t *testing.T) {
	t.Parallel()
	for _, kind := range latticeKinds {
		t.Run(kind.name, func(t *testing.T) {
			t.Parallel()
			// A level a third of the field's range, at 24 samples to a cell.
			level := kind.maxLevel / 3
			m, err := kind.build(t.Context(), latticeSpec(level, 24))
			require.NoError(t, err)
			report := m.Report()
			require.True(t, report.Closed, "every edge has two triangles")
			require.False(t, report.Inverted, "wound outward")
			require.Zero(t, report.NonManifoldEdges)
			volume, err := m.Volume()
			require.NoError(t, err)
			want := fieldFraction(kind.field, level) * 20 * 20 * 20
			require.InDelta(t, want, volume.Value.Base(), 0.02*want)
			box := m.Bounds()
			require.True(t, box.Min.Equal(r3.Vec{X: 1, Y: 2, Z: 3}, 1e-9))
			require.True(t, box.Max.Equal(r3.Vec{X: 21, Y: 22, Z: 23}, 1e-9), "the box's faces cap the lattice")
		})
	}
}

// TestNeoviusAndIWPFollowTheirOwnFieldAtAThinWall reads the Neovius and IWP kinds at a thin wall, where the solid
// fraction depends on every term of the field, so a wrong coefficient moves it by several per cent. A thin wall needs
// 48 samples to a cell to follow the field within a per cent.
func TestNeoviusAndIWPFollowTheirOwnFieldAtAThinWall(t *testing.T) {
	t.Parallel()
	for _, kind := range latticeKinds {
		if kind.name != "Neovius" && kind.name != "IWP" && kind.name != "Fischer-Koch S" && kind.name != "Lidinoid" {
			continue
		}
		t.Run(kind.name, func(t *testing.T) {
			t.Parallel()
			level := 0.15 * kind.maxLevel
			// One cell of the lattice is enough: it repeats, so its solid fraction is the box's.
			spec := decad.LatticeSpec{
				Min: r3.Vec{}, Max: r3.Vec{X: 10, Y: 10, Z: 10},
				Cell: units.Millimeters(10), Level: units.Scalar(level), Resolution: 48,
			}
			m, err := kind.build(t.Context(), spec)
			require.NoError(t, err)
			volume, err := m.Volume()
			require.NoError(t, err)
			want := fieldFraction(kind.field, level) * 10 * 10 * 10
			require.InDelta(t, want, volume.Value.Base(), 0.025*want)
		})
	}
}

func TestLatticeKindsConvergeAndThicken(t *testing.T) {
	t.Parallel()
	for _, kind := range latticeKinds {
		t.Run(kind.name, func(t *testing.T) {
			t.Parallel()
			level := kind.maxLevel / 3
			want := fieldFraction(kind.field, level) * 20 * 20 * 20
			var errs []float64
			for _, res := range []int{8, 24} {
				m, err := kind.build(t.Context(), latticeSpec(level, res))
				require.NoError(t, err)
				v, err := m.Volume()
				require.NoError(t, err)
				errs = append(errs, math.Abs(v.Value.Base()-want))
			}
			require.Less(t, errs[1], errs[0]/2, "a finer grid follows the lattice more closely")
			volumes := make([]float64, 3)
			for i, share := range []float64{0.15, 0.3, 0.5} {
				m, err := kind.build(t.Context(), latticeSpec(share*kind.maxLevel, 12))
				require.NoError(t, err)
				v, err := m.Volume()
				require.NoError(t, err)
				volumes[i] = v.Value.Base()
			}
			require.Less(t, volumes[0], volumes[1])
			require.Less(t, volumes[1], volumes[2])
		})
	}
}

func TestLatticeKindsDifferFromTheGyroidAndEachOther(t *testing.T) {
	t.Parallel()
	// The same level and box give different surfaces: the kinds' volumes at a level of 0.6
	// are 20³ times the solid fraction of each field, which differ well beyond the grid's error.
	volumes := map[string]float64{}
	for name, build := range map[string]func(context.Context, decad.LatticeSpec) (*decad.MeshBody, error){
		"gyroid": decad.GyroidLattice, "schwarz": decad.SchwarzPLattice, "diamond": decad.DiamondLattice,
		"neovius": decad.NeoviusLattice, "iwp": decad.IWPLattice,
		"fischerkoch": decad.FischerKochSLattice, "lidinoid": decad.LidinoidLattice,
	} {
		m, err := build(t.Context(), latticeSpec(0.6, 12))
		require.NoError(t, err)
		v, err := m.Volume()
		require.NoError(t, err)
		volumes[name] = v.Value.Base()
	}
	require.Greater(t, math.Abs(volumes["gyroid"]-volumes["schwarz"]), 0.05*volumes["gyroid"])
	require.Greater(t, math.Abs(volumes["gyroid"]-volumes["diamond"]), 0.05*volumes["gyroid"])
	require.Greater(t, math.Abs(volumes["schwarz"]-volumes["diamond"]), 0.05*volumes["schwarz"])
	require.Greater(t, math.Abs(volumes["neovius"]-volumes["schwarz"]), 0.05*volumes["neovius"])
	require.Greater(t, math.Abs(volumes["iwp"]-volumes["gyroid"]), 0.05*volumes["iwp"])
	require.Greater(t, math.Abs(volumes["fischerkoch"]-volumes["gyroid"]), 0.05*volumes["fischerkoch"])
	require.Greater(t, math.Abs(volumes["lidinoid"]-volumes["diamond"]), 0.05*volumes["lidinoid"])
}

func TestLatticeKindsRefusals(t *testing.T) {
	t.Parallel()
	for _, kind := range latticeKinds {
		t.Run(kind.name, func(t *testing.T) {
			t.Parallel()
			good := latticeSpec(kind.maxLevel/3, 8)
			with := func(edit func(*decad.LatticeSpec)) error {
				s := good
				edit(&s)
				_, err := kind.build(t.Context(), s)
				return err
			}
			// The largest level of the field and zero are not walls.
			require.ErrorIs(t, with(func(s *decad.LatticeSpec) { s.Level = units.Scalar(0) }), decad.ErrDegenerate)
			require.ErrorIs(t, with(func(s *decad.LatticeSpec) { s.Level = units.Scalar(kind.maxLevel) }), decad.ErrDegenerate)
			require.ErrorIs(t, with(func(s *decad.LatticeSpec) { s.Level = units.Scalar(kind.maxLevel + 0.1) }), decad.ErrDegenerate)
			require.NoError(t, with(func(s *decad.LatticeSpec) { s.Level = units.Scalar(kind.maxLevel * 0.95) }))
			require.ErrorIs(t, with(func(s *decad.LatticeSpec) { s.Level = units.Millimeters(1) }), decad.ErrUnitKind)
			require.ErrorIs(t, with(func(s *decad.LatticeSpec) { s.Cell = units.Millimeters(-1) }), decad.ErrNegativeMagnitude)
			require.ErrorIs(t, with(func(s *decad.LatticeSpec) { s.Resolution = 3 }), decad.ErrDegenerate)
			require.ErrorIs(t, with(func(s *decad.LatticeSpec) { s.Max = s.Min }), decad.ErrDegenerate)
			_, err := kind.build(nil, good) // a nil context is the case under test
			require.ErrorIs(t, err, decad.ErrDegenerate)
		})
	}
}
