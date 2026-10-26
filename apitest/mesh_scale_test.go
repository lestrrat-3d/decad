package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func scalarOf(f float64) units.Value { return units.Scalar(f) }

func unitCubeMesh(t *testing.T) *decad.MeshBody {
	t.Helper()
	v := []r3.Vec{{}, {X: 1}, {X: 1, Y: 1}, {Y: 1}, {Z: 1}, {X: 1, Z: 1}, {X: 1, Y: 1, Z: 1}, {Y: 1, Z: 1}}
	tri := [][3]int{
		{0, 2, 1}, {0, 3, 2}, {4, 5, 6}, {4, 6, 7}, {0, 1, 5}, {0, 5, 4},
		{1, 2, 6}, {1, 6, 5}, {2, 3, 7}, {2, 7, 6}, {3, 0, 4}, {3, 4, 7},
	}
	m, err := decad.NewMeshBody(t.Context(), v, tri)
	require.NoError(t, err)
	return m
}

func TestScaleMeshStretchesEachAxis(t *testing.T) {
	t.Parallel()
	got, err := decad.ScaleMesh(t.Context(), unitCubeMesh(t), r3.Vec{}, scalarOf(2), scalarOf(3), scalarOf(5))
	require.NoError(t, err)
	require.True(t, got.Report().Closed)
	volume, err := got.Volume()
	require.NoError(t, err)
	require.InDelta(t, 30, volume.Value.Base(), 1e-9)
	box := got.Bounds()
	require.True(t, box.Max.Equal(r3.NewVec(2, 3, 5), 1e-12))
	area, err := got.Area()
	require.NoError(t, err)
	require.InDelta(t, 2*(2*3+3*5+2*5), area.Value.Base(), 1e-9)
}

func TestScaleMeshAboutAnAnchorKeepsTheAnchorFixed(t *testing.T) {
	t.Parallel()
	anchor := r3.Vec{X: 1, Y: 1, Z: 1}
	got, err := decad.ScaleMesh(t.Context(), unitCubeMesh(t), anchor, scalarOf(2), scalarOf(2), scalarOf(2))
	require.NoError(t, err)
	box := got.Bounds()
	require.True(t, box.Min.Equal(r3.NewVec(-1, -1, -1), 1e-12))
	require.True(t, box.Max.Equal(anchor, 1e-12))
}

func TestScaleMeshMirrorsWithANegativeFactor(t *testing.T) {
	t.Parallel()
	for _, f := range [][3]float64{{-1, 1, 1}, {-1, -1, 1}, {-2, -3, -4}} {
		got, err := decad.ScaleMesh(t.Context(), unitCubeMesh(t), r3.Vec{}, scalarOf(f[0]), scalarOf(f[1]), scalarOf(f[2]))
		require.NoError(t, err)
		require.True(t, got.Report().Closed)
		volume, err := got.Volume()
		require.NoError(t, err, "a mirrored mesh stays wound outward")
		require.InDelta(t, math.Abs(f[0]*f[1]*f[2]), volume.Value.Base(), 1e-9)
	}
	got, err := decad.ScaleMesh(t.Context(), unitCubeMesh(t), r3.Vec{}, scalarOf(-1), scalarOf(1), scalarOf(1))
	require.NoError(t, err)
	require.True(t, got.Bounds().Min.Equal(r3.NewVec(-1, 0, 0), 1e-12))
}

func TestScaleMeshDoesNotChangeItsSource(t *testing.T) {
	t.Parallel()
	src := unitCubeMesh(t)
	_, err := decad.ScaleMesh(t.Context(), src, r3.Vec{}, scalarOf(9), scalarOf(9), scalarOf(9))
	require.NoError(t, err)
	require.True(t, src.Bounds().Max.Equal(r3.NewVec(1, 1, 1), 0))
}

func TestScaleMeshRefusals(t *testing.T) {
	t.Parallel()
	m := unitCubeMesh(t)
	one := scalarOf(1)
	_, err := decad.ScaleMesh(t.Context(), m, r3.Vec{}, scalarOf(1), scalarOf(0), one)
	require.ErrorIs(t, err, decad.ErrDegenerate)
	_, err = decad.ScaleMesh(t.Context(), m, r3.Vec{}, one, one, units.Millimeters(2))
	require.ErrorIs(t, err, decad.ErrUnitKind)
	_, err = decad.ScaleMesh(t.Context(), m, r3.Vec{}, scalarOf(math.NaN()), one, one)
	require.ErrorIs(t, err, decad.ErrNotFinite)
	_, err = decad.ScaleMesh(t.Context(), m, r3.Vec{X: math.Inf(1)}, one, one, one)
	require.ErrorIs(t, err, decad.ErrNotFinite)
	_, err = decad.ScaleMesh(t.Context(), nil, r3.Vec{}, one, one, one)
	require.ErrorIs(t, err, decad.ErrDegenerate)
	_, err = decad.ScaleMesh(nil, m, r3.Vec{}, one, one, one) //nolint:staticcheck // a nil context is the case under test
	require.ErrorIs(t, err, decad.ErrDegenerate)
	_, err = decad.ScaleMesh(t.Context(), m, r3.Vec{}, scalarOf(math.MaxFloat64), scalarOf(math.MaxFloat64), scalarOf(math.MaxFloat64))
	require.ErrorIs(t, err, decad.ErrUnsupported, "an area past the largest float has no bound")
}

func TestScaleMeshOverflowingVerticesAreNotFinite(t *testing.T) {
	t.Parallel()
	big, err := decad.ScaleMesh(t.Context(), unitCubeMesh(t), r3.Vec{}, scalarOf(1e200), scalarOf(1e200), scalarOf(1e200))
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.Nil(t, big)
	m := unitCubeMesh(t)
	_, err = decad.ScaleMesh(t.Context(), m, r3.Vec{X: 1e308}, scalarOf(1e10), scalarOf(1), scalarOf(1))
	require.ErrorIs(t, err, decad.ErrNotFinite)
}

func TestScaleBodyMeshStretchesASolid(t *testing.T) {
	t.Parallel()
	block := boxBody(t, decad.New(), 0, 0, 10, 20, 30)
	m, err := decad.ScaleBodyMesh(t.Context(), block, r3.Vec{}, scalarOf(2), scalarOf(1), scalarOf(0.5), units.Millimeters(0.1))
	require.NoError(t, err)
	require.True(t, m.Report().Closed)
	volume, err := m.Volume()
	require.NoError(t, err)
	require.InDelta(t, 10*20*30*1.0, volume.Value.Base(), 1e-9, "a block of flat faces keeps its volume exactly, scaled by |x·y·z| = 1")
	box := m.Bounds()
	require.True(t, box.Max.Equal(r3.NewVec(20, 20, 15), 1e-12))
	// The source is untouched.
	original, err := block.Volume()
	require.NoError(t, err)
	require.InDelta(t, 6000, original.Value.Base(), 1e-9)
}

func TestScaleBodyMeshStretchesARodIntoAnEllipticalOne(t *testing.T) {
	t.Parallel()
	rod := circleBodyAtZ(t, decad.New(), 0, 5, 0, 20)
	m, err := decad.ScaleBodyMesh(t.Context(), rod, r3.Vec{}, scalarOf(2), scalarOf(1), scalarOf(1), units.Millimeters(0.01))
	require.NoError(t, err)
	volume, err := m.Volume()
	require.NoError(t, err)
	// An ellipse of semi-axes 10 and 5 over 20 mm, to the tessellation's own error.
	want := math.Pi * 10 * 5 * 20
	require.InDelta(t, want, volume.Value.Base(), 0.005*want)
	box := m.Bounds()
	// The polygon of the rod's rim lies within the chord tolerance, 0.01 mm, of the circle.
	require.InDelta(t, 10, box.Max.X, 2*0.01)
	require.InDelta(t, 5, box.Max.Y, 0.01)
}

func TestScaleBodyMeshRefusals(t *testing.T) {
	t.Parallel()
	block := boxBody(t, decad.New(), 0, 0, 10, 20, 30)
	one := scalarOf(1)
	_, err := decad.ScaleBodyMesh(t.Context(), nil, r3.Vec{}, one, one, one, units.Millimeters(0.1))
	require.ErrorIs(t, err, decad.ErrDegenerate)
	_, err = decad.ScaleBodyMesh(t.Context(), block, r3.Vec{}, one, scalarOf(0), one, units.Millimeters(0.1))
	require.ErrorIs(t, err, decad.ErrDegenerate)
	_, err = decad.ScaleBodyMesh(t.Context(), block, r3.Vec{}, one, one, one, units.Degrees(1))
	require.ErrorIs(t, err, decad.ErrUnitKind)
}
