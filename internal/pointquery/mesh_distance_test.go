package pointquery_test

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/pointquery"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// cubeMesh is the outward-oriented 10 mm cube with its two z faces' triangles
// attributed to bottom and top.
func cubeMesh() (*topo.Mesh, *topo.Face, *topo.Face) {
	bottom, top, wall := &topo.Face{}, &topo.Face{}, &topo.Face{}
	m := &topo.Mesh{
		Vertices: []r3.Vec{
			{X: 0, Y: 0, Z: 0}, {X: 10, Y: 0, Z: 0}, {X: 10, Y: 10, Z: 0}, {X: 0, Y: 10, Z: 0},
			{X: 0, Y: 0, Z: 10}, {X: 10, Y: 0, Z: 10}, {X: 10, Y: 10, Z: 10}, {X: 0, Y: 10, Z: 10},
		},
		Triangles: [][3]int{
			{0, 2, 1}, {0, 3, 2}, {4, 5, 6}, {4, 6, 7},
			{0, 1, 5}, {0, 5, 4}, {1, 2, 6}, {1, 6, 5}, {2, 3, 7}, {2, 7, 6}, {3, 0, 4}, {3, 4, 7},
		},
		Bound: 0.25, VolSymDiff: 0.5, BoundaryOK: true, SymDiffOK: true,
		FaceBound: map[*topo.Face]float64{bottom: 0.125, top: 0.125, wall: 0.125},
	}
	m.Source = []*topo.Face{bottom, bottom, top, top, wall, wall, wall, wall, wall, wall, wall, wall}
	return m, bottom, top
}

func TestBodyDistance(t *testing.T) {
	t.Parallel()
	ctx := t.Context()
	p := r3.Vec{X: 5, Y: 5, Z: 14}

	mesh, _, _ := cubeMesh()
	outside, err := pointquery.BodyDistance(ctx, p, mesh)
	require.NoError(t, err)
	require.InDelta(t, 4.0, outside.Value.Base(), 1e-9, "midpoint of [3.75, 4.25]")
	require.GreaterOrEqual(t, outside.Bound.Base(), 0.25)
	require.Equal(t, measurement.Approximate, outside.Exactness)

	inside, err := pointquery.BodyDistance(ctx, r3.Vec{X: 5, Y: 5, Z: 5}, mesh)
	require.NoError(t, err)
	require.Zero(t, inside.Value.Base())
	require.Zero(t, inside.Bound.Base())
	require.Equal(t, measurement.Exact, inside.Exactness)

	t.Run("no volume proof leaves the lower bound at zero", func(t *testing.T) {
		unproven, _, _ := cubeMesh()
		unproven.SymDiffOK = false
		got, err := pointquery.BodyDistance(ctx, p, unproven)
		require.NoError(t, err)
		require.InDelta(t, 2.125, got.Value.Base(), 1e-9, "midpoint of [0, 4.25]")
		require.GreaterOrEqual(t, got.Bound.Base(), 2.125)
	})
	t.Run("a point nearer than the displacement bound clamps at zero", func(t *testing.T) {
		near, _, _ := cubeMesh()
		got, err := pointquery.BodyDistance(ctx, r3.Vec{X: 5, Y: 5, Z: 10.125}, near)
		require.NoError(t, err)
		require.GreaterOrEqual(t, got.Value.Base()-got.Bound.Base(), -1e-12, "the lower end never goes negative")
	})
	for _, tc := range []struct {
		name string
		edit func(*topo.Mesh)
	}{
		{"unverified boundary", func(m *topo.Mesh) { m.BoundaryOK = false }},
		{"non-finite bound", func(m *topo.Mesh) { m.Bound = math.NaN() }},
		{"infinite bound", func(m *topo.Mesh) { m.Bound = math.Inf(1) }},
		{"negative bound", func(m *topo.Mesh) { m.Bound = -1 }},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			m, _, _ := cubeMesh()
			tc.edit(m)
			_, err := pointquery.BodyDistance(ctx, p, m)
			require.ErrorIs(t, err, decaderr.ErrUnsupported)
		})
	}
}

func TestFaceDistance(t *testing.T) {
	t.Parallel()
	ctx := t.Context()
	p := r3.Vec{X: 5, Y: 5, Z: 14}

	mesh, bottom, top := cubeMesh()
	got, err := pointquery.FaceDistance(ctx, p, mesh, top)
	require.NoError(t, err)
	require.InDelta(t, 4.0, got.Value.Base(), 1e-9)
	require.GreaterOrEqual(t, got.Bound.Base(), 0.125)

	far, err := pointquery.FaceDistance(ctx, p, mesh, bottom)
	require.NoError(t, err)
	require.InDelta(t, 14.0, far.Value.Base(), 1e-9)

	near, err := pointquery.FaceDistance(ctx, r3.Vec{X: 5, Y: 5, Z: 10.0625}, mesh, top)
	require.NoError(t, err)
	require.GreaterOrEqual(t, near.Value.Base()-near.Bound.Base(), -1e-12, "the lower end never goes negative")

	for _, tc := range []struct {
		name string
		edit func(*topo.Mesh, *topo.Face)
		want error
	}{
		{"unverified boundary", func(m *topo.Mesh, _ *topo.Face) { m.BoundaryOK = false }, decaderr.ErrUnsupported},
		{"source and triangles disagree", func(m *topo.Mesh, _ *topo.Face) { m.Source = m.Source[:3] }, decaderr.ErrBooleanFailed},
		{"face has no triangles", func(m *topo.Mesh, f *topo.Face) {
			for i, s := range m.Source {
				if s == f {
					m.Source[i] = nil
				}
			}
		}, decaderr.ErrUnsupported},
		{"face has no displacement bound", func(m *topo.Mesh, f *topo.Face) { delete(m.FaceBound, f) }, decaderr.ErrBooleanFailed},
		{"non-finite displacement bound", func(m *topo.Mesh, f *topo.Face) { m.FaceBound[f] = math.NaN() }, decaderr.ErrUnsupported},
		{"negative displacement bound", func(m *topo.Mesh, f *topo.Face) { m.FaceBound[f] = -1 }, decaderr.ErrUnsupported},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			m, _, face := cubeMesh()
			tc.edit(m, face)
			_, err := pointquery.FaceDistance(ctx, p, m, face)
			require.ErrorIs(t, err, tc.want)
		})
	}
}

func TestRatIntervalMeasurement(t *testing.T) {
	t.Parallel()
	exact, ok := pointquery.RatIntervalMeasurement(big.NewRat(3, 1), big.NewRat(3, 1))
	require.True(t, ok)
	require.Equal(t, 3.0, exact.Value.Base())
	require.Zero(t, exact.Bound.Base())
	require.Equal(t, measurement.Exact, exact.Exactness)

	wide, ok := pointquery.RatIntervalMeasurement(big.NewRat(1, 1), big.NewRat(3, 1))
	require.True(t, ok)
	require.Equal(t, 2.0, wide.Value.Base())
	require.Equal(t, 1.0, wide.Bound.Base())
	require.Equal(t, measurement.Approximate, wide.Exactness)

	huge := new(big.Rat).SetInt(new(big.Int).Lsh(big.NewInt(1), 5000))
	_, ok = pointquery.RatIntervalMeasurement(huge, huge)
	require.False(t, ok)
}
