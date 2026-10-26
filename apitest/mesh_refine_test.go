package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is agent-docs/mesh-import-design.md §10's public-surface tests for
// RefineMesh, on the unit cube of mesh_scale_test.go.

func longestEdge(m *decad.MeshBody) float64 {
	v := m.Vertices()
	longest := 0.0
	for _, t := range m.Triangles() {
		for e := range 3 {
			longest = math.Max(longest, v[t[(e+1)%3]].Sub(v[t[e]]).Len())
		}
	}
	return longest
}

func TestRefineMeshShortensEdgesAndKeepsTheSolid(t *testing.T) {
	t.Parallel()
	cube := unitCubeMesh(t)
	require.Greater(t, longestEdge(cube), 1.4, "the cube's face diagonals are √2")
	for _, limit := range []float64{1.2, 0.5, 0.3} {
		got, err := decad.RefineMesh(t.Context(), cube, units.Millimeters(limit))
		require.NoError(t, err)
		require.LessOrEqual(t, longestEdge(got), limit, "no edge longer than %g", limit)
		report := got.Report()
		require.True(t, report.Closed, "a closed mesh stays closed at %g", limit)
		require.False(t, report.Inverted)
		require.Zero(t, report.DegenerateTriangles)
		require.Greater(t, report.Triangles, 12)
		volume, err := got.Volume()
		require.NoError(t, err)
		require.InDelta(t, 1, volume.Value.Base(), 1e-12, "the volume is the cube's")
		area, err := got.Area()
		require.NoError(t, err)
		require.InDelta(t, 6, area.Value.Base(), 1e-9, "the area is the cube's")
		box := got.Bounds()
		require.True(t, box.Min.Equal(r3.Vec{}, 0))
		require.True(t, box.Max.Equal(r3.NewVec(1, 1, 1), 0))
	}
}

func TestRefineMeshKeepsEveryOriginalCorner(t *testing.T) {
	t.Parallel()
	cube := unitCubeMesh(t)
	got, err := decad.RefineMesh(t.Context(), cube, units.Millimeters(0.4))
	require.NoError(t, err)
	have := map[r3.Vec]bool{}
	for _, v := range got.Vertices() {
		have[v] = true
	}
	for _, v := range cube.Vertices() {
		require.True(t, have[v], "corner %v is a vertex of the refined mesh", v)
	}
}

func TestRefineMeshWithNoLongEdgeIsACopy(t *testing.T) {
	t.Parallel()
	cube := unitCubeMesh(t)
	got, err := decad.RefineMesh(t.Context(), cube, units.Millimeters(5))
	require.NoError(t, err)
	require.Equal(t, cube.Report().Triangles, got.Report().Triangles)
	require.Equal(t, cube.Vertices(), got.Vertices())
	require.False(t, got == cube)
}

func TestRefineMeshRefusals(t *testing.T) {
	t.Parallel()
	cube := unitCubeMesh(t)
	_, err := decad.RefineMesh(t.Context(), cube, units.Millimeters(0))
	require.ErrorIs(t, err, decad.ErrDegenerate)
	_, err = decad.RefineMesh(t.Context(), cube, units.Millimeters(-1))
	require.ErrorIs(t, err, decad.ErrNegativeMagnitude)
	_, err = decad.RefineMesh(t.Context(), cube, units.Degrees(1))
	require.ErrorIs(t, err, decad.ErrUnitKind)
	_, err = decad.RefineMesh(t.Context(), nil, units.Millimeters(1))
	require.ErrorIs(t, err, decad.ErrDegenerate)
	_, err = decad.RefineMesh(nil, cube, units.Millimeters(1)) //nolint:staticcheck // a nil context is the case under test
	require.ErrorIs(t, err, decad.ErrDegenerate)
	// A limit that asks for more than five million triangles.
	_, err = decad.RefineMesh(t.Context(), cube, units.Millimeters(0.001))
	require.ErrorIs(t, err, decad.ErrUnsupported)
}
