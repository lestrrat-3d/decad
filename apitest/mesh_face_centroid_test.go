package apitest_test

import (
	"sort"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// This file is agent-docs/mesh-solid-design.md §7's public-surface tests: the area centroid of each planar face of a
// mesh solid, read exactly from its vertices.

func TestMeshSolidFaceCentroidsAreTheFacesAreaCentroids(t *testing.T) {
	t.Parallel()
	// A triangular prism: legs 10 and 6 along x and y, 4 tall. The slanted face is a 4 x |(-10, 6)| rectangle, so its
	// centroid is the midpoint of the hypotenuse at mid height, and the triangles' centroid is a third of the way in.
	verts := []r3.Vec{
		r3.NewVec(0, 0, 0), r3.NewVec(10, 0, 0), r3.NewVec(0, 6, 0),
		r3.NewVec(0, 0, 4), r3.NewVec(10, 0, 4), r3.NewVec(0, 6, 4),
	}
	tris := [][3]int{{0, 2, 1}, {3, 4, 5}, {0, 1, 4}, {0, 4, 3}, {0, 3, 5}, {0, 5, 2}, {1, 2, 5}, {1, 5, 4}}
	m, err := decad.NewMeshBody(t.Context(), verts, tris)
	require.NoError(t, err)
	body, err := decad.SolidFromMesh(t.Context(), decad.New(), m)
	require.NoError(t, err)
	var got []r3.Vec
	for _, f := range body.Faces() {
		c, err := f.Centroid()
		require.NoError(t, err)
		require.LessOrEqual(t, c.Bound.Base(), 1e-12)
		got = append(got, c.Value)
	}
	sort.Slice(got, func(i, j int) bool {
		a, b := got[i], got[j]
		if a.X != b.X {
			return a.X < b.X
		}
		if a.Y != b.Y {
			return a.Y < b.Y
		}
		return a.Z < b.Z
	})
	want := []r3.Vec{
		r3.NewVec(0, 3, 2), r3.NewVec(10.0/3, 2, 0), r3.NewVec(10.0/3, 2, 4), r3.NewVec(5, 0, 2), r3.NewVec(5, 3, 2),
	}
	require.Len(t, got, len(want))
	for i := range want {
		require.InDelta(t, want[i].X, got[i].X, 1e-12)
		require.InDelta(t, want[i].Y, got[i].Y, 1e-12)
		require.InDelta(t, want[i].Z, got[i].Z, 1e-12)
	}
}

func TestMeshSolidFaceCentroidWeighsUnequalTrianglesByArea(t *testing.T) {
	t.Parallel()
	// A square top face split into a small and a large triangle by an off-centre diagonal would pull an unweighted
	// mean of the triangles' centroids off the square's centre.
	verts := []r3.Vec{
		r3.NewVec(0, 0, 0), r3.NewVec(8, 0, 0), r3.NewVec(8, 8, 0), r3.NewVec(0, 8, 0),
		r3.NewVec(0, 0, 2), r3.NewVec(8, 0, 2), r3.NewVec(8, 8, 2), r3.NewVec(0, 8, 2),
		r3.NewVec(4, 0, 2),
	}
	// The y = 0 side is split at (4, 0, 2), so the top face is a triangle fan of unequal triangles.
	tris := [][3]int{
		{0, 3, 2}, {0, 2, 1},
		{4, 8, 6}, {4, 6, 7}, {8, 5, 6},
		{0, 1, 5}, {0, 5, 8}, {0, 8, 4},
		{3, 7, 6}, {3, 6, 2},
		{0, 4, 7}, {0, 7, 3},
		{1, 2, 6}, {1, 6, 5},
	}
	m, err := decad.NewMeshBody(t.Context(), verts, tris)
	require.NoError(t, err)
	body, err := decad.SolidFromMesh(t.Context(), decad.New(), m)
	require.NoError(t, err)
	top := body.Faces()
	var found bool
	for _, f := range top {
		c, err := f.Centroid()
		require.NoError(t, err)
		if c.Value.Z == 2 && c.Value.X > 3 && c.Value.X < 5 && c.Value.Y > 3 && c.Value.Y < 5 {
			require.InDelta(t, 4, c.Value.X, 1e-12)
			require.InDelta(t, 4, c.Value.Y, 1e-12)
			found = true
		}
	}
	require.True(t, found, "the top face has its centroid at the square's centre")
}
