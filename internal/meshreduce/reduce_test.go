package meshreduce_test

import (
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/meshreduce"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// grid is an n x n grid of unit quads on z = 0, two counter-clockwise triangles
// each, with its (n+1)² vertices numbered along x then y.
func grid(n int) ([]r3.Vec, [][3]int) {
	var v []r3.Vec
	for y := 0; y <= n; y++ {
		for x := 0; x <= n; x++ {
			v = append(v, r3.NewVec(float64(x), float64(y), 0))
		}
	}
	var t [][3]int
	at := func(x, y int) int { return y*(n+1) + x }
	for y := range n {
		for x := range n {
			t = append(t, [3]int{at(x, y), at(x+1, y), at(x+1, y+1)}, [3]int{at(x, y), at(x+1, y+1), at(x, y+1)})
		}
	}
	return v, t
}

func area(v []r3.Vec, tris [][3]int) float64 {
	total := 0.0
	for _, t := range tris {
		total += v[t[1]].Sub(v[t[0]]).Cross(v[t[2]].Sub(v[t[0]])).Len() / 2
	}
	return total
}

func TestReduceAGridByCellsOfFour(t *testing.T) {
	t.Parallel()
	v, tris := grid(16)
	got, err := meshreduce.Reduce(t.Context(), v, tris, 4)
	require.NoError(t, err)
	// Along each axis the vertices 0..3 share a cell with mean 1.5, then 4..7
	// with 5.5, 8..11 with 9.5, 12..15 with 13.5, and 16 stands alone.
	require.Len(t, got.Vertices, 25)
	require.Len(t, got.Triangles, 2*4*4)
	xs := map[float64]bool{}
	for _, p := range got.Vertices {
		xs[p.X] = true
		require.Zero(t, p.Z)
	}
	require.Equal(t, map[float64]bool{1.5: true, 5.5: true, 9.5: true, 13.5: true, 16: true}, xs)
	require.InDelta(t, 14.5*14.5, area(got.Vertices, got.Triangles), 1e-9, "the grid spans 1.5 to 16 each way")
	require.Equal(t, 289-25, got.MergedVertices)
	require.Equal(t, len(tris)-len(got.Triangles), got.DroppedTriangles)
	// The corner vertex (0, 0) moves to (1.5, 1.5), the farthest of any.
	require.InDelta(t, math.Hypot(1.5, 1.5), got.MaxMove, 1e-12)
}

func TestReduceKeepsTriangleWinding(t *testing.T) {
	t.Parallel()
	v, tris := grid(16)
	got, err := meshreduce.Reduce(t.Context(), v, tris, 4)
	require.NoError(t, err)
	for _, tri := range got.Triangles {
		n := got.Vertices[tri[1]].Sub(got.Vertices[tri[0]]).Cross(got.Vertices[tri[2]].Sub(got.Vertices[tri[0]]))
		require.Positive(t, n.Z, "a triangle of the plane still faces +Z")
	}
}

func TestReduceWithACellSmallerThanTheSpacingChangesNothing(t *testing.T) {
	t.Parallel()
	v, tris := grid(4)
	got, err := meshreduce.Reduce(t.Context(), v, tris, 0.5)
	require.NoError(t, err)
	require.Len(t, got.Vertices, 25)
	require.Len(t, got.Triangles, len(tris))
	require.Zero(t, got.MaxMove)
	require.Zero(t, got.MergedVertices)
	require.Zero(t, got.DroppedTriangles)
}

func TestReduceToOneCellDropsEverything(t *testing.T) {
	t.Parallel()
	v, tris := grid(4)
	got, err := meshreduce.Reduce(t.Context(), v, tris, 100)
	require.NoError(t, err)
	require.Empty(t, got.Triangles)
	require.Empty(t, got.Vertices)
}

func TestReduceReadsTheContext(t *testing.T) {
	t.Parallel()
	v, tris := grid(4)
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	_, err := meshreduce.Reduce(ctx, v, tris, 2)
	require.ErrorIs(t, err, context.Canceled)
}

func TestReduceDropsATriangleThatRepeatsAnothersCells(t *testing.T) {
	t.Parallel()
	v := []r3.Vec{
		r3.NewVec(0, 0, 0), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0),
		r3.NewVec(0.1, 0, 0), r3.NewVec(1.1, 0, 0), r3.NewVec(0, 1.1, 0),
	}
	tris := [][3]int{{0, 1, 2}, {3, 4, 5}}
	got, err := meshreduce.Reduce(t.Context(), v, tris, 1)
	require.NoError(t, err)
	require.Len(t, got.Triangles, 1, "both triangles span the same three cells")
	require.Equal(t, 1, got.DroppedTriangles)
	require.Len(t, got.Vertices, 3)
}

func TestSurfaceBoundRefusesAReductionThatLeavesACollapsedPieceUncovered(t *testing.T) {
	t.Parallel()
	// A 10 mm triangle and, apart from it, one 0.1 mm triangle. A 1 mm cell collapses the small one to a
	// point that no kept triangle has as a corner, so part of the input has no counterpart in the result.
	verts := []r3.Vec{
		{X: 0, Y: 0}, {X: 10, Y: 0}, {X: 0, Y: 10},
		{X: 20, Y: 20}, {X: 20.1, Y: 20}, {X: 20, Y: 20.1},
	}
	tris := [][3]int{{0, 1, 2}, {3, 4, 5}}
	got, err := meshreduce.Reduce(t.Context(), verts, tris, 1)
	require.NoError(t, err)
	require.Len(t, got.Triangles, 1)
	require.False(t, got.Covered)
	_, ok := meshreduce.SurfaceBound(got, verts)
	require.False(t, ok)
	// With a cell no vertex shares, nothing collapses and the bound is the rounding alone.
	whole, err := meshreduce.Reduce(t.Context(), verts, tris, 0.01)
	require.NoError(t, err)
	require.True(t, whole.Covered)
	bound, ok := meshreduce.SurfaceBound(whole, verts)
	require.True(t, ok)
	require.Less(t, bound, 1e-9)
	_, _, found, err := meshreduce.ReduceWithin(t.Context(), verts, tris, 0.2)
	require.NoError(t, err)
	require.True(t, found, "some cell proves 0.2 mm")
}
