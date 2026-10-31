package retainedblend

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestPossibleSharedVertexKeepsEuclideanMatches(t *testing.T) {
	tol := 1.0
	for _, delta := range []r3.Vec{
		{},
		{X: tol},
		{Y: -tol},
		{X: math.Sqrt(0.5), Y: math.Sqrt(0.5)},
		{Z: math.Nextafter(tol, 0)},
	} {
		require.LessOrEqual(t, delta.Len(), tol)
		require.True(t, possibleSharedVertex(delta, tol))
	}
}

func TestPossibleSharedVertexLeavesLengthCheckForNearbyPoints(t *testing.T) {
	tol := 1.0
	nearby := r3.Vec{X: 1.5 * tol}
	require.True(t, possibleSharedVertex(nearby, tol))
	require.Greater(t, nearby.Len(), tol)

	atFilterBoundary := r3.Vec{Y: 2 * tol}
	require.True(t, possibleSharedVertex(atFilterBoundary, tol))
	require.Greater(t, atFilterBoundary.Len(), tol)

	beyondFilterBoundary := r3.Vec{Y: math.Nextafter(2*tol, math.Inf(1))}
	require.False(t, possibleSharedVertex(beyondFilterBoundary, tol))
}

func TestSharedVertexIndexMatchesLinearScan(t *testing.T) {
	tol := 1.0
	verts := []r3.Vec{
		{X: -2.01}, // Negative cell; within tolerance across its lower boundary.
		{X: 0.01},  // A second match, in a different cell.
		{X: -0.01}, // A later match confirms that the earliest matching ID wins.
		{X: 20},
	}
	shared := []int{0, 1, 2, 3}
	index := newSharedVertexIndex(tol)
	for id, p := range verts {
		index.add(id, p)
	}
	for _, query := range []r3.Vec{
		{X: -1.99}, // Crosses a bucket boundary at -2.
		{},         // Multiple matches choose the earliest ID.
		{X: 20},
		{X: -20},
	} {
		got := index.find(verts, shared, query, tol)
		want := scanSharedVertices(verts, shared, query, tol)
		require.Equal(t, want, got)
	}
	require.Equal(t, 1, index.find(verts, shared, r3.Vec{}, tol))
}

func TestSharedVertexIndexFallsBackForUnquantizablePoints(t *testing.T) {
	tol := 1.0
	point := r3.Vec{X: math.MaxFloat64}
	verts := []r3.Vec{point}
	shared := []int{0}
	index := newSharedVertexIndex(tol)
	index.add(0, point)
	require.Len(t, index.unindexed, 1)
	require.Equal(t, 0, index.find(verts, shared, point, tol))

	invalidQuery := r3.Vec{X: math.NaN()}
	require.Equal(t, scanSharedVertices(verts, shared, invalidQuery, tol),
		index.find(verts, shared, invalidQuery, tol))

	verts = append(verts, r3.Vec{X: math.Inf(1)})
	shared = append(shared, 1)
	index.add(1, verts[1])
	require.Equal(t, scanSharedVertices(verts, shared, r3.Vec{}, tol), index.find(verts, shared, r3.Vec{}, tol))
}
