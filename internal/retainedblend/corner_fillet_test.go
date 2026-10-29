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
