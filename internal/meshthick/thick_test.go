package meshthick_test

import (
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/meshthick"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// box returns the outward-wound mesh of the axis-aligned box [0,x] x [0,y] x [0,z].
func box(x, y, z float64) ([]r3.Vec, [][3]int) {
	v := []r3.Vec{
		r3.NewVec(0, 0, 0), r3.NewVec(x, 0, 0), r3.NewVec(x, y, 0), r3.NewVec(0, y, 0),
		r3.NewVec(0, 0, z), r3.NewVec(x, 0, z), r3.NewVec(x, y, z), r3.NewVec(0, y, z),
	}
	t := [][3]int{
		{0, 2, 1}, {0, 3, 2}, {4, 5, 6}, {4, 6, 7}, {0, 1, 5}, {0, 5, 4},
		{2, 3, 7}, {2, 7, 6}, {1, 2, 6}, {1, 6, 5}, {3, 0, 4}, {3, 4, 7},
	}
	return v, t
}

func thinnest(t *testing.T, v []r3.Vec, tris [][3]int) meshthick.Result {
	t.Helper()
	got, err := meshthick.Min(proofbound.NewWorkBudget(t.Context()), v, tris)
	require.NoError(t, err)
	return got
}

func TestMinOfABoxIsItsShortestSide(t *testing.T) {
	t.Parallel()
	v, tris := box(20, 10, 3)
	got := thinnest(t, v, tris)
	require.True(t, got.Found)
	require.InDelta(t, 3, got.Thickness, 1e-9, "the 3 mm thickness, from the big faces' centres")
}

func TestMinOfACubeIsItsSide(t *testing.T) {
	t.Parallel()
	v, tris := box(5, 5, 5)
	got := thinnest(t, v, tris)
	require.True(t, got.Found)
	require.InDelta(t, 5, got.Thickness, 1e-9)
}

func TestMinFindsAThinWallBesideACavity(t *testing.T) {
	t.Parallel()
	// A 10 mm cube with a 6 mm cube cavity pushed against one side, leaving a 1
	// mm wall there and 3 mm on the others.
	v, tris := box(10, 10, 10)
	inner, itris := box(6, 6, 6)
	for i := range inner {
		inner[i] = inner[i].Add(r3.NewVec(3, 3, 3))
	}
	// Push the cavity to 1 mm from z = 0.
	for i := range inner {
		inner[i] = inner[i].Add(r3.NewVec(0, 0, -2))
	}
	n := len(v)
	v = append(v, inner...)
	for _, tri := range itris {
		tris = append(tris, [3]int{tri[0] + n, tri[2] + n, tri[1] + n}) // the cavity winds inward
	}
	got := thinnest(t, v, tris)
	require.True(t, got.Found)
	require.InDelta(t, 1, got.Thickness, 1e-9, "the wall between the cavity floor and the bottom face")
}

func TestMinOfAWedgeIsItsThinEnd(t *testing.T) {
	t.Parallel()
	// A wedge 10 mm tall at x = 0 and 0.5 mm tall at x = 10, 4 mm deep.
	v := []r3.Vec{
		r3.NewVec(0, 0, 0), r3.NewVec(10, 0, 0), r3.NewVec(10, 4, 0), r3.NewVec(0, 4, 0),
		r3.NewVec(0, 0, 10), r3.NewVec(10, 0, 0.5), r3.NewVec(10, 4, 0.5), r3.NewVec(0, 4, 10),
	}
	_, tris := box(1, 1, 1)
	got := thinnest(t, v, tris)
	require.True(t, got.Found)
	require.Less(t, got.Thickness, 10.0)
	require.Greater(t, got.Thickness, 0.0)
	require.True(t, !math.IsInf(got.Thickness, 0))
}

func TestMinRefusesNothingOnEmptyMeshes(t *testing.T) {
	t.Parallel()
	got, err := meshthick.Min(proofbound.NewWorkBudget(t.Context()), nil, nil)
	require.NoError(t, err)
	require.False(t, got.Found)
}

func TestMinChecksCancellation(t *testing.T) {
	t.Parallel()
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	v, tris := box(5, 5, 5)
	_, err := meshthick.Min(proofbound.NewPollWorkBudget(ctx), v, tris)
	require.Error(t, err)
}
