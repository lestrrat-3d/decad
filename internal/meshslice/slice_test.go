package meshslice_test

import (
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/meshslice"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// cube returns a 2 mm cube at the origin, wound counter-clockwise seen from outside.
func cube() ([]r3.Vec, [][3]int) {
	v := []r3.Vec{
		r3.NewVec(0, 0, 0), r3.NewVec(2, 0, 0), r3.NewVec(2, 2, 0), r3.NewVec(0, 2, 0),
		r3.NewVec(0, 0, 2), r3.NewVec(2, 0, 2), r3.NewVec(2, 2, 2), r3.NewVec(0, 2, 2),
	}
	t := [][3]int{
		{0, 2, 1}, {0, 3, 2}, {4, 5, 6}, {4, 6, 7}, {0, 1, 5}, {0, 5, 4},
		{2, 3, 7}, {2, 7, 6}, {1, 2, 6}, {1, 6, 5}, {3, 0, 4}, {3, 4, 7},
	}
	return v, t
}

var up = r3.NewVec(0, 0, 1)

func TestSliceCubeAtMidHeightIsACounterClockwiseSquare(t *testing.T) {
	t.Parallel()
	v, tris := cube()
	got, err := meshslice.Slice(t.Context(), v, tris, r3.NewVec(0, 0, 1), up)
	require.NoError(t, err)
	require.Len(t, got, 1)
	require.Len(t, got[0].Points, 8, "four sides, each crossed by two triangles")
	require.InDelta(t, 4, got[0].Area(up), 1e-12, "a 2 x 2 square, counter-clockwise from +Z")
	for _, p := range got[0].Points {
		require.InDelta(t, 1, p.Z, 1e-12)
		require.True(t, math.Abs(p.X) < 1e-12 || math.Abs(p.X-2) < 1e-12 || math.Abs(p.Y) < 1e-12 || math.Abs(p.Y-2) < 1e-12,
			"every point lies on the square's boundary: %v", p)
	}
}

func TestSliceFromTheOtherSideReversesTheOutline(t *testing.T) {
	t.Parallel()
	v, tris := cube()
	got, err := meshslice.Slice(t.Context(), v, tris, r3.NewVec(0, 0, 1), r3.NewVec(0, 0, -1))
	require.NoError(t, err)
	require.Len(t, got, 1)
	require.InDelta(t, 4, got[0].Area(r3.NewVec(0, 0, -1)), 1e-12, "counter-clockwise seen from the new normal")
	require.InDelta(t, -4, got[0].Area(up), 1e-12)
}

func TestSliceThroughACoplanarFaceCountsTheOnPlaneSideAsAbove(t *testing.T) {
	t.Parallel()
	v, tris := cube()
	// The top face lies in z = 2: its vertices count as above, so the cut
	// runs along the four side walls just under it and is the same square.
	got, err := meshslice.Slice(t.Context(), v, tris, r3.NewVec(0, 0, 2), up)
	require.NoError(t, err)
	require.Len(t, got, 1)
	require.InDelta(t, 4, got[0].Area(up), 1e-12)
	// The bottom face at z = 0 is also on the plane: all vertices are on the
	// normal's side, so nothing crosses and the cut is empty.
	empty, err := meshslice.Slice(t.Context(), v, tris, r3.NewVec(0, 0, 0), up)
	require.NoError(t, err)
	require.Empty(t, empty)
}

func TestSliceObliquePlaneThroughTheCornersAndPastTheCube(t *testing.T) {
	t.Parallel()
	v, tris := cube()
	// The plane x + y + z = 3 cuts the cube through its centre in a regular
	// hexagon of side √2, whose area is 3√3/2 · 2.
	n, _ := r3.NewVec(1, 1, 1).Normalize()
	got, err := meshslice.Slice(t.Context(), v, tris, r3.NewVec(1, 1, 1), n)
	require.NoError(t, err)
	require.Len(t, got, 1)
	require.InDelta(t, 3*math.Sqrt(3), got[0].Area(n), 1e-9)
	missed, err := meshslice.Slice(t.Context(), v, tris, r3.NewVec(0, 0, 5), up)
	require.NoError(t, err)
	require.Empty(t, missed, "a plane past the cube cuts nothing")
}

func TestSliceTwoShellsGivesTwoOutlines(t *testing.T) {
	t.Parallel()
	v, tris := cube()
	shifted := make([]r3.Vec, len(v))
	for i, p := range v {
		shifted[i] = p.Add(r3.NewVec(5, 0, 0))
	}
	all := append(append([]r3.Vec(nil), v...), shifted...)
	more := append([][3]int(nil), tris...)
	for _, tri := range tris {
		more = append(more, [3]int{tri[0] + 8, tri[1] + 8, tri[2] + 8})
	}
	got, err := meshslice.Slice(t.Context(), all, more, r3.NewVec(0, 0, 1), up)
	require.NoError(t, err)
	require.Len(t, got, 2)
	require.InDelta(t, 8, got[0].Area(up)+got[1].Area(up), 1e-12)
}

func TestSliceInsideOutShellIsAClockwiseHole(t *testing.T) {
	t.Parallel()
	v, tris := cube()
	// An inner cube wound inward is a cavity: its outline runs clockwise.
	inner := make([]r3.Vec, len(v))
	for i, p := range v {
		inner[i] = p.Scale(0.5).Add(r3.NewVec(0.5, 0.5, 0.5))
	}
	all := append(append([]r3.Vec(nil), v...), inner...)
	more := append([][3]int(nil), tris...)
	for _, tri := range tris {
		more = append(more, [3]int{tri[0] + 8, tri[2] + 8, tri[1] + 8})
	}
	got, err := meshslice.Slice(t.Context(), all, more, r3.NewVec(0, 0, 1), up)
	require.NoError(t, err)
	require.Len(t, got, 2)
	total := got[0].Area(up) + got[1].Area(up)
	require.InDelta(t, 4-1, total, 1e-12, "the 2 x 2 square less the 1 x 1 cavity")
}

func TestSliceRefusesAnOpenMesh(t *testing.T) {
	t.Parallel()
	v, tris := cube()
	_, err := meshslice.Slice(t.Context(), v, tris[2:], r3.NewVec(0, 0, 1), up) // no bottom
	require.NoError(t, err, "an open bottom leaves the mid-height cut closed")
	_, err = meshslice.Slice(t.Context(), v, tris[:11], r3.NewVec(0, 0, 1), up) // a missing side triangle
	require.ErrorIs(t, err, meshslice.ErrOpen)
}

func TestSliceChecksCancellation(t *testing.T) {
	t.Parallel()
	v, tris := cube()
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	_, err := meshslice.Slice(ctx, v, tris, r3.NewVec(0, 0, 1), up)
	require.ErrorIs(t, err, context.Canceled)
}
