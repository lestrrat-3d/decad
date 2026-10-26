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

// This file is agent-docs/point-containment-design.md's public-surface tests
// for Body.ClosestPoint and Face.ClosestPoint, on a 10 mm box at the origin.

func TestClosestPointOfBox(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	box := boxBody(t, doc, 0, 0, 10, 10, 10)
	for _, tc := range []struct {
		name string
		at   r3.Vec
		want r3.Vec
	}{
		{"inside is its own", r3.NewVec(5, 5, 5), r3.NewVec(5, 5, 5)},
		{"on the boundary is its own", r3.NewVec(0, 5, 5), r3.NewVec(0, 5, 5)},
		{"beside a face", r3.NewVec(12, 3, 4), r3.NewVec(10, 3, 4)},
		{"past an edge", r3.NewVec(12, 13, 4), r3.NewVec(10, 10, 4)},
		{"past a corner", r3.NewVec(11, -2, 14), r3.NewVec(10, 0, 10)},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			got, err := box.ClosestPoint(t.Context(), tc.at, units.Millimeters(0.1))
			require.NoError(t, err)
			require.True(t, got.Point.Equal(tc.want, 1e-12), "point %v, want %v", got.Point, tc.want)
			// The point's own distance from the query matches the distance reading.
			distance, err := box.DistanceToPoint(t.Context(), tc.at, units.Millimeters(0.1))
			require.NoError(t, err)
			require.Equal(t, distance, got.Distance)
			require.LessOrEqual(t, math.Abs(tc.at.Sub(got.Point).Len()-got.Distance.Value.Base()),
				got.Distance.Bound.Base()+got.OffSurface.Base()+1e-12)
		})
	}
}

func TestClosestPointOffSurfaceStatesTheMeshBound(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	box := boxBody(t, doc, 0, 0, 10, 10, 10)
	inside, err := box.ClosestPoint(t.Context(), r3.NewVec(5, 5, 5), units.Millimeters(0.1))
	require.NoError(t, err)
	require.Zero(t, inside.OffSurface.Base(), "an interior query point is its own closest point")
	outside, err := box.ClosestPoint(t.Context(), r3.NewVec(12, 3, 4), units.Millimeters(0.1))
	require.NoError(t, err)
	require.GreaterOrEqual(t, outside.OffSurface.Base(), 0.0)
	require.Less(t, outside.OffSurface.Base(), 1e-9, "a box has planar faces, so its mesh departs from it only by rounding")
}

func TestClosestPointOfFace(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	box := boxBody(t, doc, 0, 0, 10, 10, 10)
	var top *decad.Face
	for _, f := range box.Faces() {
		n, err := f.NormalAt(r3.NewVec(5, 5, 10))
		if err == nil && n.Value.Equal(r3.NewVec(0, 0, 1), 1e-12) {
			top = f
		}
	}
	require.NotNil(t, top)
	// A point beside the box is nearest, on the top face, to the top rim.
	got, err := top.ClosestPoint(t.Context(), r3.NewVec(14, 3, 4), units.Millimeters(0.1))
	require.NoError(t, err)
	require.True(t, got.Point.Equal(r3.NewVec(10, 3, 10), 1e-12), "%v", got.Point)
	// A point above the face is nearest at its foot on the face.
	got, err = top.ClosestPoint(t.Context(), r3.NewVec(2, 7, 25), units.Millimeters(0.1))
	require.NoError(t, err)
	require.True(t, got.Point.Equal(r3.NewVec(2, 7, 10), 1e-12), "%v", got.Point)
}

func TestClosestPointRefusals(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	box := boxBody(t, doc, 0, 0, 10, 10, 10)
	_, err := box.ClosestPoint(context.Context(nil), r3.NewVec(12, 5, 5), units.Millimeters(0.1))
	require.ErrorIs(t, err, decad.ErrDegenerate)
	_, err = box.ClosestPoint(t.Context(), r3.NewVec(math.NaN(), 0, 0), units.Millimeters(0.1))
	require.ErrorIs(t, err, decad.ErrNotFinite)
	_, err = box.ClosestPoint(t.Context(), r3.NewVec(12, 5, 5), units.Degrees(1))
	require.ErrorIs(t, err, decad.ErrUnitKind)
	var nilFace *decad.Face
	_, err = nilFace.ClosestPoint(t.Context(), r3.NewVec(1, 1, 1), units.Millimeters(0.1))
	require.ErrorIs(t, err, decad.ErrDegenerate)
}
