package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is agent-docs/orthogonal-move-design.md's public-surface tests for MoveFace.

func TestMoveFacePushesAndPullsFacesOfABox(t *testing.T) {
	t.Parallel()
	for name, tc := range map[string]struct {
		facing   r3.Vec
		distance float64
		volume   float64
		lo, hi   r3.Vec
	}{
		"top out":   {r3.NewVec(0, 0, 1), 2, 10 * 8 * 8, r3.NewVec(0, 0, 0), r3.NewVec(10, 8, 8)},
		"top in":    {r3.NewVec(0, 0, 1), -2, 10 * 8 * 4, r3.NewVec(0, 0, 0), r3.NewVec(10, 8, 4)},
		"right out": {r3.NewVec(1, 0, 0), 3, 13 * 8 * 6, r3.NewVec(0, 0, 0), r3.NewVec(13, 8, 6)},
		"left in":   {r3.NewVec(-1, 0, 0), -4, 6 * 8 * 6, r3.NewVec(4, 0, 0), r3.NewVec(10, 8, 6)},
		"front out": {r3.NewVec(0, -1, 0), 1.5, 10 * 9.5 * 6, r3.NewVec(0, -1.5, 0), r3.NewVec(10, 8, 6)},
		"bottom in": {r3.NewVec(0, 0, -1), -1, 10 * 8 * 5, r3.NewVec(0, 0, 1), r3.NewVec(10, 8, 6)},
	} {
		t.Run(name, func(t *testing.T) {
			t.Parallel()
			doc, block := blockSolid(t, r3.NewVec(0, 0, 0), r3.NewVec(10, 8, 6))
			out, err := decad.MoveFace(t.Context(), block, decad.Faces(decad.Facing(tc.facing)), units.Millimeters(tc.distance))
			require.NoError(t, err)
			require.Equal(t, []*decad.Body{out}, doc.Bodies())
			operations := doc.OperationHistory().Operations()
			require.Equal(t, decad.OperationMoveFace, operations[len(operations)-1].Kind())
			volume, err := out.Volume()
			require.NoError(t, err)
			require.LessOrEqual(t, math.Abs(volume.Value.Base()-tc.volume), volume.Bound.Base()+1e-9)
			box, err := out.Bounds()
			require.NoError(t, err)
			require.True(t, box.Min.Equal(tc.lo, 1e-9), "min %v", box.Min)
			require.True(t, box.Max.Equal(tc.hi, 1e-9), "max %v", box.Max)
			require.Len(t, out.Faces(), 6, "a moved face is still a face, and its neighbours follow it")
		})
	}
}

func TestMoveFaceOnAnLShape(t *testing.T) {
	t.Parallel()
	left := decad.Faces(decad.Facing(r3.NewVec(-1, 0, 0))) // the one face at x = 0, over y 0..4
	volumeOf := func(distance float64) (decad.Measurement, error) {
		out, err := decad.MoveFace(t.Context(), lPrismSolid(t), left, units.Millimeters(distance))
		if err != nil {
			return decad.Measurement{}, err
		}
		return out.Volume()
	}
	for distance, want := range map[float64]float64{1: 24 + 4*2, -1: 24 - 4*2, -2: 24 - 2*4*2} {
		volume, err := volumeOf(distance)
		require.NoError(t, err, "distance %g", distance)
		require.LessOrEqual(t, math.Abs(volume.Value.Base()-want), volume.Bound.Base()+1e-9, "distance %g", distance)
	}
	// Pulling it in by 3 would pass the notch: the prism would leave the L.
	_, err := volumeOf(-3)
	require.ErrorIs(t, err, decad.ErrDegenerate)
	// The widened arm holds a point beyond the old face.
	out, err := decad.MoveFace(t.Context(), lPrismSolid(t), left, units.Millimeters(1))
	require.NoError(t, err)
	inside, err := out.LocatePoint(t.Context(), r3.NewVec(-0.5, 3, 1), units.Millimeters(0.01))
	require.NoError(t, err)
	require.Equal(t, decad.PointInside, inside)
}

func TestMoveFaceRefusals(t *testing.T) {
	t.Parallel()
	doc, block := blockSolid(t, r3.NewVec(0, 0, 0), r3.NewVec(10, 8, 6))
	top := decad.Faces(decad.Facing(r3.NewVec(0, 0, 1)))
	move := func(sel decad.FaceSelector, d units.Value) error {
		_, err := decad.MoveFace(t.Context(), block, sel, d)
		return err
	}
	require.ErrorIs(t, move(top, units.Millimeters(0)), decad.ErrDegenerate)
	require.ErrorIs(t, move(top, units.Millimeters(-7)), decad.ErrDegenerate, "out through the opposite wall")
	require.ErrorIs(t, move(top, units.Millimeters(-6)), decad.ErrDegenerate, "all the way through")
	require.ErrorIs(t, move(top, units.Degrees(5)), decad.ErrUnitKind)
	require.ErrorIs(t, move(top, units.Millimeters(math.NaN())), decad.ErrNotFinite)
	require.ErrorIs(t, move(decad.Faces(decad.Planar()), units.Millimeters(1)), decad.ErrDegenerate, "six faces are not one")
	require.ErrorIs(t, move(nil, units.Millimeters(1)), decad.ErrDegenerate)
	_, err := decad.MoveFace(nil, block, top, units.Millimeters(1)) //nolint:staticcheck // a nil context is the case under test
	require.ErrorIs(t, err, decad.ErrDegenerate)
	// A body that is not a faceted orthogonal solid keeps its refusal.
	prism := boxBody(t, doc, 30, 0, 40, 8, 6)
	_, err = decad.MoveFace(t.Context(), prism, decad.Faces(decad.Facing(r3.NewVec(0, 0, 1))), units.Millimeters(1))
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.Len(t, doc.Bodies(), 2, "every refusal leaves the document alone")
}
