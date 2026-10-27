package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is agent-docs/mesh-solid-design.md §3's public-surface tests for Stretch.

func TestStretchScalesEachAxisOfAMeshSolid(t *testing.T) {
	t.Parallel()
	doc, block := blockSolid(t, r3.NewVec(0, 0, 0), r3.NewVec(10, 8, 6))
	out, err := decad.Stretch(t.Context(), block, r3.NewVec(0, 0, 0), units.Scalar(2), units.Scalar(0.5), units.Scalar(3))
	require.NoError(t, err)
	require.Equal(t, []*decad.Body{out}, doc.Bodies())
	operations := doc.OperationHistory().Operations()
	require.Equal(t, decad.OperationStretch, operations[len(operations)-1].Kind())
	volume, err := out.Volume()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(volume.Value.Base()-10*8*6*2*0.5*3), volume.Bound.Base()+1e-9)
	box, err := out.Bounds()
	require.NoError(t, err)
	require.True(t, box.Max.Equal(r3.NewVec(20, 4, 18), 1e-9), "max %v", box.Max)
	require.Len(t, out.Faces(), 6)
}

func TestStretchAboutAnAnchorAndOfAnLShape(t *testing.T) {
	t.Parallel()
	l := lPrismSolid(t)
	out, err := decad.Stretch(t.Context(), l, r3.NewVec(2, 0, 0), units.Scalar(1.5), units.Scalar(1), units.Scalar(2))
	require.NoError(t, err)
	volume, err := out.Volume()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(volume.Value.Base()-24*1.5*2), volume.Bound.Base()+1e-9)
	box, err := out.Bounds()
	require.NoError(t, err)
	// x = 0 moves to 2 + (0−2)·1.5 = −1.
	require.InDelta(t, -1, box.Min.X, 1e-9)
	inside, err := out.LocatePoint(t.Context(), r3.NewVec(-0.5, 1, 1), units.Millimeters(0.01))
	require.NoError(t, err)
	require.Equal(t, decad.PointInside, inside)
}

func TestStretchMirrorsOnANegativeFactor(t *testing.T) {
	t.Parallel()
	_, block := blockSolid(t, r3.NewVec(0, 0, 0), r3.NewVec(10, 8, 6))
	out, err := decad.Stretch(t.Context(), block, r3.NewVec(0, 0, 0), units.Scalar(-1), units.Scalar(1), units.Scalar(1))
	require.NoError(t, err)
	volume, err := out.Volume()
	require.NoError(t, err)
	require.Greater(t, volume.Value.Base(), 0.0, "the mirrored solid stays wound outward")
	box, err := out.Bounds()
	require.NoError(t, err)
	require.InDelta(t, -10, box.Min.X, 1e-9)
}

func TestStretchRefusals(t *testing.T) {
	t.Parallel()
	doc, block := blockSolid(t, r3.NewVec(0, 0, 0), r3.NewVec(10, 8, 6))
	one := units.Scalar(1)
	o := r3.NewVec(0, 0, 0)
	stretch := func(b *decad.Body, x, y, z units.Value) error {
		_, err := decad.Stretch(t.Context(), b, o, x, y, z)
		return err
	}
	require.ErrorIs(t, stretch(block, units.Scalar(0), one, one), decad.ErrDegenerate)
	require.ErrorIs(t, stretch(block, units.Millimeters(2), one, one), decad.ErrUnitKind)
	require.ErrorIs(t, stretch(block, units.Scalar(math.Inf(1)), one, one), decad.ErrNotFinite)
	_, err := decad.Stretch(t.Context(), block, r3.NewVec(math.NaN(), 0, 0), one, one, one)
	require.ErrorIs(t, err, decad.ErrNotFinite)
	_, err = decad.Stretch(nil, block, o, one, one, one) //nolint:staticcheck // a nil context is the case under test
	require.ErrorIs(t, err, decad.ErrDegenerate)
	prism := boxBody(t, doc, 30, 0, 40, 8, 6)
	require.ErrorIs(t, stretch(prism, units.Scalar(2), one, one), decad.ErrUnsupported)
	require.Len(t, doc.Bodies(), 2, "every refusal leaves the document alone")
}
