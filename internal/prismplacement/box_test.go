package prismplacement_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/prismplacement"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestExactBoxInFrame(t *testing.T) {
	ref, err := r3.NewFrame(r3.Vec{}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	require.NoError(t, err)
	// The source u axis is -Y, its v axis is +Z and its normal is -X, so the
	// swept extent runs from x=5 down to x=2.
	source, err := r3.NewFrame(r3.Vec{X: 5}, r3.Vec{Y: -1}, r3.Vec{Z: 1})
	require.NoError(t, err)

	box, ok := prismplacement.ExactBoxInFrame(0, 2, 0, 1, 0, 3, source, ref)
	require.True(t, ok)
	require.Equal(t, prismplacement.Box{Lo: [3]float64{2, -2, 0}, Hi: [3]float64{5, 0, 1}}, box)

	// An empty extent is not a box.
	_, ok = prismplacement.ExactBoxInFrame(0, 2, 0, 1, 1, 1, source, ref)
	require.False(t, ok)
}

func TestBoxesShareFacePatch(t *testing.T) {
	unit := prismplacement.Box{Hi: [3]float64{1, 1, 1}}
	right := prismplacement.Box{Lo: [3]float64{1, 0, 0}, Hi: [3]float64{2, 1, 1}}
	edge := prismplacement.Box{Lo: [3]float64{1, 1, 0}, Hi: [3]float64{2, 2, 1}}

	require.True(t, prismplacement.BoxesShareFacePatch(unit, right, 2))
	require.False(t, prismplacement.BoxesShareFacePatch(unit, right, 0))
	require.False(t, prismplacement.BoxesShareFacePatch(unit, edge, 2))
}

func TestBoxProfile(t *testing.T) {
	profile := prismplacement.BoxProfile(prismplacement.Box{Hi: [3]float64{2, 1, 0}})
	require.Empty(t, profile.Holes)
	require.Len(t, profile.Outer.Segments, 4)
	second, ok := profile.Outer.Segments[1].(sectionrecord.LineSeg)
	require.True(t, ok)
	require.Equal(t, sectionrecord.LineSeg{
		Start: sectionrecord.Point2{U: 2, V: 0}, End: sectionrecord.Point2{U: 2, V: 1}, TStart: 0, TEnd: 1,
	}, second)
}
