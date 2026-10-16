package apitest_test

import (
	"errors"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestBrepUnionSectionFeedsExtrude(t *testing.T) {
	doc := decad.New()
	base := boxBodyAtZ(t, doc, -5, -5, 5, 5, 0, 10)
	lug := boxBodyAtZ(t, doc, 3, -2, 7, 2, 8, 4)
	joined, err := decad.Union(t.Context(), base, lug)
	require.NoError(t, err)
	joinedVolume, err := joined.Volume()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(joinedVolume.Value.Base()-1048), joinedVolume.Bound.Base())
	frame, err := r3.NewFrame(r3.Vec{Z: 9}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	require.NoError(t, err)
	section, profiles, err := joined.Section(t.Context(), frame)
	require.NoError(t, err)
	require.Equal(t, 1, len(profiles))
	reading, err := decad.MeasureProfile(section, profiles[0])
	require.NoError(t, err)
	area, err := reading.Area()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(area.Value.Base()-108), area.Bound.Base())
	child, err := decad.Extrude(t.Context(), decad.New(), section, profiles[0],
		decad.Distance{D: units.Millimeters(2), Dir: decad.Along})
	require.NoError(t, err)
	volume, err := child.Volume()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(volume.Value.Base()-216), volume.Bound.Base())
	mesh, err := child.Tessellate(t.Context(), units.Millimeters(0.1),
		decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
}

func TestBrepUnionSectionPlacementAndRefusals(t *testing.T) {
	doc := decad.New()
	base := boxBodyAtZ(t, doc, -5, -5, 5, 5, 0, 10)
	lug := boxBodyAtZ(t, doc, 3, -2, 7, 2, 8, 4)
	joined, err := decad.Union(t.Context(), base, lug)
	require.NoError(t, err)
	for _, tc := range []struct {
		z, area float64
	}{{z: 4, area: 100}, {z: 11, area: 16}} {
		frame, frameErr := r3.NewFrame(r3.Vec{Z: tc.z}, r3.Vec{X: 1}, r3.Vec{Y: 1})
		require.NoError(t, frameErr)
		section, profiles, sectionErr := joined.Section(t.Context(), frame)
		require.NoError(t, sectionErr)
		require.Equal(t, 1, len(profiles))
		reading, measureErr := decad.MeasureProfile(section, profiles[0])
		require.NoError(t, measureErr)
		area, areaErr := reading.Area()
		require.NoError(t, areaErr)
		require.LessOrEqual(t, math.Abs(area.Value.Base()-tc.area), area.Bound.Base())
	}
	for _, z := range []float64{0, 8, 10, 12} {
		frame, frameErr := r3.NewFrame(r3.Vec{Z: z}, r3.Vec{X: 1}, r3.Vec{Y: 1})
		require.NoError(t, frameErr)
		_, _, sectionErr := joined.Section(t.Context(), frame)
		require.True(t, errors.Is(sectionErr, decad.ErrUnsupported), "z=%g: %v", z, sectionErr)
	}
	reversed, err := r3.NewFrame(r3.Vec{Z: 9}, r3.Vec{X: -1}, r3.Vec{Y: -1})
	require.NoError(t, err)
	_, _, err = joined.Section(t.Context(), reversed)
	require.ErrorIs(t, err, decad.ErrUnsupported)

	move, err := r3.Translation(r3.Vec{X: 20, Y: 30, Z: 1})
	require.NoError(t, err)
	placed, err := decad.Transform(t.Context(), joined, move)
	require.NoError(t, err)
	frame, err := r3.NewFrame(r3.Vec{X: 20, Y: 30, Z: 10}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	require.NoError(t, err)
	section, profiles, err := placed.Section(t.Context(), frame)
	require.NoError(t, err)
	require.Equal(t, 1, len(profiles))
	reading, err := decad.MeasureProfile(section, profiles[0])
	require.NoError(t, err)
	area, err := reading.Area()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(area.Value.Base()-108), area.Bound.Base())
}

func TestBrepSectionRefusesBodyWithoutCertifiedSlabs(t *testing.T) {
	_, drilled := crossDrilledBar(t)
	frame, err := r3.NewFrame(r3.Vec{Z: 10}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	require.NoError(t, err)
	_, _, err = drilled.Section(t.Context(), frame)
	require.ErrorIs(t, err, decad.ErrUnsupported)
}
