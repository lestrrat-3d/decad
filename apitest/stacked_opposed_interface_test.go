package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestStackedOpposedInterfacePockets(t *testing.T) {
	t.Parallel()
	for _, firstTop := range []bool{true, false} {
		name := map[bool]string{true: "top-first", false: "bottom-first"}[firstTop]
		t.Run(name, func(t *testing.T) {
			t.Parallel()
			doc := decad.New()
			plate := boxBody(t, doc, -10, -10, 10, 10, 10)
			top := boxBodyAtZ(t, doc, -6, -2, -2, 2, 6, 4)
			bottom := boxBodyAtZ(t, doc, 2, -2, 6, 2, 0, 6)
			first, second := top, bottom
			if !firstTop {
				first, second = bottom, top
			}
			part, err := decad.Cut(t.Context(), plate, first)
			require.NoError(t, err)
			part, err = decad.Cut(t.Context(), part, second)
			require.NoError(t, err)
			require.False(t, anyFaceIsFaceted(part))
			require.Len(t, part.Faces(), 16)
			volume, err := part.Volume()
			require.NoError(t, err)
			require.Equal(t, decad.Exact, volume.Exactness)
			require.Equal(t, 3840.0, volumeMM(t, volume))
			report, err := doc.Verify(t.Context())
			require.NoError(t, err)
			require.Equal(t, decad.Sound, report.Status)
			mesh, err := part.Tessellate(t.Context(), units.Millimeters(0.1))
			require.NoError(t, err)
			require.True(t, mesh.VolumeVerified())
			requireWatertight(t, mesh)
			rotation, err := r3.Rotation(r3.NewVec(1, 2, 3), units.Degrees(37))
			require.NoError(t, err)
			placed, err := part.Placed(t.Context(), rotation)
			require.NoError(t, err)
			require.False(t, anyFaceIsFaceted(placed))
			placedVolume, err := placed.Volume()
			require.NoError(t, err)
			require.LessOrEqual(t, math.Abs(volumeMM(t, placedVolume)-3840), boundMM3(t, placedVolume))
			placedMesh, err := placed.Tessellate(t.Context(), units.Millimeters(0.1))
			require.NoError(t, err)
			require.True(t, placedMesh.VolumeVerified())
			requireWatertight(t, placedMesh)
		})
	}
}

func TestStackedOpposedRoundBoresAcceptLaterThroughHole(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	plate := boxBody(t, doc, -10, -10, 10, 10, 10)
	top := circleBodyAtZ(t, doc, -5, 2, 6, 4)
	part, err := decad.Cut(t.Context(), plate, top)
	require.NoError(t, err)
	bottom := circleBodyAtZ(t, doc, 5, 2, 0, 6)
	part, err = decad.Cut(t.Context(), part, bottom)
	require.NoError(t, err)
	require.False(t, anyFaceIsFaceted(part))
	volume, err := part.Volume()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(volumeMM(t, volume)-(4000-40*math.Pi)), boundMM3(t, volume))
	mesh, err := part.Tessellate(t.Context(), units.Millimeters(0.1))
	require.NoError(t, err)
	require.True(t, mesh.VolumeVerified())
	requireWatertight(t, mesh)
	through := circleBodyAtZ(t, doc, 0, 1, 0, 10)
	part, err = decad.Cut(t.Context(), part, through)
	require.NoError(t, err)
	require.False(t, anyFaceIsFaceted(part))
	volume, err = part.Volume()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(volumeMM(t, volume)-(4000-50*math.Pi)), boundMM3(t, volume))
	mesh, err = part.Tessellate(t.Context(), units.Millimeters(0.1))
	require.NoError(t, err)
	require.True(t, mesh.VolumeVerified())
	requireWatertight(t, mesh)
}
