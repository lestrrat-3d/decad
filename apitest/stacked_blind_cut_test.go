package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestStackedBlindCutSplitsExistingSlab(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name       string
		z, height  float64
		wantVolume float64
	}{
		{name: "from-top", z: 4, height: 6, wantVolume: 3840},
		{name: "from-bottom", z: 0, height: 8, wantVolume: 3808},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc := decad.New()
			plate := boxBody(t, doc, -10, -10, 10, 10, 10)
			firstTool := boxBodyAtZ(t, doc, -6, -2, -2, 2, 6, 4)
			part, err := decad.Cut(t.Context(), plate, firstTool)
			require.NoError(t, err)
			secondTool := boxBodyAtZ(t, doc, 2, -2, 6, 2, tc.z, tc.height)
			part, err = decad.Cut(t.Context(), part, secondTool)
			require.NoError(t, err)
			require.False(t, anyFaceIsFaceted(part))
			volume, err := part.Volume()
			require.NoError(t, err)
			require.Equal(t, tc.wantVolume, volumeMM(t, volume))
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
			require.LessOrEqual(t, math.Abs(volumeMM(t, placedVolume)-tc.wantVolume), boundMM3(t, placedVolume))
			placedMesh, err := placed.Tessellate(t.Context(), units.Millimeters(0.1))
			require.NoError(t, err)
			requireWatertight(t, placedMesh)
		})
	}
}

func TestStackedBlindCutPreservesCounterboreShoulder(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	plate := boxBody(t, doc, -10, -10, 10, 10, 10)
	through := circleBodyAtZ(t, doc, 0, 2, 0, 10)
	part, err := decad.Cut(t.Context(), plate, through)
	require.NoError(t, err)
	wide := circleBodyAtZ(t, doc, 0, 4, 7, 3)
	part, err = decad.Cut(t.Context(), part, wide)
	require.NoError(t, err)
	side := circleBodyAtZ(t, doc, 8, 1, 4, 6)
	part, err = decad.Cut(t.Context(), part, side)
	require.NoError(t, err)
	require.False(t, anyFaceIsFaceted(part))
	volume, err := part.Volume()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(volumeMM(t, volume)-(4000-82*math.Pi)), boundMM3(t, volume))
	mesh, err := part.Tessellate(t.Context(), units.Millimeters(0.1))
	require.NoError(t, err)
	require.True(t, mesh.VolumeVerified())
	requireWatertight(t, mesh)
}
