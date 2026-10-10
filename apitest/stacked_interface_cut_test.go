package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestStackedBlindCutReusesExactInterface(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name string
		z    float64
	}{
		{name: "top", z: 6},
		{name: "bottom", z: 0},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc := decad.New()
			plate := boxBody(t, doc, -10, -10, 10, 10, 10)
			first := boxBodyAtZ(t, doc, -6, -2, -2, 2, tc.z, 4)
			part, err := decad.Cut(t.Context(), plate, first)
			require.NoError(t, err)
			second := boxBodyAtZ(t, doc, 2, -2, 6, 2, tc.z, 4)
			part, err = decad.Cut(t.Context(), part, second)
			require.NoError(t, err)
			require.False(t, anyFaceIsFaceted(part))
			volume, err := part.Volume()
			require.NoError(t, err)
			require.Equal(t, 3872.0, volumeMM(t, volume))
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
			require.LessOrEqual(t, math.Abs(volumeMM(t, placedVolume)-3872), boundMM3(t, placedVolume))
			placedMesh, err := placed.Tessellate(t.Context(), units.Millimeters(0.1))
			require.NoError(t, err)
			require.True(t, placedMesh.VolumeVerified())
			requireWatertight(t, placedMesh)
		})
	}
}
