package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestCrossingBlindPocketOnThroughHole(t *testing.T) {
	for _, tc := range []struct {
		name       string
		pocketY0   float64
		pocketY1   float64
		wantVolume float64
		wantX      float64
		wantY      float64
		wantZ      float64
	}{
		{name: "shared-line", pocketY0: -2, pocketY1: 2, wantVolume: 3792,
			wantX: 248.0 / 3792, wantY: 0, wantZ: 18816.0 / 3792},
		{name: "crossing-corners", pocketY0: -1, pocketY1: 3, wantVolume: 3788,
			wantX: 250.0 / 3788, wantY: -58.0 / 3788, wantZ: 18784.0 / 3788},
	} {
		t.Run(tc.name, func(t *testing.T) {
			doc := decad.New()
			plate := boxBody(t, doc, -10, -10, 10, 10, 10)
			through := boxBodyAtZ(t, doc, -4, -2, 0, 2, 0, 10)
			part, err := decad.Cut(t.Context(), plate, through)
			require.NoError(t, err)
			pocket := boxBodyAtZ(t, doc, -1, tc.pocketY0, 3, tc.pocketY1, 6, 4)
			part, err = decad.Cut(t.Context(), part, pocket)
			require.NoError(t, err)
			require.False(t, anyFaceIsFaceted(part))
			requireEveryEdgeOnTwoFaces(t, part)
			volume, err := part.Volume()
			require.NoError(t, err)
			require.Equal(t, decad.Exact, volume.Exactness)
			require.Equal(t, tc.wantVolume, volumeMM(t, volume))
			centroid, err := part.Centroid()
			require.NoError(t, err)
			for _, axis := range []struct{ got, want float64 }{
				{centroid.Value.X, tc.wantX},
				{centroid.Value.Y, tc.wantY},
				{centroid.Value.Z, tc.wantZ},
			} {
				require.LessOrEqual(t, math.Abs(axis.got-axis.want), centroid.Bound.Base())
			}
			mesh, err := part.Tessellate(t.Context(), units.Millimeters(0.1),
				decad.WithVerification(decad.VerifyAll))
			require.NoError(t, err)
			require.True(t, mesh.VolumeVerified())
			requireWatertight(t, mesh)
			report, err := doc.Verify(t.Context())
			require.NoError(t, err)
			require.Equal(t, decad.Sound, report.Status)
		})
	}
}
