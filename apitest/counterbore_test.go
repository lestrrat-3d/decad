package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestCounterboreCutBuildsAnalyticShoulderInEitherOrder(t *testing.T) {
	t.Parallel()
	for _, throughFirst := range []bool{true, false} {
		for _, fromTop := range []bool{true, false} {
			name := map[bool]string{true: "through-first", false: "counterbore-first"}[throughFirst]
			name += map[bool]string{true: "/top", false: "/bottom"}[fromTop]
			t.Run(name, func(t *testing.T) {
				t.Parallel()
				doc := decad.New()
				plate := boxBody(t, doc, -10, -10, 10, 10, 10)
				counterboreZ := 0.0
				if fromTop {
					counterboreZ = 7
				}
				var part *decad.Body
				var err error
				if throughFirst {
					hole := circleBodyAtZ(t, doc, 0, 2, 0, 10)
					part, err = decad.Cut(t.Context(), plate, hole)
					require.NoError(t, err)
					counterbore := circleBodyAtZ(t, doc, 0, 4, counterboreZ, 3)
					part, err = decad.Cut(t.Context(), part, counterbore)
				} else {
					counterbore := circleBodyAtZ(t, doc, 0, 4, counterboreZ, 3)
					part, err = decad.Cut(t.Context(), plate, counterbore)
					require.NoError(t, err)
					hole := circleBodyAtZ(t, doc, 0, 2, 0, 10)
					part, err = decad.Cut(t.Context(), part, hole)
				}
				require.NoError(t, err)
				require.False(t, anyFaceIsFaceted(part))
				require.Len(t, part.Faces(), 9)
				volume, err := part.Volume()
				require.NoError(t, err)
				require.LessOrEqual(t, math.Abs(volumeMM(t, volume)-(4000-76*math.Pi)), boundMM3(t, volume))
				require.Less(t, boundMM3(t, volume), 1e-9)
				mesh, err := part.Tessellate(t.Context(), units.Millimeters(0.1))
				require.NoError(t, err)
				require.True(t, mesh.VolumeVerified())
				requireWatertight(t, mesh)
				rotation, err := r3.Rotation(r3.NewVec(1, 2, 3), units.Degrees(37))
				require.NoError(t, err)
				placed, err := part.Placed(t.Context(), rotation)
				require.NoError(t, err)
				placedVolume, err := placed.Volume()
				require.NoError(t, err)
				require.LessOrEqual(t,
					math.Abs(volumeMM(t, placedVolume)-(4000-76*math.Pi)), boundMM3(t, placedVolume))
				placedMesh, err := placed.Tessellate(t.Context(), units.Millimeters(0.1))
				require.NoError(t, err)
				requireWatertight(t, placedMesh)
			})
		}
	}
}

func TestCounterboreCutKeepsAnOutsideThroughHole(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	plate := boxBody(t, doc, -10, -10, 10, 10, 10)
	centerHole := circleBodyAtZ(t, doc, 0, 2, 0, 10)
	part, err := decad.Cut(t.Context(), plate, centerHole)
	require.NoError(t, err)
	sideHole := circleBodyAtZ(t, doc, 8, 1, 0, 10)
	part, err = decad.Cut(t.Context(), part, sideHole)
	require.NoError(t, err)
	counterbore := circleBodyAtZ(t, doc, 0, 4, 7, 3)
	part, err = decad.Cut(t.Context(), part, counterbore)
	require.NoError(t, err)
	require.False(t, anyFaceIsFaceted(part))
	require.Len(t, part.Faces(), 10)
	volume, err := part.Volume()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(volumeMM(t, volume)-(4000-86*math.Pi)), boundMM3(t, volume))
	mesh, err := part.Tessellate(t.Context(), units.Millimeters(0.1))
	require.NoError(t, err)
	requireWatertight(t, mesh)
}
