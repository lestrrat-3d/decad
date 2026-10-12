package decad

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestRevolveSectorCapFanRequiresAdjacentCompleteSourceRing(t *testing.T) {
	positions := map[int]int{10: 0, 11: 1, 12: 2, 13: 3}
	cell, fan, err := revolveSectorCapFan([3]int{2, 11, 10}, 7, 2, 7, positions, 4)
	require.NoError(t, err)
	require.True(t, fan)
	require.Equal(t, 0, cell)
	for _, tc := range []struct {
		tri   [3]int
		group int
	}{
		{tri: [3]int{2, 10, 12}, group: 7},
		{tri: [3]int{2, 10, 11}, group: 8},
		{tri: [3]int{2, 10, 99}, group: 7},
		{tri: [3]int{2, 2, 10}, group: 7},
	} {
		_, _, err := revolveSectorCapFan(tc.tri, tc.group, 2, 7, positions, 4)
		require.ErrorIs(t, err, ErrUnsupported)
	}
}

func TestRevolveSectorStraightBandCrossIsStrictAtBothRadii(t *testing.T) {
	outer, capPoint := Point2{U: 5, V: 2.5}, Point2{U: 6, V: 1.5}
	cross, ok := revolveSectorStraightBandCross(outer, capPoint, 2)
	require.True(t, ok)
	require.Equal(t, 5.5, cross.x)
	require.LessOrEqual(t, cross.round, 1e-14)
	require.Equal(t, 1.0, cross.slopeLo)
	require.Equal(t, 1.0, cross.slopeHi)
	for _, radius := range []float64{1.5, math.Nextafter(1.5, 0), 2.5, math.Nextafter(2.5, math.Inf(1))} {
		_, ok := revolveSectorStraightBandCross(outer, capPoint, radius)
		require.False(t, ok)
	}
}

func TestRevolveSectorTriangleClearChecksInteriorRadialDistance(t *testing.T) {
	verts := []r3.Vec{{Y: 3, Z: -3}, {Y: 3, Z: 3}, {Y: 5, Z: 0}}
	require.True(t, revolveSectorTriangleClear([3]int{0, 1, 2}, verts, 2.5))
	require.False(t, revolveSectorTriangleClear([3]int{0, 1, 2}, verts, 3.1))
	verts = []r3.Vec{{Y: 3}, {Z: 3}, {Y: 3, Z: 3}}
	require.False(t, revolveSectorTriangleClear([3]int{0, 1, 2}, verts, 2.5))
}
