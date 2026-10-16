package decad

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/revolveshell"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestRevolveSectorStraightBandCrossIsStrictAtBothRadii(t *testing.T) {
	outer, capPoint := planePoint{U: 5, V: 2.5}, planePoint{U: 6, V: 1.5}
	cross, ok := revolveshell.RevolveSectorStraightBandCross(outer, capPoint, 2)
	require.True(t, ok)
	require.Equal(t, 5.5, cross.X)
	require.LessOrEqual(t, cross.Round, 1e-14)
	require.Equal(t, 1.0, cross.SlopeLo)
	require.Equal(t, 1.0, cross.SlopeHi)
	for _, radius := range []float64{1.5, math.Nextafter(1.5, 0), 2.5, math.Nextafter(2.5, math.Inf(1))} {
		_, ok := revolveshell.RevolveSectorStraightBandCross(outer, capPoint, radius)
		require.False(t, ok)
	}
}

func TestRevolveSectorTriangleClearChecksInteriorRadialDistance(t *testing.T) {
	verts := []r3.Vec{{Y: 3, Z: -3}, {Y: 3, Z: 3}, {Y: 5, Z: 0}}
	require.True(t, revolveshell.RevolveSectorTriangleClear([3]int{0, 1, 2}, verts, 2.5))
	require.False(t, revolveshell.RevolveSectorTriangleClear([3]int{0, 1, 2}, verts, 3.1))
	verts = []r3.Vec{{Y: 3}, {Z: 3}, {Y: 3, Z: 3}}
	require.False(t, revolveshell.RevolveSectorTriangleClear([3]int{0, 1, 2}, verts, 2.5))
}
