package decad

import (
	"testing"

	"github.com/stretchr/testify/require"
)

func TestStationedTwistOptionalRefinementKeepsOriginalWorkCeiling(t *testing.T) {
	poly := twistPolygon{
		points:      make([]planePoint, 8),
		hasArc:      true,
		maxRadiusL1: 7,
		maxEdgeL1:   5,
	}
	base, refined, err := stationedTwistCellCounts(poly, 0.5235987755982988)
	require.NoError(t, err)
	require.Greater(t, refined, base)

	base, refined, err = stationedTwistCellCounts(poly, 6)
	require.NoError(t, err)
	require.Greater(t, base, 2)
	require.Equal(t, base, refined)
}
