package decad

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/twistsweep"
	"github.com/stretchr/testify/require"
)

func TestStationedTwistOptionalRefinementKeepsOriginalWorkCeiling(t *testing.T) {
	poly := twistsweep.TwistPolygon{
		Points:      make([]planePoint, 8),
		HasArc:      true,
		MaxRadiusL1: 7,
		MaxEdgeL1:   5,
	}
	base, refined, err := twistsweep.StationedTwistCellCounts(poly, 0.5235987755982988)
	require.NoError(t, err)
	require.Greater(t, refined, base)

	base, refined, err = twistsweep.StationedTwistCellCounts(poly, 6)
	require.NoError(t, err)
	require.Greater(t, base, 2)
	require.Equal(t, base, refined)
}
