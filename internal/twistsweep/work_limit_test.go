package twistsweep_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/twistsweep"
	"github.com/stretchr/testify/require"
)

func TestStationedTwistWorkLimitDetails(t *testing.T) {
	poly := twistsweep.TwistPolygon{
		Points: make([]sectionrecord.Point2, 1000), MaxRadiusL1: 1,
	}
	_, _, err := twistsweep.StationedTwistCellCounts(poly, 0.5)
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
	var detail *proofbound.WorkLimitError
	require.ErrorAs(t, err, &detail)
	require.Equal(t, "facet_pairs", detail.Resource)
	require.Equal(t, uint64(17_973_010), detail.Observed)
	require.Equal(t, uint64(8_000_000), detail.Maximum)
}
