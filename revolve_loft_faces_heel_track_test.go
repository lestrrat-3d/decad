package decad

import (
	"math/big"
	"testing"

	"github.com/stretchr/testify/require"
)

func TestCurvedHeelTrackUsesLiveMovingRoots(t *testing.T) {
	blank, tooth, _ := neutralArcSeamSource(t)
	proof, ok, err := certifyRevolveLoftFacesSource(t.Context(), blank, tooth)
	require.NoError(t, err)
	require.True(t, ok)
	budget := big.NewRat(1, 50)
	track, built, err := revolveLoftFacesHeelTrackOf(t.Context(), proof, budget)
	require.NoError(t, err)
	require.True(t, built)
	require.NotNil(t, track)
	require.Greater(t, len(track.stations), 2)
	require.LessOrEqual(t, track.calls, 256)
	require.True(t, revolveLoftFacesHeelTrackCapEnds(track,
		track.region.lower, track.region.upper))
	for index, station := range track.stations {
		require.True(t, revolveLoftFacesHeelTrackPlace(&station, budget))
		require.Equal(t, station.point[0].X, station.point[1].X)
		for event := range station.point {
			require.Less(t, station.sourceBound[event], 0.020000000001)
		}
		if index > 0 {
			require.Less(t, track.stations[index-1].point[0].X,
				station.point[0].X)
			for event := range station.box {
				for axis := range station.box[event] {
					previous := track.stations[index-1].box[event][axis]
					current := station.box[event][axis]
					require.LessOrEqual(t, previous.lo.Cmp(current.hi), 0)
					require.LessOrEqual(t, current.lo.Cmp(previous.hi), 0)
				}
			}
		}
	}
	var refused *revolveLoftFacesHeelTrack
	refused, built, err = revolveLoftFacesHeelTrackOf(t.Context(), proof, nil)
	require.NoError(t, err)
	require.False(t, built)
	require.Nil(t, refused)
}
