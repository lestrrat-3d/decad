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

	changed := *proof
	changed.section = proof.section
	changed.section.spans = append([]revolveSectionSpan(nil), proof.section.spans...)
	changed.section.spans[track.region.blank[0].outer].end.V += 0.125
	refused, built, err = revolveLoftFacesHeelTrackOf(t.Context(), &changed, budget)
	require.NoError(t, err)
	require.False(t, built)
	require.Nil(t, refused)

	originalRevision := proof.trim.source.source.revision
	proof.trim.source.source.revision++
	defer func() { proof.trim.source.source.revision = originalRevision }()
	refused, built, err = revolveLoftFacesHeelTrackOf(t.Context(), proof, budget)
	require.NoError(t, err)
	require.False(t, built)
	require.Nil(t, refused)
}

func TestCurvedHeelTrackRefusesRoundedAwaySides(t *testing.T) {
	tiny := new(big.Rat).SetFrac(big.NewInt(1),
		new(big.Int).Lsh(big.NewInt(1), 1075))
	value := func(v *big.Rat) loftFaceInterval {
		return loftFaceInterval{lo: v, hi: v}
	}
	for _, test := range []struct {
		name   string
		y0, z0 *big.Rat
	}{
		{"positive Y rounds to zero", tiny, lfOne()},
		{"positive Z rounds to zero", lfOne(), tiny},
	} {
		t.Run(test.name, func(t *testing.T) {
			station := revolveLoftFacesHeelTrackStation{box: [2][3]loftFaceInterval{
				{value(lfOne()), value(test.y0), value(test.z0)},
				{value(lfOne()), value(lfOne()), value(big.NewRat(-1, 1))},
			}}
			require.False(t, revolveLoftFacesHeelTrackPlace(&station, big.NewRat(1, 50)))
		})
	}
}
