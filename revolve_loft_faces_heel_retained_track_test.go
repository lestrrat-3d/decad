package decad

import (
	"math/big"
	"testing"

	"github.com/stretchr/testify/require"
)

func TestCurvedRetainedHeelTracksUseLiveSourceOwners(t *testing.T) {
	blank, tooth, _ := neutralArcSeamSource(t)
	proof, ok, err := certifyRevolveLoftFacesSource(t.Context(), blank, tooth)
	require.NoError(t, err)
	require.True(t, ok)
	budget := big.NewRat(1, 50)
	tracks, ok, err := revolveLoftFacesHeelRetainedTracksOf(t.Context(), proof,
		budget)
	require.NoError(t, err)
	require.True(t, ok)
	for _, track := range []*revolveLoftFacesHeelRetainedTrack{
		tracks.birth, tracks.terminal} {
		require.NotEmpty(t, track.stations)
		require.NotEmpty(t, track.slabs)
		require.NotEmpty(t, track.arc)
		for _, bracket := range track.bracket {
			require.NotNil(t, bracket)
			require.NotEmpty(t, bracket.members)
			require.LessOrEqual(t,
				revolveLoftFacesHeelBoxRadius(bracket.box).Cmp(budget), 0)
		}
		for _, slab := range track.slabs {
			require.LessOrEqual(t, slab.time.lo.Cmp(slab.time.hi), 0)
			for _, root := range slab.root {
				require.NotNil(t, root)
				require.NotEmpty(t, root.members)
				require.Less(t,
					revolveLoftFacesHeelBoxRadius(root.box).Cmp(big.NewRat(1, 50)), 1)
				for _, member := range root.members {
					require.NotNil(t, member.face)
					require.Equal(t, proof.sourceRefs[member.band][member.segment], member.ref)
				}
			}
		}
	}
	t.Logf("retained heel source tracks: birth %d slabs/%d stations, terminal %d slabs/%d stations",
		len(tracks.birth.slabs), len(tracks.birth.stations),
		len(tracks.terminal.slabs), len(tracks.terminal.stations))
	for _, track := range []*revolveLoftFacesHeelRetainedTrack{tracks.birth, tracks.terminal} {
		first, last := track.stations[0], track.stations[len(track.stations)-1]
		t.Logf("track endpoints: first=%v/%v last=%v/%v arc=%v..%v",
			first.point[0], first.point[1], last.point[0], last.point[1],
			track.arc[0].point, track.arc[len(track.arc)-1].point)
	}
}
