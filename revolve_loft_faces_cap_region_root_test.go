package decad

import (
	"math/big"
	"testing"

	"github.com/stretchr/testify/require"
)

func TestCurvedHeelRegionBoxesLiveMovingJoint(t *testing.T) {
	blank, tooth, _ := neutralArcSeamSource(t)
	proof, ok, err := certifyRevolveLoftFacesSource(t.Context(), blank, tooth)
	require.NoError(t, err)
	require.True(t, ok)
	region, ok, err := revolveLoftFacesHeelRegionEventGraph(t.Context(), proof)
	require.NoError(t, err)
	require.True(t, ok)
	budget := big.NewRat(1, 50) // 0.02 mm L1 corner radius
	joint := false
	for index, slice := range region.slices {
		for eventIndex, event := range slice.events {
			if event.kind != revolveLoftFacesLineFitContact {
				continue
			}
			joint = true
			box, certified, boxErr := revolveLoftFacesHeelEventBox(t.Context(), proof,
				region, index, eventIndex, slice.lo, slice.hi, budget)
			require.NoError(t, boxErr)
			require.True(t, certified, "slice %d", index)
			require.NotEmpty(t, box.members)
			require.LessOrEqual(t, len(box.members), 2)
			require.LessOrEqual(t, revolveLoftFacesHeelBoxRadius(box.box).Cmp(budget), 0)
			for _, member := range box.members {
				require.Equal(t, slice.band, member.band)
				require.NotNil(t, member.face)
			}
		}
	}
	require.True(t, joint)
}

func TestCurvedHeelRegionBoxRejectsChangedStoredCell(t *testing.T) {
	blank, tooth, _ := neutralArcSeamSource(t)
	proof, ok, err := certifyRevolveLoftFacesSource(t.Context(), blank, tooth)
	require.NoError(t, err)
	require.True(t, ok)
	region, ok, err := revolveLoftFacesHeelRegionEventGraph(t.Context(), proof)
	require.NoError(t, err)
	require.True(t, ok)
	for index, slice := range region.slices {
		for eventIndex, event := range slice.events {
			if event.kind != revolveLoftFacesLineFitContact {
				continue
			}
			changed := *region
			changed.slices = append([]revolveLoftFacesHeelRegionSlice(nil), region.slices...)
			changed.slices[index].events[eventIndex].members =
				append([]revolveLoftFacesMappedCell(nil), event.members...)
			changed.slices[index].events[eventIndex].members[0].sLo = big.NewRat(1, 2)
			box, certified, boxErr := revolveLoftFacesHeelEventBox(t.Context(), proof,
				&changed, index, eventIndex, slice.lo, slice.hi, big.NewRat(1, 50))
			require.NoError(t, boxErr)
			require.False(t, certified)
			require.Nil(t, box)

			changed.slices[index] = slice
			changed.slices[index].lo = lfAdd(slice.lo, big.NewRat(1, 1<<20))
			box, certified, boxErr = revolveLoftFacesHeelEventBox(t.Context(), proof,
				&changed, index, eventIndex, slice.lo, slice.hi, big.NewRat(1, 50))
			require.NoError(t, boxErr)
			require.False(t, certified)
			require.Nil(t, box)

			originalRevision := proof.trim.source.source.revision
			proof.trim.source.source.revision++
			defer func() { proof.trim.source.source.revision = originalRevision }()
			box, certified, boxErr = revolveLoftFacesHeelEventBox(t.Context(), proof,
				region, index, eventIndex, slice.lo, slice.hi, big.NewRat(1, 50))
			require.NoError(t, boxErr)
			require.False(t, certified)
			require.Nil(t, box)
			return
		}
	}
	t.Fatal("the real neutral heel cap has no line/Fit event")
}

func TestCurvedHeelRegionBoxesNeutralEventStations(t *testing.T) {
	blank, tooth, _ := neutralArcSeamSource(t)
	proof, ok, err := certifyRevolveLoftFacesSource(t.Context(), blank, tooth)
	require.NoError(t, err)
	require.True(t, ok)
	region, ok, err := revolveLoftFacesHeelRegionEventGraph(t.Context(), proof)
	require.NoError(t, err)
	require.True(t, ok)
	for index, slice := range region.slices {
		for eventIndex, event := range slice.events {
			station := slice.lo
			if !slice.point {
				station = lfDiv(lfAdd(slice.lo, slice.hi), big.NewRat(2, 1))
			}
			box, certified, boxErr := revolveLoftFacesHeelEventBoxFromCurrent(t.Context(),
				proof, region, index, eventIndex, station, station, big.NewRat(1, 50))
			require.NoError(t, boxErr)
			require.True(t, certified, "slice %d event %d kind %d member %d/%d",
				index, eventIndex, event.kind, event.members[0].segment, event.members[0].span)
			require.NotNil(t, box)
		}
	}
}

func TestCurvedHeelRegionBoxRefusesBroadTimeBudget(t *testing.T) {
	blank, tooth, _ := neutralArcSeamSource(t)
	proof, ok, err := certifyRevolveLoftFacesSource(t.Context(), blank, tooth)
	require.NoError(t, err)
	require.True(t, ok)
	region, ok, err := revolveLoftFacesHeelRegionEventGraph(t.Context(), proof)
	require.NoError(t, err)
	require.True(t, ok)
	require.Greater(t, len(region.slices), 1)
	slice := region.slices[1]
	require.False(t, slice.point)
	box, certified, boxErr := revolveLoftFacesHeelEventBoxFromCurrent(t.Context(),
		proof, region, 1, 0, slice.lo, slice.hi, big.NewRat(1, 50))
	require.NoError(t, boxErr)
	require.False(t, certified)
	require.Nil(t, box)
	station := lfDiv(lfAdd(slice.lo, slice.hi), big.NewRat(2, 1))
	box, certified, boxErr = revolveLoftFacesHeelEventBoxFromCurrent(t.Context(),
		proof, region, 1, 0, station, station, big.NewRat(1, 50))
	require.NoError(t, boxErr)
	require.True(t, certified)
	require.NotNil(t, box)
}
