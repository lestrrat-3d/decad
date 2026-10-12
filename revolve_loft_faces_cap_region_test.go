package decad

import (
	"math/big"
	"testing"

	"github.com/stretchr/testify/require"
)

func TestCurvedHeelRegionCertifiesContinuousSource(t *testing.T) {
	blank, tooth, _ := neutralArcSeamSource(t)
	proof, ok, err := certifyRevolveLoftFacesSource(t.Context(), blank, tooth)
	require.NoError(t, err)
	require.True(t, ok)

	region, ok, err := certifyRevolveLoftFacesHeelRegion(t.Context(), proof)
	require.NoError(t, err)
	require.True(t, ok)
	require.NotNil(t, region)
	require.Equal(t, 10, len(region.slices))
	require.Len(t, region.blank, 2)
	require.Equal(t, region.lower.band+1, region.upper.band)
	require.Equal(t, region.lower.circle.x.Cmp(region.blank[0].lo), 0)
	require.Equal(t, region.upper.circle.x.Cmp(region.blank[len(region.blank)-1].hi), 0)
	for _, slice := range region.slices {
		require.Equal(t, -1, slice.events[0].sign)
		require.Equal(t, 1, slice.events[1].sign)
	}
	loop, simple, err := revolveLoftFacesSimpleSourceLoop(t.Context(), proof.trim)
	require.NoError(t, err)
	require.True(t, simple)
	loopIndex := make(map[revolveLoftFacesHeelSourceKey]int, len(loop.cells))
	for index, source := range loop.cells {
		key := revolveLoftFacesHeelSourceKey{source.segment, source.span}
		if source.arc != nil {
			key.span = -1
		}
		loopIndex[key] = index
	}
	jointFound := false
	for _, slice := range region.slices {
		for _, event := range slice.events {
			if event.kind != revolveLoftFacesLineFitContact {
				continue
			}
			jointFound = true
			require.True(t, revolveLoftFacesHeelRegionJointCrosses(proof.trim,
				region.meridian, loopIndex, event))
			changed := event
			changed.sign = -event.sign
			require.False(t, revolveLoftFacesHeelRegionJointCrosses(proof.trim,
				region.meridian, loopIndex, changed))
		}
	}
	require.True(t, jointFound)
	cells, ok, err := revolveLoftFacesFiniteSpanCells(t.Context(), proof.trim, region.meridian)
	require.NoError(t, err)
	require.True(t, ok)
	for index := range cells {
		if cells[index].band == region.lower.band &&
			cells[index].side == revolveLoftFacesCrossed &&
			revolveLoftFacesTimeInCell(region.lower.time, cells[index]) {
			cells[index].side = revolveLoftFacesInside
			break
		}
	}
	_, ok = revolveLoftFacesHeelRegionSliceAt(proof.trim, loop, region.meridian,
		cells, region.lower.band, region.lower.time, lfOne(), false)
	require.False(t, ok, "a missing true crossing must not certify a cap region")
}

func TestCurvedHeelRegionRejectsChangedBlankSection(t *testing.T) {
	blank, tooth, _ := neutralArcSeamSource(t)
	proof, ok, err := certifyRevolveLoftFacesSource(t.Context(), blank, tooth)
	require.NoError(t, err)
	require.True(t, ok)
	region, ok, err := certifyRevolveLoftFacesHeelRegion(t.Context(), proof)
	require.NoError(t, err)
	require.True(t, ok)
	require.NotNil(t, region)
	changedArc := *region.lower
	changedArc.time = lfAdd(changedArc.time, big.NewRat(1, 1000))
	blankSlices, current := revolveLoftFacesHeelRegionBlankInside(t.Context(), proof,
		&changedArc, region.upper)
	require.False(t, current)
	require.Nil(t, blankSlices)

	changed := *proof
	changed.section = proof.section
	changed.section.spans = append([]revolveSectionSpan(nil), proof.section.spans...)
	changed.section.spans[region.blank[0].outer].end.V += 0.125
	rejected, ok, err := certifyRevolveLoftFacesHeelRegion(t.Context(), &changed)
	require.NoError(t, err)
	require.False(t, ok)
	require.Nil(t, rejected)
}

func TestCurvedHeelRegionRejectsStaleSource(t *testing.T) {
	blank, trimmed, trim := neutralArcSeamSource(t)
	proof, ok, err := certifyRevolveLoftFacesSource(t.Context(), blank, trimmed)
	require.NoError(t, err)
	require.True(t, ok)

	originalRevision := trim.source.source.revision
	trim.source.source.revision++
	defer func() { trim.source.source.revision = originalRevision }()
	region, ok, err := certifyRevolveLoftFacesHeelRegion(t.Context(), proof)
	require.NoError(t, err)
	require.False(t, ok)
	require.Nil(t, region)
}
