package decad

import (
	"testing"

	"github.com/stretchr/testify/require"
)

func TestCurvedLoftFacesArcSeamPatchesShareCertifiedVertices(t *testing.T) {
	blank, trimmed, trim := neutralArcSeamSource(t)
	sectionIndex, contact, stations := neutralArcSeamContact(t, blank, trimmed, trim)
	seam, blankPatch, loftPatch, ok, err := revolveLoftFacesCertifiedArcPatches(t.Context(),
		blank, trimmed, sectionIndex, contact.segment, contact.band,
		loftFaceInterval{Lo: contact.tLo, Hi: contact.tHi}, stations)
	require.NoError(t, err)
	require.True(t, ok)
	require.Len(t, blankPatch.triangles, 4*(len(stations)-1))
	require.Equal(t, len(blankPatch.triangles), len(loftPatch.triangles))
	maxBlank, maxLoft := 0.0, 0.0
	for index := range seam {
		require.Equal(t, blankPatch.vertices[blankPatch.seam[index]],
			loftPatch.vertices[loftPatch.seam[index]])
		require.Less(t, seam[index].blankBound, 0.1)
		require.Less(t, seam[index].loftBound, 0.1)
		maxBlank = max(maxBlank, seam[index].blankBound)
		maxLoft = max(maxLoft, seam[index].loftBound)
	}
	for _, bound := range blankPatch.vertexBounds {
		require.Less(t, bound, 0.1)
	}
	for _, bound := range loftPatch.vertexBounds {
		require.Less(t, bound, 0.1)
	}
	t.Logf("source-aligned radial seam: %d stations, maximum vertex bounds blank %.9g mm, loft %.9g mm",
		len(seam), maxBlank, maxLoft)
}

func neutralArcSeamContact(t *testing.T, blank, trimmed *Body, trim *loftFacesTrimRecord) (
	int, revolveLoftFacesMappedCell, []int,
) {
	t.Helper()
	proof, ok, err := certifyRevolveLoftFacesSource(t.Context(), blank, trimmed)
	require.NoError(t, err)
	require.True(t, ok)
	census, ok, err := revolveLoftFacesCensus(t.Context(), proof)
	require.NoError(t, err)
	require.True(t, ok)

	var section revolveSectionSpan
	sectionIndex := -1
	var contact revolveLoftFacesMappedCell
	found := false
	for _, pair := range census.pairs {
		if pair.verdict != revolveLoftFacesPairSourceCandidate {
			continue
		}
		for _, cell := range pair.sourceCells {
			mapped := cell.mapped
			if mapped.side != revolveLoftFacesJointContact ||
				mapped.jointKind != revolveLoftFacesWholeArcContact || found {
				continue
			}
			section = proof.section.Spans[pair.sectionIndex]
			sectionIndex = pair.sectionIndex
			contact = mapped
			found = true
		}
	}
	require.True(t, found)
	require.NotNil(t, section.Face)
	require.NotNil(t, contact.jointArc)
	require.NotNil(t, contact.tLo)
	require.NotNil(t, contact.tHi)
	var stations []int
	for index, group := range trim.StationGroup {
		if group == contact.segment {
			stations = append(stations, index)
		}
	}
	require.NotEmpty(t, stations)
	stations = append(stations, (stations[len(stations)-1]+1)%len(trim.StationGroup))
	return sectionIndex, contact, stations
}

func TestCurvedArcSeamRefusesChangedStationAndIncompleteRun(t *testing.T) {
	t.Run("changed station", func(t *testing.T) {
		blank, trimmed, trim := neutralArcSeamSource(t)
		sectionIndex, contact, stations := neutralArcSeamContact(t, blank, trimmed, trim)
		trim.Source.SourceStation[stations[1]].U += 0.001
		require.False(t, pointSourceCurrent(trim.Source))
		_, _, _, ok, err := revolveLoftFacesCertifiedArcPatches(t.Context(), blank, trimmed,
			sectionIndex, contact.segment, contact.band,
			loftFaceInterval{Lo: contact.tLo, Hi: contact.tHi}, stations)
		require.NoError(t, err)
		require.False(t, ok)
	})
	t.Run("changed station bound", func(t *testing.T) {
		blank, trimmed, trim := neutralArcSeamSource(t)
		sectionIndex, contact, stations := neutralArcSeamContact(t, blank, trimmed, trim)
		require.Positive(t, trim.Source.SourceStationRound)
		trim.Source.SourceStationRound = 0
		require.False(t, pointSourceCurrent(trim.Source))
		_, _, _, ok, err := revolveLoftFacesCertifiedArcPatches(t.Context(), blank, trimmed,
			sectionIndex, contact.segment, contact.band,
			loftFaceInterval{Lo: contact.tLo, Hi: contact.tHi}, stations)
		require.NoError(t, err)
		require.False(t, ok)
	})
	t.Run("truncated arc run", func(t *testing.T) {
		blank, trimmed, trim := neutralArcSeamSource(t)
		sectionIndex, contact, stations := neutralArcSeamContact(t, blank, trimmed, trim)
		require.Greater(t, len(stations), 2)
		_, _, _, ok, err := revolveLoftFacesCertifiedArcPatches(t.Context(), blank, trimmed,
			sectionIndex, contact.segment, contact.band,
			loftFaceInterval{Lo: contact.tLo, Hi: contact.tHi}, stations[:2])
		require.NoError(t, err)
		require.False(t, ok)
	})
}
