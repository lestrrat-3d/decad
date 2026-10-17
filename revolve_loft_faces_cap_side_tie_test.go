package decad

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/revolveloftfaces"
	"github.com/stretchr/testify/require"
)

func TestCurvedHeelArcEndpointsBindToLiveSideCells(t *testing.T) {
	blank, trimmed, _ := neutralArcSeamSource(t)
	proof, ok, err := certifyRevolveLoftFacesSource(t.Context(), blank, trimmed)
	require.NoError(t, err)
	require.True(t, ok)
	census, ok, err := revolveLoftFacesCensus(t.Context(), proof)
	require.NoError(t, err)
	require.True(t, ok)
	count := 0
	verticalCount := 0
	slopedCount := 0
	for _, pair := range census.pairs {
		if pair.verdict != revolveLoftFacesPairToolArc {
			continue
		}
		ties, certified, err := revolveLoftFacesHeelArcSideTiesOf(t.Context(), proof, pair.toolArc)
		require.NoError(t, err)
		require.True(t, certified, "blank section %d", pair.sectionIndex)
		require.Equal(t, pair.toolArc.negativeZ.ref, ties.negativeZ.boundary.ref)
		require.Equal(t, pair.toolArc.positiveZ.ref, ties.positiveZ.boundary.ref)
		for _, tie := range [...]revolveLoftFacesCapSideTie{ties.negativeZ, ties.positiveZ} {
			require.NotNil(t, tie.boundary.face)
			require.True(t, tie.sourceCell != nil || tie.verticalCell != nil)
			require.False(t, tie.sourceCell != nil && tie.verticalCell != nil)
			if tie.verticalCell != nil {
				verticalCount++
			} else {
				slopedCount++
			}
		}
		count++
	}
	require.Equal(t, 2, count)
	require.Equal(t, 2, verticalCount)
	require.Equal(t, 2, slopedCount)
}

func TestCurvedHeelArcSideTiesRejectChangedSourceRecord(t *testing.T) {
	blank, trimmed, _ := neutralArcSeamSource(t)
	proof, ok, err := certifyRevolveLoftFacesSource(t.Context(), blank, trimmed)
	require.NoError(t, err)
	require.True(t, ok)
	census, ok, err := revolveLoftFacesCensus(t.Context(), proof)
	require.NoError(t, err)
	require.True(t, ok)
	for _, pair := range census.pairs {
		if pair.verdict != revolveLoftFacesPairToolArc {
			continue
		}
		arc := *pair.toolArc
		arc.time = new(big.Rat).Add(arc.time, big.NewRat(1, 1<<20))
		ties, certified, err := revolveLoftFacesHeelArcSideTiesOf(t.Context(), proof, &arc)
		require.NoError(t, err)
		require.False(t, certified)
		require.Nil(t, ties)

		arc = *pair.toolArc
		arc.negativeZ.parameter.Lo = new(big.Rat).Add(
			arc.negativeZ.parameter.Lo, big.NewRat(1, 1<<60))
		ties, certified, err = revolveLoftFacesHeelArcSideTiesOf(t.Context(), proof, &arc)
		require.NoError(t, err)
		require.False(t, certified)
		require.Nil(t, ties)
	}
}

func TestCurvedHeelArcSideTieTimeEndpointOwnership(t *testing.T) {
	cell := revolveloftfaces.RevolveLoftFacesMappedCell{TLo: big.NewRat(1, 4), THi: big.NewRat(3, 4)}
	require.True(t, revolveLoftFacesTimeInCell(big.NewRat(1, 4), cell))
	require.True(t, revolveLoftFacesTimeInCell(big.NewRat(1, 2), cell))
	require.True(t, revolveLoftFacesTimeInCell(big.NewRat(3, 4), cell))
	cell.TLoOpen, cell.THiOpen = true, true
	require.False(t, revolveLoftFacesTimeInCell(big.NewRat(1, 4), cell))
	require.True(t, revolveLoftFacesTimeInCell(big.NewRat(1, 2), cell))
	require.False(t, revolveLoftFacesTimeInCell(big.NewRat(3, 4), cell))
}
