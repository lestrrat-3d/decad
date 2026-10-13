package decad

import (
	"math/big"
	"testing"

	"github.com/stretchr/testify/require"
)

func TestCurvedHeelArcSideTiesUseCallerCensus(t *testing.T) {
	blank, trimmed, _ := neutralArcSeamSource(t)
	proof, ok, err := certifyRevolveLoftFacesSource(t.Context(), blank, trimmed)
	require.NoError(t, err)
	require.True(t, ok)
	graph, ok, err := revolveLoftFacesContactGraphOf(t.Context(), proof)
	require.NoError(t, err)
	require.True(t, ok)
	require.Len(t, graph.arcs, 2)

	fresh := make([]*revolveLoftFacesHeelArcSideTies, len(graph.arcs))
	for i := range graph.arcs {
		arc := &graph.arcs[i]
		ties, tied, err := revolveLoftFacesHeelArcSideTiesOf(t.Context(), proof, arc.arc)
		require.NoError(t, err)
		require.True(t, tied)
		fresh[i] = ties
		for endpoint, tie := range [...]revolveLoftFacesCapSideTie{ties.negativeZ, ties.positiveZ} {
			id, found := -1, false
			if tie.sourceCell != nil {
				id, found = graph.sourceCellID(arc.sectionIndex, arc.arc.circle.blankFace, *tie.sourceCell)
			} else if tie.verticalCell != nil {
				id, found = graph.verticalCellID(arc.sectionIndex, arc.arc.circle.blankFace, *tie.verticalCell)
			}
			require.True(t, found)
			require.Equal(t, arc.endpointCellIDs[endpoint], id, "arc %d endpoint %d", i, endpoint)
		}
	}

	census, ok, err := revolveLoftFacesCensus(t.Context(), proof)
	require.NoError(t, err)
	require.True(t, ok)
	for i := range graph.arcs {
		arc := graph.arcs[i].arc
		ties, tied, err := revolveLoftFacesHeelArcSideTiesIn(t.Context(), proof, census, arc)
		require.NoError(t, err)
		require.True(t, tied)
		want := [...]revolveLoftFacesCapSideTie{fresh[i].negativeZ, fresh[i].positiveZ}
		for endpoint, got := range [...]revolveLoftFacesCapSideTie{ties.negativeZ, ties.positiveZ} {
			require.True(t, revolveLoftFacesToolCapBoundaryEqual(want[endpoint].boundary, got.boundary))
			require.True(t, (got.sourceCell != nil) != (got.verticalCell != nil))
			require.Equal(t, want[endpoint].sourceCell != nil, got.sourceCell != nil)
			if got.sourceCell != nil {
				require.True(t, revolveLoftFacesSameMappedCell(want[endpoint].sourceCell.mapped, got.sourceCell.mapped))
				require.Equal(t, want[endpoint].sourceCell.ref, got.sourceCell.ref)
			} else {
				require.True(t, revolveLoftFacesGraphSameVerticalCell(*want[endpoint].verticalCell, *got.verticalCell))
			}
		}

		ties, tied, err = revolveLoftFacesHeelArcSideTiesIn(t.Context(), proof,
			&revolveLoftFacesPairCensus{}, arc)
		require.NoError(t, err)
		require.False(t, tied)
		require.Nil(t, ties)

		// The verdicts come from the census passed in, not from a fresh one.
		noPairs := *census
		noPairs.pairs = nil
		ties, tied, err = revolveLoftFacesHeelArcSideTiesIn(t.Context(), proof, &noPairs, arc)
		require.NoError(t, err)
		require.False(t, tied)
		require.Nil(t, ties)

		moved := *arc
		moved.time = new(big.Rat).Add(arc.time, big.NewRat(1, 1<<20))
		ties, tied, err = revolveLoftFacesHeelArcSideTiesIn(t.Context(), proof, census, &moved)
		require.NoError(t, err)
		require.False(t, tied)
		require.Nil(t, ties)
	}
}
