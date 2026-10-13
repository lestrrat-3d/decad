package decad

import (
	"math/big"
	"testing"

	"github.com/stretchr/testify/require"
)

func TestCurvedHeelCapEndpointsHaveSourceBounds(t *testing.T) {
	blank, trimmed, _ := neutralArcSeamSource(t)
	proof, ok, err := certifyRevolveLoftFacesSource(t.Context(), blank, trimmed)
	require.NoError(t, err)
	require.True(t, ok)
	census, ok, err := revolveLoftFacesCensus(t.Context(), proof)
	require.NoError(t, err)
	require.True(t, ok)
	arcs, endpoints := 0, 0
	checkedMutation := false
	for _, pair := range census.pairs {
		if pair.verdict != revolveLoftFacesPairToolArc {
			continue
		}
		arcs++
		vertices, built, buildErr := revolveLoftFacesCapArcSeamVertices(
			t.Context(), proof, pair.toolArc)
		require.NoError(t, buildErr)
		require.True(t, built)
		for i, vertex := range vertices {
			endpoints++
			require.Less(t, vertex.blankBound, 0.1)
			require.Less(t, vertex.loftBound, 0.1)
			require.Greater(t, vertex.point.Y, 0.0)
			if i == 0 {
				require.Less(t, vertex.point.Z, 0.0)
			} else {
				require.Greater(t, vertex.point.Z, 0.0)
			}
			t.Logf("heel endpoint section=%d side=%d bound=%.9g mm",
				pair.sectionIndex, i, vertex.blankBound)
		}
		if !checkedMutation {
			changedTime := *pair.toolArc
			changedTime.time = lfAdd(changedTime.time, big.NewRat(1, 1<<20))
			_, accepted, mutationErr := revolveLoftFacesCapArcSeamVertices(
				t.Context(), proof, &changedTime)
			require.NoError(t, mutationErr)
			require.False(t, accepted)
			changedBox := *pair.toolArc
			changedBox.negativeZ.box[1].Lo = lfAdd(
				changedBox.negativeZ.box[1].Lo, big.NewRat(1, 1<<20))
			_, accepted, mutationErr = revolveLoftFacesCapArcSeamVertices(
				t.Context(), proof, &changedBox)
			require.NoError(t, mutationErr)
			require.False(t, accepted)
			checkedMutation = true
		}
	}
	require.Equal(t, 2, arcs)
	require.Equal(t, 4, endpoints)
	require.True(t, checkedMutation)
}
