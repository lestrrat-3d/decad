package decad

import (
	"math/big"
	"testing"

	"github.com/stretchr/testify/require"
)

func TestRevolveLoftFacesToolCapArcBindsLiveSourceCells(t *testing.T) {
	blank, tooth, _ := neutralArcSeamSource(t)
	proof, ok, err := certifyRevolveLoftFacesSource(t.Context(), blank, tooth)
	require.NoError(t, err)
	require.True(t, ok)
	sides, ok := revolveLoftFacesToolCapSidesOf(proof)
	require.True(t, ok)
	box, err := lfSourceBox(proof.trim.source)
	require.NoError(t, err)
	checked := 0
	for _, circle := range sides.circles {
		if circle.tool != 1 || circle.sourceMiss {
			continue
		}
		span := proof.section.spans[circle.span]
		var arc *revolveLoftFacesToolCapOccupiedArc
		var certified bool
		if span.start.U == span.end.U {
			arc, certified, err = revolveLoftFacesHeelVerticalOccupiedArc(t.Context(), proof, circle)
		} else {
			arc, certified, err = revolveLoftFacesHeelSlopedOccupiedArc(t.Context(), proof, circle)
		}
		require.NoError(t, err)
		require.True(t, certified)
		band, time, unique := revolveLoftFacesConstantXBand(proof.trim.maps, box, circle.x)
		require.True(t, unique)
		require.Equal(t, band, arc.band)
		require.Zero(t, time.Cmp(arc.time))
		current, currentErr := revolveLoftFacesToolCapOccupiedArcCurrent(t.Context(), proof, arc)
		require.NoError(t, currentErr)
		require.True(t, current)

		changed := *arc
		changed.band++
		current, currentErr = revolveLoftFacesToolCapOccupiedArcCurrent(t.Context(), proof, &changed)
		require.NoError(t, currentErr)
		require.False(t, current, "another cap band cannot own the same arc")
		changed = *arc
		changed.time = new(big.Rat).Add(arc.time, big.NewRat(1, 1<<20))
		current, currentErr = revolveLoftFacesToolCapOccupiedArcCurrent(t.Context(), proof, &changed)
		require.NoError(t, currentErr)
		require.False(t, current, "another time cannot own the same source cells")
		changed = *arc
		changed.negativeZ.parameter.lo = new(big.Rat).Add(
			arc.negativeZ.parameter.lo, arc.negativeZ.parameter.hi)
		changed.negativeZ.parameter.lo.Quo(changed.negativeZ.parameter.lo, big.NewRat(2, 1))
		current, currentErr = revolveLoftFacesToolCapOccupiedArcCurrent(t.Context(), proof, &changed)
		require.NoError(t, currentErr)
		require.False(t, current, "a narrowed root bracket is not the certified source bracket")
		changed = *arc
		changed.positiveZ.face = &Face{}
		current, currentErr = revolveLoftFacesToolCapOccupiedArcCurrent(t.Context(), proof, &changed)
		require.NoError(t, currentErr)
		require.False(t, current, "another retained face cannot own the source crossing")
		checked++
	}
	require.Equal(t, 2, checked, "the real source has vertical and sloped heel arcs")
}
