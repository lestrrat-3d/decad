package decad

import (
	"testing"

	"github.com/stretchr/testify/require"
)

func TestRevolveLoftFacesShortArcSupportsOnlyItsChordOutside(t *testing.T) {
	arc := arcSeg{Center: planePoint{},
		Start: planePoint{U: 4, V: -0.5}, End: planePoint{U: 4, V: 0.5},
		TStart: 0, TEnd: 1}
	require.True(t, revolveLoftFacesSimpleArcChordSide(arc, 0, lfF(4), 1))
	require.False(t, revolveLoftFacesSimpleArcChordSide(arc, 0, lfF(4), -1))
	require.True(t, revolveLoftFacesSimpleArcMonotoneSide(arc, 1, lfF(-0.5), 1))
	require.True(t, revolveLoftFacesSimpleArcMonotoneSide(arc, 1, lfF(0.5), -1))
	require.False(t, revolveLoftFacesSimpleArcMonotoneSide(arc, 0, lfF(4), 1),
		"the bulging coordinate is not monotone")
}

func TestRevolveLoftFacesArcRefusesUnequalEndpointRadius(t *testing.T) {
	s, profile := neutralSimpleSourceSketch(t, false)
	trim := neutralSimpleSourceRecord(t, s, profile)
	arc := trim.Source.Profile.Outer.Segments[1].(arcSeg)
	arc.End.U += 0.01
	_, certified := revolveLoftFacesSimpleArc(trim, 1, arc, false)
	require.False(t, certified)
}
