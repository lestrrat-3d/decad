package decad

import (
	"math/big"
	"testing"

	"github.com/stretchr/testify/require"
)

func TestCurvedHeelTerminalUsesLiveWholeArc(t *testing.T) {
	blank, tooth, trim := neutralArcSeamSource(t)
	proof, ok, err := certifyRevolveLoftFacesSource(t.Context(), blank, tooth)
	require.NoError(t, err)
	require.True(t, ok)
	region, ok, err := certifyRevolveLoftFacesHeelRegion(t.Context(), proof)
	require.NoError(t, err)
	require.True(t, ok)
	for _, arc := range []*revolveLoftFacesToolCapOccupiedArc{region.lower, region.upper} {
		cut, built, cutErr := revolveLoftFacesHeldCapCutPath(t.Context(), proof, arc)
		require.NoError(t, cutErr)
		require.True(t, built)
		require.NotEmpty(t, cut.points)
	}
	track, built, err := revolveLoftFacesHeelTrackOf(t.Context(), proof, big.NewRat(1, 50))
	require.NoError(t, err)
	require.True(t, built)
	require.Greater(t, len(track.stations), 2)

	terminal, ok, err := certifyRevolveLoftFacesHeelTerminal(t.Context(), proof)
	require.NoError(t, err)
	require.True(t, ok)
	require.NotNil(t, terminal)
	require.Same(t, proof.heelFace, terminal.capFace)
	require.Same(t, trim.source.source.entities[4], terminal.arc)
	require.True(t, terminal.upper.circle.x.Cmp(terminal.rootX.lo) < 0)
	require.True(t, terminal.rootX.hi.Cmp(lfF(terminal.meridian.end.U)) < 0)
	require.True(t, terminal.root.lo.Cmp(terminal.root.hi) < 0)
	require.True(t, terminal.root.hi.Cmp(terminal.endTime) < 0)
	require.NotSame(t, terminal.ends[0], terminal.ends[1])
	arcIndex := -1
	for index, entity := range trim.source.source.entities {
		if entity == terminal.arc {
			arcIndex = index
			break
		}
	}
	require.Positive(t, arcIndex)
	for index, line := range terminal.lines {
		joint, current := certifyRevolveLoftFacesLineArcJoint(t.Context(), trim,
			terminal.meridian, line, arcIndex, terminal.upper.band, index == 0)
		require.True(t, current)
		require.Same(t, terminal.ends[index], joint.vertex.point)
		require.Same(t, terminal.arc, joint.vertex.arc)
		require.Equal(t, 0, terminal.root.lo.Cmp(joint.roots[0].time.lo))
		require.Equal(t, 0, terminal.root.hi.Cmp(joint.roots[0].time.hi))
	}

	loop, simple, err := revolveLoftFacesSimpleSourceLoop(t.Context(), trim)
	require.NoError(t, err)
	require.True(t, simple)
	cells, classified, err := revolveLoftFacesFiniteSpanCells(t.Context(),
		trim, terminal.meridian)
	require.NoError(t, err)
	require.True(t, classified)
	require.True(t, revolveLoftFacesHeelTerminalCells(trim, loop, cells, terminal))
	changedSide := false
	for i := range cells {
		if cells[i].band == terminal.upper.band &&
			cells[i].side == revolveLoftFacesOutside &&
			cells[i].tLo.Cmp(terminal.root.hi) == 0 {
			cells[i].side = revolveLoftFacesInside
			require.False(t, revolveLoftFacesHeelTerminalCells(trim, loop, cells, terminal))
			changedSide = true
			break
		}
	}
	require.True(t, changedSide)

	changed := *proof
	changed.section = proof.section
	changed.section.spans = append([]revolveSectionSpan(nil), proof.section.spans...)
	changed.section.spans[changed.section.axisSpan].end.U += 0.125
	rejected, ok, err := certifyRevolveLoftFacesHeelTerminal(t.Context(), &changed)
	require.NoError(t, err)
	require.False(t, ok)
	require.Nil(t, rejected)

	originalRevision := trim.source.source.revision
	trim.source.source.revision++
	defer func() { trim.source.source.revision = originalRevision }()
	rejected, ok, err = certifyRevolveLoftFacesHeelTerminal(t.Context(), proof)
	require.NoError(t, err)
	require.False(t, ok)
	require.Nil(t, rejected)
}
