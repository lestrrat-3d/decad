package decad

import (
	"testing"

	"github.com/stretchr/testify/require"
)

func TestCurvedHeelTerminalUsesLiveWholeArc(t *testing.T) {
	blank, tooth, trim := neutralArcSeamSource(t)
	proof, ok, err := certifyRevolveLoftFacesSource(t.Context(), blank, tooth)
	require.NoError(t, err)
	require.True(t, ok)
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
}
