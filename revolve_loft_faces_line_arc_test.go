package decad

import (
	"testing"

	"github.com/lestrrat-3d/sketch"
	"github.com/stretchr/testify/require"
)

func TestCurvedLineArcJointUsesLiveNeutralSource(t *testing.T) {
	_, _, trim := neutralArcSeamSource(t)
	section := revolveSectionSpan{
		start: Point2{U: 6, V: 2}, end: Point2{U: 6.5, V: 1.5},
	}
	m, b := revolveLoftFacesMeridian(section)
	require.Equal(t, 0, m.Cmp(lfSub(lfZero(), trim.heel.slope)))
	require.Equal(t, 0, b.Cmp(trim.heel.apex))
	segments := trim.source.profile.Outer.Segments
	arcIndex := -1
	for i, entity := range trim.source.source.entities {
		if _, ok := entity.(*sketch.Arc); ok &&
			i > 0 {
			arcIndex = i
		}
	}
	require.Positive(t, arcIndex)
	lineIndices := []int{(arcIndex - 1 + len(segments)) % len(segments),
		(arcIndex + 1) % len(segments)}
	var terminalTime loftFaceInterval
	for _, lineIndex := range lineIndices {
		_, ok := trim.source.source.entities[lineIndex].(*sketch.Line)
		require.True(t, ok)
		lineAtWalkEnd := (lineIndex+1)%len(segments) == arcIndex
		joint, ok := certifyRevolveLoftFacesLineArcJoint(t.Context(), trim,
			section, lineIndex, arcIndex, 5, lineAtWalkEnd)
		require.True(t, ok)
		require.Len(t, joint.roots, 1)
		require.True(t, joint.roots[0].finite)
		require.True(t, joint.vertex.point == joint.vertex.arc.Start ||
			joint.vertex.point == joint.vertex.arc.End)
		require.Equal(t, 0, joint.roots[0].time.lo.Cmp(joint.lineCells[0].time.lo))
		require.Equal(t, 0, joint.roots[0].time.hi.Cmp(joint.lineCells[0].time.hi))
		if terminalTime.lo == nil {
			terminalTime = joint.roots[0].time
		} else {
			require.Equal(t, 0, terminalTime.lo.Cmp(joint.roots[0].time.lo))
			require.Equal(t, 0, terminalTime.hi.Cmp(joint.roots[0].time.hi))
		}
	}
	cells, ok, err := revolveLoftFacesFiniteSpanCells(t.Context(), trim, section)
	require.NoError(t, err)
	require.True(t, ok)
	arcContacts := 0
	for _, cell := range cells {
		if cell.segment == arcIndex && cell.band == 5 &&
			cell.jointKind == revolveLoftFacesWholeArcContact {
			arcContacts++
			require.Equal(t, 0, terminalTime.lo.Cmp(cell.tLo))
			require.Equal(t, 0, terminalTime.hi.Cmp(cell.tHi))
		}
	}
	require.Equal(t, 1, arcContacts)
	for _, lineIndex := range lineIndices {
		lineContacts, fitContacts := 0, 0
		for _, cell := range cells {
			if cell.segment != lineIndex || cell.band != 5 {
				continue
			}
			if cell.jointKind == revolveLoftFacesLineArcContact {
				lineContacts++
				require.Equal(t, revolveLoftFacesJointContact, cell.side)
				require.Same(t, trim.source.source.entities[arcIndex], cell.jointArc)
				require.NotNil(t, cell.jointPoint)
				require.Equal(t, 0, terminalTime.lo.Cmp(cell.tLo))
				require.Equal(t, 0, terminalTime.hi.Cmp(cell.tHi))
			}
			if cell.jointKind == revolveLoftFacesLineFitContact {
				fitContacts++
				require.Equal(t, revolveLoftFacesAxialMiss, cell.side)
			}
		}
		require.Equal(t, 1, lineContacts)
		require.Equal(t, 1, fitContacts)
	}

	original := trim.source.profile.Outer.Segments[lineIndices[0]]
	changed := original.(lineSeg)
	changed.Start.U += 0.01
	trim.source.profile.Outer.Segments[lineIndices[0]] = changed
	defer func() { trim.source.profile.Outer.Segments[lineIndices[0]] = original }()
	_, ok = revolveLoftFacesLineArcVertexOf(trim, lineIndices[0], arcIndex, true)
	require.False(t, ok)
	trim.source.profile.Outer.Segments[lineIndices[0]] = original

	originalArc := trim.source.profile.Outer.Segments[arcIndex]
	changedArc := originalArc.(arcSeg)
	changedArc.Start.U += 0.01
	trim.source.profile.Outer.Segments[arcIndex] = changedArc
	defer func() { trim.source.profile.Outer.Segments[arcIndex] = originalArc }()
	_, ok = revolveLoftFacesLineArcVertexOf(trim, lineIndices[0], arcIndex, true)
	require.False(t, ok)
}
