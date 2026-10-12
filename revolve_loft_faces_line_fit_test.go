package decad

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/sketch"
	"github.com/stretchr/testify/require"
)

func TestCurvedLineFitJointUsesLiveNeutralSource(t *testing.T) {
	blank, trimmed, trim := neutralArcSeamSource(t)
	proof, ok, err := certifyRevolveLoftFacesSource(t.Context(), blank, trimmed)
	require.NoError(t, err)
	require.True(t, ok)
	section := revolveSectionSpan{
		start: Point2{U: 4.7, V: 3.3}, end: Point2{U: 6, V: 2},
	}
	m, b := revolveLoftFacesMeridian(section)
	require.Equal(t, 0, m.Cmp(lfSub(lfZero(), trim.heel.slope)))
	require.Equal(t, 0, b.Cmp(trim.heel.apex))
	require.Same(t, trim, proof.trim)

	fitIndex := -1
	for index, segment := range trim.source.profile.Outer.Segments {
		if _, isFit := segment.(fitSplineSeg); isFit {
			fitIndex = index
			break
		}
	}
	require.NotEqual(t, -1, fitIndex)
	lineIndex := (fitIndex - 1 + len(trim.source.profile.Outer.Segments)) %
		len(trim.source.profile.Outer.Segments)
	_, isLine := trim.source.source.entities[lineIndex].(*sketch.Line)
	require.True(t, isLine)
	joint, ok := certifyRevolveLoftFacesLineFitJoint(t.Context(), trim,
		section, lineIndex, fitIndex, 5, false)
	require.True(t, ok)
	require.Len(t, joint.roots, 1)
	require.True(t, joint.roots[0].finite)
	require.Equal(t, 0, joint.roots[0].time.lo.Cmp(joint.lineCells[0].time.lo))
	require.Equal(t, 0, joint.roots[0].time.hi.Cmp(joint.fitCells[0].time.hi))

	cells, ok, err := revolveLoftFacesFiniteSpanCells(t.Context(), trim, section)
	require.NoError(t, err)
	require.True(t, ok)
	lineContacts, fitContacts, openGaps := 0, 0, 0
	for _, cell := range cells {
		if cell.band == 5 && (cell.segment == lineIndex || cell.segment == fitIndex) &&
			(cell.tLoOpen || cell.tHiOpen) {
			openGaps++
		}
		if cell.jointKind != revolveLoftFacesLineFitContact ||
			cell.jointPoint != joint.vertex.point || cell.band != 5 {
			continue
		}
		require.Same(t, joint.vertex.point, cell.jointPoint)
		require.False(t, cell.tLoOpen)
		require.False(t, cell.tHiOpen)
		require.Equal(t, 0, cell.tLo.Cmp(joint.roots[0].time.lo))
		require.Equal(t, 0, cell.tHi.Cmp(joint.roots[0].time.hi))
		if cell.segment == lineIndex {
			lineContacts++
		}
		if cell.segment == fitIndex {
			fitContacts++
		}
	}
	require.Equal(t, 1, lineContacts)
	require.Equal(t, 1, fitContacts)
	require.Positive(t, openGaps)
	blankSide := proof.section.spans[2]
	fitSpans, spanOK := revolveLoftFacesStraightOrFitSpans(trim, fitIndex,
		trim.source.profile.Outer.Segments[fitIndex])
	require.True(t, spanOK)
	require.True(t, revolveLoftFacesSpanAxialMiss(trim.maps[2], trim.maps[3],
		fitSpans[0], blankSide, lfZero(), lfOne(), lfZero(), lfOne()))
	sideCells, ok, err := revolveLoftFacesFiniteSpanCells(t.Context(), trim, blankSide)
	require.NoError(t, err)
	require.True(t, ok)
	foundMiss := false
	for _, cell := range sideCells {
		if cell.segment == fitIndex && cell.span == 0 && cell.band == 2 {
			require.Equal(t, revolveLoftFacesAxialMiss, cell.side)
			foundMiss = true
		}
	}
	require.True(t, foundMiss)
	contactCells, ok, err := revolveLoftFacesFiniteSpanCells(t.Context(), trim,
		proof.section.spans[3])
	require.NoError(t, err)
	require.True(t, ok)
	foundJoint := false
	for _, cell := range contactCells {
		if cell.segment == fitIndex && cell.span == 0 && cell.band == 4 &&
			cell.jointKind == revolveLoftFacesFitKnotContact &&
			cell.side == revolveLoftFacesJointContact {
			foundJoint = true
		}
	}
	require.True(t, foundJoint)
	joints, jointOK, err := revolveLoftFacesEnumerateNonverticalJoints(t.Context(),
		trim, proof.section.spans[3])
	require.NoError(t, err)
	require.True(t, jointOK)
	var lineJoint *revolveLoftFacesLineFitJoint
	var knotJoint *revolveLoftFacesFitKnotJoint
	for index := range joints.lineFit {
		joint := &joints.lineFit[index]
		if joint.vertex.fitSegment == fitIndex && joint.band == 4 && !joint.vertex.fitAtEnd {
			lineJoint = joint
		}
	}
	for index := range joints.fitKnots {
		joint := &joints.fitKnots[index]
		if joint.vertex.segment == fitIndex && joint.vertex.leftIndex == 0 && joint.band == 4 {
			knotJoint = joint
		}
	}
	require.NotNil(t, lineJoint)
	require.NotNil(t, knotJoint)
	merged, mergedOK, err := revolveLoftFacesMergeLineFitKnotCells(t.Context(), trim,
		proof.section.spans[3], fitIndex, 0, 4, fitSpans[0], lineJoint, knotJoint, true)
	require.NoError(t, err)
	require.True(t, mergedOK)
	require.NotEmpty(t, merged)
	changedJoint := *lineJoint
	changedJoint.vertex.point = knotJoint.vertex.point
	_, mergedOK, err = revolveLoftFacesMergeLineFitKnotCells(t.Context(), trim,
		proof.section.spans[3], fitIndex, 0, 4, fitSpans[0], &changedJoint, knotJoint, true)
	require.NoError(t, err)
	require.False(t, mergedOK)
	manifest, ok, err := revolveLoftFacesAllSideManifest(t.Context(), proof)
	require.NoError(t, err)
	require.True(t, ok)
	require.NotEmpty(t, manifest.nonvertical)
	census, ok, err := revolveLoftFacesCensus(t.Context(), proof)
	require.NoError(t, err)
	require.True(t, ok)
	require.Len(t, census.pairs, len(blank.Faces())*len(trimmed.Faces()))
	original := trim.source.profile.Outer.Segments[lineIndex]
	changed := original.(lineSeg)
	changed.Start.U += 0.01
	trim.source.profile.Outer.Segments[lineIndex] = changed
	defer func() { trim.source.profile.Outer.Segments[lineIndex] = original }()
	_, ok = revolveLoftFacesLineFitVertexOf(trim, lineIndex, fitIndex, false)
	require.False(t, ok)
	trim.source.profile.Outer.Segments[lineIndex] = original

	originalFit := trim.source.profile.Outer.Segments[fitIndex]
	changedFit := originalFit.(fitSplineSeg)
	changedFit.Fit = append([]sectionrecord.Point2(nil), changedFit.Fit...)
	changedFit.Fit[1].U += 0.01
	trim.source.profile.Outer.Segments[fitIndex] = changedFit
	defer func() { trim.source.profile.Outer.Segments[fitIndex] = originalFit }()
	_, ok = revolveLoftFacesLineFitVertexOf(trim, lineIndex, fitIndex, false)
	require.False(t, ok)
	trim.source.profile.Outer.Segments[fitIndex] = originalFit

	originalRevision := trim.source.source.revision
	trim.source.source.revision++
	_, ok = revolveLoftFacesLineFitVertexOf(trim, lineIndex, fitIndex, false)
	require.False(t, ok)
	trim.source.source.revision = originalRevision
	_, ok = revolveLoftFacesLineFitVertexOf(trim, lineIndex, fitIndex, false)
	require.True(t, ok)
}
