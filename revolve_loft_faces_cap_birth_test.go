package decad

import (
	"testing"

	"github.com/stretchr/testify/require"
)

func TestCurvedHeelBirthUsesLiveTipArc(t *testing.T) {
	blank, tooth, trim := neutralArcSeamSource(t)
	proof, ok, err := certifyRevolveLoftFacesSource(t.Context(), blank, tooth)
	require.NoError(t, err)
	require.True(t, ok)
	birth, ok, err := certifyRevolveLoftFacesHeelBirth(t.Context(), proof)
	require.NoError(t, err)
	require.True(t, ok)
	require.NotNil(t, birth)
	require.Same(t, proof.heelFace, birth.capFace)
	require.Same(t, trim.source.source.entities[1], birth.arc)
	require.True(t, birth.rootX.hi.Cmp(birth.lower.circle.x) < 0)
	require.NotEmpty(t, birth.slices)
	require.Equal(t, birth.lower.time.Cmp(birth.slices[len(birth.slices)-1].hi), 0)

	loop, simple, err := revolveLoftFacesSimpleSourceLoop(t.Context(), trim)
	require.NoError(t, err)
	require.True(t, simple)
	cells, classified, err := revolveLoftFacesFiniteSpanCells(t.Context(), trim, birth.meridian)
	require.NoError(t, err)
	require.True(t, classified)
	require.True(t, revolveLoftFacesHeelBirthCells(trim, loop, cells, birth))
	changedSide := false
	for i := range cells {
		if cells[i].band == birth.lower.band && cells[i].side == revolveLoftFacesInside &&
			cells[i].tLo.Cmp(lfZero()) == 0 &&
			cells[i].tHi.Cmp(birth.root.lo) == 0 {
			cells[i].side = revolveLoftFacesOutside
			require.False(t, revolveLoftFacesHeelBirthCells(trim, loop, cells, birth))
			cells[i].side = revolveLoftFacesInside
			changedSide = true
			break
		}
	}
	require.True(t, changedSide)
	changedJoint := false
	for i := range cells {
		if cells[i].band == birth.lower.band &&
			cells[i].side == revolveLoftFacesJointContact &&
			cells[i].jointKind == revolveLoftFacesRadialArcContact &&
			cells[i].tLo.Cmp(birth.root.lo) == 0 {
			cells[i].jointPoint = nil
			require.False(t, revolveLoftFacesHeelBirthCells(trim, loop, cells, birth))
			changedJoint = true
			break
		}
	}
	require.True(t, changedJoint)

	changed := *proof
	changed.section = proof.section
	changed.section.spans = append([]revolveSectionSpan(nil), proof.section.spans...)
	changed.section.spans[birth.lower.circle.span].end.V += 0.125
	rejected, ok, err := certifyRevolveLoftFacesHeelBirth(t.Context(), &changed)
	require.NoError(t, err)
	require.False(t, ok)
	require.Nil(t, rejected)
}
