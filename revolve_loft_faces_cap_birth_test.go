package decad

import (
	"math/big"
	"testing"

	"github.com/stretchr/testify/require"
)

func TestCurvedHeelBirthUsesLiveTipArc(t *testing.T) {
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
	require.True(t, region.upper.circle.x.Cmp(terminal.rootX.lo) < 0)

	birth, ok, err := certifyRevolveLoftFacesHeelBirth(t.Context(), proof)
	require.NoError(t, err)
	require.True(t, ok)
	require.NotNil(t, birth)
	require.Same(t, proof.heelFace, birth.capFace)
	require.Same(t, trim.source.source.entities[1], birth.arc)
	require.True(t, birth.rootX.hi.Cmp(birth.lower.circle.x) < 0)
	require.NotEmpty(t, birth.slices)
	require.Equal(t, birth.lower.time.Cmp(birth.slices[len(birth.slices)-1].hi), 0)
	changedOrder := *birth
	changedOrder.slices = append([]revolveLoftFacesHeelRegionSlice(nil), birth.slices...)
	changedOrder.slices[0].events[0], changedOrder.slices[0].events[1] =
		changedOrder.slices[0].events[1], changedOrder.slices[0].events[0]
	require.False(t, revolveLoftFacesHeelBirthZSides(t.Context(), proof, &changedOrder),
		"the two crossing boxes must stay on opposite Z sides")

	loop, simple, err := revolveLoftFacesSimpleSourceLoop(t.Context(), trim)
	require.NoError(t, err)
	require.True(t, simple)
	cells, classified, err := revolveLoftFacesFiniteSpanCells(t.Context(), trim, birth.meridian)
	require.NoError(t, err)
	require.True(t, classified)
	require.True(t, revolveLoftFacesHeelBirthCells(trim, loop, cells, birth))
	changedOverlap := *birth
	changedOverlap.root.hi = birth.lower.time
	require.False(t, revolveLoftFacesHeelBirthCells(trim, loop, cells, &changedOverlap))
	tipRefused := false
	for i := range cells {
		if cells[i].band == birth.lower.band &&
			cells[i].jointKind == revolveLoftFacesWholeArcContact &&
			cells[i].jointArc == birth.arc {
			arc := cells[i].jointArc
			cells[i].jointArc = nil
			require.False(t, revolveLoftFacesHeelBirthCells(trim, loop, cells, birth))
			cells[i].jointArc = arc
			tipRefused = true
			break
		}
	}
	require.True(t, tipRefused)
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

	originalRevision := trim.source.source.revision
	trim.source.source.revision++
	defer func() { trim.source.source.revision = originalRevision }()
	rejected, ok, err = certifyRevolveLoftFacesHeelBirth(t.Context(), proof)
	require.NoError(t, err)
	require.False(t, ok)
	require.Nil(t, rejected)
}
