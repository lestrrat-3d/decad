package decad

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/sketch"
	"github.com/stretchr/testify/require"
)

func TestCurvedMappedSourceWithPinnedArcEnd(t *testing.T) {
	blank, tooth, trim := neutralArcSeamSourceWithInnerArcEnd(t,
		[][2]float64{{2.8, -0.24}, {3.5, -0.3}, {4.2, -0.35}},
		[][2]float64{{2.8, 0.24}, {3.5, 0.3}, {4.2, 0.35}},
		math.Nextafter(0.2, 1))
	proof, ok, err := certifyRevolveLoftFacesSource(t.Context(), blank, tooth)
	require.NoError(t, err)
	require.True(t, ok)
	section := proof.section.spans[proof.section.rootSpan]
	_, ok, err = revolveLoftFacesFiniteSpanCells(t.Context(), trim, section)
	require.NoError(t, err)
	require.True(t, ok)
	_, ok, err = revolveLoftFacesNonverticalManifest(t.Context(), proof)
	require.NoError(t, err)
	require.True(t, ok)
	jointSection := revolveSectionSpan{start: Point2{U: 7, V: 2.15}, end: Point2{U: 9, V: 2.15}}
	lineArc, valid := revolveLoftFacesLineArcVertexAt(trim, 3, 4, true)
	require.True(t, valid)
	require.Same(t, trim.source.source.entities[4].(*sketch.Arc).End, lineArc.point)
	arcRecord := trim.source.profile.Outer.Segments[4].(arcSeg)
	require.Positive(t, loftmesh.ArcNaturalEndRadialUpper(arcRecord))
	require.LessOrEqual(t, loftmesh.ArcNaturalEndRadialUpper(arcRecord), trim.source.sourceStationRound)
	_, radial := mappedArcRadialTimeBand(t.Context(), trim, 4, 7, jointSection)
	require.True(t, radial)
	joints, valid, err := revolveLoftFacesEnumerateNonverticalJoints(t.Context(), trim, jointSection)
	require.NoError(t, err)
	require.True(t, valid)
	require.NotEmpty(t, joints.lineArcs)
	cells, ok, err := revolveLoftFacesFiniteSpanCells(t.Context(), trim, jointSection)
	require.NoError(t, err)
	require.True(t, ok)
	lineContacts, arcContacts := 0, 0
	for _, cell := range cells {
		if cell.segment == 3 && cell.band == 7 && cell.jointKind == revolveLoftFacesLineArcContact {
			lineContacts++
			require.Same(t, lineArc.point, cell.jointPoint)
		}
		if cell.segment == 4 && cell.band == 7 && cell.jointKind == revolveLoftFacesWholeArcContact {
			arcContacts++
		}
	}
	require.Equal(t, 1, lineContacts)
	require.Equal(t, 1, arcContacts)
	t.Run("missing station residual refuses", func(t *testing.T) {
		before := trim.source.sourceStationRound
		defer func() { trim.source.sourceStationRound = before }()
		trim.source.sourceStationRound = 0
		_, accepted := mappedArcRadialTimeBand(t.Context(), trim, 4, 7, jointSection)
		require.False(t, accepted)
	})
	t.Run("changed Arc record refuses", func(t *testing.T) {
		before := trim.source.profile.Outer.Segments[4]
		defer func() { trim.source.profile.Outer.Segments[4] = before }()
		changed := arcRecord
		changed.End.V += 0.01
		trim.source.profile.Outer.Segments[4] = changed
		_, accepted := revolveLoftFacesLineArcVertexAt(trim, 3, 4, true)
		require.False(t, accepted)
		_, accepted = mappedArcRadialTimeBand(t.Context(), trim, 4, 7, jointSection)
		require.False(t, accepted)
	})
	t.Run("changed Line record refuses", func(t *testing.T) {
		before := trim.source.profile.Outer.Segments[3]
		defer func() { trim.source.profile.Outer.Segments[3] = before }()
		changed := before.(lineSeg)
		changed.End.V += 0.01
		trim.source.profile.Outer.Segments[3] = changed
		_, accepted := revolveLoftFacesLineArcVertexAt(trim, 3, 4, true)
		require.False(t, accepted)
	})
	t.Run("nonadjacent pair refuses", func(t *testing.T) {
		_, accepted := revolveLoftFacesLineArcVertexAt(trim, 5, 4, true)
		require.False(t, accepted)
	})
}
