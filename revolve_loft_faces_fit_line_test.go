package decad

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/sketch"
	"github.com/stretchr/testify/require"
)

func TestFiniteRevolveFitLineJointKeepsBothLiveSourceSides(t *testing.T) {
	_, _, trim := neutralArcSeamSource(t)
	vertex, ok := revolveLoftFacesFitLineVertexAt(trim, 0, 5, false)
	require.True(t, ok)
	require.Same(t, trim.Source.source.entities[0].(*sketch.FitSpline).Fit[0], vertex.point)
	other, ok := revolveLoftFacesFitLineVertexAt(trim, 2, 3, true)
	require.True(t, ok)
	require.False(t, other.lineAtEnd)
	band := 3
	first := trim.Maps[band].At(vertex.coordinate[0], vertex.coordinate[1])
	last := trim.Maps[band+1].At(vertex.coordinate[0], vertex.coordinate[1])
	x0, _ := first[0].Float64()
	x1, _ := last[0].Float64()
	y0, _ := first[1].Float64()
	y1, _ := last[1].Float64()
	z0, _ := first[2].Float64()
	z1, _ := last[2].Float64()
	r0, r1 := math.Hypot(y0, z0), math.Hypot(y1, z1)
	require.NotEqual(t, r0, r1)
	section := revolveSectionSpan{Start: planePoint{U: math.Min(x0, x1) - 0.05, V: (r0 + r1) / 2},
		End: planePoint{U: math.Max(x0, x1) + 0.05, V: (r0 + r1) / 2}}
	joint, ok := certifyRevolveLoftFacesFitLineJoint(t.Context(), trim, section, 0, 5, band, false)
	require.True(t, ok)
	require.Len(t, joint.roots, 1)
	require.True(t, joint.roots[0].Finite)
	for _, part := range []struct {
		segment int
		span    freeform.BezierSpan
	}{
		{segment: 0, span: vertex.fitSpan},
		{segment: 5, span: vertex.lineSpan},
	} {
		cells, certified, err := revolveLoftFacesFitLinePartitionCells(t.Context(),
			trim, section, part.segment, 0, band, part.span, nil, false, nil,
			[]*revolveLoftFacesFitLineJoint{&joint})
		require.NoError(t, err)
		require.True(t, certified)
		require.NotEmpty(t, cells)
		contacts := 0
		for _, cell := range cells {
			if cell.side == revolveLoftFacesJointContact {
				contacts++
				require.Equal(t, revolveLoftFacesFitLineContact, cell.jointKind)
				require.Same(t, vertex.point, cell.jointPoint)
			}
		}
		require.Equal(t, 1, contacts)
	}
	joints, certified, err := revolveLoftFacesEnumerateNonverticalJoints(t.Context(), trim, section)
	require.NoError(t, err)
	require.True(t, certified)
	found := false
	for _, candidate := range joints.fitLines {
		if candidate.band == band && candidate.vertex.point == vertex.point &&
			candidate.vertex.fitSegment == 0 && candidate.vertex.lineSegment == 5 {
			found = true
		}
	}
	require.True(t, found)
	mapped, certified, err := revolveLoftFacesFiniteSpanCells(t.Context(), trim, section)
	require.NoError(t, err)
	require.True(t, certified)
	for _, segment := range []int{0, 5} {
		matched := false
		for _, cell := range mapped {
			if cell.segment == segment && cell.band == band &&
				cell.side == revolveLoftFacesJointContact &&
				cell.jointKind == revolveLoftFacesFitLineContact &&
				cell.jointPoint == vertex.point {
				matched = true
			}
		}
		require.True(t, matched)
	}
	t.Run("changed recorded line endpoint refuses", func(t *testing.T) {
		original := trim.Source.profile.Outer.Segments[5]
		defer func() { trim.Source.profile.Outer.Segments[5] = original }()
		changed := original.(lineSeg)
		changed.End.U += 0.001
		trim.Source.profile.Outer.Segments[5] = changed
		_, valid := revolveLoftFacesFitLineVertexAt(trim, 0, 5, false)
		require.False(t, valid)
	})
}
