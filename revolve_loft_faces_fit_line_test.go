package decad

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/revolveloftfaces"
	"github.com/lestrrat-3d/sketch"
	"github.com/stretchr/testify/require"
)

func TestFiniteRevolveFitLineJointKeepsBothLiveSourceSides(t *testing.T) {
	_, _, trim := neutralArcSeamSource(t)
	vertex, ok := revolveloftfaces.RevolveLoftFacesFitLineVertexAt(trim, 0, 5, false)
	require.True(t, ok)
	require.Same(t, trim.Source.Source.Entities[0].(*sketch.FitSpline).Fit[0], vertex.Point)
	other, ok := revolveloftfaces.RevolveLoftFacesFitLineVertexAt(trim, 2, 3, true)
	require.True(t, ok)
	require.False(t, other.LineAtEnd)
	band := 3
	first := trim.Maps[band].At(vertex.Coordinate[0], vertex.Coordinate[1])
	last := trim.Maps[band+1].At(vertex.Coordinate[0], vertex.Coordinate[1])
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
	joint, ok := revolveloftfaces.CertifyRevolveLoftFacesFitLineJoint(t.Context(), trim, section, 0, 5, band, false)
	require.True(t, ok)
	require.Len(t, joint.Roots, 1)
	require.True(t, joint.Roots[0].Finite)
	for _, part := range []struct {
		segment int
		span    freeform.BezierSpan
	}{
		{segment: 0, span: vertex.FitSpan},
		{segment: 5, span: vertex.LineSpan},
	} {
		cells, certified, err := revolveloftfaces.RevolveLoftFacesFitLinePartitionCells(t.Context(),
			trim, section, part.segment, 0, band, part.span, nil, false, nil,
			[]*revolveloftfaces.RevolveLoftFacesFitLineJoint{&joint})
		require.NoError(t, err)
		require.True(t, certified)
		require.NotEmpty(t, cells)
		contacts := 0
		for _, cell := range cells {
			if cell.Side == revolveloftfaces.RevolveLoftFacesJointContact {
				contacts++
				require.Equal(t, revolveloftfaces.RevolveLoftFacesFitLineContact, cell.JointKind)
				require.Same(t, vertex.Point, cell.JointPoint)
			}
		}
		require.Equal(t, 1, contacts)
	}
	joints, certified, err := revolveloftfaces.RevolveLoftFacesEnumerateNonverticalJoints(t.Context(), trim, section)
	require.NoError(t, err)
	require.True(t, certified)
	found := false
	for _, candidate := range joints.FitLines {
		if candidate.Band == band && candidate.Vertex.Point == vertex.Point &&
			candidate.Vertex.FitSegment == 0 && candidate.Vertex.LineSegment == 5 {
			found = true
		}
	}
	require.True(t, found)
	mapped, certified, err := revolveloftfaces.RevolveLoftFacesFiniteSpanCells(t.Context(), trim, section)
	require.NoError(t, err)
	require.True(t, certified)
	for _, segment := range []int{0, 5} {
		matched := false
		for _, cell := range mapped {
			if cell.Segment == segment && cell.Band == band &&
				cell.Side == revolveloftfaces.RevolveLoftFacesJointContact &&
				cell.JointKind == revolveloftfaces.RevolveLoftFacesFitLineContact &&
				cell.JointPoint == vertex.Point {
				matched = true
			}
		}
		require.True(t, matched)
	}
	t.Run("changed recorded line endpoint refuses", func(t *testing.T) {
		original := trim.Source.Profile.Outer.Segments[5]
		defer func() { trim.Source.Profile.Outer.Segments[5] = original }()
		changed := original.(lineSeg)
		changed.End.U += 0.001
		trim.Source.Profile.Outer.Segments[5] = changed
		_, valid := revolveloftfaces.RevolveLoftFacesFitLineVertexAt(trim, 0, 5, false)
		require.False(t, valid)
	})
}
