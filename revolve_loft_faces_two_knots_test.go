package decad

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/sketch"
	"github.com/stretchr/testify/require"
)

func TestFiniteRevolveTwoFitKnotsOnOneSourceSpan(t *testing.T) {
	_, _, trim := neutralArcSeamSourceWithFit(t,
		[][2]float64{{2.8, -0.24}, {3.45, -0.29}, {3.5, -0.30}, {4.2, -0.35}},
		[][2]float64{{2.8, 0.24}, {3.45, 0.29}, {3.5, 0.30}, {4.2, 0.35}})
	fit := trim.source.source.entities[0].(*sketch.FitSpline)
	require.Len(t, fit.Fit, 4)
	spans, ok := revolveLoftFacesStraightOrFitSpans(trim, 0,
		trim.source.profile.Outer.Segments[0])
	require.True(t, ok)
	require.Len(t, spans, 3)
	for band := 0; band+1 < len(trim.maps); band++ {
		first := trim.maps[band]
		last := trim.maps[band+1]
		var radii [2][2]float64
		minimumX, maximumX := math.Inf(1), math.Inf(-1)
		for i, point := range fit.Fit[1:3] {
			for j, mapped := range []loftFaceMap{first, last} {
				xyz := mapped.at(lfF(point.X()), lfF(point.Y()))
				x, _ := xyz[0].Float64()
				y, _ := xyz[1].Float64()
				z, _ := xyz[2].Float64()
				minimumX = math.Min(minimumX, x)
				maximumX = math.Max(maximumX, x)
				radii[i][j] = math.Hypot(y, z)
			}
		}
		low := math.Max(math.Min(radii[0][0], radii[0][1]),
			math.Min(radii[1][0], radii[1][1]))
		high := math.Min(math.Max(radii[0][0], radii[0][1]),
			math.Max(radii[1][0], radii[1][1]))
		if high-low < 0.01 {
			continue
		}
		section := revolveSectionSpan{
			start: Point2{U: minimumX - 1, V: (low + high) / 2},
			end:   Point2{U: maximumX + 1, V: (low + high) / 2},
		}
		joints, certified, err := revolveLoftFacesEnumerateNonverticalJoints(t.Context(), trim, section)
		require.NoError(t, err)
		require.True(t, certified)
		var knots []*revolveLoftFacesFitKnotJoint
		for i := range joints.fitKnots {
			joint := &joints.fitKnots[i]
			if joint.vertex.segment == 0 && joint.band == band &&
				(joint.vertex.leftIndex == 0 || joint.vertex.leftIndex == 1) {
				knots = append(knots, joint)
			}
		}
		require.Len(t, knots, 2)
		require.NotSame(t, knots[0].vertex.point, knots[1].vertex.point)
		cells, certified, err := revolveLoftFacesPartitionJointCells(t.Context(),
			trim, section, 0, 1, band, spans[1], knots, nil, nil, nil)
		require.NoError(t, err)
		require.True(t, certified)
		contacts := map[*sketch.Point]int{}
		for _, cell := range cells {
			if cell.side == revolveLoftFacesJointContact {
				require.Equal(t, revolveLoftFacesFitKnotContact, cell.jointKind)
				contacts[cell.jointPoint]++
			}
		}
		require.Equal(t, 1, contacts[fit.Fit[1]])
		require.Equal(t, 1, contacts[fit.Fit[2]])
		mapped, certified, err := revolveLoftFacesFiniteSpanCells(t.Context(), trim, section)
		require.NoError(t, err)
		require.True(t, certified)
		found := 0
		for _, cell := range mapped {
			if cell.segment == 0 && cell.span == 1 && cell.band == band &&
				cell.side == revolveLoftFacesJointContact {
				found++
			}
		}
		require.Equal(t, 2, found)
		t.Run("overlapping time roots refuse", func(t *testing.T) {
			changed := *knots[1]
			changed.roots = append([]revolveLoftFacesEndpointRoot(nil), changed.roots...)
			changed.rightCells = append([]revolveLoftFacesJointCell(nil), changed.rightCells...)
			changed.roots[0].time = knots[0].roots[0].time
			changed.rightCells[0].time = changed.roots[0].time
			_, certified, _ := revolveLoftFacesPartitionJointCells(t.Context(),
				trim, section, 0, 1, band, spans[1],
				[]*revolveLoftFacesFitKnotJoint{knots[0], &changed}, nil, nil, nil)
			require.False(t, certified)
		})
		t.Run("wrong source span refuses", func(t *testing.T) {
			_, certified, _ := revolveLoftFacesPartitionJointCells(t.Context(),
				trim, section, 0, 1, band, spans[0], knots, nil, nil, nil)
			require.False(t, certified)
		})
		return
	}
	t.Fatal("no mapped band crosses both adjacent fitted knots")
}
