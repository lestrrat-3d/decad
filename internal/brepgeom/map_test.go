package brepgeom_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/brepgeom"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/stretchr/testify/require"
)

// TestMapSegment pins docs/brep-modify-design.md §5.4's plane-local map
// between two faces across one reference axis: the XY frame (identity
// embed) and a frame across the same axis whose u runs along reference y and
// whose v runs along reference x, a swap that reflects the plane. A line
// keeps its range, an arc swaps its ends and reads its range as 1−t, and a
// circle flips its sense. The rotation by a quarter turn (u along y, v along
// −x) keeps every sense. Faces whose normals lie on different axes have no
// such map. Shown to fail with Reflects' sign inverted (every arc then kept
// its ends under the swap).
func TestMapSegment(t *testing.T) {
	t.Parallel()
	xy := brepgeom.Embed{Axis: [3]int{0, 1, 2}, Sign: [3]float64{1, 1, 1}}
	swap := brepgeom.Embed{Axis: [3]int{1, 0, 2}, Sign: [3]float64{1, 1, -1}}
	turn := brepgeom.Embed{Axis: [3]int{1, 0, 2}, Sign: [3]float64{1, -1, 1}}
	p := func(u, v float64) sectionrecord.Point2 { return sectionrecord.Point2{U: u, V: v} }

	m, ok := brepgeom.NewMap2(xy, swap)
	require.True(t, ok)
	require.True(t, m.Reflects())
	u, v := m.Point(1, 2)
	require.Equal(t, [2]float64{2, 1}, [2]float64{u, v})

	line, err := brepgeom.MapSegment(m, sectionrecord.LineSeg{Start: p(0, 0), End: p(3, 1), TStart: 1, TEnd: 0})
	require.NoError(t, err)
	require.Equal(t, sectionrecord.LineSeg{Start: p(0, 0), End: p(1, 3), TStart: 1, TEnd: 0}, line)
	arc, err := brepgeom.MapSegment(m, sectionrecord.ArcSeg{Center: p(2, 2), Start: p(0, 2), End: p(2, 0), TStart: 0, TEnd: 1})
	require.NoError(t, err)
	require.Equal(t, sectionrecord.ArcSeg{Center: p(2, 2), Start: p(0, 2), End: p(2, 0), TStart: 1, TEnd: 0}, arc)
	circle, err := brepgeom.MapSegment(m, sectionrecord.CircleSeg{Center: p(5, 1), CCW: true, TStart: 0, TEnd: 1})
	require.NoError(t, err)
	require.Equal(t, sectionrecord.CircleSeg{Center: p(1, 5), CCW: false, TStart: 1, TEnd: 0}, circle)

	m, ok = brepgeom.NewMap2(xy, turn)
	require.True(t, ok)
	require.False(t, m.Reflects())
	arc, err = brepgeom.MapSegment(m, sectionrecord.ArcSeg{Center: p(2, 2), Start: p(0, 2), End: p(2, 0), TStart: 0, TEnd: 1})
	require.NoError(t, err)
	require.Equal(t, sectionrecord.ArcSeg{Center: p(2, -2), Start: p(2, 0), End: p(0, -2), TStart: 0, TEnd: 1}, arc)

	_, ok = brepgeom.NewMap2(xy, brepgeom.Embed{Axis: [3]int{0, 2, 1}, Sign: [3]float64{1, 1, -1}})
	require.False(t, ok)
	_, err = brepgeom.MapSegment(m, sectionrecord.EllipseSeg{})
	require.Error(t, err)
}
