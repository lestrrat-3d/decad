package brepgeom_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/brepgeom"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// TestReverseSegmentWalksACircleTheOtherWay pins agent-docs/modify-general-design.md
// §3.3 step 5's rule for a whole circle: the same centre and radius, the CCW
// sense flipped and the range swapped, so reversing twice is the identity. A
// line and an arc keep their ends and swap their range as before.
func TestReverseSegmentWalksACircleTheOtherWay(t *testing.T) {
	t.Parallel()
	circle := sectionrecord.CircleSeg{Center: sectionrecord.Point2{U: 1, V: 2}, Radius: units.Millimeters(3), CCW: true, TStart: 0, TEnd: 1}
	got, ok := brepgeom.ReverseSegment(circle).(sectionrecord.CircleSeg)
	require.True(t, ok)
	require.Equal(t, circle.Center, got.Center)
	require.Equal(t, circle.Radius, got.Radius)
	require.False(t, got.CCW)
	require.Equal(t, [2]float64{1, 0}, [2]float64{got.TStart, got.TEnd})
	require.Equal(t, circle, brepgeom.ReverseSegment(got))

	line := sectionrecord.LineSeg{Start: sectionrecord.Point2{U: 0, V: 0}, End: sectionrecord.Point2{U: 5, V: 0}, TStart: 0, TEnd: 1}
	gotLine, ok := brepgeom.ReverseSegment(line).(sectionrecord.LineSeg)
	require.True(t, ok)
	require.Equal(t, [2]float64{1, 0}, [2]float64{gotLine.TStart, gotLine.TEnd})
	require.Equal(t, line.Start, gotLine.Start)
	require.Equal(t, line.End, gotLine.End)
}
