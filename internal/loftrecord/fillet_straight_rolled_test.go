package loftrecord

import (
	"math"
	"testing"

	"github.com/stretchr/testify/require"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/units"
)

// rootStep is a radius-3 root circle fragment about the origin followed by a
// radial line of length 0.1 leaving its end, then next. The line's corner with
// the circle is concave, so a radius-ρ fillet sets back about ρ along it.
func rootStep(t *testing.T, next sectionrecord.CurveSegment) ([]sectionrecord.CurveSegment, []survey2d.SideWalk) {
	t.Helper()
	const tEnd, length = 0.97, 0.1
	a := 2 * math.Pi * tEnd
	foot := sectionrecord.Point2{U: 3 * math.Cos(a), V: 3 * math.Sin(a)}
	tip := sectionrecord.Point2{U: (3 + length) * math.Cos(a), V: (3 + length) * math.Sin(a)}
	segments := []sectionrecord.CurveSegment{
		sectionrecord.CircleSeg{Radius: units.Millimeters(3), CCW: true, TStart: 0.9, TEnd: tEnd},
		sectionrecord.LineSeg{Start: foot, End: tip, TEnd: 1},
		next,
	}
	walks := make([]survey2d.SideWalk, len(segments))
	for i := range 2 {
		w, err := boundarywalk.WalkOf(segments[i], freeform.NewFreeformWork())
		require.NoError(t, err)
		walks[i] = survey2d.SideWalk{SegmentWalk: w, Segs: []int{i}}
	}
	return segments, walks
}

func TestRolledLinesNeedAConsumedLineEndingAtAFitSpline(t *testing.T) {
	t.Parallel()
	fit := sectionrecord.FitSplineSeg{Fit: []sectionrecord.Point2{{U: 4}, {U: 5}}, TEnd: 1}
	other := sectionrecord.CircleSeg{Center: sectionrecord.Point2{U: 9}, Radius: units.Millimeters(1),
		CCW: true, TStart: 0.2, TEnd: 0.3}
	t.Run("a consumed line before a fit spline", func(t *testing.T) {
		t.Parallel()
		segments, walks := rootStep(t, fit)
		require.Equal(t, map[int]struct{}{1: {}}, rolledLines(segments, walks, []int{1}, 0.45))
	})
	t.Run("a line the fillet only trims", func(t *testing.T) {
		t.Parallel()
		segments, walks := rootStep(t, fit)
		require.Empty(t, rolledLines(segments, walks, []int{1}, 0.05))
	})
	t.Run("a consumed line before another circle fragment", func(t *testing.T) {
		t.Parallel()
		segments, walks := rootStep(t, other)
		require.Empty(t, rolledLines(segments, walks, []int{1}, 0.45))
	})
	t.Run("an unselected corner", func(t *testing.T) {
		t.Parallel()
		segments, walks := rootStep(t, fit)
		require.Empty(t, rolledLines(segments, walks, nil, 0.45))
	})
}

func TestDropSegmentsMovesEachCornerToTheNextKeptSegment(t *testing.T) {
	t.Parallel()
	segments := make([]sectionrecord.CurveSegment, 6)
	walks := make([]survey2d.SideWalk, 6)
	for i := range segments {
		segments[i] = sectionrecord.LineSeg{Start: sectionrecord.Point2{U: float64(i)}, TEnd: 1}
		walks[i].Length = float64(i)
	}
	kept, keptWalks, origin, corners := dropSegments(segments, walks, []int{2, 3, 5, 0},
		map[int]struct{}{1: {}, 3: {}, 5: {}})
	require.Equal(t, []int{0, 2, 4}, origin)
	require.Len(t, kept, 3)
	for i, o := range origin {
		require.Equal(t, segments[o], kept[i])
		require.Equal(t, walks[o].Length, keptWalks[i].Length)
	}
	// Corner 2 keeps its segment; corner 3 moves to segment 4; corner 5 wraps to
	// segment 0, which corner 0 also names.
	require.Equal(t, []int{1, 2, 0, 0}, corners)
}
