package boundarywalk

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/stretchr/testify/require"
)

func TestWholeWalkUsesRecordedEndpoints(t *testing.T) {
	t.Parallel()
	start := Point2{U: 4.0 / 7.0, V: 2}
	end := Point2{U: 10.0 / 3.0, V: 2}
	require.NotEqual(t, end.U, start.U+(end.U-start.U),
		"this line's rounded chord evaluation misses its recorded end")
	line := LineSeg{Start: start, End: end, TEnd: 1}
	walk, err := WalkOf(line, nil)
	require.NoError(t, err)
	require.Equal(t, [4]float64{start.U, start.V, end.U, end.V},
		[4]float64{walk.StartU, walk.StartV, walk.EndU, walk.EndV})
	line.TStart, line.TEnd = 1, 0
	reversed, err := WalkOf(line, nil)
	require.NoError(t, err)
	require.Equal(t, [4]float64{end.U, end.V, start.U, start.V},
		[4]float64{reversed.StartU, reversed.StartV, reversed.EndU, reversed.EndV})

	const cu, cv, radius = 10.3, 9.7, 4.7
	arc := ArcSeg{
		Center: Point2{U: cu, V: cv},
		Start:  Point2{U: cu + radius*math.Cos(0.05), V: cv + radius*math.Sin(0.05)},
		End:    Point2{U: cu + radius*math.Cos(0.75), V: cv + radius*math.Sin(0.75)},
		TEnd:   1,
	}
	walk, err = WalkOf(arc, nil)
	require.NoError(t, err)
	require.Equal(t, [4]float64{arc.Start.U, arc.Start.V, arc.End.U, arc.End.V},
		[4]float64{walk.StartU, walk.StartV, walk.EndU, walk.EndV})
	arc.TStart, arc.TEnd = 1, 0
	reversed, err = WalkOf(arc, nil)
	require.NoError(t, err)
	require.Equal(t, [4]float64{arc.End.U, arc.End.V, arc.Start.U, arc.Start.V},
		[4]float64{reversed.StartU, reversed.StartV, reversed.EndU, reversed.EndV})
}

// A trimmed line's walked point rounds the product t·(end − start) before
// adding start, on every target. The fragment is the chord of a radius-10
// half-disk cut at u = ±6 by a hollow ball's axis lines (sketch reports
// t = 0.2 and 0.8). Rounded separately, −10 + 0.8·20 is exactly 6, the start
// of the recorded axis line from (6, 0) to (10, 0), so the two walks
// coalesce into one axis wall. Fused into one multiply-add (arm64, or amd64
// built with GOAMD64=v3), the sum is one ulp above 6 and the walks stay split,
// which the revolve tessellation refuses as two on-axis segments at one
// junction.
//
// Shown to fail under GOAMD64=v3 with lerp2's float64() conversion removed:
// the walk ends at 6.000000000000001 and two walks remain.
func TestTrimmedLineWalkRoundsProductBeforeSum(t *testing.T) {
	t.Parallel()
	chord := LineSeg{Start: Point2{U: -10}, End: Point2{U: 10}, TStart: 0.2, TEnd: 0.8}
	walk, err := WalkOf(chord, nil)
	require.NoError(t, err)
	tEnd, span := chord.TEnd, chord.End.U-chord.Start.U
	product := float64(tEnd * span)
	require.Equal(t, math.Float64bits(chord.Start.U+product), math.Float64bits(walk.EndU))
	require.Equal(t, 6.0, walk.EndU)
	require.Equal(t, -6.0, walk.StartU)

	wall := LineSeg{Start: Point2{U: 6}, End: Point2{U: 10}, TEnd: 1}
	next, err := WalkOf(wall, nil)
	require.NoError(t, err)
	walks, err := CoalesceChainWalksContext(t.Context(), []survey2d.SideWalk{
		{SegmentWalk: walk, Segs: []int{0}},
		{SegmentWalk: next, Segs: []int{1}},
	})
	require.NoError(t, err)
	require.Len(t, walks, 1)
	require.Equal(t, []int{0, 1}, walks[0].Segs)
	require.Equal(t, [2]float64{-6, 10}, [2]float64{walks[0].StartU, walks[0].EndU})
}
