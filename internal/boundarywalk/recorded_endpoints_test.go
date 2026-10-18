package boundarywalk

import (
	"math"
	"testing"

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
