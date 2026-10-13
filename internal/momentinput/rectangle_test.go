package momentinput_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/stretchr/testify/require"
)

func rectangleLine(u0, v0, u1, v1, tStart, tEnd float64) sectionrecord.CurveSegment {
	return sectionrecord.LineSeg{
		Start: sectionrecord.Point2{U: u0, V: v0}, End: sectionrecord.Point2{U: u1, V: v1},
		TStart: tStart, TEnd: tEnd,
	}
}

func rectangleProfile(segments ...sectionrecord.CurveSegment) momentinput.Profile {
	return momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: segments}}
}

func TestRectangleBounds(t *testing.T) {
	// The CCW (0,0)-(2,1) loop, with the right side stored reversed.
	whole := rectangleProfile(
		rectangleLine(0, 0, 2, 0, 0, 1),
		rectangleLine(2, 0, 2, 1, 1, 0),
		rectangleLine(2, 1, 0, 1, 0, 1),
		rectangleLine(0, 1, 0, 0, 0, 1),
	)
	u0, u1, v0, v1, ok := momentinput.RectangleBounds(whole)
	require.True(t, ok)
	require.Equal(t, [4]float64{0, 2, 0, 1}, [4]float64{u0, u1, v0, v1})

	// A side that does not cover its whole line is not a rectangle side.
	partial := rectangleProfile(
		rectangleLine(0, 0, 2, 0, 0, 0.5),
		rectangleLine(2, 0, 2, 1, 0, 1),
		rectangleLine(2, 1, 0, 1, 0, 1),
		rectangleLine(0, 1, 0, 0, 0, 1),
	)
	u0, u1, v0, v1, ok = momentinput.RectangleBounds(partial)
	require.False(t, ok, "%v %v %v %v", u0, u1, v0, v1)

	// Two bottom sides plus right and left leave the top uncovered.
	duplicate := rectangleProfile(
		rectangleLine(0, 0, 2, 0, 0, 1),
		rectangleLine(2, 0, 2, 1, 0, 1),
		rectangleLine(2, 0, 0, 0, 0, 1),
		rectangleLine(0, 1, 0, 0, 0, 1),
	)
	u0, u1, v0, v1, ok = momentinput.RectangleBounds(duplicate)
	require.False(t, ok, "%v %v %v %v", u0, u1, v0, v1)
}
