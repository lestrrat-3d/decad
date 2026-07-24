package decad

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func overflowSquare(side float64) ProfileRecord {
	line := func(u0, v0, u1, v1 float64) CurveSegment {
		return LineSeg{
			Start:  Point2{U: u0, V: v0},
			End:    Point2{U: u1, V: v1},
			TStart: 0,
			TEnd:   1,
		}
	}
	return ProfileRecord{Outer: LoopRecord{Segments: []CurveSegment{
		line(0, 0, side, 0),
		line(side, 0, side, side),
		line(side, side, 0, side),
		line(0, side, 0, 0),
	}}}
}

func overflowFrame(t *testing.T) r3.Frame {
	t.Helper()
	frame, err := r3.NewFrame(
		r3.NewVec(0, 0, 0),
		r3.NewVec(1, 0, 0),
		r3.NewVec(0, 1, 0),
	)
	require.NoError(t, err)
	return frame
}

func TestEvalPrismRejectsOverflowedMeasurements(t *testing.T) {
	profile := overflowSquare(1e154)
	body, err := evalPrism(New(), StepRef(0), prismPayload{
		profile: profile,
		frame:   overflowFrame(t),
		z1:      100,
		xform:   r3.Identity(),
	})
	require.ErrorIs(t, err, ErrNotFinite)
	require.Nil(t, body)
}

func TestEvalRevolveRejectsOverflowedMeasurements(t *testing.T) {
	profile := ProfileRecord{Outer: LoopRecord{Segments: []CurveSegment{
		LineSeg{Start: Point2{U: 1e154}, End: Point2{U: 2e154}, TStart: 0, TEnd: 1},
		LineSeg{Start: Point2{U: 2e154}, End: Point2{U: 2e154, V: 1}, TStart: 0, TEnd: 1},
		LineSeg{Start: Point2{U: 2e154, V: 1}, End: Point2{U: 1e154, V: 1}, TStart: 0, TEnd: 1},
		LineSeg{Start: Point2{U: 1e154, V: 1}, End: Point2{U: 1e154}, TStart: 0, TEnd: 1},
	}}}
	body, err := evalRevolve(New(), StepRef(0), revolvePayload{
		profile: profile,
		frame:   overflowFrame(t),
		ax: axisFrame{
			dV: -1,
		},
		phi1:  2 * math.Pi,
		full:  true,
		xform: r3.Identity(),
	})
	require.ErrorIs(t, err, ErrNotFinite)
	require.Nil(t, body)
}

func TestEvalCupRejectsOverflowedMeasurements(t *testing.T) {
	body, err := evalCup(New(), StepRef(0), cupPayload{
		outer:  overflowSquare(10),
		cavity: overflowSquare(1),
		frame:  overflowFrame(t),
		zOpen:  1e307,
		zCav:   1,
		xform:  r3.Identity(),
	})
	require.ErrorIs(t, err, ErrNotFinite)
	require.Nil(t, body)
}
