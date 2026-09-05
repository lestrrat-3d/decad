package decad

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
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
	}, newFreeformWork())
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

func TestEvalCupIgnoresUnusedOverflowedSecondMoments(t *testing.T) {
	const outerSide = 1e100
	body, err := evalCup(New(), StepRef(0), cupPayload{
		outer:  overflowSquare(outerSide),
		cavity: overflowSquare(outerSide / 2),
		frame:  overflowFrame(t),
		zOpen:  2,
		zOuter: 0,
		zCav:   1,
		xform:  r3.Identity(),
	})
	require.NoError(t, err)
	require.NotNil(t, body)
	require.True(t, body.IsSolid())

	volume, err := body.Volume()
	require.NoError(t, err)
	got, err := volume.Value.In(units.CubicMillimeter)
	require.NoError(t, err)
	require.False(t, math.IsInf(got, 0))
	require.InDelta(t, 1.75e200, got, 1e185)
}

func TestAnalyticCircularEdgesCarryLengthBounds(t *testing.T) {
	circle := ProfileRecord{Outer: LoopRecord{Segments: []CurveSegment{
		CircleSeg{
			Center: Point2{},
			Radius: units.Millimeters(10),
			TStart: 0,
			TEnd:   1,
			CCW:    true,
		},
	}}}
	prism, err := evalPrism(New(), StepRef(0), prismPayload{
		profile: circle,
		frame:   overflowFrame(t),
		z1:      2,
		xform:   r3.Identity(),
	}, newFreeformWork())
	require.NoError(t, err)
	circular := 0
	for _, edge := range prism.Edges() {
		if _, ok := edge.Curve().(Circle3); !ok {
			continue
		}
		circular++
		length, err := edge.Length()
		require.NoError(t, err)
		require.Positive(t, length.Bound.Mag())
	}
	require.Equal(t, 2, circular)

	revolveProfile := ProfileRecord{Outer: LoopRecord{Segments: []CurveSegment{
		LineSeg{Start: Point2{U: 0, V: 5}, End: Point2{U: 10, V: 5}, TStart: 0, TEnd: 1},
		LineSeg{Start: Point2{U: 10, V: 5}, End: Point2{U: 10, V: 15}, TStart: 0, TEnd: 1},
		LineSeg{Start: Point2{U: 10, V: 15}, End: Point2{U: 0, V: 15}, TStart: 0, TEnd: 1},
		LineSeg{Start: Point2{U: 0, V: 15}, End: Point2{U: 0, V: 5}, TStart: 0, TEnd: 1},
	}}}
	rp := revolvePayload{
		profile: revolveProfile,
		frame:   overflowFrame(t),
		ax:      axisFrame{dU: 1},
		phi0:    0,
		phi1:    2 * math.Pi,
		full:    true,
		xform:   r3.Identity(),
	}
	full, err := evalRevolve(New(), StepRef(0), rp)
	require.NoError(t, err)
	for _, edge := range full.Edges() {
		if _, ok := edge.Curve().(Circle3); !ok {
			continue
		}
		length, err := edge.Length()
		require.NoError(t, err)
		require.Positive(t, length.Bound.Mag())
	}

	rp.phi1 = math.Pi / 2
	rp.full = false
	partial, err := evalRevolve(New(), StepRef(0), rp)
	require.NoError(t, err)
	for _, edge := range partial.Edges() {
		if _, ok := edge.Curve().(Arc3); !ok {
			continue
		}
		length, err := edge.Length()
		require.NoError(t, err)
		require.Positive(t, length.Bound.Mag())
	}
}

func TestCoalescedAnalyticEdgesCarryLengthBounds(t *testing.T) {
	profile := ProfileRecord{Outer: LoopRecord{Segments: []CurveSegment{
		LineSeg{Start: Point2{U: 0, V: 0}, End: Point2{U: 1, V: 1}, TStart: 0, TEnd: 1},
		LineSeg{Start: Point2{U: 1, V: 1}, End: Point2{U: 2, V: 2}, TStart: 0, TEnd: 1},
		LineSeg{Start: Point2{U: 2, V: 2}, End: Point2{U: 1, V: 3}, TStart: 0, TEnd: 1},
		LineSeg{Start: Point2{U: 1, V: 3}, End: Point2{U: 0, V: 2}, TStart: 0, TEnd: 1},
		LineSeg{Start: Point2{U: 0, V: 2}, End: Point2{U: 0, V: 0}, TStart: 0, TEnd: 1},
	}}}
	body, err := evalPrism(New(), StepRef(0), prismPayload{
		profile: profile,
		frame:   overflowFrame(t),
		z1:      1,
		xform:   r3.Identity(),
	}, newFreeformWork())
	require.NoError(t, err)
	coalesced := 0
	for _, edge := range body.Edges() {
		length, err := edge.Length()
		require.NoError(t, err)
		got, err := length.Value.In(units.Millimeter)
		require.NoError(t, err)
		if math.Abs(got-2*math.Sqrt2) > 1e-12 {
			continue
		}
		coalesced++
		require.Positive(t, length.Bound.Mag())
	}
	require.Equal(t, 2, coalesced)
}
