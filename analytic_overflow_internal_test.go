package decad

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func overflowSquare(side float64) profileRecord {
	line := func(u0, v0, u1, v1 float64) curveSegment {
		return lineSeg{
			Start:  planePoint{U: u0, V: v0},
			End:    planePoint{U: u1, V: v1},
			TStart: 0,
			TEnd:   1,
		}
	}
	return profileRecord{Outer: loopRecord{Segments: []curveSegment{
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
	t.Parallel()
	profile := overflowSquare(1e154)
	body, err := evalPrism(New(), producerID(0), prismrecord.Payload{
		Profile: profile,
		Frame:   overflowFrame(t),
		Z1:      100,
		Xform:   r3.Identity(),
	}, freeform.NewFreeformWork())
	require.ErrorIs(t, err, ErrNotFinite)
	require.Nil(t, body)
}

func TestEvalRevolveRejectsOverflowedMeasurements(t *testing.T) {
	t.Parallel()
	profile := profileRecord{Outer: loopRecord{Segments: []curveSegment{
		lineSeg{Start: planePoint{U: 1e154}, End: planePoint{U: 2e154}, TStart: 0, TEnd: 1},
		lineSeg{Start: planePoint{U: 2e154}, End: planePoint{U: 2e154, V: 1}, TStart: 0, TEnd: 1},
		lineSeg{Start: planePoint{U: 2e154, V: 1}, End: planePoint{U: 1e154, V: 1}, TStart: 0, TEnd: 1},
		lineSeg{Start: planePoint{U: 1e154, V: 1}, End: planePoint{U: 1e154}, TStart: 0, TEnd: 1},
	}}}
	body, err := evalRevolve(New(), producerID(0), revolvePayload{
		Profile: profile,
		Frame:   overflowFrame(t),
		Ax: axisFrame{
			DV: -1,
		},
		Phi1:  2 * math.Pi,
		Full:  true,
		Xform: r3.Identity(),
	})
	require.ErrorIs(t, err, ErrNotFinite)
	require.Nil(t, body)
}

func TestEvalCupRejectsOverflowedMeasurements(t *testing.T) {
	t.Parallel()
	body, err := evalCup(New(), producerID(0), cupView{
		Outer:  overflowSquare(10),
		Cavity: overflowSquare(1),
		Frame:  overflowFrame(t),
		ZOpen:  1e307,
		ZCav:   1,
		Xform:  r3.Identity(),
	})
	require.ErrorIs(t, err, ErrNotFinite)
	require.Nil(t, body)
}

func TestEvalCupIgnoresUnusedOverflowedSecondMoments(t *testing.T) {
	t.Parallel()
	const outerSide = 1e100
	body, err := evalCup(New(), producerID(0), cupView{
		Outer:  overflowSquare(outerSide),
		Cavity: overflowSquare(outerSide / 2),
		Frame:  overflowFrame(t),
		ZOpen:  2,
		ZOuter: 0,
		ZCav:   1,
		Xform:  r3.Identity(),
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
	t.Parallel()
	circle := profileRecord{Outer: loopRecord{Segments: []curveSegment{
		circleSeg{
			Center: planePoint{},
			Radius: units.Millimeters(10),
			TStart: 0,
			TEnd:   1,
			CCW:    true,
		},
	}}}
	prism, err := evalPrism(New(), producerID(0), prismrecord.Payload{
		Profile: circle,
		Frame:   overflowFrame(t),
		Z1:      2,
		Xform:   r3.Identity(),
	}, freeform.NewFreeformWork())
	require.NoError(t, err)
	circular := 0
	for _, edge := range prism.Edges() {
		if _, ok := edge.n.Curve.(surfacegeom.Circle3); !ok {
			continue
		}
		circular++
		length, err := edge.Length()
		require.NoError(t, err)
		require.Positive(t, length.Bound.Mag())
	}
	require.Equal(t, 2, circular)

	revolveProfile := profileRecord{Outer: loopRecord{Segments: []curveSegment{
		lineSeg{Start: planePoint{U: 0, V: 5}, End: planePoint{U: 10, V: 5}, TStart: 0, TEnd: 1},
		lineSeg{Start: planePoint{U: 10, V: 5}, End: planePoint{U: 10, V: 15}, TStart: 0, TEnd: 1},
		lineSeg{Start: planePoint{U: 10, V: 15}, End: planePoint{U: 0, V: 15}, TStart: 0, TEnd: 1},
		lineSeg{Start: planePoint{U: 0, V: 15}, End: planePoint{U: 0, V: 5}, TStart: 0, TEnd: 1},
	}}}
	rp := revolvePayload{
		Profile: revolveProfile,
		Frame:   overflowFrame(t),
		Ax:      axisFrame{DU: 1},
		Phi0:    0,
		Phi1:    2 * math.Pi,
		Full:    true,
		Xform:   r3.Identity(),
	}
	full, err := evalRevolve(New(), producerID(0), rp)
	require.NoError(t, err)
	for _, edge := range full.Edges() {
		if _, ok := edge.n.Curve.(surfacegeom.Circle3); !ok {
			continue
		}
		length, err := edge.Length()
		require.NoError(t, err)
		require.Positive(t, length.Bound.Mag())
	}

	rp.Phi1 = math.Pi / 2
	rp.Full = false
	partial, err := evalRevolve(New(), producerID(0), rp)
	require.NoError(t, err)
	for _, edge := range partial.Edges() {
		if _, ok := edge.n.Curve.(surfacegeom.Arc3); !ok {
			continue
		}
		length, err := edge.Length()
		require.NoError(t, err)
		require.Positive(t, length.Bound.Mag())
	}
}

func TestCoalescedAnalyticEdgesCarryLengthBounds(t *testing.T) {
	t.Parallel()
	profile := profileRecord{Outer: loopRecord{Segments: []curveSegment{
		lineSeg{Start: planePoint{U: 0, V: 0}, End: planePoint{U: 1, V: 1}, TStart: 0, TEnd: 1},
		lineSeg{Start: planePoint{U: 1, V: 1}, End: planePoint{U: 2, V: 2}, TStart: 0, TEnd: 1},
		lineSeg{Start: planePoint{U: 2, V: 2}, End: planePoint{U: 1, V: 3}, TStart: 0, TEnd: 1},
		lineSeg{Start: planePoint{U: 1, V: 3}, End: planePoint{U: 0, V: 2}, TStart: 0, TEnd: 1},
		lineSeg{Start: planePoint{U: 0, V: 2}, End: planePoint{U: 0, V: 0}, TStart: 0, TEnd: 1},
	}}}
	body, err := evalPrism(New(), producerID(0), prismrecord.Payload{
		Profile: profile,
		Frame:   overflowFrame(t),
		Z1:      1,
		Xform:   r3.Identity(),
	}, freeform.NewFreeformWork())
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
