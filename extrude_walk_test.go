package decad

import (
	"math"
	"testing"

	"github.com/stretchr/testify/require"
)

// A recorded segment's walk states the record's own endpoints at the two
// natural bounds. Each analytic kind's parameterization runs Start → End over
// [0, 1] (record.go), so its value at t = 0 is Start and at t = 1 is End,
// exactly — while the float routes to those points (a line's chord lerp, an
// arc's atan2 plus sweep) need not land back on them.
//
// Both fixtures below are chosen so that route MISSES, and each states that
// premise before asserting the walk. What depends on it: buildPrismScene
// (prism_boolean.go) creates one sketch point per walked endpoint, so a walk
// landing an ulp off the vertex two segments share would offer sketch two points
// where the record states one, and RecordProfile would then refuse the region
// the arrangement admits on its own proximity threshold.
func TestWholeSegmentWalkStatesTheRecordedEndpoints(t *testing.T) {
	t.Run("line", func(t *testing.T) {
		// 4/7 → 10/3 is a pair the chord formula does not round-trip: the
		// difference needs a finer grid than its own exponent carries, so it
		// rounds, and start + (end − start) then lands one ulp short of end.
		start := Point2{U: 4.0 / 7.0, V: 2}
		end := Point2{U: 10.0 / 3.0, V: 2}
		require.NotEqual(t, end.U, start.U+1.0*(end.U-start.U),
			`premise: evaluating the chord formula at t = 1 misses this endpoint`)

		u0, v0 := lerp2(start, end, 0)
		require.Equal(t, start, Point2{U: u0, V: v0})
		u1, v1 := lerp2(start, end, 1)
		require.Equal(t, end, Point2{U: u1, V: v1})

		// An interior parameter names no recorded coordinate, so the formula
		// remains what states it.
		uq, vq := lerp2(start, end, 0.25)
		require.Equal(t, start.U+0.25*(end.U-start.U), uq)
		require.Equal(t, start.V+0.25*(end.V-start.V), vq)

		w, err := walkOf(LineSeg{Start: start, End: end, TStart: 0, TEnd: 1}, nil)
		require.NoError(t, err)
		require.Equal(t,
			[4]float64{start.U, start.V, end.U, end.V},
			[4]float64{w.startU, w.startV, w.endU, w.endV})

		// A reversed whole edge records TStart = 1, TEnd = 0 (seam.go), so the
		// walk's own ends swap while each still states a recorded coordinate.
		rev, err := walkOf(LineSeg{Start: start, End: end, TStart: 1, TEnd: 0}, nil)
		require.NoError(t, err)
		require.Equal(t,
			[4]float64{end.U, end.V, start.U, start.V},
			[4]float64{rev.startU, rev.startV, rev.endU, rev.endV})
	})

	t.Run("arc", func(t *testing.T) {
		const cu, cv, r = 10.3, 9.7, 4.7
		th0, th1 := 0.05, 0.75
		seg := ArcSeg{
			Center: Point2{U: cu, V: cv},
			Start:  Point2{U: cu + r*math.Cos(th0), V: cv + r*math.Sin(th0)},
			End:    Point2{U: cu + r*math.Cos(th1), V: cv + r*math.Sin(th1)},
			TStart: 0, TEnd: 1,
		}

		w, err := walkOf(seg, nil)
		require.NoError(t, err)

		// Premise, read off the walk's OWN published circular model rather than
		// a second copy of walkOf's formula: evaluating that model at the end
		// angle it derived does not land back on the recorded endpoint.
		sin, cos := math.Sincos(w.th1)
		require.NotEqual(t, seg.End, Point2{U: w.cU + w.radius*cos, V: w.cV + w.radius*sin},
			`premise: the walk's own end angle misses this recorded endpoint`)

		require.Equal(t,
			[4]float64{seg.Start.U, seg.Start.V, seg.End.U, seg.End.V},
			[4]float64{w.startU, w.startV, w.endU, w.endV})

		rev, err := walkOf(ArcSeg{
			Center: seg.Center, Start: seg.Start, End: seg.End,
			TStart: 1, TEnd: 0,
		}, nil)
		require.NoError(t, err)
		require.Equal(t,
			[4]float64{seg.End.U, seg.End.V, seg.Start.U, seg.Start.V},
			[4]float64{rev.startU, rev.startV, rev.endU, rev.endV})

		// A trimmed bound keeps the circular model's own value: the record
		// states no coordinate there, and this seam never invents one.
		part, err := walkOf(ArcSeg{
			Center: seg.Center, Start: seg.Start, End: seg.End,
			TStart: 0, TEnd: 0.5,
		}, nil)
		require.NoError(t, err)
		require.Equal(t, [2]float64{seg.Start.U, seg.Start.V}, [2]float64{part.startU, part.startV})
		psin, pcos := math.Sincos(part.th1)
		require.Equal(t,
			[2]float64{part.cU + part.radius*pcos, part.cV + part.radius*psin},
			[2]float64{part.endU, part.endV})
	})
}
