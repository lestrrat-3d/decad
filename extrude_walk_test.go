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
// Each arm states that premise before asserting the walk, and states it where
// it is decidable. The line's route is a chord formula over two exact float64
// constants, and its own multiply is by one — exact whether or not the build
// contracts the multiply-add — so whether that route misses follows from
// IEEE-754 arithmetic alone and one fixture can require it. The arc's route runs
// through atan2 and cos/sin, whose last bit does differ between platforms: a
// build that contracts a multiply-add rounds once where another rounds twice, so
// whether any single arc misses is a fact about the host rather than about this
// seam. The arc arm therefore measures the premise over the family the diagnosis
// covered, on the running platform, and requires that family to hold a miss;
// that is what keeps its walk assertions non-vacuous wherever the test runs.
//
// What depends on it: buildPrismScene (prism_boolean.go) creates one sketch
// point per walked endpoint, so a walk landing an ulp off the vertex two
// segments share would offer sketch two points where the record states one, and
// RecordProfile would then refuse the region the arrangement admits on its own
// proximity threshold.
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

		// Observation, not a requirement: whether THIS arc's own end angle
		// lands back on its recorded endpoint depends on the host's arithmetic.
		// The family below is what states the premise.
		t.Logf("this fixture's recorded arc end is %v; its walk's own end angle reaches %v",
			seg.End, walkEndFromModel(w))

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
		require.Equal(t, walkEndFromModel(part), Point2{U: part.endU, V: part.endV})

		// The premise, stated over the family the diagnosis measured rather
		// than over one fixture: across these arcs at least one walk's own end
		// angle misses its recorded endpoint, so at least one of the endpoint
		// assertions this loop makes would fail if a whole segment's walk
		// answered its circular model at that bound instead of the record.
		var missed, total int
		for _, c := range []struct{ cu, cv, r float64 }{
			{10.3, 9.7, 4.7},
			{-3.35, 0.15, 1.9},
			{0.7, -12.25, 21.3},
			{1000 + 1.0/3.0, 2.0 / 7.0, 0.35},
		} {
			for i := range 5 {
				a0 := 0.05 + 0.71*float64(i)
				for j := range 4 {
					a1 := a0 + 0.13 + 0.79*float64(j)
					fam := ArcSeg{
						Center: Point2{U: c.cu, V: c.cv},
						Start:  Point2{U: c.cu + c.r*math.Cos(a0), V: c.cv + c.r*math.Sin(a0)},
						End:    Point2{U: c.cu + c.r*math.Cos(a1), V: c.cv + c.r*math.Sin(a1)},
						TStart: 0, TEnd: 1,
					}
					fw, err := walkOf(fam, nil)
					require.NoError(t, err)
					require.Equal(t,
						[4]float64{fam.Start.U, fam.Start.V, fam.End.U, fam.End.V},
						[4]float64{fw.startU, fw.startV, fw.endU, fw.endV},
						`the walk states this arc's own recorded endpoints`)
					total++
					if walkEndFromModel(fw) != fam.End {
						missed++
					}
				}
			}
		}
		t.Logf("%d of %d arcs in the family have a walk end angle that misses the recorded endpoint", missed, total)
		require.Positive(t, missed,
			`premise: this platform's arc family holds an end angle that misses its recorded endpoint`)
	})
}

// walkEndFromModel re-derives a circular walk's far endpoint from the walk's OWN
// published model — its centre, radius and end angle — rather than from a second
// copy of walkOf's formula. It is the coordinate the walk would carry at that
// bound if it answered its model there instead of the record.
func walkEndFromModel(w segmentWalk) Point2 {
	sin, cos := math.Sincos(w.th1)
	return Point2{U: w.cU + w.radius*cos, V: w.cV + w.radius*sin}
}
