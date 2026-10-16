package momentinput

import (
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func captureRectangle(trimmed, reversed bool) Profile {
	points := []Point2{{}, {U: 12}, {U: 12, V: 10}, {V: 10}}
	loop := LoopRecord{}
	for i, a := range points {
		b := points[(i+1)%len(points)]
		line := LineSeg{Start: a, End: b, TEnd: 1}
		if trimmed {
			du, dv := b.U-a.U, b.V-a.V
			line.Start = Point2{U: a.U - du/2, V: a.V - dv/2}
			line.End = Point2{U: b.U + du/2, V: b.V + dv/2}
			line.TStart, line.TEnd = .25, .75
		}
		if reversed {
			line.Start, line.End = line.End, line.Start
			line.TStart, line.TEnd = 1-line.TStart, 1-line.TEnd
		}
		loop.Segments = append(loop.Segments, line)
	}
	return Profile{Outer: loop}
}

func captureWalkFloats(w survey2d.SegmentWalk) []float64 {
	return []float64{w.StartU, w.StartV, w.EndU, w.EndV, w.StartBound.U, w.StartBound.V, w.EndBound.U, w.EndBound.V,
		w.TanInU, w.TanInV, w.TanOutU, w.TanOutV, w.TanInBound, w.TanOutBound, w.Length, w.LengthBound, w.LengthUpper,
		w.CoordUpper, w.AxisRadiusUpper, w.AxisMomentUpper, w.StartVBound, w.EndVBound, w.CVBound, w.CU, w.CV,
		w.Radius, w.RadiusBound, w.Th0, w.Th1}
}

func requireCapturedWalk(t *testing.T, want, got survey2d.SegmentWalk) {
	t.Helper()
	a, b := captureWalkFloats(want), captureWalkFloats(got)
	for i := range a {
		require.Equal(t, math.Float64bits(a[i]), math.Float64bits(b[i]), "walk float %d", i)
	}
	require.Equal(t, want.Kind, got.Kind)
	require.Equal(t, want.Closed, got.Closed)
	require.Equal(t, want.Reversed, got.Reversed)
	require.Equal(t, want.FitInterpolated, got.FitInterpolated)
	require.True(t, want.Spans == nil && got.Spans == nil, "analytic spans must be nil")
}

func TestAnalyticWalkCaptureMatchesResolution(t *testing.T) {
	circle := Profile{Outer: LoopRecord{Segments: []CurveSegment{CircleSeg{
		Center: Point2{U: 3, V: 4}, Radius: units.Millimeters(10), CCW: true, TEnd: 1}}}}
	hole := CircleSeg{Center: Point2{U: 3, V: 4}, Radius: units.Millimeters(2), CCW: false, TStart: 1}
	circle.Holes = []LoopRecord{{Segments: []CurveSegment{hole}}}
	arc := Profile{Outer: LoopRecord{Segments: []CurveSegment{
		ArcSeg{Start: Point2{U: 2}, End: Point2{U: -2}, TEnd: 1},
		LineSeg{Start: Point2{U: -2}, End: Point2{U: 2}, TEnd: 1},
	}}}
	trimmedArc := arc
	trimmedArc.Outer.Segments = append([]CurveSegment(nil), arc.Outer.Segments...)
	trimmedArc.Outer.Segments[0] = ArcSeg{Start: Point2{U: 2}, End: Point2{U: -2}, TStart: .125, TEnd: .875}
	trimmedCircle := Profile{Outer: LoopRecord{Segments: []CurveSegment{
		CircleSeg{Radius: units.Millimeters(2), CCW: true, TEnd: .5},
		LineSeg{Start: Point2{U: -2}, End: Point2{U: 2}, TEnd: 1},
	}}}
	withLineHole := captureRectangle(true, false)
	withLineHole.Holes = []LoopRecord{{Segments: []CurveSegment{
		LineSeg{Start: Point2{U: 2, V: 2}, End: Point2{U: 2, V: 3}, TEnd: 1},
		LineSeg{Start: Point2{U: 2, V: 3}, End: Point2{U: 3, V: 3}, TEnd: 1},
		LineSeg{Start: Point2{U: 3, V: 3}, End: Point2{U: 3, V: 2}, TEnd: 1},
		LineSeg{Start: Point2{U: 3, V: 2}, End: Point2{U: 2, V: 2}, TEnd: 1},
	}}}
	for i, record := range []Profile{captureRectangle(false, false), captureRectangle(true, false), captureRectangle(true, true),
		circle, arc, trimmedArc, trimmedCircle, withLineHole} {
		for _, nilWork := range []bool{false, true} {
			work := freeform.NewFreeformWork()
			if nilWork {
				work = nil
			} else {
				work.Spent = freeform.FreeformWorkLimit
				work.ReconstructionSpent = freeform.ReconstructionWorkLimit
			}
			ig, captured, err := record.EvaluatorIntegralsWithAnalyticWalksContext(t.Context(), freeform.MomentFirstOrder, work)
			require.NoError(t, err, "case %d", i)
			require.NotNil(t, captured, "case %d", i)
			require.True(t, captured.Reusable(record), "case %d", i)
			require.Zero(t, captured.Spent)
			require.Zero(t, captured.ReconstructionSpent)
			require.Nil(t, captured.ReadCharges)
			want, err := ResolveProfileWalks(record, work)
			require.NoError(t, err)
			for li, loop := range append([]LoopRecord{record.Outer}, record.Holes...) {
				require.Equal(t, len(loop.Segments), len(captured.LoopWalks(li)))
				for si := range loop.Segments {
					requireCapturedWalk(t, want.At(li, si), captured.At(li, si))
				}
			}
			old, err := record.EvaluatorIntegralsUncheckedContext(t.Context(), freeform.MomentFirstOrder, work)
			require.NoError(t, err)
			for k, pair := range [][2]float64{{old.Area, ig.Area}, {old.AreaBound, ig.AreaBound}, {old.Mu, ig.Mu},
				{old.MuBound, ig.MuBound}, {old.Mv, ig.Mv}, {old.MvBound, ig.MvBound}, {old.CoordUpper, ig.CoordUpper}} {
				require.Equal(t, math.Float64bits(pair[0]), math.Float64bits(pair[1]), "integral field %d", k)
			}
			if work != nil {
				require.Equal(t, freeform.FreeformWorkLimit, work.Spent)
				require.Equal(t, freeform.ReconstructionWorkLimit, work.ReconstructionSpent)
			}
		}
	}
}

func TestAnalyticWalkCaptureDisqualification(t *testing.T) {
	record := captureRectangle(false, false)
	pointer := record
	pointer.Outer.Segments = append([]CurveSegment(nil), record.Outer.Segments...)
	line := pointer.Outer.Segments[0].(LineSeg)
	pointer.Outer.Segments[0] = &line
	unitsRecord := Profile{Outer: LoopRecord{Segments: []CurveSegment{CircleSeg{Radius: units.Centimeters(1), CCW: true, TEnd: 1}}}}
	for _, input := range []Profile{pointer, unitsRecord} {
		_, got, err := input.EvaluatorIntegralsWithAnalyticWalksContext(t.Context(), freeform.MomentFirstOrder, nil)
		require.NoError(t, err)
		require.NotNil(t, got)
		require.False(t, got.Reusable(input), "normalization must not weaken identity")
	}
	// An analytic prefix allocated capture arrays before the free-form segment.
	mixed := record
	mixed.Outer.Segments = append([]CurveSegment(nil), record.Outer.Segments...)
	mixed.Outer.Segments[1] = FitSplineSeg{Fit: []Point2{{U: 12}, {U: 12, V: 10}}, TEnd: 1}
	capture := analyticWalkCapture{source: mixed}
	pre, err := validateFieldsWithCapture(nil, mixed, nil, &capture)
	require.NoError(t, err)
	require.True(t, capture.disabled)
	require.Nil(t, capture.outer)
	require.Nil(t, capture.holes)
	require.Empty(t, capture.source.Outer.Segments)
	require.Nil(t, capture.take(pre.Record))
	_, got, err := mixed.EvaluatorIntegralsWithAnalyticWalksContext(t.Context(), freeform.MomentFirstOrder, nil)
	require.NoError(t, err)
	require.Nil(t, got)
	// Either measured counter changing disqualifies capture permanently.
	for _, deltas := range [][2]uint64{{1, 0}, {0, 1}} {
		c := analyticWalkCapture{source: record}
		walk, err := boundarywalk.WalkOf(record.Outer.Segments[0], nil)
		require.NoError(t, err)
		c.retain(walk, 7, 11, 7, 11)
		require.NotNil(t, c.outer)
		c.segmentIndex = 1
		c.retain(walk, 7, 11, 7+deltas[0], 11+deltas[1])
		require.True(t, c.disabled)
		require.Nil(t, c.outer)
		c.retain(walk, 7, 11, 7, 11)
		require.Nil(t, c.outer)
	}
	c := analyticWalkCapture{source: record}
	require.Nil(t, c.outer, "capture allocation is lazy")
	require.Nil(t, c.take(record), "an incomplete set cannot be published")
}

type capturePollContext struct {
	context.Context //nolint:containedctx // deterministic cancellation wrapper used only within one test call.
	calls, cancelAt int
}

func (c *capturePollContext) Err() error {
	c.calls++
	if c.calls == c.cancelAt {
		return context.Canceled
	}
	return nil
}

func TestAnalyticWalkCapturePreservesErrorsAndPolling(t *testing.T) {
	record := captureRectangle(true, false)
	for stop := 1; stop <= 12; stop++ {
		a := &capturePollContext{Context: t.Context(), cancelAt: stop}
		b := &capturePollContext{Context: t.Context(), cancelAt: stop}
		_, oldErr := record.EvaluatorIntegralsUncheckedContext(a, freeform.MomentFirstOrder, nil)
		_, capture, newErr := record.EvaluatorIntegralsWithAnalyticWalksContext(b, freeform.MomentFirstOrder, nil)
		require.Equal(t, a.calls, b.calls)
		require.Equal(t, oldErr == nil, newErr == nil)
		if oldErr != nil {
			require.EqualError(t, newErr, oldErr.Error())
			require.Nil(t, capture)
		}
	}
	malformed := record
	malformed.Outer.Segments = append([]CurveSegment(nil), record.Outer.Segments...)
	malformed.Outer.Segments[1] = LineSeg{Start: Point2{U: math.NaN()}, TEnd: 1}
	_, oldErr := malformed.EvaluatorIntegralsUncheckedContext(t.Context(), freeform.MomentFirstOrder, nil)
	_, capture, newErr := malformed.EvaluatorIntegralsWithAnalyticWalksContext(t.Context(), freeform.MomentFirstOrder, nil)
	require.ErrorIs(t, newErr, ErrNotFinite)
	require.EqualError(t, newErr, oldErr.Error())
	require.Nil(t, capture)
	// With a nil caller, the actual preflight work object is created internally.
	c := analyticWalkCapture{source: record}
	pre, err := validateFieldsWithCapture(nil, record, nil, &c)
	require.NoError(t, err)
	require.NotNil(t, pre.Work)
	require.Zero(t, pre.Work.Spent)
	require.Zero(t, pre.Work.ReconstructionSpent)
	require.NotNil(t, c.take(pre.Record))
}
