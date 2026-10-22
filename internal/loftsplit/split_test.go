package loftsplit_test

import (
	"testing"

	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/loftsplit"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
)

func square() momentinput.Profile {
	p := func(u, v float64) sectionrecord.Point2 { return sectionrecord.Point2{U: u, V: v} }
	return momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
		sectionrecord.LineSeg{Start: p(0, 0), End: p(4, 0), TStart: 0, TEnd: 1},
		sectionrecord.LineSeg{Start: p(4, 0), End: p(4, 4), TStart: 0, TEnd: 1},
		sectionrecord.LineSeg{Start: p(4, 4), End: p(0, 4), TStart: 0, TEnd: 1},
		sectionrecord.LineSeg{Start: p(0, 4), End: p(0, 0), TStart: 0, TEnd: 1},
	}}}
}

func TestApplyCutsASegmentIntoSubRangesOfTheSameCurve(t *testing.T) {
	t.Parallel()
	in := square()
	out, err := loftsplit.Apply(in, []loftsplit.Split{{Loop: 0, Segment: 1, At: []float64{0.25, 0.5}}})
	require.NoError(t, err)
	require.Len(t, out.Outer.Segments, 6)
	want := [][2]float64{{0, 1}, {0, 0.25}, {0.25, 0.5}, {0.5, 1}, {0, 1}, {0, 1}}
	for i, seg := range out.Outer.Segments {
		line := seg.(sectionrecord.LineSeg)
		require.Equal(t, want[i], [2]float64{line.TStart, line.TEnd}, "segment %d", i)
		if i >= 1 && i <= 3 {
			require.Equal(t, sectionrecord.Point2{U: 4, V: 0}, line.Start, "a piece keeps the entity's own end points")
			require.Equal(t, sectionrecord.Point2{U: 4, V: 4}, line.End)
		}
	}
	require.Len(t, in.Outer.Segments, 4, "the input is not modified")
}

func TestApplyKeepsTheWalkOrderOfAReversedRange(t *testing.T) {
	t.Parallel()
	in := square()
	back := in.Outer.Segments[0].(sectionrecord.LineSeg)
	back.TStart, back.TEnd = 1, 0.5
	in.Outer.Segments[0] = back
	out, err := loftsplit.Apply(in, []loftsplit.Split{{Loop: 0, Segment: 0, At: []float64{0.5}}})
	require.NoError(t, err)
	first, second := out.Outer.Segments[0].(sectionrecord.LineSeg), out.Outer.Segments[1].(sectionrecord.LineSeg)
	require.Equal(t, [2]float64{1, 0.75}, [2]float64{first.TStart, first.TEnd})
	require.Equal(t, [2]float64{0.75, 0.5}, [2]float64{second.TStart, second.TEnd})
}

func TestApplyKeepsTheWindingOfAClosedKind(t *testing.T) {
	t.Parallel()
	circle := sectionrecord.CircleSeg{Radius: mmValue(2), CCW: true, TStart: 0, TEnd: 1}
	out, err := loftsplit.Apply(momentinput.Profile{Outer: sectionrecord.LoopRecord{
		Segments: []sectionrecord.CurveSegment{circle}}}, []loftsplit.Split{{Loop: 0, Segment: 0, At: []float64{0.5}}})
	require.NoError(t, err)
	require.Len(t, out.Outer.Segments, 2)
	for _, seg := range out.Outer.Segments {
		require.True(t, seg.(sectionrecord.CircleSeg).CCW)
	}
}

func TestApplyDoesNotShareASplitSegmentsSlices(t *testing.T) {
	t.Parallel()
	fit := sectionrecord.FitSplineSeg{Fit: []sectionrecord.Point2{{U: 0, V: 0}, {U: 1, V: 1}, {U: 2, V: 0}}, TStart: 0, TEnd: 1}
	in := momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{fit}}}
	out, err := loftsplit.Apply(in, []loftsplit.Split{{Loop: 0, Segment: 0, At: []float64{0.5}}})
	require.NoError(t, err)
	out.Outer.Segments[0].(sectionrecord.FitSplineSeg).Fit[0].U = 99
	require.Equal(t, float64(0), in.Outer.Segments[0].(sectionrecord.FitSplineSeg).Fit[0].U)
	require.Equal(t, float64(0), out.Outer.Segments[1].(sectionrecord.FitSplineSeg).Fit[0].U)
}

func TestApplyRefusals(t *testing.T) {
	t.Parallel()
	for name, splits := range map[string][]loftsplit.Split{
		"loop out of range":    {{Loop: 1, Segment: 0, At: []float64{0.5}}},
		"segment out of range": {{Loop: 0, Segment: 4, At: []float64{0.5}}},
		"no fraction":          {{Loop: 0, Segment: 0}},
		"zero":                 {{Loop: 0, Segment: 0, At: []float64{0}}},
		"one":                  {{Loop: 0, Segment: 0, At: []float64{1}}},
		"decreasing":           {{Loop: 0, Segment: 0, At: []float64{0.6, 0.4}}},
		"repeated":             {{Loop: 0, Segment: 0, At: []float64{0.5, 0.5}}},
		"split twice":          {{Loop: 0, Segment: 0, At: []float64{0.5}}, {Loop: 0, Segment: 0, At: []float64{0.7}}},
	} {
		_, err := loftsplit.Apply(square(), splits)
		require.ErrorIs(t, err, decaderr.ErrDegenerate, name)
	}
	// A fraction too close to the end rounds onto it.
	_, err := loftsplit.Apply(square(), []loftsplit.Split{{Loop: 0, Segment: 0, At: []float64{1 - 1e-17}}})
	require.ErrorIs(t, err, decaderr.ErrDegenerate)
}

func mmValue(v float64) units.Value { return units.Millimeters(v) }
