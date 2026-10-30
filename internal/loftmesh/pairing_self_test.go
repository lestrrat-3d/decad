package loftmesh

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/stretchr/testify/require"
)

func selfPairFixture() (sectionrecord.CurveSegment, survey2d.SegmentWalk) {
	point := func(u, d int64) freeform.RatPoint {
		return freeform.RatPoint{U: big.NewRat(u, d), V: new(big.Rat)}
	}
	span := freeform.BezierSpan{point(0, 1), point(1, 1000), point(1, 1000), point(1, 1)}
	segment := sectionrecord.SplineSeg{
		Control: []sectionrecord.Point2{{U: 0}, {U: 1.0 / 3}, {U: 2.0 / 3}, {U: 1}},
		TStart:  0,
		TEnd:    1,
	}
	walk := survey2d.SegmentWalk{Kind: survey2d.WalkFreeform, Spans: []freeform.BezierSpan{span}}
	return segment, walk
}

func TestSelfFreeformCellMatchesTwoSideAndKeepsIndependentOutputs(t *testing.T) {
	segment, walk := selfPairFixture()
	work0, work1 := freeform.NewFreeformWork(), freeform.NewFreeformWork()
	want, err := FreeformCellPoints(walk, walk, 1e-2, 128, work0, work1)
	require.NoError(t, err)

	got0, got1 := freeform.NewFreeformWork(), freeform.NewFreeformWork()
	got, reused := selfFreeformCell(segment, segment, walk, walk, 1e-2, 128, got0, got1)
	require.True(t, reused)
	require.Equal(t, want, got)
	require.Equal(t, *work0, *got0, "the first record pays the original one-side charge")
	require.Equal(t, *work1, *got1, "the second record pays its own one-side charge")
	require.Greater(t, len(got.MatchedDelta), 1, "the matched-departure rule still bisects the span")

	otherStation := got.Stations1[0]
	got.Stations0[0].U = -100
	require.Equal(t, otherStation, got.Stations1[0], "the mirrored station slices do not alias")
	otherArc := got.ArcUpper1[0]
	got.ArcUpper0[0] = -100
	require.Equal(t, otherArc, got.ArcUpper1[0], "the mirrored arc-bound slices do not alias")
	otherEnergy := got.Energy1[0]
	got.Energy0[0] = -100
	require.Equal(t, otherEnergy, got.Energy1[0], "the mirrored energy slices do not alias")
}

func TestPairRecordsReusesIdenticalFreeformPair(t *testing.T) {
	segment, walk := selfPairFixture()
	profile := momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{segment}}}
	work0, work1 := freeform.NewFreeformWork(), freeform.NewFreeformWork()
	want, err := FreeformCellPoints(walk, walk, 1e-2, StationCap(1), work0, work1)
	require.NoError(t, err)

	got0, got1 := freeform.NewFreeformWork(), freeform.NewFreeformWork()
	pairs, sectionDelta, matchedDelta, round, err := PairRecords(profile, profile, []int{0},
		[][]survey2d.SegmentWalk{{walk}}, [][]survey2d.SegmentWalk{{walk}}, 1e-2, got0, got1)
	require.NoError(t, err)
	require.Len(t, pairs, 1)
	require.Equal(t, want.Stations0, pairs[0].V)
	require.Equal(t, want.Stations1, pairs[0].W)
	require.Equal(t, want.ArcUpper0, pairs[0].ArcUpperV)
	require.Equal(t, want.ArcUpper1, pairs[0].ArcUpperW)
	require.Equal(t, want.MatchedDelta, pairs[0].MatchedDelta)
	require.Equal(t, want.Energy0, pairs[0].TangentEnergyV)
	require.Equal(t, want.Energy1, pairs[0].TangentEnergyW)
	require.Equal(t, want.Sagitta, sectionDelta)
	wantMatched := 0.0
	for _, value := range want.MatchedDelta {
		wantMatched = max(wantMatched, value)
	}
	require.Equal(t, wantMatched, matchedDelta)
	require.Equal(t, want.Round, round)
	require.Equal(t, *work0, *got0)
	require.Equal(t, *work1, *got1)
}

func TestSelfFreeformCellFallsBackForChangedRecordOrWalk(t *testing.T) {
	segment, walk := selfPairFixture()
	changed := segment.(sectionrecord.SplineSeg)
	changed.Control = append([]sectionrecord.Point2(nil), changed.Control...)
	changed.Control[2].V = 0.25
	shiftedWalk := walk
	shiftedWalk.Spans = []freeform.BezierSpan{{
		{U: big.NewRat(0, 1), V: big.NewRat(0, 1)},
		{U: big.NewRat(1, 1000), V: big.NewRat(1, 4)},
		{U: big.NewRat(1, 1000), V: big.NewRat(0, 1)},
		{U: big.NewRat(1, 1), V: big.NewRat(0, 1)},
	}}
	for _, tc := range []struct {
		name string
		s0   sectionrecord.CurveSegment
		w1   survey2d.SegmentWalk
	}{
		{name: "changed record", s0: changed, w1: walk},
		{name: "offset walk", s0: segment, w1: shiftedWalk},
	} {
		t.Run(tc.name, func(t *testing.T) {
			work0, work1 := freeform.NewFreeformWork(), freeform.NewFreeformWork()
			before0, before1 := *work0, *work1
			_, reused := selfFreeformCell(segment, tc.s0, walk, tc.w1, 1e-2, 128, work0, work1)
			require.False(t, reused)
			require.Equal(t, before0, *work0)
			require.Equal(t, before1, *work1)
		})
	}
	work0, work1 := freeform.NewFreeformWork(), freeform.NewFreeformWork()
	work1.Spent = 1
	before0, before1 := *work0, *work1
	_, reused := selfFreeformCell(segment, segment, walk, walk, 1e-2, 128, work0, work1)
	require.False(t, reused, "unequal counter state takes the original two-side path")
	require.Equal(t, before0, *work0)
	require.Equal(t, before1, *work1)
}

func TestSelfFreeformCellRefusalsLeaveCountersForTwoSideFallback(t *testing.T) {
	segment, walk := selfPairFixture()
	t.Run("station cap", func(t *testing.T) {
		work0, work1 := freeform.NewFreeformWork(), freeform.NewFreeformWork()
		before0, before1 := *work0, *work1
		_, reused := selfFreeformCell(segment, segment, walk, walk, 1e-2, 1, work0, work1)
		require.False(t, reused)
		require.Equal(t, before0, *work0)
		require.Equal(t, before1, *work1)
		_, err := FreeformCellPoints(walk, walk, 1e-2, 1, work0, work1)
		require.ErrorIs(t, err, freeform.ErrTooManyChords)
	})

	t.Run("work limit", func(t *testing.T) {
		measure0, measure1 := freeform.NewFreeformWork(), freeform.NewFreeformWork()
		_, err := FreeformCellPoints(walk, walk, 1e-2, 128, measure0, measure1)
		require.NoError(t, err)
		limit := measure0.Spent
		require.Positive(t, limit)
		work0 := &freeform.FreeformWork{Spent: 1, Limit: limit}
		work1 := &freeform.FreeformWork{Spent: 1, Limit: limit}
		before0, before1 := *work0, *work1
		_, reused := selfFreeformCell(segment, segment, walk, walk, 1e-2, 128, work0, work1)
		require.False(t, reused)
		require.Equal(t, before0, *work0)
		require.Equal(t, before1, *work1)
		_, err = FreeformCellPoints(walk, walk, 1e-2, 128, work0, work1)
		require.ErrorIs(t, err, decaderr.ErrUnsupported)
	})
}
