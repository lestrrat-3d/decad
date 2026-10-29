package loftmesh

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/stretchr/testify/require"
)

// TestLoftFreeformWalkBisectsOnMatched pairs a cubic with itself whose
// control points all lie on its chord segment, two of them clustered at
// u = 1/1000: its sagitta is exactly 0 while its parameter-matched departure
// is above 0.3 (freeform's TestSpanMatchedDeltaUpperEnclosesWhatTheSagittaMisses).
// The walk must still bisect until every accepted cell's matched departure
// meets the target (agent-docs/loft-gear-bounds-design.md §5).
//
// Shown to fail: dropping the matched arm of SagittaStationWalk.WalkCell's
// acceptance test leaves one cell whose matched value is above the target.
func TestLoftFreeformWalkBisectsOnMatched(t *testing.T) {
	t.Parallel()
	point := func(un, ud int64) freeform.RatPoint {
		return freeform.RatPoint{U: big.NewRat(un, ud), V: new(big.Rat)}
	}
	span := freeform.BezierSpan{point(0, 1), point(1, 1000), point(1, 1000), point(1, 1)}
	w := survey2d.SegmentWalk{Kind: survey2d.WalkFreeform, Spans: []freeform.BezierSpan{span}}
	const target = 1e-2
	cell, err := FreeformCellPoints(w, w, target, freeform.MaxChordsPerWalk, nil, nil)
	require.NoError(t, err)

	require.Zero(t, cell.Sagitta, "the sagitta alone never asks this span for a bisection")
	require.Greater(t, len(cell.MatchedDelta), 1, "the matched departure forces a bisection")
	require.Len(t, cell.Stations0, len(cell.MatchedDelta), "one station per accepted cell")
	worst := 0.0
	for k, md := range cell.MatchedDelta {
		require.LessOrEqual(t, md, target, "cell %d's matched departure must meet the target", k)
		worst = math.Max(worst, md)
	}
	require.Positive(t, worst, "the curve departs from its chords, so the matched values cannot all be 0")
}

func TestCommonSpanCountRespectsStationShare(t *testing.T) {
	for _, tc := range []struct {
		a, b, share, want int
		ok                bool
	}{
		{4, 5, 20, 20, true},
		{4, 6, 12, 12, true},
		{4, 5, 19, 0, false},
		{150, 151, 8192, 0, false},
	} {
		got, ok := commonSpanCount(tc.a, tc.b, tc.share)
		if got != tc.want || ok != tc.ok {
			t.Fatalf("commonSpanCount(%d, %d, %d) = (%d, %t), want (%d, %t)",
				tc.a, tc.b, tc.share, got, ok, tc.want, tc.ok)
		}
	}
}
