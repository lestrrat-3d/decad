package boundarywalk_test

import (
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestWholeCircleWalkReplayMatchesOrdinaryWalk(t *testing.T) {
	for _, tc := range []struct {
		name   string
		center sectionrecord.Point2
		radius float64
		ccw    bool
	}{
		{"forward", sectionrecord.Point2{U: 2, V: -3}, 1.5, true},
		{"reverse", sectionrecord.Point2{U: 2, V: -3}, 1.5, false},
		{"far center", sectionrecord.Point2{U: 1e12, V: -1e12}, 0.125, false},
		{"maximum center", sectionrecord.Point2{U: math.MaxFloat64}, 1, true},
		{"small radius", sectionrecord.Point2{U: 0.25, V: -0.5}, math.SmallestNonzeroFloat64, true},
	} {
		t.Run(tc.name, func(t *testing.T) {
			seg := sectionrecord.CircleSeg{Center: tc.center, Radius: units.Millimeters(tc.radius), CCW: tc.ccw, TEnd: 1}
			if !tc.ccw {
				seg.TStart, seg.TEnd = 1, 0
			}
			ordinary, err := boundarywalk.WalkOf(seg, nil)
			require.NoError(t, err)
			budget := proofbound.NewReplayTrialBudget()
			bounded, err := boundarywalk.WholeCircleWalkReplay(t.Context(), seg, budget)
			require.NoError(t, err)
			require.Equal(t, ordinary, bounded)
			for _, pair := range [...][2]float64{
				{ordinary.StartBound.U, bounded.StartBound.U},
				{ordinary.StartBound.V, bounded.StartBound.V},
				{ordinary.EndBound.U, bounded.EndBound.U},
				{ordinary.EndBound.V, bounded.EndBound.V},
				{ordinary.LengthBound, bounded.LengthBound},
			} {
				require.Equal(t, math.Float64bits(pair[0]), math.Float64bits(pair[1]))
			}
			work, records := budget.Usage()
			require.Positive(t, work)
			require.Positive(t, records)
			require.Positive(t, budget.Bytes())
		})
	}
}

func TestWholeCircleWalkReplayOneShortAndCancel(t *testing.T) {
	seg := sectionrecord.CircleSeg{Radius: units.Millimeters(1.5), CCW: true, TEnd: 1}
	measured := proofbound.NewReplayTrialBudget()
	_, err := boundarywalk.WholeCircleWalkReplay(t.Context(), seg, measured)
	require.NoError(t, err)
	w, r := measured.Usage()
	b := measured.Bytes()
	for _, limits := range [...][3]uint64{{w - 1, r, b}, {w, r - 1, b}, {w, r, b - 1}} {
		budget, err := proofbound.NewReplayBudgetLimits(limits[0], limits[1], limits[2])
		require.NoError(t, err)
		_, err = boundarywalk.WholeCircleWalkReplay(t.Context(), seg, budget)
		require.Error(t, err)
	}
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	budget := proofbound.NewReplayTrialBudget()
	_, err = boundarywalk.WholeCircleWalkReplay(ctx, seg, budget)
	require.ErrorIs(t, err, context.Canceled)
	w, r = budget.Usage()
	require.Zero(t, w)
	require.Zero(t, r)
	require.Zero(t, budget.Bytes())
}
