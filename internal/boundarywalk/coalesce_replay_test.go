package boundarywalk_test

import (
	"context"
	"testing"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/stretchr/testify/require"
)

func TestCoalesceWalksNoMergeReplayMatchesOrdinaryAndRefusesCandidate(t *testing.T) {
	line := func(a, b sectionrecord.Point2, i int) survey2d.SideWalk {
		w, err := boundarywalk.WalkOf(sectionrecord.LineSeg{Start: a, End: b, TEnd: 1}, nil)
		require.NoError(t, err)
		return survey2d.SideWalk{SegmentWalk: w, Segs: []int{i}}
	}
	a := sectionrecord.Point2{}
	b := sectionrecord.Point2{U: 1}
	c := sectionrecord.Point2{U: 1, V: 1}
	corner := []survey2d.SideWalk{line(a, b, 0), line(b, c, 1), line(c, a, 2)}
	want, err := boundarywalk.CoalesceWalksContext(t.Context(), corner)
	require.NoError(t, err)
	meter := proofbound.NewReplayTrialBudget()
	got, err := boundarywalk.CoalesceWalksNoMergeReplay(t.Context(), corner, meter)
	require.NoError(t, err)
	require.Equal(t, want, got)
	w, r := meter.Usage()
	bytes := meter.Bytes()
	for _, limits := range [...][3]uint64{{w - 1, r, bytes}, {w, r - 1, bytes}, {w, r, bytes - 1}} {
		short, e := proofbound.NewReplayBudgetLimits(limits[0], limits[1], limits[2])
		require.NoError(t, e)
		result, e := boundarywalk.CoalesceWalksNoMergeReplay(t.Context(), corner, short)
		require.Error(t, e)
		require.Nil(t, result)
	}

	straight := []survey2d.SideWalk{line(a, b, 0), line(b, sectionrecord.Point2{U: 2}, 1)}
	result, err := boundarywalk.CoalesceWalksNoMergeReplay(t.Context(), straight,
		proofbound.NewReplayTrialBudget())
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
	require.Nil(t, result)

	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	result, err = boundarywalk.CoalesceWalksNoMergeReplay(ctx, corner,
		proofbound.NewReplayTrialBudget())
	require.ErrorIs(t, err, context.Canceled)
	require.Nil(t, result)
}
