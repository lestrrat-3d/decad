package momentinput_test

import (
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/stretchr/testify/require"
)

func TestFirstMomentsReplayBoundProducerCache(t *testing.T) {
	points := []sectionrecord.Point2{{}, {U: 3}, {U: 3, V: 2}, {V: 2}}
	loop := sectionrecord.LoopRecord{Segments: make([]sectionrecord.CurveSegment, 4)}
	for i := range loop.Segments {
		loop.Segments[i] = sectionrecord.LineSeg{Start: points[i], End: points[(i+1)%4], TEnd: 1}
	}
	profile := momentinput.Profile{Outer: loop}
	walks, err := momentinput.ResolveProfileWalks(profile, freeform.NewFreeformWork())
	require.NoError(t, err)
	computed, err := profile.EvaluatorIntegralsContext(t.Context(), freeform.MomentFirstOrder,
		freeform.NewFreeformWork())
	require.NoError(t, err)
	cache := walks.WithIntegrals(computed, freeform.MomentFirstOrder)
	budget := proofbound.NewReplayTrialBudget()
	got, ok, err := cache.FirstMomentsReplay(t.Context(), profile, budget)
	require.NoError(t, err)
	require.True(t, ok)
	for _, pair := range [][2]float64{{got.Area, computed.Area}, {got.AreaBound, computed.AreaBound},
		{got.Mu, computed.Mu}, {got.MuBound, computed.MuBound}, {got.Mv, computed.Mv},
		{got.MvBound, computed.MvBound}} {
		require.Equal(t, math.Float64bits(pair[1]), math.Float64bits(pair[0]))
	}
	require.Nil(t, got.Exact.Area, "a replay read must not share a rational pointer")
	work, records := budget.Usage()
	require.Equal(t, uint64(513), work)
	require.Zero(t, records)
	require.Zero(t, budget.Bytes())

	exact, err := proofbound.NewReplayBudgetLimits(work, 1, 1)
	require.NoError(t, err)
	_, ok, err = cache.FirstMomentsReplay(t.Context(), profile, exact)
	require.NoError(t, err)
	require.True(t, ok)
	short, err := proofbound.NewReplayBudgetLimits(work-1, 1, 1)
	require.NoError(t, err)
	_, ok, err = cache.FirstMomentsReplay(t.Context(), profile, short)
	require.Error(t, err)
	require.False(t, ok)

	changed := momentinput.Profile{Outer: sectionrecord.LoopRecord{Segments: append([]sectionrecord.CurveSegment(nil), loop.Segments...)}}
	line := changed.Outer.Segments[0].(sectionrecord.LineSeg)
	line.Start.U = math.Copysign(0, -1)
	changed.Outer.Segments[0] = line
	_, ok, err = cache.FirstMomentsReplay(t.Context(), changed, proofbound.NewReplayTrialBudget())
	require.NoError(t, err)
	require.False(t, ok)
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	cancelled := proofbound.NewReplayTrialBudget()
	_, ok, err = cache.FirstMomentsReplay(ctx, profile, cancelled)
	require.ErrorIs(t, err, context.Canceled)
	require.False(t, ok)
	work, records = cancelled.Usage()
	require.Zero(t, work)
	require.Zero(t, records)
	require.Zero(t, cancelled.Bytes())
}
