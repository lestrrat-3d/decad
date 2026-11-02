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

func TestProfileWalksLoopMatchesReplayExactRecord(t *testing.T) {
	loop := sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
		sectionrecord.LineSeg{Start: sectionrecord.Point2{U: 0, V: 1},
			End: sectionrecord.Point2{U: 2, V: 1}, TEnd: 1},
	}}
	profile := momentinput.Profile{Outer: loop}
	walks, err := momentinput.ResolveProfileWalks(profile, freeform.NewFreeformWork())
	require.NoError(t, err)

	exact, err := proofbound.NewReplayBudgetLimits(64, 1, 1)
	require.NoError(t, err)
	ok, err := walks.LoopMatchesReplay(t.Context(), 0, loop, exact)
	require.NoError(t, err)
	require.True(t, ok)
	w, r := exact.Usage()
	require.Equal(t, uint64(64), w)
	require.Zero(t, r)
	require.Zero(t, exact.Bytes())

	short, err := proofbound.NewReplayBudgetLimits(63, 1, 1)
	require.NoError(t, err)
	ok, err = walks.LoopMatchesReplay(t.Context(), 0, loop, short)
	require.Error(t, err)
	require.False(t, ok)
	w, r = short.Usage()
	require.Zero(t, w)
	require.Zero(t, r)
	require.Zero(t, short.Bytes())

	changed := sectionrecord.LineSeg{Start: sectionrecord.Point2{U: math.Copysign(0, -1), V: 1},
		End: sectionrecord.Point2{U: 2, V: 1}, TEnd: 1}
	ok, err = walks.LoopMatchesReplay(t.Context(), 0,
		sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{changed}}, proofbound.NewReplayTrialBudget())
	require.NoError(t, err)
	require.False(t, ok)

	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	cancelled := proofbound.NewReplayTrialBudget()
	ok, err = walks.LoopMatchesReplay(ctx, 0, loop, cancelled)
	require.ErrorIs(t, err, context.Canceled)
	require.False(t, ok)
	w, r = cancelled.Usage()
	require.Zero(t, w)
	require.Zero(t, r)
	require.Zero(t, cancelled.Bytes())
}
