package boundarywalk_test

import (
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestCircleWalkReplayMatchesRecordedGeometry(t *testing.T) {
	for _, seg := range []sectionrecord.CircleSeg{
		{Center: sectionrecord.Point2{U: 1.2, V: -3.4}, Radius: units.Millimeters(3.3), CCW: true, TEnd: 1},
		{Center: sectionrecord.Point2{U: 1e12, V: -1e12}, Radius: units.Millimeters(.003), CCW: true, TEnd: 1},
	} {
		ordinary, err := boundarywalk.WalkOf(seg, nil)
		require.NoError(t, err)
		got, err := boundarywalk.CircleWalkReplay(t.Context(), seg, proofbound.NewReplayTrialBudget())
		require.NoError(t, err)
		require.Equal(t, ordinary.Kind, got.Kind)
		require.Equal(t, ordinary.Closed, got.Closed)
		require.Equal(t, [8]float64{ordinary.StartU, ordinary.StartV, ordinary.EndU, ordinary.EndV, ordinary.CU, ordinary.CV, ordinary.Radius, ordinary.Length}, [8]float64{got.StartU, got.StartV, got.EndU, got.EndV, got.CU, got.CV, got.Radius, got.Length})
		require.GreaterOrEqual(t, got.StartBound.U, ordinary.StartBound.U)
		require.GreaterOrEqual(t, got.StartBound.V, ordinary.StartBound.V)
		require.GreaterOrEqual(t, got.EndBound.U, ordinary.EndBound.U)
		require.GreaterOrEqual(t, got.EndBound.V, ordinary.EndBound.V)
		require.GreaterOrEqual(t, got.LengthBound, ordinary.LengthBound)
	}
}

func TestCircleWalkReplayLimitsAndRefusals(t *testing.T) {
	seg := sectionrecord.CircleSeg{Center: sectionrecord.Point2{U: 1.2, V: -3.4}, Radius: units.Millimeters(3.3), CCW: true, TEnd: 1}
	run := func(ctx context.Context, b *proofbound.ReplayBudget) error {
		_, err := boundarywalk.CircleWalkReplay(ctx, seg, b)
		return err
	}
	base := proofbound.NewReplayTrialBudget()
	require.NoError(t, run(t.Context(), base))
	w, r := base.Usage()
	n := base.Bytes()
	for _, mode := range []string{"exact", "work", "records", "bytes"} {
		t.Run(mode, func(t *testing.T) {
			b := proofbound.NewReplayTrialBudget()
			ww, rr, nn := proofbound.ReplayTrialWork-w, proofbound.ReplayTrialRecords-r, proofbound.ReplayTrialBytes-n
			switch mode {
			case "work":
				ww++
			case "records":
				rr++
			case "bytes":
				nn++
			}
			require.NoError(t, b.SpendWork(t.Context(), ww))
			require.NoError(t, b.ReserveRecords(t.Context(), rr))
			require.NoError(t, b.ReserveBytes(t.Context(), nn))
			err := run(t.Context(), b)
			if mode == "exact" {
				require.NoError(t, err)
			} else {
				require.ErrorIs(t, err, decaderr.ErrUnsupported)
			}
		})
	}
	seg.TEnd = .5
	require.ErrorIs(t, run(t.Context(), proofbound.NewReplayTrialBudget()), decaderr.ErrUnsupported)
	seg.TEnd = 1
	seg.Radius = units.Inches(1)
	require.ErrorIs(t, run(t.Context(), proofbound.NewReplayTrialBudget()), decaderr.ErrUnsupported)
	seg.Radius = units.Millimeters(3.3)
	seg.Center.U = math.Inf(1)
	require.ErrorIs(t, run(t.Context(), proofbound.NewReplayTrialBudget()), decaderr.ErrUnsupported)
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	seg.Center.U = 0
	require.ErrorIs(t, run(ctx, proofbound.NewReplayTrialBudget()), context.Canceled)
}
