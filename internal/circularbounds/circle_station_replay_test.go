package circularbounds_test

import (
	"context"
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/circularbounds"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestCircleStationReplayBoundsIndependentInterval(t *testing.T) {
	seg := sectionrecord.CircleSeg{Center: sectionrecord.Point2{U: 1.2, V: -3.4}, Radius: units.Millimeters(3.3), CCW: true, TEnd: 1}
	for _, tc := range []struct{ k, n int }{{1, 4}, {1, 13}, {4, 13}, {12, 13}, {7, 8}} {
		s, c := math.Sincos(2 * math.Pi * float64(tc.k) / float64(tc.n))
		u, v := seg.Center.U+3.3*c, seg.Center.V+3.3*s
		got, err := circularbounds.CircleStationReplay(t.Context(), seg, tc.k, tc.n, u, v, proofbound.NewReplayTrialBudget())
		require.NoError(t, err)
		iu, iv, ok := circularbounds.EndpointInterval(circularbounds.RecordSegment(seg), big.NewRat(int64(tc.k), int64(tc.n)))
		require.True(t, ok)
		require.GreaterOrEqual(t, got.U, proofbound.IntervalFloatError(iu, u))
		require.GreaterOrEqual(t, got.V, proofbound.IntervalFloatError(iv, v))
	}
}

func TestCircleStationReplayLimitsAndRefusals(t *testing.T) {
	seg := sectionrecord.CircleSeg{Radius: units.Millimeters(3.3), CCW: true, TEnd: 1}
	run := func(ctx context.Context, b *proofbound.ReplayBudget) error {
		_, err := circularbounds.CircleStationReplay(ctx, seg, 1, 13, 1, 2, b)
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
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	require.ErrorIs(t, run(ctx, proofbound.NewReplayTrialBudget()), context.Canceled)
}
