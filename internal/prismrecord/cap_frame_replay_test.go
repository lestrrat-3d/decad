package prismrecord_test

import (
	"context"
	"testing"

	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestCapFrameSignedReplayParityAndLimits(t *testing.T) {
	frame, err := r3.NewFrame(r3.Vec{X: 1e12}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	require.NoError(t, err)
	pp := prismrecord.Payload{Frame: frame, Xform: r3.Identity()}
	for _, flip := range []bool{false, true} {
		meter := proofbound.NewReplayTrialBudget()
		got, err := pp.CapFrameSignedReplay(t.Context(), 17.5, flip, meter)
		require.NoError(t, err)
		want, err := pp.CapFrame(17.5, flip)
		require.NoError(t, err)
		require.Equal(t, want, got)
		work, records := meter.Usage()
		bytes := meter.Bytes()
		for _, limits := range [...][3]uint64{{work - 1, records, bytes},
			{work, records - 1, bytes}, {work, records, bytes - 1}} {
			short, e := proofbound.NewReplayBudgetLimits(limits[0], limits[1], limits[2])
			require.NoError(t, e)
			_, e = pp.CapFrameSignedReplay(t.Context(), 17.5, flip, short)
			require.Error(t, e)
		}
	}
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	meter := proofbound.NewReplayTrialBudget()
	_, err = pp.CapFrameSignedReplay(ctx, 17.5, false, meter)
	require.ErrorIs(t, err, context.Canceled)
	work, records := meter.Usage()
	require.Zero(t, work)
	require.Zero(t, records)
	require.Zero(t, meter.Bytes())
}
