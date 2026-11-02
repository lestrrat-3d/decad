package prismrecord_test

import (
	"context"
	"testing"

	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestChargeMapSignedReplayMatchesOrdinaryAndLimits(t *testing.T) {
	frame, err := r3.NewFrame(r3.Vec{X: 17}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	require.NoError(t, err)
	body := &topo.Body{Solid: true, Volume: measurement.Measurement{Value: units.CubicMillimeters(3)},
		Area: measurement.Measurement{Value: units.SquareMillimeters(5)}}
	want := &topo.Body{Solid: body.Solid, Volume: body.Volume, Area: body.Area}
	require.NoError(t, prismrecord.ChargeMap(want, frame, r3.Identity()))
	meter := proofbound.NewReplayTrialBudget()
	require.NoError(t, prismrecord.ChargeMapSignedReplay(t.Context(), body, frame, r3.Identity(), meter))
	require.Equal(t, want.Volume, body.Volume)
	require.Equal(t, want.Area, body.Area)
	work, records := meter.Usage()
	require.Equal(t, uint64(128), work)
	require.Zero(t, records)
	require.Zero(t, meter.Bytes())
	short, err := proofbound.NewReplayBudgetLimits(work-1, 1, 1)
	require.NoError(t, err)
	require.Error(t, prismrecord.ChargeMapSignedReplay(t.Context(), body, frame, r3.Identity(), short))
	tilted, err := r3.NewFrame(r3.Vec{}, r3.Vec{X: 1, Y: 1}, r3.Vec{X: -1, Y: 1})
	require.NoError(t, err)
	require.Error(t, prismrecord.ChargeMapSignedReplay(t.Context(), body, tilted, r3.Identity(),
		proofbound.NewReplayTrialBudget()))
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	meter = proofbound.NewReplayTrialBudget()
	require.ErrorIs(t, prismrecord.ChargeMapSignedReplay(ctx, body, frame, r3.Identity(), meter), context.Canceled)
	work, records = meter.Usage()
	require.Zero(t, work)
	require.Zero(t, records)
	require.Zero(t, meter.Bytes())
}
