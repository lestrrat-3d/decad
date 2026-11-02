package prismrecord_test

import (
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestPointBoundSignedReplayParityAndLimits(t *testing.T) {
	for _, origin := range []r3.Vec{{}, {X: 1e12, Y: -1e12}} {
		frame, err := r3.NewFrame(origin, r3.Vec{X: 1}, r3.Vec{Y: 1})
		require.NoError(t, err)
		pp := prismrecord.Payload{Frame: frame, Xform: r3.Identity()}
		u, v, z := proofbound.MeasuredScalar(1.1, 0.01),
			proofbound.MeasuredScalar(2.3, 0.02), proofbound.MeasuredScalar(4.7, 0.03)
		meter := proofbound.NewReplayTrialBudget()
		got, bound, err := pp.PointBoundSignedReplay(t.Context(), u, v, z, meter)
		require.NoError(t, err)
		want := pp.Point(u.Value, v.Value, z.Value)
		require.Equal(t, math.Float64bits(want.X), math.Float64bits(got.X))
		require.Equal(t, math.Float64bits(want.Y), math.Float64bits(got.Y))
		require.Equal(t, math.Float64bits(want.Z), math.Float64bits(got.Z))
		require.Equal(t, math.Float64bits(pp.PointBound(u, v, z)), math.Float64bits(bound))
		work, records := meter.Usage()
		bytes := meter.Bytes()
		for _, limits := range [...][3]uint64{{work - 1, records, bytes},
			{work, records - 1, bytes}, {work, records, bytes - 1}} {
			short, err := proofbound.NewReplayBudgetLimits(limits[0], limits[1], limits[2])
			require.NoError(t, err)
			_, _, err = pp.PointBoundSignedReplay(t.Context(), u, v, z, short)
			require.Error(t, err)
		}
	}
	frame, err := r3.NewFrame(r3.Vec{}, r3.Vec{X: 1, Y: 1}, r3.Vec{X: -1, Y: 1})
	require.NoError(t, err)
	_, _, err = (prismrecord.Payload{Frame: frame, Xform: r3.Identity()}).PointBoundSignedReplay(
		t.Context(), proofbound.ExactScalar(1), proofbound.ExactScalar(2), proofbound.ExactScalar(3),
		proofbound.NewReplayTrialBudget())
	require.Error(t, err)
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	meter := proofbound.NewReplayTrialBudget()
	_, _, err = (prismrecord.Payload{Frame: frame, Xform: r3.Identity()}).PointBoundSignedReplay(
		ctx, proofbound.ExactScalar(1), proofbound.ExactScalar(2), proofbound.ExactScalar(3), meter)
	require.ErrorIs(t, err, context.Canceled)
	work, records := meter.Usage()
	require.Zero(t, work)
	require.Zero(t, records)
	require.Zero(t, meter.Bytes())
}
