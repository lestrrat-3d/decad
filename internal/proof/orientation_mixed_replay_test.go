package proof_test

import (
	"context"
	"math"
	"math/rand"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestReplayOrientSignMixed(t *testing.T) {
	ctx := t.Context()
	rng := rand.New(rand.NewSource(912))
	for range 256 {
		point := func() r3.Vec {
			return r3.Vec{X: math.Ldexp(rng.Float64()-.5, rng.Intn(80)-40), Y: math.Ldexp(rng.Float64()-.5, rng.Intn(80)-40), Z: math.Ldexp(rng.Float64()-.5, rng.Intn(80)-40)}
		}
		a, b, c, d := point(), point(), point(), point()
		exact := proof.XptOf(d)
		got, err := proof.ReplayOrientSignMixed(ctx, a, b, c, exact, proofbound.NewReplayTrialBudget())
		require.NoError(t, err)
		require.Equal(t, proof.OrientSignMixed(a, b, c, exact), got)
	}
	a, b, c := r3.Vec{}, r3.Vec{X: 1}, r3.Vec{Y: 1}
	for _, d := range []r3.Vec{{X: .5, Y: .25}, {Z: 1}, {Z: -1}} {
		exact := proof.XptOf(d)
		got, err := proof.ReplayOrientSignMixed(ctx, a, b, c, exact, proofbound.NewReplayTrialBudget())
		require.NoError(t, err)
		require.Equal(t, proof.OrientSignMixed(a, b, c, exact), got)
	}
}

func TestReplayOrientSignMixedLimits(t *testing.T) {
	ctx := t.Context()
	a, b, c, d := r3.Vec{}, r3.Vec{X: 1}, r3.Vec{Y: 1}, proof.XptOf(r3.Vec{Z: 1})
	meter := proofbound.NewReplayTrialBudget()
	_, err := proof.ReplayOrientSignMixed(ctx, a, b, c, d, meter)
	require.NoError(t, err)
	work, records := meter.Usage()
	bytes := meter.Bytes()
	for _, kind := range []string{"work", "records", "bytes"} {
		t.Run(kind, func(t *testing.T) {
			m := proofbound.NewReplayTrialBudget()
			w, r, z := proofbound.ReplayTrialWork-work, proofbound.ReplayTrialRecords-records, proofbound.ReplayTrialBytes-bytes
			switch kind {
			case "work":
				w++
			case "records":
				r++
			case "bytes":
				z++
			}
			require.NoError(t, m.SpendWork(ctx, w))
			require.NoError(t, m.ReserveRecords(ctx, r))
			require.NoError(t, m.ReserveBytes(ctx, z))
			_, err := proof.ReplayOrientSignMixed(ctx, a, b, c, d, m)
			require.ErrorIs(t, err, decaderr.ErrUnsupported)
		})
	}
	cancelled, cancel := context.WithCancel(ctx)
	cancel()
	_, err = proof.ReplayOrientSignMixed(cancelled, a, b, c, d, proofbound.NewReplayTrialBudget())
	require.ErrorIs(t, err, context.Canceled)
}
