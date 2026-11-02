package prismboolean_test

import (
	"context"
	"testing"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismboolean"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topo/topotest"
	"github.com/stretchr/testify/require"
)

func capReplayBody(t *testing.T) *topo.Body {
	t.Helper()
	payload := prism(t, momentinput.Profile{Outer: square(0, 0, 2)}, 3)
	body, err := prismrecord.Build(t.Context(), topo.Env{Doc: &topo.Document{}, Ref: 7, Alloc: topotest.Alloc{}},
		payload, freeform.NewFreeformWork())
	require.NoError(t, err)
	require.True(t, body.HasStrictPrismBuildSeal())
	return body
}

func TestReplayPrismCap(t *testing.T) {
	t.Parallel()
	body := capReplayBody(t)
	for _, end := range []bool{false, true} {
		meter := proofbound.NewReplayTrialBudget()
		face, ok, err := prismboolean.ReplayPrismCap(t.Context(), body, end, meter)
		require.NoError(t, err)
		require.True(t, ok)
		require.True(t, face.Body == body)
		role := provenance.RoleCapStart
		if end {
			role = provenance.RoleCapEnd
		}
		require.Equal(t, role, face.Origins[0].Role)
		work, records := meter.Usage()
		exact, err := proofbound.NewReplayBudgetLimits(work, records, meter.Bytes())
		require.NoError(t, err)
		again, ok, err := prismboolean.ReplayPrismCap(t.Context(), body, end, exact)
		require.NoError(t, err)
		require.True(t, ok)
		require.True(t, face == again)
		short, err := proofbound.NewReplayBudgetLimits(work-1, records, meter.Bytes())
		require.NoError(t, err)
		_, _, err = prismboolean.ReplayPrismCap(t.Context(), body, end, short)
		var limit *proofbound.WorkLimitError
		require.ErrorAs(t, err, &limit)
	}
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	_, _, err := prismboolean.ReplayPrismCap(ctx, body, false, proofbound.NewReplayTrialBudget())
	require.ErrorIs(t, err, context.Canceled)
	face, ok, err := prismboolean.ReplayPrismCap(t.Context(), body, false, proofbound.NewReplayTrialBudget())
	require.NoError(t, err)
	require.True(t, ok)
	shell := body.Lumps[0].Shells[0]
	shell.Faces = append(shell.Faces, face)
	_, ok, err = prismboolean.ReplayPrismCap(t.Context(), body, false, proofbound.NewReplayTrialBudget())
	require.NoError(t, err)
	require.False(t, ok)
}
