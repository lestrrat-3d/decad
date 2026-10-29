package revolveplan

import (
	"context"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/stretchr/testify/require"
)

func TestFreeEndOwnership(t *testing.T) {
	body := &topo.Body{}
	edge := &topo.Edge{Faces: []*topo.Face{{Body: body}}}
	body.Payload = ChainPayload{Chains: []sectionrecord.ChainRecord{{Segments: make([]sectionrecord.CurveSegment, 2)}}, freeEnds: []chainFreeEnd{{edge, 0, 1, false}}}
	budget := proofbound.NewWorkBudget(t.Context())
	ci, si, start, err := FreeEnd(t.Context(), budget, body, edge)
	require.NoError(t, err)
	require.Equal(t, 0, ci)
	require.Equal(t, 1, si)
	require.False(t, start)
	ci, _, _, err = FreeEnd(t.Context(), budget, body, &topo.Edge{})
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
	require.Zero(t, ci)
	other := &topo.Body{Payload: body.Payload}
	ci, _, _, err = FreeEnd(t.Context(), budget, other, edge)
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
	require.Zero(t, ci)
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	ci, _, _, err = FreeEnd(ctx, proofbound.NewWorkBudget(ctx), body, edge)
	require.ErrorIs(t, err, context.Canceled)
	require.Zero(t, ci)
}
