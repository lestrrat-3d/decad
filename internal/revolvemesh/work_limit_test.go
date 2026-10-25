package revolvemesh_test

import (
	"context"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolvemesh"
	"github.com/stretchr/testify/require"
)

func TestRevolveContactAuditWorkLimitDetails(t *testing.T) {
	data := make([]revolvemesh.RevolveAuditTri, 4001)
	tris := make([][3]int, 4001)
	err := revolvemesh.RevolveContactAudit(proofbound.NewWorkBudget(t.Context()), data, tris, 0)
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
	var detail *proofbound.WorkLimitError
	require.ErrorAs(t, err, &detail)
	require.Equal(t, "facet_pairs", detail.Resource)
	require.Equal(t, uint64(8_002_000), detail.Observed)
	require.Equal(t, uint64(8_000_000), detail.Maximum)
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	require.Same(t, ctx.Err(), revolvemesh.RevolveContactAudit(proofbound.NewWorkBudget(ctx), data, tris, 0))
}
