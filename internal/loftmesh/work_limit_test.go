package loftmesh_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestLoftAuditWorkLimitDetails(t *testing.T) {
	verts := []r3.Vec{{}, {X: 2}, {X: 2, Y: 2}, {Y: 2}}
	tris := [][3]int{{0, 1, 2}, {0, 2, 3}}
	budget := proofbound.NewWorkBudget(t.Context())
	err := loftmesh.LoftCrossingAuditBounded(budget, verts, tris, 1, 100)
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
	var detail *proofbound.WorkLimitError
	require.ErrorAs(t, err, &detail)
	require.Equal(t, "mesh_facets", detail.Resource)
	require.Equal(t, uint64(2), detail.Observed)
	require.Equal(t, uint64(1), detail.Maximum)
	err = loftmesh.LoftCrossingAuditBounded(budget, verts, tris, 2, 0)
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
	require.ErrorIs(t, err, loftmesh.ErrLoftAuditCeiling)
	require.ErrorAs(t, err, &detail)
	require.Equal(t, "pair_enumeration_steps", detail.Resource)
	require.Equal(t, uint64(1), detail.Observed)
	require.Zero(t, detail.Maximum)
	require.NoError(t, loftmesh.LoftCrossingAuditBounded(budget, verts, tris, 2, 100))
}
