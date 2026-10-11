package loftmesh

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestLoftCrossingAuditBoundedUsesCallerLimits(t *testing.T) {
	verts := []r3.Vec{
		r3.NewVec(0, 0, 0), r3.NewVec(1, 0, 0),
		r3.NewVec(1, 1, 0), r3.NewVec(0, 1, 0),
	}
	tris := [][3]int{{0, 1, 2}, {0, 2, 3}}
	audit := func(maxTriangles int, maxScans uint64) error {
		return LoftCrossingAuditBounded(
			proofbound.NewWorkBudget(t.Context()), verts, tris, maxTriangles, maxScans)
	}

	err := audit(1, 1)
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
	require.ErrorContains(t, err, "triangles exceed")
	err = audit(2, 0)
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
	require.ErrorContains(t, err, "candidate pair count exceeds")
	require.NoError(t, audit(2, 1))

	work, err := LoftCrossingAuditWork(
		proofbound.NewWorkBudget(t.Context()), verts, tris, loftGenericShortcuts)
	require.NoError(t, err)
	require.Equal(t, 1, work.Candidates)
}

func TestLoftCrossingAuditBoundedKeepsExactContactVerdict(t *testing.T) {
	verts := []r3.Vec{
		r3.NewVec(0, 0, 0), r3.NewVec(2, 0, 0), r3.NewVec(0, 2, 0),
		r3.NewVec(0.25, 0.25, 0), r3.NewVec(1, 0.25, 0), r3.NewVec(0.25, 1, 0),
	}
	tris := [][3]int{{0, 1, 2}, {3, 4, 5}}
	want := LoftCrossingAudit(proofbound.NewWorkBudget(t.Context()), verts, tris)
	got := LoftCrossingAuditBounded(proofbound.NewWorkBudget(t.Context()), verts, tris, 2, 1)
	require.ErrorIs(t, want, decaderr.ErrDegenerate)
	require.EqualError(t, got, want.Error())
}
