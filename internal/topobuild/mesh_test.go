package topobuild_test

import (
	"context"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/stretchr/testify/require"
)

func TestMeshWritesKeepSourcesAndLargestBound(t *testing.T) {
	t.Parallel()
	a, b := &topo.Face{}, &topo.Face{}
	m := &topo.Mesh{}
	topobuild.AddTriangle(m, [3]int{0, 1, 2}, a)
	topobuild.AddTriangle(m, [3]int{0, 2, 3}, b)
	require.Equal(t, [][3]int{{0, 1, 2}, {0, 2, 3}}, m.Triangles)
	require.Len(t, m.Source, 2)
	require.True(t, m.Source[0] == a && m.Source[1] == b, "each facet keeps its source face")

	topobuild.SetFaceBound(m, a, 0)
	v, ok := m.FaceBound[a]
	require.True(t, ok, "a zero bound is still written")
	require.Zero(t, v)
	topobuild.SetFaceBound(m, a, 0.5)
	topobuild.SetFaceBound(m, a, 0.25)
	topobuild.SetFaceBound(m, b, 0.125)
	require.InDelta(t, 0.5, m.FaceBound[a], 0)
	require.InDelta(t, 0.125, m.FaceBound[b], 0)
	require.InDelta(t, 0.5, m.Bound, 0)
}

func TestRequireClosedMeshLiftsTheRefusal(t *testing.T) {
	t.Parallel()
	tetra := &topo.Mesh{Triangles: [][3]int{{0, 2, 1}, {0, 1, 3}, {1, 2, 3}, {0, 3, 2}}}
	require.NoError(t, topobuild.RequireClosedMesh(tetra))

	open := &topo.Mesh{Triangles: tetra.Triangles[:3]}
	err := topobuild.RequireClosedMesh(open)
	require.ErrorIs(t, err, decaderr.ErrDegenerate)
	require.EqualError(t, err, decaderr.ErrDegenerate.Error()+
		": the chorded boundary could not be triangulated into a watertight mesh")
}

func TestLiftAuditError(t *testing.T) {
	t.Parallel()
	unsupported := topobuild.LiftAuditError(&tessellation.AuditError{Sentinel: tessellation.Unsupported, Detail: "x"})
	require.ErrorIs(t, unsupported, decaderr.ErrUnsupported)
	require.EqualError(t, unsupported, decaderr.ErrUnsupported.Error()+": x")
	degenerate := topobuild.LiftAuditError(&tessellation.AuditError{Sentinel: tessellation.Degenerate, Detail: "y"})
	require.EqualError(t, degenerate, decaderr.ErrDegenerate.Error()+": y")
	require.ErrorIs(t, topobuild.LiftAuditError(context.Canceled), context.Canceled)
	require.NoError(t, topobuild.LiftAuditError(nil))
}
