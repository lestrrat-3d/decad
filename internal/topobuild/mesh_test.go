package topobuild_test

import (
	"context"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
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

func TestVertexBoundsRecordAndDerived(t *testing.T) {
	t.Parallel()
	a, b := &topo.Face{}, &topo.Face{}
	twoFaces := func() *topo.Mesh {
		m := &topo.Mesh{Vertices: make([]r3.Vec, 4)}
		topobuild.AddTriangle(m, [3]int{0, 1, 2}, a)
		topobuild.AddTriangle(m, [3]int{0, 2, 3}, b)
		return m
	}

	t.Run("SetPublishesFaceMaxima", func(t *testing.T) {
		t.Parallel()
		m := twoFaces()
		beta := []float64{0.125, 0.5, 0.25, 0.0625}
		topobuild.SetVertexBounds(m, beta)
		beta[1] = 8
		require.Equal(t, []float64{0.125, 0.5, 0.25, 0.0625}, m.VertexBound, "the mesh keeps its own copy")
		require.InDelta(t, 0.5, m.FaceBound[a], 0)
		require.InDelta(t, 0.25, m.FaceBound[b], 0)
		require.InDelta(t, 0.5, m.Bound, 0)
	})

	t.Run("RecordIsReadBack", func(t *testing.T) {
		t.Parallel()
		m := twoFaces()
		topobuild.SetVertexBounds(m, []float64{0.125, 0.5, 0.25, 0.0625})
		got, err := topobuild.VertexBounds(m)
		require.NoError(t, err)
		require.Equal(t, []float64{0.125, 0.5, 0.25, 0.0625}, got)
		got[0] = 8
		require.InDelta(t, 0.125, m.VertexBound[0], 0, "the returned slice is the caller's own")
	})

	t.Run("DerivedFromFaceBounds", func(t *testing.T) {
		t.Parallel()
		m := twoFaces()
		topobuild.SetFaceBound(m, a, 0.5)
		topobuild.SetFaceBound(m, b, 0.25)
		got, err := topobuild.VertexBounds(m)
		require.NoError(t, err)
		require.Equal(t, []float64{0.5, 0.5, 0.5, 0.25}, got)
		require.Nil(t, m.VertexBound, "a derived reading publishes no record")
	})

	t.Run("MissingFaceBoundRefuses", func(t *testing.T) {
		t.Parallel()
		m := twoFaces()
		topobuild.SetFaceBound(m, a, 0.5)
		_, err := topobuild.VertexBounds(m)
		require.ErrorIs(t, err, decaderr.ErrBooleanFailed)
	})
}
