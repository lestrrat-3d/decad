package meshbool

import (
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func retainedMeshEdgeFixture(t *testing.T) (*topo.Mesh, *BoolMesh, *topo.Edge) {
	t.Helper()
	verts := []r3.Vec{{X: 0}, {X: 1}, {Y: 1}, {Z: 1}}
	tris := [][3]int{{0, 1, 2}, {1, 0, 3}}
	a, b := &topo.Face{}, &topo.Face{}
	e := &topo.Edge{RetainedCurve: true, Curve: surfacegeom.Line3{}, Faces: []*topo.Face{a, b},
		Start: &topo.Vertex{Position: r3.NewVec(-1, 0, 0), Bound: units.Millimeters(0)},
		End:   &topo.Vertex{Position: r3.NewVec(2, 0, 0), Bound: units.Millimeters(0)}}
	a.Loops = []*topo.Loop{{CoEdges: []topo.CoEdge{{Edge: e, Forward: true}}}}
	b.Loops = []*topo.Loop{{CoEdges: []topo.CoEdge{{Edge: e, Forward: false}}}}
	m := &topo.Mesh{Vertices: verts, Triangles: tris, Source: []*topo.Face{a, b}, BoundaryOK: true}
	bm, err := PrepContext(t.Context(), verts, tris, []int{0, 1})
	require.NoError(t, err)
	return m, bm, e
}

func TestRetainedEdgeAtMeshEdge(t *testing.T) {
	for _, tc := range []struct {
		name string
		edit func(*topo.Mesh, *BoolMesh, *topo.Edge)
		want bool
	}{
		{name: "shared retained edge", want: true},
		{name: "uncertain endpoint", edit: func(_ *topo.Mesh, _ *BoolMesh, e *topo.Edge) { e.Start.Bound = units.Millimeters(1e-9) }},
		{name: "different edge", edit: func(_ *topo.Mesh, _ *BoolMesh, e *topo.Edge) { e.RetainedCurve = false }},
		{name: "off the line", edit: func(m *topo.Mesh, bm *BoolMesh, _ *topo.Edge) {
			v := append([]r3.Vec(nil), m.Vertices...)
			v[1].Y = math.SmallestNonzeroFloat64
			m.Vertices, bm.Verts = v, v
		}},
		{name: "outside finite edge", edit: func(_ *topo.Mesh, _ *BoolMesh, e *topo.Edge) { e.End.Position.X = 0.5 }},
		{name: "different sources", edit: func(m *topo.Mesh, _ *BoolMesh, _ *topo.Edge) { m.Source[1] = &topo.Face{} }},
		{name: "unverified mesh", edit: func(m *topo.Mesh, _ *BoolMesh, _ *topo.Edge) { m.BoundaryOK = false }},
		{name: "ambiguous edges", edit: func(m *topo.Mesh, _ *BoolMesh, e *topo.Edge) {
			other := *e
			m.Source[0].Loops[0].CoEdges = append(m.Source[0].Loops[0].CoEdges, topo.CoEdge{Edge: &other})
			m.Source[1].Loops[0].CoEdges = append(m.Source[1].Loops[0].CoEdges, topo.CoEdge{Edge: &other})
		}},
		{name: "stale prepared mesh", edit: func(m *topo.Mesh, _ *BoolMesh, _ *topo.Edge) {
			m.Triangles = append([][3]int(nil), m.Triangles...)
			m.Triangles[1][2] = 2
		}},
	} {
		t.Run(tc.name, func(t *testing.T) {
			m, bm, e := retainedMeshEdgeFixture(t)
			if tc.edit != nil {
				tc.edit(m, bm, e)
			}
			got, err := RetainedEdgeAtMeshEdge(t.Context(), m, bm, 0, 0, proofbound.NewWorkBudget(t.Context()))
			require.NoError(t, err)
			require.Equal(t, tc.want, got != nil)
			if tc.want {
				require.Same(t, e, got)
			}
		})
	}
	m, bm, _ := retainedMeshEdgeFixture(t)
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	_, err := RetainedEdgeAtMeshEdge(ctx, m, bm, 0, 0, proofbound.NewWorkBudget(ctx))
	require.ErrorIs(t, err, context.Canceled)
	budget := &proofbound.WorkBudget{StepFn: func() error { return proofbound.ErrWallWorkBudget }, ErrFn: func() error { return nil }}
	_, err = RetainedEdgeAtMeshEdge(t.Context(), m, bm, 0, 0, budget)
	require.ErrorIs(t, err, proofbound.ErrWallWorkBudget)
}
