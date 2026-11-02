package booleandispatch_test

import (
	"context"
	"testing"

	"github.com/lestrrat-3d/decad/internal/booleandispatch"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/orthoshell"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func orthogonalPairBox(lo, hi r3.Vec) facetedrecord.Payload {
	return facetedrecord.Payload{
		Verts: []r3.Vec{
			{X: lo.X, Y: lo.Y, Z: lo.Z}, {X: hi.X, Y: lo.Y, Z: lo.Z},
			{X: hi.X, Y: hi.Y, Z: lo.Z}, {X: lo.X, Y: hi.Y, Z: lo.Z},
			{X: lo.X, Y: lo.Y, Z: hi.Z}, {X: hi.X, Y: lo.Y, Z: hi.Z},
			{X: hi.X, Y: hi.Y, Z: hi.Z}, {X: lo.X, Y: hi.Y, Z: hi.Z},
		},
		Tris: [][3]int{
			{0, 3, 2}, {0, 2, 1}, {4, 5, 6}, {4, 6, 7},
			{0, 1, 5}, {0, 5, 4}, {3, 7, 6}, {3, 6, 2},
			{0, 4, 7}, {0, 7, 3}, {1, 2, 6}, {1, 6, 5},
		},
	}
}

func orthogonalPairVolume(part orthoshell.Part) float64 {
	volume := 0.0
	for _, tri := range part.Tris {
		volume += part.Verts[tri[0]].Dot(part.Verts[tri[1]].Cross(part.Verts[tri[2]])) / 6
	}
	return volume
}

type orthogonalPairMeshes struct {
	t     *testing.T
	want  *topo.Body
	mesh  *topo.Mesh
	err   error
	calls int
}

func (m *orthogonalPairMeshes) VerifyAll(_ context.Context, body *topo.Body, tolMM float64) (*topo.Mesh, error) {
	m.t.Helper()
	require.True(m.t, body == m.want, "the mesher receives the other operand")
	require.Equal(m.t, 0.1, tolMM)
	m.calls++
	return m.mesh, m.err
}

func TestOrthogonalPairSelectsOperation(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name   string
		op     meshbool.OperationKind
		volume float64
	}{
		{"union", meshbool.OpUnion, 10},
		{"cut", meshbool.OpCut, 6},
		{"intersect", meshbool.OpIntersect, 2},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			a := &topo.Body{Payload: orthogonalPairBox(r3.Vec{}, r3.Vec{X: 2, Y: 2, Z: 2})}
			b := &topo.Body{Payload: orthogonalPairBox(r3.Vec{X: 1}, r3.Vec{X: 3, Y: 1, Z: 2})}
			meshes := &orthogonalPairMeshes{t: t}
			part, ok := booleandispatch.OrthogonalPair(t.Context(), tc.op, a, b, meshes)
			require.True(t, ok)
			require.InDelta(t, tc.volume, orthogonalPairVolume(part), 1e-12)
			require.Zero(t, meshes.calls, "faceted operands do not need tessellation")
		})
	}
}

func TestOrthogonalPairTessellatesOnlyOtherOperand(t *testing.T) {
	t.Parallel()
	for _, side := range []string{"first", "second"} {
		t.Run(side, func(t *testing.T) {
			t.Parallel()
			pa := orthogonalPairBox(r3.Vec{}, r3.Vec{X: 2, Y: 2, Z: 2})
			pb := orthogonalPairBox(r3.Vec{X: 1}, r3.Vec{X: 3, Y: 1, Z: 2})
			a, b := &topo.Body{Payload: pa}, &topo.Body{Payload: pb}
			other, payload := b, pb
			if side == "first" {
				other, payload = a, pa
			}
			other.Payload = prismrecord.Payload{}
			meshes := &orthogonalPairMeshes{
				t: t, want: other, mesh: &topo.Mesh{Vertices: payload.Verts, Triangles: payload.Tris},
			}
			part, ok := booleandispatch.OrthogonalPair(t.Context(), meshbool.OpCut, a, b, meshes)
			require.True(t, ok)
			require.InDelta(t, 6.0, orthogonalPairVolume(part), 1e-12)
			require.Equal(t, 1, meshes.calls)
		})
	}
}

func TestOrthogonalPairKeepsAnalyticPairOnItsOwnRoutes(t *testing.T) {
	t.Parallel()
	a := &topo.Body{Payload: prismrecord.Payload{}}
	b := &topo.Body{Payload: prismrecord.Payload{}}
	meshes := &orthogonalPairMeshes{t: t}
	_, ok := booleandispatch.OrthogonalPair(t.Context(), meshbool.OpUnion, a, b, meshes)
	require.False(t, ok)
	require.Zero(t, meshes.calls)
}

func TestOrthogonalPairRejectsUnprovedOtherOperand(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name  string
		edit  func(*topo.Body, *orthogonalPairMeshes)
		calls int
	}{
		{"no payload", func(body *topo.Body, _ *orthogonalPairMeshes) { body.Payload = nil }, 0},
		{"sheet", func(body *topo.Body, _ *orthogonalPairMeshes) { body.Kind = topo.BodySheet }, 0},
		{"mesh bound", func(_ *topo.Body, meshes *orthogonalPairMeshes) { meshes.mesh.Bound = 0.25 }, 1},
		{"no triangles", func(_ *topo.Body, meshes *orthogonalPairMeshes) { meshes.mesh.Triangles = nil }, 1},
		{"mesher refusal", func(_ *topo.Body, meshes *orthogonalPairMeshes) { meshes.err = context.Canceled }, 1},
		{"faceted vertex bound", func(body *topo.Body, meshes *orthogonalPairMeshes) {
			body.Payload = facetedrecord.Payload{
				Verts: meshes.mesh.Vertices, Tris: meshes.mesh.Triangles, VertexBound: []float64{0.25},
			}
			meshes.mesh.Bound = 0.25
		}, 1},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			a := &topo.Body{Payload: orthogonalPairBox(r3.Vec{}, r3.Vec{X: 2, Y: 2, Z: 2})}
			pb := orthogonalPairBox(r3.Vec{X: 1}, r3.Vec{X: 3, Y: 1, Z: 2})
			b := &topo.Body{Payload: prismrecord.Payload{}}
			meshes := &orthogonalPairMeshes{
				t: t, want: b, mesh: &topo.Mesh{Vertices: pb.Verts, Triangles: pb.Tris},
			}
			tc.edit(b, meshes)
			part, ok := booleandispatch.OrthogonalPair(t.Context(), meshbool.OpUnion, a, b, meshes)
			require.False(t, ok)
			require.Zero(t, len(part.Verts))
			require.Zero(t, len(part.Tris))
			require.Equal(t, tc.calls, meshes.calls)
		})
	}
}

func TestOrthogonalPairDeclinesUnknownOperationAndGridRefusals(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name string
		op   meshbool.OperationKind
		edit func(*facetedrecord.Payload, *facetedrecord.Payload)
	}{
		{"unknown operation", meshbool.OperationKind(99), func(_, _ *facetedrecord.Payload) {}},
		{"slanted triangle", meshbool.OpUnion, func(a, _ *facetedrecord.Payload) { a.Verts[0].X = 0.25 }},
		{"empty intersection", meshbool.OpIntersect, func(_, b *facetedrecord.Payload) {
			*b = orthogonalPairBox(r3.Vec{X: 4}, r3.Vec{X: 5, Y: 1, Z: 1})
		}},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			pa := orthogonalPairBox(r3.Vec{}, r3.Vec{X: 2, Y: 2, Z: 2})
			pb := orthogonalPairBox(r3.Vec{X: 1}, r3.Vec{X: 3, Y: 1, Z: 2})
			tc.edit(&pa, &pb)
			meshes := &orthogonalPairMeshes{t: t}
			part, ok := booleandispatch.OrthogonalPair(t.Context(), tc.op,
				&topo.Body{Payload: pa}, &topo.Body{Payload: pb}, meshes)
			require.False(t, ok)
			require.Zero(t, len(part.Verts))
			require.Zero(t, len(part.Tris))
			require.Zero(t, meshes.calls)
		})
	}
}
