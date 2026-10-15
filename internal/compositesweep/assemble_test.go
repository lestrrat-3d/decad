package compositesweep_test

import (
	"context"
	"fmt"
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad/internal/compositesweep"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// ownerAlloc stamps every node with a recognisable Owner, the way root's
// handle allocator does with the handle that holds the node.
type ownerAlloc struct{}

func (ownerAlloc) NewBody() *topo.Body           { return &topo.Body{Owner: "body"} }
func (ownerAlloc) NewLump() *topo.Lump           { return &topo.Lump{Owner: "lump"} }
func (ownerAlloc) NewFaceShell() *topo.FaceShell { return &topo.FaceShell{Owner: "shell"} }
func (ownerAlloc) NewFace() *topo.Face           { return &topo.Face{Owner: "face"} }
func (ownerAlloc) NewLoop() *topo.Loop           { return &topo.Loop{Owner: "loop"} }
func (ownerAlloc) NewEdge() *topo.Edge           { return &topo.Edge{Owner: "edge"} }
func (ownerAlloc) NewVertex() *topo.Vertex       { return &topo.Vertex{Owner: "vertex"} }

// boxSpan builds one closed unit-square prism span from z0 to z1 the way the
// prism builder lays it out: four walls side(0,i), and two caps whose loops
// walk the bottom and top ring edges forward in profile order.
func boxSpan(z0, z1 float64) *topo.Body {
	corners := [4]r3.Vec{r3.NewVec(0, 0, 0), r3.NewVec(1, 0, 0), r3.NewVec(1, 1, 0), r3.NewVec(0, 1, 0)}
	body := &topo.Body{Solid: true}
	var bottom, top [4]*topo.Vertex
	for i, c := range corners {
		bottom[i] = &topo.Vertex{Position: r3.NewVec(c.X, c.Y, z0)}
		top[i] = &topo.Vertex{Position: r3.NewVec(c.X, c.Y, z1)}
	}
	var bottomEdges, topEdges, vertical [4]*topo.Edge
	for i := range corners {
		bottomEdges[i] = &topo.Edge{Start: bottom[i], End: bottom[(i+1)%4]}
		topEdges[i] = &topo.Edge{Start: top[i], End: top[(i+1)%4]}
		vertical[i] = &topo.Edge{Start: bottom[i], End: top[i]}
	}
	link := func(f *topo.Face) *topo.Face {
		for _, ce := range f.Loops[0].CoEdges {
			ce.Edge.Faces = append(ce.Edge.Faces, f)
		}
		return f
	}
	var faces []*topo.Face
	startCo, endCo := make([]topo.CoEdge, 4), make([]topo.CoEdge, 4)
	for i := range corners {
		startCo[i] = topo.CoEdge{Edge: bottomEdges[i], Forward: true}
		endCo[i] = topo.CoEdge{Edge: topEdges[i], Forward: true}
	}
	faces = append(faces, link(&topo.Face{
		Area: 1, Body: body, Origins: []provenance.Ref{{Role: provenance.RoleCapStart}},
		Loops: []*topo.Loop{{Outer: true, CoEdges: startCo}},
	}))
	for i := range corners {
		faces = append(faces, link(&topo.Face{
			Area: z1 - z0, Body: body, Origins: []provenance.Ref{{Role: fmt.Sprintf("side(0,%d)", i)}},
			Loops: []*topo.Loop{{Outer: true, CoEdges: []topo.CoEdge{
				{Edge: bottomEdges[i], Forward: true},
				{Edge: vertical[(i+1)%4], Forward: true},
				{Edge: topEdges[i], Forward: false},
				{Edge: vertical[i], Forward: false},
			}}},
		}))
	}
	faces = append(faces, link(&topo.Face{
		Area: 1, Body: body, Origins: []provenance.Ref{{Role: provenance.RoleCapEnd}},
		Loops: []*topo.Loop{{Outer: true, CoEdges: endCo}},
	}))
	body.Lumps = []*topo.Lump{{Shells: []*topo.FaceShell{{Faces: faces}}}}
	return body
}

func boxParts(t *testing.T, n int) []compositesweep.SpanPart {
	t.Helper()
	parts := make([]compositesweep.SpanPart, n)
	for i := range parts {
		part, err := compositesweep.SpanPartOf(boxSpan(float64(i), float64(i+1)))
		require.NoError(t, err)
		parts[i] = part
	}
	return parts
}

func acceptSpans(ctx context.Context, _ []compositesweep.SpanPart) error { return ctx.Err() }

func vertexCount(body *topo.Body) int {
	seen := map[*topo.Vertex]struct{}{}
	for _, e := range topobuild.Edges(body) {
		seen[e.Start] = struct{}{}
		seen[e.End] = struct{}{}
	}
	return len(seen)
}

func faceRoles(body *topo.Body) []string {
	var roles []string
	for _, f := range topobuild.Faces(body) {
		roles = append(roles, f.Origins[0].Role)
	}
	return roles
}

func TestAssembleSewsTwoBoxSpans(t *testing.T) {
	t.Parallel()
	parts := boxParts(t, 2)
	discarded := parts[0].EndCap.Loops[0].CoEdges[0].Edge
	kept := parts[1].StartCap.Loops[0].CoEdges[0].Edge
	var audited int
	audit := func(ctx context.Context, spans []compositesweep.SpanPart) error {
		audited = len(spans)
		return ctx.Err()
	}
	body, err := compositesweep.Assemble(t.Context(), topo.Env{Ref: 7, Alloc: ownerAlloc{}}, parts, false,
		audit, compositesweep.SewJoin, false)
	require.NoError(t, err)
	require.Equal(t, 2, audited)
	require.Equal(t, "body", body.Owner)
	require.True(t, body.Solid)
	require.Equal(t, topo.BodySolid, body.Kind)
	require.Equal(t, provenance.Ref{Producer: 7, Role: provenance.RoleBody}, body.Origin)
	require.Len(t, body.Lumps, 1)
	require.Equal(t, "lump", body.Lumps[0].Owner)
	require.Equal(t, "shell", body.Lumps[0].Shells[0].Owner)
	require.Equal(t, []string{
		provenance.RoleCapStart,
		"side(0,0,0)", "side(0,0,1)", "side(0,0,2)", "side(0,0,3)",
		"side(1,0,0)", "side(1,0,1)", "side(1,0,2)", "side(1,0,3)",
		provenance.RoleCapEnd,
	}, faceRoles(body))
	edges := topobuild.Edges(body)
	require.Len(t, edges, 20)
	require.Equal(t, 12, vertexCount(body))
	// Identity checks only: a failing assertion must not dump the node graph.
	for i, e := range edges {
		require.Lenf(t, e.Faces, 2, "edge %d", i)
		require.Falsef(t, e == discarded, "edge %d is the discarded end rim", i)
		for _, f := range e.Faces {
			require.Truef(t, f.Body == body, "edge %d has a face of another body", i)
		}
	}
	require.True(t, slices.Contains(edges, kept), "the next span's start rim edge is kept")
	// The kept rim edge now ends at the second span's own section vertices.
	require.Equal(t, r3.NewVec(0, 0, 1), kept.Start.Position)
	require.Equal(t, r3.NewVec(1, 0, 1), kept.End.Position)
}

func TestAssembleSurfaceResultOmitsOuterCaps(t *testing.T) {
	t.Parallel()
	body, err := compositesweep.Assemble(t.Context(), topo.Env{Alloc: ownerAlloc{}}, boxParts(t, 2), true,
		acceptSpans, compositesweep.SewJoin, false)
	require.NoError(t, err)
	require.False(t, body.Solid)
	require.Equal(t, topo.BodySheet, body.Kind)
	require.Equal(t, []string{
		"side(0,0,0)", "side(0,0,1)", "side(0,0,2)", "side(0,0,3)",
		"side(1,0,0)", "side(1,0,1)", "side(1,0,2)", "side(1,0,3)",
	}, faceRoles(body))
	require.Len(t, body.Lumps, 1)
	require.True(t, body.Lumps[0].Shells[0].Open)
	free := 0
	for _, e := range topobuild.Edges(body) {
		if len(e.Faces) == 1 {
			free++
		}
	}
	require.Equal(t, 8, free, "each outer rim keeps its four edges on one wall")
}

func TestAssembleTwistJoinMatchesByPosition(t *testing.T) {
	t.Parallel()
	parts := boxParts(t, 2)
	// Rotate the next span's start-cap walk: positional pairing must still
	// find each rim edge, where index pairing would not.
	co := parts[1].StartCap.Loops[0].CoEdges
	parts[1].StartCap.Loops[0].CoEdges = append(co[1:], co[0])
	body, err := compositesweep.Assemble(t.Context(), topo.Env{Alloc: ownerAlloc{}}, parts, false,
		acceptSpans, compositesweep.SewTwistLoftJoin, false)
	require.NoError(t, err)
	require.Len(t, topobuild.Edges(body), 20)
	require.Equal(t, 12, vertexCount(body))
}

func TestAssembleRefusals(t *testing.T) {
	t.Parallel()
	t.Run("one span", func(t *testing.T) {
		t.Parallel()
		_, err := compositesweep.Assemble(t.Context(), topo.Env{Alloc: ownerAlloc{}}, boxParts(t, 1), false,
			acceptSpans, compositesweep.SewJoin, false)
		require.ErrorIs(t, err, decaderr.ErrDegenerate)
		require.EqualError(t, err, decaderr.ErrDegenerate.Error()+": a composite sweep requires at least two spans")
	})
	t.Run("missing end cap", func(t *testing.T) {
		t.Parallel()
		body := boxSpan(0, 1)
		faces := body.Lumps[0].Shells[0].Faces
		body.Lumps[0].Shells[0].Faces = faces[:len(faces)-1]
		_, err := compositesweep.SpanPartOf(body)
		require.EqualError(t, err, decaderr.ErrUnsupported.Error()+": a sweep span does not carry both path-oriented caps")
	})
	t.Run("loop count", func(t *testing.T) {
		t.Parallel()
		parts := boxParts(t, 2)
		parts[1].StartCap.Loops = append(parts[1].StartCap.Loops, parts[1].StartCap.Loops[0])
		err := compositesweep.ValidateRims(t.Context(), parts)
		require.EqualError(t, err, decaderr.ErrUnsupported.Error()+": adjacent sweep sections have different loop counts")
	})
	t.Run("join stage", func(t *testing.T) {
		t.Parallel()
		_, err := compositesweep.Assemble(t.Context(), topo.Env{Alloc: ownerAlloc{}}, boxParts(t, 2), false,
			acceptSpans, func(context.Context, *topo.Face, *topo.Face, *topo.Body, map[*topo.Edge]struct{}) error {
				return decaderr.ErrUnsupported
			}, false)
		require.EqualError(t, err, "sweep path join 1: "+decaderr.ErrUnsupported.Error())
	})
	t.Run("zero area face", func(t *testing.T) {
		t.Parallel()
		body := boxSpan(0, 1)
		body.Lumps[0].Shells[0].Faces[1].Area = 0
		err := compositesweep.AuditBoundary(t.Context(), body, nil)
		require.EqualError(t, err, decaderr.ErrUnsupported.Error()+": a composite sweep face is not proven to have positive area")
	})
}

func TestLiveFacesKeepsOwnFacesOnce(t *testing.T) {
	t.Parallel()
	body, other := &topo.Body{}, &topo.Body{}
	a, b, c := &topo.Face{Body: body}, &topo.Face{Body: other}, &topo.Face{Body: body}
	live := compositesweep.LiveFaces([]*topo.Face{nil, a, b, a, c}, body)
	require.Len(t, live, 2)
	require.True(t, live[0] == a && live[1] == c, "LiveFaces keeps a then c")
}
