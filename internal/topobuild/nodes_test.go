package topobuild_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
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

// TestNewKeepsTheAllocatorsOwner fills a node from a literal: every field
// comes from the literal except Owner, which stays the allocator's.
func TestNewKeepsTheAllocatorsOwner(t *testing.T) {
	t.Parallel()
	a := ownerAlloc{}
	f := topobuild.NewFace(a, topo.Face{Area: 2, Owner: "literal"})
	require.Equal(t, "face", f.Owner)
	require.Equal(t, 2.0, f.Area)
	l := topobuild.NewLoop(a, topo.Loop{Outer: true, Owner: "literal"})
	require.Equal(t, "loop", l.Owner)
	require.True(t, l.Outer)
	e := topobuild.NewEdge(a, topo.Edge{Length: 3, Owner: "literal"})
	require.Equal(t, "edge", e.Owner)
	require.Equal(t, 3.0, e.Length)
	v := topobuild.NewVertex(a, topo.Vertex{Owner: "literal"})
	require.Equal(t, "vertex", v.Owner)
}

// TestSheetLumpsSplitsConnectedPieces builds two faces that share an edge and
// a third that shares none: two lumps, each one open shell from the
// allocator, faces in input order; the walkers read them back in that order
// with each edge once.
func TestSheetLumpsSplitsConnectedPieces(t *testing.T) {
	t.Parallel()
	shared, e1, e2, e3 := &topo.Edge{}, &topo.Edge{}, &topo.Edge{}, &topo.Edge{}
	face := func(edges ...*topo.Edge) *topo.Face {
		l := &topo.Loop{}
		for _, e := range edges {
			l.CoEdges = append(l.CoEdges, topo.CoEdge{Edge: e, Forward: true})
		}
		return &topo.Face{Loops: []*topo.Loop{l}}
	}
	f1, f2, f3 := face(shared, e1), face(e2, shared), face(e3)
	faces := []*topo.Face{f1, f2, f3}
	require.NoError(t, topobuild.AttachFaceLoops(t.Context(), faces))
	require.Equal(t, []*topo.Face{f1, f2}, shared.Faces)

	lumps := topobuild.SheetLumps(ownerAlloc{}, faces)
	require.Len(t, lumps, 2)
	require.Equal(t, "lump", lumps[0].Owner)
	require.Len(t, lumps[0].Shells, 1)
	require.Equal(t, "shell", lumps[0].Shells[0].Owner)
	require.Equal(t, []*topo.Face{f1, f2}, lumps[0].Shells[0].Faces)
	require.True(t, lumps[0].Shells[0].Open)
	require.False(t, lumps[0].Shells[0].Void)
	require.Equal(t, []*topo.Face{f3}, lumps[1].Shells[0].Faces)

	body := &topo.Body{Lumps: lumps}
	require.Equal(t, faces, topobuild.Faces(body))
	require.Equal(t, []*topo.Edge{shared, e1, e2, e3}, topobuild.Edges(body))

	require.Nil(t, topobuild.SheetLumps(ownerAlloc{}, nil), `no faces, no lumps`)
	require.Nil(t, topobuild.Faces(&topo.Body{}))
	require.Nil(t, topobuild.Edges(&topo.Body{}))
}
