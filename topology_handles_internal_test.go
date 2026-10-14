package decad

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// requireOwned walks every node reachable from b's node and requires each to
// carry an Owner of its handle type, and each node pointer to map back to the
// handle the public accessors return.
func requireOwned(t *testing.T, b *Body) {
	t.Helper()
	require.Same(t, b, handleOf[Body](b.n.Owner))
	require.IsType(t, (*Document)(nil), b.n.Doc.Owner)
	for _, ln := range b.n.Lumps {
		require.IsType(t, (*Lump)(nil), ln.Owner)
		for _, sn := range ln.Shells {
			require.IsType(t, (*Shell)(nil), sn.Owner)
		}
	}
	for _, fn := range b.n.Faces() {
		require.IsType(t, (*Face)(nil), fn.Owner)
		require.Same(t, b, bodyOf(fn.Body))
		for _, ln := range fn.Loops {
			require.IsType(t, (*Loop)(nil), ln.Owner)
		}
	}
	for _, en := range b.n.Edges() {
		require.IsType(t, (*Edge)(nil), en.Owner)
		require.IsType(t, (*Vertex)(nil), en.Start.Owner)
		require.IsType(t, (*Vertex)(nil), en.End.Owner)
	}
	for _, f := range b.Faces() {
		require.Same(t, f, faceOf(&f.n))
	}
	for _, e := range b.Edges() {
		require.Same(t, e, edgeOf(&e.n))
	}
	for _, v := range b.Vertices() {
		require.Same(t, v, vertexOf(&v.n))
	}
	require.NoError(t, dualCheck(b))
}

// requireLiveNodes requires the document node's live set to map back to the
// handle live set, in order.
func requireLiveNodes(t *testing.T, d *Document) {
	t.Helper()
	live := d.liveBodies()
	nodes := d.n.Live()
	require.Len(t, nodes, len(live))
	for i, n := range nodes {
		require.Same(t, live[i], bodyOf(n))
	}
}

// TestNodeBuiltBodiesAreOwned checks an Extrude, a Transform and a Copy
// result: every node reachable from the body has an Owner of the right type,
// every handle maps back from its own node, and the legacy fields agree.
func TestNodeBuiltBodiesAreOwned(t *testing.T) {
	t.Parallel()
	doc := New()
	box := internalBoxBodyAtZ(t, doc, 0, 0, 4, 4, 0, 4)
	requireOwned(t, box)
	requireLiveNodes(t, doc)

	move, err := r3.Translation(r3.NewVec(10, 0, 0))
	require.NoError(t, err)
	moved, err := Transform(t.Context(), box, move)
	require.NoError(t, err)
	requireOwned(t, moved)
	require.False(t, doc.n.IsLive(&box.n), `Transform retires the source node too`)
	requireLiveNodes(t, doc)

	copied, err := Copy(t.Context(), moved, WithCopyPlacement(move))
	require.NoError(t, err)
	requireOwned(t, copied)
	require.True(t, doc.n.IsLive(&moved.n))
	require.True(t, doc.n.IsLive(&copied.n))
	requireLiveNodes(t, doc)

	require.NoError(t, doc.Remove(moved))
	require.NoError(t, doc.Remove(copied))
	require.Nil(t, doc.n.Live())
}

// TestNodeMappingNilAndBare shows the mapping keeps nil-versus-empty (an
// empty node slice maps to nil, as the accessors' append copies do) and
// refuses a node no root allocator built.
func TestNodeMappingNilAndBare(t *testing.T) {
	t.Parallel()
	require.Nil(t, mapNodes([]*topo.Face{}, faceOf))
	require.Nil(t, mapNodes([]*topo.Face(nil), faceOf))
	require.Nil(t, coEdgesOf([]topo.CoEdge{}))
	require.Nil(t, featureRefsOf(nil))
	require.Nil(t, faceOf(nil))
	require.Panics(t, func() { faceOf(&topo.Face{}) })
}
