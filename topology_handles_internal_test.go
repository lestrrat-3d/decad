package decad

import (
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// TestFaceBodyIsTheBodyHandle pins the Face.body link: the body node a face
// holds maps back to the handle whose Faces returned the face, for an extruded
// box, a Transform result and a Copy. Face has no public body accessor, so the
// check reads the node link directly.
func TestFaceBodyIsTheBodyHandle(t *testing.T) {
	t.Parallel()
	doc := New()
	box := internalBoxBody(t, doc, 0, 0, 10, 10, 5)
	move, err := r3.Translation(r3.NewVec(3, 0, 0))
	require.NoError(t, err)
	// Transform retires its operand, so the copy is taken first.
	copied, err := Copy(t.Context(), box, WithCopyPlacement(move))
	require.NoError(t, err)
	moved, err := Transform(t.Context(), box, move)
	require.NoError(t, err)

	for name, b := range map[string]*Body{"extruded": box, "transformed": moved, "copied": copied} {
		faces := b.Faces()
		require.Len(t, faces, 6, name)
		for _, f := range faces {
			require.NotNil(t, f.n.Body, name)
			require.Same(t, b, bodyHandle(f.n.Body), name+": a face's body node maps to the body that returned it")
			require.Same(t, &b.n, bodyNode(b), name)
		}
	}
	require.Nil(t, bodyHandle(nil))
	require.Nil(t, bodyNode(nil))
}

// TestPrismBodyNodesAreBorn pins handleAlloc: every node the internal prism
// builder allocates is born inside its root handle, for an extruded box, its
// Transform result and a Copy. Each node reachable from the body maps back
// through Owner to the handle that holds it, and the handles the root
// accessors return are those same handles.
func TestPrismBodyNodesAreBorn(t *testing.T) {
	t.Parallel()
	doc := New()
	box := internalBoxBody(t, doc, 0, 0, 10, 10, 5)
	move, err := r3.Translation(r3.NewVec(3, 0, 0))
	require.NoError(t, err)
	copied, err := Copy(t.Context(), box, WithCopyPlacement(move))
	require.NoError(t, err)
	moved, err := Transform(t.Context(), box, move)
	require.NoError(t, err)

	for name, b := range map[string]*Body{"extruded": box, "transformed": moved, "copied": copied} {
		require.Same(t, b, bodyHandle(&b.n), name)
		require.Same(t, &doc.n, b.n.Doc, name)
		require.Len(t, b.n.Lumps, 1, name)
		for _, l := range b.n.Lumps {
			require.IsType(t, (*Lump)(nil), l.Owner, name)
			require.Same(t, l, lumpNode(lumpHandle(l)), name)
			for _, s := range l.Shells {
				require.IsType(t, (*FaceShell)(nil), s.Owner, name)
				require.Same(t, s, faceShellNode(faceShellHandle(s)), name)
			}
		}
		faces := b.Faces()
		require.Len(t, faces, 6, name)
		for _, f := range faces {
			require.Same(t, f, faceHandle(&f.n), name)
			require.Same(t, b, bodyHandle(f.n.Body), name)
			for _, l := range f.n.Loops {
				require.IsType(t, (*Loop)(nil), l.Owner, name)
				require.Same(t, l, loopNode(loopHandle(l)), name)
				for _, ce := range l.CoEdges {
					require.IsType(t, (*Edge)(nil), ce.Edge.Owner, name)
					require.Same(t, ce.Edge, edgeNode(edgeHandle(ce.Edge)), name)
					require.IsType(t, (*Vertex)(nil), ce.Edge.Start.Owner, name)
					require.IsType(t, (*Vertex)(nil), ce.Edge.End.Owner, name)
				}
			}
		}
		require.Len(t, b.Edges(), 12, name)
		require.Len(t, b.Vertices(), 8, name)
	}
}
