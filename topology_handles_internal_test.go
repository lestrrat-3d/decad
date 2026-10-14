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
