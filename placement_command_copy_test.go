package decad

import (
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// copyMirrorPlane deep-copies the selector a MirrorFace holds, as a value and
// as a non-nil pointer, and the pointee of a MirrorFrame pointer
// (agent-docs/command-api-design.md rule A3). A change to the caller's query
// after the call reaches only the original.
func TestCopyMirrorPlaneDeepCopiesItsSelector(t *testing.T) {
	t.Parallel()
	q := Faces(Planar())
	value := copyMirrorPlane(MirrorFace{Face: q}).(MirrorFace).Face.(*FaceQuery)
	pointer := copyMirrorPlane(&MirrorFace{Face: q}).(*MirrorFace).Face.(*FaceQuery)
	requireIndependentCopies(t, q, map[string]*FaceQuery{"value": value, "pointer": pointer})

	frame := &MirrorFrame{Frame: r3.Frame{}}
	cp := copyMirrorPlane(frame).(*MirrorFrame)
	require.NotSame(t, frame, cp)

	t.Run("nil and selector-free values pass through", func(t *testing.T) {
		t.Parallel()
		require.Nil(t, copyMirrorPlane(nil))
		require.Equal(t, MirrorPlane((*MirrorFace)(nil)), copyMirrorPlane((*MirrorFace)(nil)))
		require.Equal(t, MirrorPlane((*MirrorFrame)(nil)), copyMirrorPlane((*MirrorFrame)(nil)))
		require.Equal(t, MirrorPlane(MirrorFrame{}), copyMirrorPlane(MirrorFrame{}))
	})
}
