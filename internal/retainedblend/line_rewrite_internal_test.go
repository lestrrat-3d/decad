package retainedblend

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// TestLineEndFaceTurn reads an end face G at a straight edge's corner V = 0 in the
// plane z = 0. A's chain leaves V toward (5, 0) and B's toward (0, 5); the
// rows (1, 0) and (0, 1) are the blend's setback points.
func TestLineEndFaceTurn(t *testing.T) {
	t.Parallel()
	x, y, z := r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1)
	verts := []r3.Vec{{}, r3.NewVec(5, 0, 0), r3.NewVec(5, 5, 0), r3.NewVec(0, 5, 0), x, y}
	end := lineEnd{g: plane{n: z}, v: 0}
	// G convex at V holds the quadrant between the chains; G reflex at V holds
	// the other three, and its loop runs the other way.
	convexG := faceLoops{loops: [][]int{{0, 1, 2, 3}}}
	reflexG := faceLoops{loops: [][]int{{0, 3, 2, 1}}}
	read := func(g faceLoops, dirs *[2]r3.Vec) (bool, error) {
		w := &work{verts: verts, beta: make([]float64, len(verts))}
		_, takes, err := w.endFace(end, g, []int{1}, []int{3}, [2]sideKind{trims, trims}, nil, nil, []int{4, 5},
			[]r3.Vec{x, y}, 1e-9, true, dirs)
		return takes, err
	}

	t.Run("a convex end face gives up the corner", func(t *testing.T) {
		t.Parallel()
		takes, err := read(convexG, &[2]r3.Vec{x, y})
		require.NoError(t, err)
		require.False(t, takes)
	})

	t.Run("a reflex end face takes the cap on the straight-edge route only", func(t *testing.T) {
		t.Parallel()
		takes, err := read(reflexG, &[2]r3.Vec{x, y})
		require.NoError(t, err)
		require.True(t, takes)
		_, err = read(reflexG, nil)
		require.ErrorIs(t, err, decaderr.ErrUnsupported)
		require.ErrorContains(t, err, "an end face's corner does not match the edge's convexity")
	})

	t.Run("a turn within the bound reads undecided and refuses", func(t *testing.T) {
		t.Parallel()
		// The held vertices show a square corner, but the tag lines meet at a
		// sine of 1e-4, under minSine: the turn is read from the tags alone.
		near, ok := r3.NewVec(-1, 1e-4, 0).Normalize()
		require.True(t, ok)
		_, err := read(convexG, &[2]r3.Vec{x, near})
		require.ErrorIs(t, err, decaderr.ErrUnsupported)
		require.ErrorContains(t, err, "an end face's turn at the edge's corner is undecided")
	})
}
