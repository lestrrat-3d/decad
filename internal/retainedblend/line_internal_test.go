package retainedblend

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// fanFace is a unit square in the plane z = 0 triangulated as a fan around a
// centre vertex at (0.5, 0.5, centreZ), the shape of a revolve's end cap.
func fanFace(centreZ float64) *work {
	fp := facetedrecord.Payload{
		Verts:       []r3.Vec{{}, {X: 1}, {X: 1, Y: 1}, {Y: 1}, {X: 0.5, Y: 0.5, Z: centreZ}},
		VertexBound: make([]float64, 5),
		Tris:        [][3]int{{0, 1, 4}, {1, 2, 4}, {2, 3, 4}, {3, 0, 4}},
		FaceOf:      []int{0, 0, 0, 0},
		Src:         []int{0, 0, 0, 0},
	}
	return newWork(fp, rim{scale: 1}, blendSpec{fillet: true, r: 1})
}

func TestFaceLoopsStrayVertex(t *testing.T) {
	t.Parallel()
	onZero := plane{n: r3.NewVec(0, 0, 1), o: r3.Vec{}}

	t.Run("a stray vertex exactly on the end plane is dropped", func(t *testing.T) {
		t.Parallel()
		fl, err := fanFace(0).faceLoops(0, &onZero)
		require.NoError(t, err)
		require.Len(t, fl.loops, 1)
		require.Len(t, fl.loops[0], 4, "the centre vertex is not on the boundary loop")
	})
	t.Run("a stray vertex off the end plane refuses", func(t *testing.T) {
		t.Parallel()
		_, err := fanFace(1e-12).faceLoops(0, &onZero)
		require.ErrorIs(t, err, decaderr.ErrUnsupported)
	})
	t.Run("a face that is not an end face refuses any stray vertex", func(t *testing.T) {
		t.Parallel()
		_, err := fanFace(0).faceLoops(0, nil)
		require.ErrorIs(t, err, decaderr.ErrUnsupported)
	})
}

func TestPlaneHolds(t *testing.T) {
	t.Parallel()
	p := plane{n: r3.NewVec(1, 0, 0), o: r3.NewVec(3, 0, 0)}

	t.Run("a point exactly on the plane is held", func(t *testing.T) {
		t.Parallel()
		require.True(t, p.holds(r3.NewVec(3, 7, -2)))
	})
	t.Run("a point off the plane by one ulp is not held", func(t *testing.T) {
		t.Parallel()
		require.False(t, p.holds(r3.NewVec(math.Nextafter(3, 4), 7, -2)))
	})
	t.Run("a point that is not finite is not held", func(t *testing.T) {
		t.Parallel()
		require.False(t, p.holds(r3.NewVec(math.Inf(1), 0, 0)))
	})
}
