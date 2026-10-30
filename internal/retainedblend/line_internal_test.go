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

// TestGatesRefuseNaN reads each grazing-angle and direction gate with a value
// that is not a number, as a tag whose normal is not a number produces: every
// gate refuses it, as it refuses a sine under minSine.
func TestGatesRefuseNaN(t *testing.T) {
	t.Parallel()
	nan := math.NaN()
	x, y, z := r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1)

	t.Run("a straight edge's end, mitre and corner planes", func(t *testing.T) {
		t.Parallel()
		require.True(t, shallow(nan))
		require.True(t, shallow(minSine/2))
		require.False(t, shallow(minSine))
		require.False(t, shallow(1))
		_, ok := intersect3(plane{n: x}, plane{n: y}, plane{n: r3.NewVec(0, 0, nan)})
		require.False(t, ok, "a plane with no normal meets no corner")
		_, ok = intersect3(plane{n: x}, plane{n: y}, plane{n: z})
		require.True(t, ok)
		require.True(t, shallow(endSine(1, x, plane{n: r3.NewVec(nan, 0, 0)})),
			"a NaN normal reaches the corner route's sine")
	})

	t.Run("a plane-cylinder edge's end plane", func(t *testing.T) {
		t.Parallel()
		require.True(t, grazes(nan, 0))
		require.True(t, grazes(4, nan))
		require.True(t, grazes(4, 4*math.Sqrt(1-minSine*minSine/4)))
		require.False(t, grazes(4, 2))
	})

	t.Run("a cylinder-cylinder edge's walls", func(t *testing.T) {
		t.Parallel()
		require.False(t, clears(nan, 1))
		require.False(t, clears(1, nan))
		require.False(t, clears(minSine/2, 1))
		require.True(t, clears(0.5, 1))
	})

	t.Run("a sphere rim's plane", func(t *testing.T) {
		t.Parallel()
		require.Error(t, crossing(nan, 5))
		require.Error(t, crossing(4, nan))
		require.Error(t, crossing(minSine*minSine, 5))
		require.NoError(t, crossing(4, 5))
	})

	t.Run("a chain end's corner turn", func(t *testing.T) {
		t.Parallel()
		require.Zero(t, cornerTurn(y, x, r3.NewVec(0, 0, nan)), "a NaN normal reads as undecided")
		require.Equal(t, 1, cornerTurn(y, x, z))
		require.Equal(t, -1, cornerTurn(y, x, z.Scale(-1)))
	})
}
