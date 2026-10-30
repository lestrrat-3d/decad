package retainedblend

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// TestCylEndSense reads which way the plane's and the wall's chains leave an
// edge end at the origin: the plane's along the tag line x, from its last
// vertex, and the wall's along the end circle's tangent ±y, from its first.
// A vertex within its bound of the corner leaves the sense undecided.
func TestCylEndSense(t *testing.T) {
	t.Parallel()
	x, y := r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0)
	const bound = 0.01

	t.Run("the plane's chain", func(t *testing.T) {
		t.Parallel()
		dir, err := lineSense(x, r3.NewVec(5, 0, 0), r3.Vec{}, bound)
		require.NoError(t, err)
		require.Equal(t, x, dir)
		dir, err = lineSense(x, r3.NewVec(-5, 0, 0), r3.Vec{}, bound)
		require.NoError(t, err)
		require.Equal(t, x.Scale(-1), dir)
		_, err = lineSense(x, r3.NewVec(0.5*bound, 0, 0), r3.Vec{}, bound)
		require.ErrorIs(t, err, decaderr.ErrUnsupported)
		require.ErrorContains(t, err, "too short a run")
	})

	t.Run("the wall's chain", func(t *testing.T) {
		t.Parallel()
		kind, err := wallSense(y, r3.NewVec(0.1, 1, 0), r3.Vec{}, bound)
		require.NoError(t, err)
		require.Equal(t, trims, kind)
		kind, err = wallSense(y, r3.NewVec(0.1, -1, 0), r3.Vec{}, bound)
		require.NoError(t, err)
		require.Equal(t, extends, kind)
		_, err = wallSense(y, r3.NewVec(0, 0.5*bound, 0), r3.Vec{}, bound)
		require.ErrorIs(t, err, decaderr.ErrUnsupported)
		require.ErrorContains(t, err, "too short a run")
		// Farther than the bound, but at under minSine of its distance.
		_, err = wallSense(y, r3.NewVec(100, 0.05, 0), r3.Vec{}, bound)
		require.ErrorIs(t, err, decaderr.ErrUnsupported)
	})
}

// TestCylRemovedArea charges the corner region A at each end of a strip of
// area 10 and a band of area 4: a convex edge takes A out of an end face
// where the wall ends with the edge, and gives A to it as the cap where the
// wall extends past it; a concave edge fills A at both ends.
func TestCylRemovedArea(t *testing.T) {
	t.Parallel()
	const strip, band, corner = 10.0, 4.0, 0.5
	for _, c := range []struct {
		name   string
		convex bool
		extB   [2]bool
		want   float64
	}{
		{"a cap at both ends", true, [2]bool{true, true}, strip + band - 2*corner},
		{"a cap at one end, a corner at the other", true, [2]bool{true, false}, strip + band},
		{"a corner at both ends", true, [2]bool{}, strip + band + 2*corner},
		{"a concave edge", false, [2]bool{}, strip + band - 2*corner},
	} {
		require.Equal(t, c.want, cylRemovedArea(strip, band, corner, c.convex, c.extB), c.name)
	}
}

// TestCapSimple reads end caps in the plane z = 0 that run from P_A = (20, 0)
// along rows to the band's corner at index 2, back along the wall's cut to
// the corner V = 0, and along the plane's line to P_A.
func TestCapSimple(t *testing.T) {
	t.Parallel()
	read := func(pts ...[2]float64) error {
		w := &work{}
		loop := make([]int, len(pts))
		for k, p := range pts {
			w.verts = append(w.verts, r3.NewVec(p[0], p[1], 0))
			loop[k] = k
		}
		return w.capSimple(r3.NewVec(0, 0, 1), loop, 2)
	}
	require.NoError(t, read([2]float64{20, 0}, [2]float64{6, 6}, [2]float64{0, 20}, [2]float64{-1, 10}, [2]float64{}))
	// The wall's cut crosses the first row segment.
	err := read([2]float64{20, 0}, [2]float64{6, 6}, [2]float64{0, 20}, [2]float64{1, 12}, [2]float64{8, 4},
		[2]float64{})
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
	require.ErrorContains(t, err, "the blend's end cap folds")
	// The wall's cut leaves the band's corner straight on from the last row:
	// the cap does not close in a cusp there.
	err = read([2]float64{20, 0}, [2]float64{6, 6}, [2]float64{0, 20}, [2]float64{-3, 27}, [2]float64{})
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
	require.ErrorContains(t, err, "the blend's end cap folds")
}
