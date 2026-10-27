package orthoshell_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/orthoshell"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// box returns the vertices and outward triangles of the box from lo to hi, and for each triangle
// which face it belongs to: 0 bottom, 1 top, 2 front, 3 back, 4 left, 5 right.
func box(lo, hi r3.Vec) ([]r3.Vec, [][3]int, []int) {
	v := []r3.Vec{
		{X: lo.X, Y: lo.Y, Z: lo.Z}, {X: hi.X, Y: lo.Y, Z: lo.Z}, {X: hi.X, Y: hi.Y, Z: lo.Z}, {X: lo.X, Y: hi.Y, Z: lo.Z},
		{X: lo.X, Y: lo.Y, Z: hi.Z}, {X: hi.X, Y: lo.Y, Z: hi.Z}, {X: hi.X, Y: hi.Y, Z: hi.Z}, {X: lo.X, Y: hi.Y, Z: hi.Z},
	}
	quads := [][4]int{{0, 3, 2, 1}, {4, 5, 6, 7}, {0, 1, 5, 4}, {3, 7, 6, 2}, {0, 4, 7, 3}, {1, 2, 6, 5}}
	var t [][3]int
	var face []int
	for f, q := range quads {
		t = append(t, [3]int{q[0], q[1], q[2]}, [3]int{q[0], q[2], q[3]})
		face = append(face, f, f)
	}
	return v, t, face
}

// volume is the signed volume of a closed outward mesh.
func volume(v []r3.Vec, tris [][3]int) float64 {
	total := 0.0
	for _, t := range tris {
		total += v[t[0]].Dot(v[t[1]].Cross(v[t[2]])) / 6
	}
	return total
}

func bounds(v []r3.Vec) (r3.Vec, r3.Vec) {
	lo, hi := v[0], v[0]
	for _, p := range v[1:] {
		lo = r3.Vec{X: math.Min(lo.X, p.X), Y: math.Min(lo.Y, p.Y), Z: math.Min(lo.Z, p.Z)}
		hi = r3.Vec{X: math.Max(hi.X, p.X), Y: math.Max(hi.Y, p.Y), Z: math.Max(hi.Z, p.Z)}
	}
	return lo, hi
}

func TestWallsOfAClosedBoxAreTheBoxLessItsCavity(t *testing.T) {
	t.Parallel()
	v, tris, _ := box(r3.NewVec(0, 0, 0), r3.NewVec(10, 8, 6))
	wv, wt, err := orthoshell.Walls(v, tris, make([]bool, len(tris)), 1, false)
	require.NoError(t, err)
	lo, hi := bounds(wv)
	require.True(t, lo.Equal(r3.NewVec(0, 0, 0), 1e-9))
	require.True(t, hi.Equal(r3.NewVec(10, 8, 6), 1e-9))
	// 480 less the cavity 8 x 6 x 4 = 192, with the inner skin wound into the void: the signed volume is the walls'.
	require.InDelta(t, 480-8*6*4, volume(wv, wt), 1e-9)
}

func TestWallsOfABoxWithATopRemovedAreOpen(t *testing.T) {
	t.Parallel()
	v, tris, face := box(r3.NewVec(0, 0, 0), r3.NewVec(10, 8, 6))
	removed := make([]bool, len(tris))
	for i, f := range face {
		removed[i] = f == 1
	}
	wv, wt, err := orthoshell.Walls(v, tris, removed, 1, false)
	require.NoError(t, err)
	// The cavity runs from the floor 1 up out of the top, 8 x 6 x 5 of the box: 480 - 240.
	require.InDelta(t, 480-8*6*5, volume(wv, wt), 1e-9)
	lo, hi := bounds(wv)
	require.True(t, lo.Equal(r3.NewVec(0, 0, 0), 1e-9))
	require.True(t, hi.Equal(r3.NewVec(10, 8, 6), 1e-9), "the walls stay inside the solid")
}

// lShape is an L-shaped prism 2 thick: the rectangles [0,4]x[0,2] and [0,2]x[2,4], twelve in area, whose
// faces on the reflex corner at (2, 2) are two rectangles of the walls, and whose top and bottom hold a
// T-junction where the arms meet, as a mesh read from a file may.
func lShape() ([]r3.Vec, [][3]int) {
	var v []r3.Vec
	idx := map[r3.Vec]int{}
	id := func(p r3.Vec) int {
		if i, ok := idx[p]; ok {
			return i
		}
		idx[p] = len(v)
		v = append(v, p)
		return len(v) - 1
	}
	var tris [][3]int
	quad := func(a, b, c, d r3.Vec) {
		tris = append(tris, [3]int{id(a), id(b), id(c)}, [3]int{id(a), id(c), id(d)})
	}
	p := func(x, y, z float64) r3.Vec { return r3.NewVec(x, y, z) }
	// Bottom z = 0 and top z = 2, each as the two arm rectangles.
	quad(p(0, 0, 0), p(0, 2, 0), p(4, 2, 0), p(4, 0, 0))
	quad(p(0, 2, 0), p(0, 4, 0), p(2, 4, 0), p(2, 2, 0))
	quad(p(0, 0, 2), p(4, 0, 2), p(4, 2, 2), p(0, 2, 2))
	quad(p(0, 2, 2), p(2, 2, 2), p(2, 4, 2), p(0, 4, 2))
	// Sides, outward.
	quad(p(0, 0, 0), p(4, 0, 0), p(4, 0, 2), p(0, 0, 2)) // y = 0
	quad(p(4, 0, 0), p(4, 2, 0), p(4, 2, 2), p(4, 0, 2)) // x = 4
	quad(p(4, 2, 0), p(2, 2, 0), p(2, 2, 2), p(4, 2, 2)) // y = 2, the arm's top
	quad(p(2, 2, 0), p(2, 4, 0), p(2, 4, 2), p(2, 2, 2)) // x = 2
	quad(p(2, 4, 0), p(0, 4, 0), p(0, 4, 2), p(2, 4, 2)) // y = 4
	quad(p(0, 4, 0), p(0, 0, 0), p(0, 0, 2), p(0, 4, 2)) // x = 0
	return v, tris
}

func TestWallsOfAnLShapeKeepTheMiteredCorner(t *testing.T) {
	t.Parallel()
	v, tris := lShape()
	require.InDelta(t, 24, volume(v, tris), 1e-9, "the fixture is a closed L of area 12 and height 2")
	wv, wt, err := orthoshell.Walls(v, tris, make([]bool, len(tris)), 0.5, false)
	require.NoError(t, err)
	// The cavity is the L eroded by 0.5: the horizontal arm [0.5,3.5] x [0.5,1.5] and the vertical arm
	// [0.5,1.5] x [1.5,3.5], a sharp corner at (1.5, 1.5), one mm in z: 3 + 2 = 5.
	require.InDelta(t, 24-5, volume(wv, wt), 1e-9)
}

func TestWallsRefusals(t *testing.T) {
	t.Parallel()
	v, tris, _ := box(r3.NewVec(0, 0, 0), r3.NewVec(10, 8, 6))
	none := make([]bool, len(tris))
	// Walls that meet leave no cavity: a box 6 thick in z with t = 3 (cube erosion by 6 swallows it).
	_, _, err := orthoshell.Walls(v, tris, none, 3, false)
	require.ErrorIs(t, err, orthoshell.ErrDegenerate)
	_, _, err = orthoshell.Walls(v, tris, none, 0, false)
	require.ErrorIs(t, err, orthoshell.ErrDegenerate)
	_, _, err = orthoshell.Walls(v, tris, none[:3], 1, false)
	require.ErrorIs(t, err, orthoshell.ErrUnsupported)
	// A slanted triangle is not square to an axis.
	slant := append([]r3.Vec{}, v...)
	slant[6] = r3.NewVec(10, 8, 5)
	_, _, err = orthoshell.Walls(slant, tris, none, 1, false)
	require.ErrorIs(t, err, orthoshell.ErrNotOrthogonal)
}

func TestOutwardWallsGrowOffTheSolid(t *testing.T) {
	t.Parallel()
	v, tris, face := box(r3.NewVec(0, 0, 0), r3.NewVec(10, 8, 6))
	none := make([]bool, len(tris))
	wv, wt, err := orthoshell.Walls(v, tris, none, 1, true)
	require.NoError(t, err)
	lo, hi := bounds(wv)
	require.True(t, lo.Equal(r3.NewVec(-1, -1, -1), 1e-9))
	require.True(t, hi.Equal(r3.NewVec(11, 9, 7), 1e-9))
	// The walls are the 12 x 10 x 8 box dilated by the cube, less the box: 960 - 480, with the box as the cavity.
	require.InDelta(t, 960-480, volume(wv, wt), 1e-9)
	removed := make([]bool, len(tris))
	for i, f := range face {
		removed[i] = f == 1
	}
	ov, ot, err := orthoshell.Walls(v, tris, removed, 1, true)
	require.NoError(t, err)
	// Open at the top: the wall ends flush with the top plane, so nothing of the 12 x 10 x 8 dilation stands above
	// z = 6 and the box's lid, 10 x 8 x 1 over its top, is left out too: 960 - 480 - (12 x 10 x 1) = 360.
	require.InDelta(t, 960-480-12*10*1, volume(ov, ot), 1e-9)
	olo, ohi := bounds(ov)
	require.True(t, olo.Equal(r3.NewVec(-1, -1, -1), 1e-9))
	require.True(t, ohi.Equal(r3.NewVec(11, 9, 6), 1e-9))
}

func TestOutwardWallsOfAnLShapeKeepTheMiteredCorner(t *testing.T) {
	t.Parallel()
	v, tris := lShape()
	wv, wt, err := orthoshell.Walls(v, tris, make([]bool, len(tris)), 0.5, true)
	require.NoError(t, err)
	// The L dilated by 0.5: [-0.5,4.5] x [-0.5,2.5] and [-0.5,2.5] x [2.5,4.5], area 15 + 6 = 21, 3 high: 63, less the L's 24.
	require.InDelta(t, 63-24, volume(wv, wt), 1e-9)
}
