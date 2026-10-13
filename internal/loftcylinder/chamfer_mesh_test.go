package loftcylinder_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/loftcylinder"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestChamferCapTriangles(t *testing.T) {
	verts := []r3.Vec{{}, {X: 4}, {X: 4, Y: 4}, {Y: 4}}
	outer := []int{0, 1, 2, 3}
	area := func(tris [][3]int) float64 {
		total := 0.0
		for _, tri := range tris {
			a, b, c := verts[tri[0]], verts[tri[1]], verts[tri[2]]
			total += ((b.X-a.X)*(c.Y-a.Y) - (c.X-a.X)*(b.Y-a.Y)) / 2
		}
		return total
	}

	top, err := loftcylinder.ChamferCapTriangles(t.Context(), verts, outer, nil, false)
	require.NoError(t, err)
	require.Len(t, top, 2)
	require.Equal(t, 16.0, area(top))

	bottom, err := loftcylinder.ChamferCapTriangles(t.Context(), verts, outer, nil, true)
	require.NoError(t, err)
	require.Len(t, bottom, len(top))
	for i, tri := range top {
		require.Equal(t, [3]int{tri[0], tri[2], tri[1]}, bottom[i])
	}

	_, err = loftcylinder.ChamferCapTriangles(t.Context(), verts, []int{0, 1, 9}, nil, false)
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
	require.ErrorContains(t, err, "cap band has an invalid outer vertex")
}

func TestChamferBandTriangles(t *testing.T) {
	tris, err := loftcylinder.ChamferBandTriangles([]int{10, 11, 12, 13}, []int{20, 21, 22},
		[]int{0, 2}, []int{1, -1}, 3)
	require.NoError(t, err)
	require.Equal(t, [][3]int{
		{11, 12, 22}, {11, 22, 20}, {10, 11, 20},
		{10, 20, 22}, {10, 22, 21}, {13, 10, 21},
		{13, 21, 22}, {12, 13, 22},
	}, tris)

	_, err = loftcylinder.ChamferBandTriangles([]int{10, 11, 12}, []int{20, 21, 22},
		[]int{0, 2}, []int{1, -1}, 3)
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
	require.ErrorContains(t, err, "loft/cylinder band has unmatched source pieces")
}

func TestChamferCompact(t *testing.T) {
	verts := []r3.Vec{{X: 1}, {X: 2}, {X: 3}, {X: 4}}
	tris := [][3]int{{0, 2, 3}}
	kept, bounds, index := loftcylinder.ChamferCompact(verts, tris, []float64{0.1, 0.2, 0.3, 0.4})
	require.Equal(t, []r3.Vec{{X: 1}, {X: 3}, {X: 4}}, kept)
	require.Equal(t, []float64{0.1, 0.3, 0.4}, bounds)
	require.Equal(t, [][3]int{{0, 1, 2}}, tris)
	require.Equal(t, []int{0, 0, 1, 2}, index)
}
