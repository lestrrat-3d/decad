package loftmesh

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestChamferMiddleHeldRootInsideClipsToSlab(t *testing.T) {
	verts := []r3.Vec{
		{X: 0, Y: 0, Z: 0},
		{X: 0.5, Y: 0, Z: 0},
		{X: 0, Y: 0.5, Z: 1},
		{X: 3, Y: 0, Z: -1},
	}
	tris := [][3]int{{0, 1, 2}, {0, 1, 3}}
	zero, one, minusOne := big.NewRat(0, 1), big.NewRat(1, 1), big.NewRat(-1, 1)

	// The far vertex lies below the slab, so the clip removes it.
	require.True(t, ChamferMiddleHeldRootInside(verts, tris, 2, 0, zero, one, 1))
	// A slab reaching down to the far vertex keeps it, outside the disk.
	require.False(t, ChamferMiddleHeldRootInside(verts, tris, 2, 0, minusOne, one, 1))

	require.False(t, ChamferMiddleHeldRootInside(verts, tris, 2, -1, zero, one, 1))
	require.False(t, ChamferMiddleHeldRootInside(verts, tris, 3, 0, zero, one, 1))
	require.False(t, ChamferMiddleHeldRootInside(verts, tris, 2, 0, one, one, 1))
	require.False(t, ChamferMiddleHeldRootInside(verts, tris, 2, 0, zero, one, 0))
}
