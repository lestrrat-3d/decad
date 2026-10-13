package shellseparation

import (
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestSlabProjectionDistinguishesEmptyClipFromInvalidBound(t *testing.T) {
	shell := Shell{
		Verts:       []r3.Vec{{}, {X: 1}, {Y: 1}},
		Tris:        [][3]int{{0, 1, 2}},
		VertexBound: []float64{0, 0, 0},
	}
	lo, hi, valid := slabProjection(shell, []int{0}, 2, 3, ratOne(), ratZero(), 1, 0)
	require.True(t, valid)
	require.Nil(t, lo)
	require.Nil(t, hi)

	shell.VertexBound = nil
	_, _, valid = slabProjection(shell, []int{0}, 2, 3, ratOne(), ratZero(), 1, 0)
	require.False(t, valid)
}

func TestDisjointLongitudinalSlabsHaveNoTransferGap(t *testing.T) {
	triangle := [][3]int{{0, 1, 2}}
	first := Shell{
		Verts: []r3.Vec{{}, {X: 1}, {Y: 1}},
		Tris:  triangle, VertexBound: make([]float64, 3),
	}
	second := Shell{
		Verts: []r3.Vec{{X: 10}, {X: 11}, {X: 10, Y: 1}},
		Tris:  triangle, VertexBound: make([]float64, 3),
	}
	gap, valid := slabsClearance(t.Context(), first, second, 0)
	require.True(t, valid)
	require.Zero(t, gap)
}
