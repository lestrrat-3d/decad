package shellseparation

import (
	"math"
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

// TestSlabsClearanceReadsTheGapOnEachSide separates two shells in Y over the
// same X slabs. The proven clearance is the gap along the direction between
// the shells' centres, so it must match the float gap between their
// projections, whichever shell comes first, and a pair that overlaps in
// every direction must be refused.
func TestSlabsClearanceReadsTheGapOnEachSide(t *testing.T) {
	triangle := [][3]int{{0, 1, 2}}
	shell := func(dy, dz float64) Shell {
		return Shell{
			Verts: []r3.Vec{{Y: dy, Z: dz}, {X: 1, Y: dy + 1, Z: dz}, {X: 1, Y: dy, Z: dz + 1}},
			Tris:  triangle, VertexBound: make([]float64, 3),
		}
	}
	near, far := shell(0, 0), shell(10, 0)
	for _, order := range [][2]Shell{{near, far}, {far, near}} {
		clearance, valid := slabsClearance(t.Context(), order[0], order[1], 0)
		require.True(t, valid)
		require.InDelta(t, 9.0, clearance, 1e-6, "the gap along Y between the two triangles")
	}

	// A pair offset in both Y and Z still has a positive clearance, at most
	// the Euclidean gap between the two triangles' nearest points.
	aside := shell(6, 8)
	for _, order := range [][2]Shell{{near, aside}, {aside, near}} {
		clearance, valid := slabsClearance(t.Context(), order[0], order[1], 0)
		require.True(t, valid)
		require.Greater(t, clearance, 9.0)
		require.Less(t, clearance, math.Hypot(6, 8))
	}

	// Two triangles whose slices cross each other over the later X slabs:
	// no direction separates them, so the pair is refused.
	crossA := Shell{
		Verts: []r3.Vec{{Y: -1, Z: -1}, {X: 1, Y: 1, Z: -1}, {X: 1, Y: -1, Z: 1}},
		Tris:  triangle, VertexBound: make([]float64, 3),
	}
	crossB := Shell{
		Verts: []r3.Vec{{Y: -1, Z: 1}, {X: 1, Y: 1, Z: 1}, {X: 1, Y: -1, Z: -1}},
		Tris:  triangle, VertexBound: make([]float64, 3),
	}
	_, valid := slabsClearance(t.Context(), crossA, crossB, 0)
	require.False(t, valid, "crossing slices have no separating direction")
	_, valid = slabsClearance(t.Context(), crossB, crossA, 0)
	require.False(t, valid)

	// The centre-to-centre direction runs nearly along Y, where the long
	// slice and the short one overlap, so only a hull axis (close to Z)
	// separates the pair. That axis can order the shells either way round.
	long := Shell{
		Verts: []r3.Vec{{Y: -10}, {X: 1, Y: 10}, {X: 1, Y: -10}},
		Tris:  triangle, VertexBound: make([]float64, 3),
	}
	short := Shell{
		Verts: []r3.Vec{{Y: 5, Z: 1}, {X: 1, Y: 5.2, Z: 1}, {X: 1, Y: 4.8, Z: 1}},
		Tris:  triangle, VertexBound: make([]float64, 3),
	}
	for _, order := range [][2]Shell{{long, short}, {short, long}} {
		clearance, valid := slabsClearance(t.Context(), order[0], order[1], 0)
		require.True(t, valid)
		require.Greater(t, clearance, 0.01)
		require.LessOrEqual(t, clearance, 1.0+1e-9)
	}
}
