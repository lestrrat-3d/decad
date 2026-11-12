package planar_test

import (
	"errors"
	"testing"

	"github.com/lestrrat-3d/decad/internal/pair/planar"
	"github.com/stretchr/testify/require"
)

// Legs of PlanarDeepVertex shown to fail (each removed in turn, fixture red,
// then restored):
//   - the margin: the plug's lower corners sit at depth exactly 2, and a
//     margin of 2 accepts them;
//   - the parity cast: without it, a vertex far from every facet of the
//     other solid reads as deep, whether outside it (the block's corners
//     against the plug at margin 2) or in a cavity (the floating box).

func deepVertex(t *testing.T, a, b planar.PlanarSolid, margin float64) bool {
	t.Helper()
	deep, err := planar.PlanarDeepVertex(&a, &b, dy(margin), noPoll)
	require.NoError(t, err)
	return deep
}

func TestPlanarDeepVertexMargin(t *testing.T) {
	block := boxSolid([3]float64{0, 0, 0}, [3]float64{10, 10, 10})
	// The plug's lower corners are 2 below the block's top face and at least
	// 4 from every other face, so their depth is exactly 2.
	plug := boxSolid([3]float64{4, 4, 8}, [3]float64{6, 6, 12})
	for _, order := range [][2]planar.PlanarSolid{{plug, block}, {block, plug}} {
		require.True(t, deepVertex(t, order[0], order[1], 0))
		require.True(t, deepVertex(t, order[0], order[1], 1.75))
		require.False(t, deepVertex(t, order[0], order[1], 2))
		require.False(t, deepVertex(t, order[0], order[1], 3))
	}

	// A plug resting on the top face has every vertex on the boundary or
	// outside, so no margin proves an overlap.
	resting := boxSolid([3]float64{4, 4, 10}, [3]float64{6, 6, 12})
	require.False(t, deepVertex(t, resting, block, 0))
	// A box in the hollow's cavity is far from every facet, yet in no
	// material.
	hollow := hollowSolid([3]float64{-20, -20, -20}, [3]float64{20, 20, 20},
		[3]float64{-15, -15, -15}, [3]float64{15, 15, 15})
	floating := boxSolid([3]float64{-1, -1, -1}, [3]float64{1, 1, 1})
	require.False(t, deepVertex(t, floating, hollow, 0.5))
	// Two crossing bars overlap with no vertex of either inside the other:
	// the witness is sufficient, not complete.
	barA := boxSolid([3]float64{-4, -1, -1}, [3]float64{4, 1, 1})
	barB := boxSolid([3]float64{-0.5, -4, -2}, [3]float64{0.5, 4, 2})
	require.False(t, deepVertex(t, barA, barB, 0))
}

func TestPlanarDeepVertexNegativeMarginKeepsSquaredBehavior(t *testing.T) {
	block := boxSolid([3]float64{0, 0, 0}, [3]float64{10, 10, 10})
	plug := boxSolid([3]float64{2, 2, 2}, [3]float64{8, 8, 8})
	for _, margin := range []float64{0, 1, 2, 3, 4} {
		for _, order := range [][2]planar.PlanarSolid{{plug, block}, {block, plug}} {
			positive := deepVertex(t, order[0], order[1], margin)
			negative := deepVertex(t, order[0], order[1], -margin)
			require.Equal(t, positive, negative, "margin ±%g", margin)
		}
	}
}

func TestPlanarDeepVertexPollsAndStops(t *testing.T) {
	block := boxSolid([3]float64{0, 0, 0}, [3]float64{10, 10, 10})
	far := boxSolid([3]float64{20, 20, 20}, [3]float64{21, 21, 21})
	stop := errors.New("stop")
	calls := 0
	_, err := planar.PlanarDeepVertex(&far, &block, dy(0), func() error {
		calls++
		if calls > 5 {
			return stop
		}
		return nil
	})
	require.ErrorIs(t, err, stop)
}

// Legs of PlanarShallowOverlap shown to fail (each removed in turn, fixture
// red, then restored):
//   - the margin: translating by a fixed length of 1 instead, the 0.25 deep
//     plug clears at a margin of 0.125;
//   - the trial relation: accepting an overlapping trial, the plug clears at
//     a margin of 0.125;
//   - the reversed normals of a's facets: read as they are, the plug no longer
//     backs out of its own facets.

func shallowOverlap(t *testing.T, a, b planar.PlanarSolid, margin float64) bool {
	t.Helper()
	result, err := planar.ClassifyPlanar(&a, &b, noPoll)
	require.NoError(t, err)
	shallow, err := planar.PlanarShallowOverlap(&a, &b, result, dy(margin), noPoll)
	require.NoError(t, err)
	return shallow
}

func TestPlanarShallowOverlap(t *testing.T) {
	block := boxSolid([3]float64{0, 0, 0}, [3]float64{10, 10, 10})
	// The plug sinks 0.25 into the block's top face. Backing out along +Z
	// (the plug moved) or -Z (the block moved) clears it.
	plug := boxSolid([3]float64{4, 4, 9.75}, [3]float64{6, 6, 12})
	for _, order := range [][2]planar.PlanarSolid{{plug, block}, {block, plug}} {
		require.True(t, shallowOverlap(t, order[0], order[1], 0.5))
		require.False(t, shallowOverlap(t, order[0], order[1], 0.125))
		require.False(t, shallowOverlap(t, order[0], order[1], 0))
	}
	// A plug in a pocket of the hollow's cavity wall: the cavity is the pocket,
	// and the plug overlaps only its floor, so backing out of the floor clears it.
	hollow := hollowSolid([3]float64{-20, -20, -20}, [3]float64{20, 20, 20},
		[3]float64{-15, -15, -15}, [3]float64{15, 15, 15})
	seated := boxSolid([3]float64{-1, -1, -15.25}, [3]float64{1, 1, -10})
	require.True(t, shallowOverlap(t, seated, hollow, 0.5))
	require.True(t, shallowOverlap(t, hollow, seated, 0.5))

	// A plate pinched 0.25 deep between two walls: no single translation
	// clears both sides, however shallow each is.
	walls := boxSolid([3]float64{-5, -5, -5}, [3]float64{-1, 5, 5})
	right := boxSolid([3]float64{1, -5, -5}, [3]float64{5, 5, 5})
	walls.Verts = append(walls.Verts, right.Verts...)
	walls.Tris = append(walls.Tris, boxTriangles(8)...)
	plate := boxSolid([3]float64{-1.25, -2, -2}, [3]float64{1.25, 2, 2})
	require.False(t, shallowOverlap(t, plate, walls, 0.5))
	// Crossing bars overlap with no deep vertex, yet every way out is longer
	// than the margin.
	barA := boxSolid([3]float64{-4, -1, -1}, [3]float64{4, 1, 1})
	barB := boxSolid([3]float64{-0.5, -4, -2}, [3]float64{0.5, 4, 2})
	require.False(t, deepVertex(t, barA, barB, 0))
	require.False(t, shallowOverlap(t, barA, barB, 1))
	// Only an overlap is read.
	resting := boxSolid([3]float64{4, 4, 10}, [3]float64{6, 6, 12})
	require.False(t, shallowOverlap(t, resting, block, 0.5))
}

func TestPlanarShallowOverlapPollsAndStops(t *testing.T) {
	block := boxSolid([3]float64{0, 0, 0}, [3]float64{10, 10, 10})
	plug := boxSolid([3]float64{4, 4, 9.75}, [3]float64{6, 6, 12})
	result, err := planar.ClassifyPlanar(&plug, &block, noPoll)
	require.NoError(t, err)
	stop := errors.New("stop")
	_, err = planar.PlanarShallowOverlap(&plug, &block, result, dy(0.5), func() error { return stop })
	require.ErrorIs(t, err, stop)
}
