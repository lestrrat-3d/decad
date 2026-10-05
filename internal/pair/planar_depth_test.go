package pair_test

import (
	"errors"
	"testing"

	"github.com/lestrrat-3d/decad/internal/pair"
	"github.com/stretchr/testify/require"
)

// Legs of PlanarDeepVertex shown to fail (each removed in turn, fixture red,
// then restored):
//   - the margin: the plug's lower corners sit at depth exactly 2, and a
//     margin of 2 accepts them;
//   - the parity cast: without it, a vertex far from every facet of the
//     other solid reads as deep, whether outside it (the block's corners
//     against the plug at margin 2) or in a cavity (the floating box).

func deepVertex(t *testing.T, a, b pair.PlanarSolid, margin float64) bool {
	t.Helper()
	deep, err := pair.PlanarDeepVertex(&a, &b, dy(margin), noPoll)
	require.NoError(t, err)
	return deep
}

func TestPlanarDeepVertexMargin(t *testing.T) {
	block := boxSolid([3]float64{0, 0, 0}, [3]float64{10, 10, 10})
	// The plug's lower corners are 2 below the block's top face and at least
	// 4 from every other face, so their depth is exactly 2.
	plug := boxSolid([3]float64{4, 4, 8}, [3]float64{6, 6, 12})
	for _, order := range [][2]pair.PlanarSolid{{plug, block}, {block, plug}} {
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

func TestPlanarDeepVertexPollsAndStops(t *testing.T) {
	block := boxSolid([3]float64{0, 0, 0}, [3]float64{10, 10, 10})
	far := boxSolid([3]float64{20, 20, 20}, [3]float64{21, 21, 21})
	stop := errors.New("stop")
	calls := 0
	_, err := pair.PlanarDeepVertex(&far, &block, dy(0), func() error {
		calls++
		if calls > 5 {
			return stop
		}
		return nil
	})
	require.ErrorIs(t, err, stop)
}
