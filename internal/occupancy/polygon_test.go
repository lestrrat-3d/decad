package occupancy_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/occupancy"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

func TestPolygonLeaf(t *testing.T) {
	t.Parallel()
	pl := newPlaced(r3.Vec{X: 1, Y: -1, Z: 2}, 0.9, r3.Vec{X: 20.5, Y: -3.25, Z: 40})
	// Clockwise on purpose: the leaf fixes the winding.
	rect := [][2]float64{{0, 0}, {0, 10}, {40, 10}, {40, 0}}
	leaf, err := occupancy.PolygonLeaf(pl.chain, rect, 0, 5, 0, 0, 0, 0, 0.001)
	require.NoError(t, err)
	pieces, err := leaf.Pieces()
	require.NoError(t, err)
	require.Len(t, pieces, 1)

	inside := pl.image(r3.Vec{X: 39.999, Y: 9.999, Z: 4.999})
	ok, err := occupancy.Misses([3]r3.Vec{inside, inside, inside}, r3.Vec{}, pieces[0], &occupancy.Budget{Limit: 1 << 40})
	require.NoError(t, err)
	require.False(t, ok, "a point inside the prism is never separated")
	outside := pl.image(r3.Vec{X: 40.01, Y: 5, Z: 2})
	ok, err = occupancy.Misses([3]r3.Vec{outside, outside, outside}, r3.Vec{}, pieces[0], &occupancy.Budget{Limit: 1 << 40})
	require.NoError(t, err)
	require.True(t, ok)

	// A facet 0.02 past the long side pokes out at least that far.
	rec := occupancy.LeafRecord(leaf)
	tri := [3]r3.Vec{pl.image(r3.Vec{X: 5, Y: 10.02, Z: 1}), pl.image(r3.Vec{X: 6, Y: 10.02, Z: 1}), pl.image(r3.Vec{X: 5, Y: 10.02, Z: 2})}
	b := rec.Bulges(tri[:], [][3]int{{0, 1, 2}}, []float64{1})
	require.GreaterOrEqual(t, b[0], 0.02)
	require.Less(t, b[0], 0.021)

	// A non-convex section is refused.
	_, err = occupancy.PolygonLeaf(pl.chain, [][2]float64{{0, 0}, {10, 0}, {5, 2}, {10, 5}, {0, 5}}, 0, 5, 0, 0, 0, 0, 0.001)
	require.Error(t, err)
}
