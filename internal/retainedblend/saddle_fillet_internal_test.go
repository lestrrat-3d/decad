package retainedblend

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/crossbore"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/stretchr/testify/require"
)

func TestChainArcs(t *testing.T) {
	t.Parallel()
	// Three facets' arcs around a window, clockwise: 1 → (corner 10) → 2,
	// 2 → 3, 3 → 1, and a held sliver whose merged entry and exit make no arc.
	arcs := map[int][]int{1: {1, 10, 2}, 2: {2, 3}, 3: {3, 1}}
	ring, err := chainArcs(arcs, nil)
	require.NoError(t, err)
	require.Equal(t, []int{3, 2, 10, 1}, ring, "the ring runs counter-clockwise")

	_, err = chainArcs(map[int][]int{1: {1, 2}, 2: {2, 1}, 5: {5, 6}, 6: {6, 5}}, nil)
	require.ErrorIs(t, err, decaderr.ErrUnsupported, "two loops")
	_, err = chainArcs(map[int][]int{1: {1, 2}, 2: {2, 3}}, nil)
	require.ErrorIs(t, err, decaderr.ErrUnsupported, "an open chain")

	ring, err = chainArcs(map[int][]int{}, []int{7, 6, 5})
	require.NoError(t, err)
	require.Equal(t, []int{5, 6, 7}, ring, "a window inside one facet is its corners")
}

func TestPoseHand(t *testing.T) {
	t.Parallel()
	require.InDelta(t, 1.0, poseHand(crossbore.Pose{Axis: [3]int{0, 1, 2}, Sign: [3]float64{1, 1, 1}}), 0)
	require.InDelta(t, 1.0, poseHand(crossbore.Pose{Axis: [3]int{0, 1, 2}, Sign: [3]float64{-1, -1, 1}}), 0)
	require.InDelta(t, 1.0, poseHand(crossbore.Pose{Axis: [3]int{2, 0, 1}, Sign: [3]float64{1, 1, 1}}), 0)
	// The Revolve shaft on x with a bore on z: local X is world z.
	require.InDelta(t, -1.0, poseHand(crossbore.Pose{Axis: [3]int{2, 1, 0}, Sign: [3]float64{1, 1, 1}}), 0)
	require.InDelta(t, -1.0, poseHand(crossbore.Pose{Axis: [3]int{0, 1, 2}, Sign: [3]float64{-1, 1, 1}}), 0)
}

func TestMonotoneRing(t *testing.T) {
	t.Parallel()
	ring, err := monotoneRing([]ringPt{{idx: 1, ang: 4}, {idx: 2, ang: 5}, {idx: 3, ang: 1}, {idx: 4, ang: 2.5}}, "ring")
	require.NoError(t, err)
	require.Equal(t, 3, ring[0].idx, "the ring starts at its least angle")

	_, err = monotoneRing([]ringPt{{idx: 1, ang: 1}, {idx: 2, ang: 3}, {idx: 3, ang: 2}, {idx: 4, ang: 5}}, "ring")
	require.ErrorIs(t, err, decaderr.ErrUnsupported, "a ring that turns back")
	_, err = monotoneRing([]ringPt{{idx: 1, ang: 0}, {idx: 2, ang: 0.5}, {idx: 3, ang: 1 + math.Pi}}, "ring")
	require.ErrorIs(t, err, decaderr.ErrUnsupported, "a gap of half a turn")
}
