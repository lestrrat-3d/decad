package facetedrecord

import (
	"testing"

	"github.com/stretchr/testify/require"
)

func TestPlacedHeldFloorGrowsByTheLargestVertexGrowth(t *testing.T) {
	fp := Payload{HeldFloor: 0.5, VertexBound: []float64{0.1, 0.4, 0.2}}

	got := placedHeldFloor(fp, []float64{0.1, 0.4 + 1e-12, 0.2 + 3e-12})
	require.GreaterOrEqual(t, got, 0.5+3e-12, "the floor covers the largest growth")
	require.Less(t, got, 0.5+1e-11, "and only that growth")

	// A body with no inherited floor, or a vertex count that changed, keeps none.
	require.Zero(t, placedHeldFloor(Payload{VertexBound: []float64{0.1}}, []float64{0.1}))
	require.Zero(t, placedHeldFloor(fp, []float64{0.1}))
}
