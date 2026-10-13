package brepgeom_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/brepgeom"
	"github.com/stretchr/testify/require"
)

// An inward planar face walks its recorded loop against the oriented solid
// boundary. Reversing only its outward normal would reverse this turn.
func TestMaterialWedgeConvexInwardPlanarFace(t *testing.T) {
	t.Parallel()
	embeds := []brepgeom.Embed{
		{Axis: [3]int{0, 1, 2}, Sign: [3]float64{1, 1, 1}},
		{Axis: [3]int{0, 2, 1}, Sign: [3]float64{1, 1, -1}},
	}
	uses := []brepgeom.Use{
		{Face: 0, Part: brepgeom.LoopSeg, From: [3]float64{0, 0, 0},
			To: [3]float64{1, 0, 0}, Outward: true},
		{Face: 1, Part: brepgeom.LoopSeg, Outward: true},
	}
	for _, outward := range []bool{true, false} {
		uses[0].Outward = outward
		convex, decided := brepgeom.MaterialWedgeConvex([2]int{0, 1}, uses, embeds, nil)
		require.True(t, decided)
		require.True(t, convex)
	}
}
