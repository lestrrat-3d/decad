package brepgeom_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/brepgeom"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/stretchr/testify/require"
)

// TestPrismWallSetMapsCarrier is the carrier half of P2(a)
// (agent-docs/brep-modify-design.md §4.4): a swept arc about (3, 4) in a wall
// frame turned a quarter turn from the section frame (u along reference y, v
// along −x) records its operand circle about (−4, 3) in the section frame,
// with its radius unchanged, and a wall with no carrier keeps none.
// Shown-to-fail: without the centre map in PrismWallSet.Add the carrier keeps
// the wall frame's (3, 4).
func TestPrismWallSetMapsCarrier(t *testing.T) {
	t.Parallel()
	wall := brepgeom.Embed{Axis: [3]int{1, 0, 2}, Sign: [3]float64{1, -1, 1}}
	section := brepgeom.Embed{Axis: [3]int{0, 1, 2}, Sign: [3]float64{1, 1, 1}}
	arc := sectionrecord.ArcSeg{Center: sectionrecord.Point2{U: 3, V: 4},
		Start: sectionrecord.Point2{U: 8, V: 4}, End: sectionrecord.Point2{U: 3, V: 9}, TStart: 0, TEnd: 1}
	face := brepgeom.PrismRectFace{Wall: arc, Z1: 10,
		Carrier: brepgeom.CircleCarrier{Center: sectionrecord.Point2{U: 3, V: 4}, Radius: 5}}
	var walls brepgeom.PrismWallSet
	work := freeform.NewFreeformWork()
	require.True(t, walls.Add(face, wall, section, 2, 0, 10, work))
	face.Carrier = brepgeom.CircleCarrier{}
	require.True(t, walls.Add(face, wall, section, 2, 0, 10, work))
	require.Equal(t, []brepgeom.CircleCarrier{
		{Center: sectionrecord.Point2{U: -4, V: 3}, Radius: 5},
		{},
	}, walls.Carriers)
	require.Equal(t, -4.0, walls.Walls[0].CU)
	require.Equal(t, 3.0, walls.Walls[0].CV)
}
