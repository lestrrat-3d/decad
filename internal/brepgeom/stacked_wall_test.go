package brepgeom_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/brepgeom"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/stretchr/testify/require"
)

// TestStackedWallRegions pins docs/shell-opening-design.md §4.3's grouping of
// one wall key's chained loops on the XY embed: counter-clockwise loops are a
// face each, one counter-clockwise loop with a clockwise one is one face whose
// hole is the clockwise loop, and two counter-clockwise loops beside a
// clockwise one, or loops at different levels, are a miss. Shown to fail with
// the per-loop rule restored (every clockwise loop then missed).
func TestStackedWallRegions(t *testing.T) {
	t.Parallel()
	xy := brepgeom.Embed{Axis: [3]int{0, 1, 2}, Sign: [3]float64{1, 1, 1}}
	// square walks the axis-aligned square [u0, u1]×[v0, v1] at level z,
	// counter-clockwise or clockwise.
	square := func(u0, v0, u1, v1, z float64, ccw bool) []brepgeom.StackedWallSegment {
		p := [][3]float64{{u0, v0, z}, {u1, v0, z}, {u1, v1, z}, {u0, v1, z}}
		if !ccw {
			p[1], p[3] = p[3], p[1]
		}
		out := make([]brepgeom.StackedWallSegment, len(p))
		for i := range p {
			out[i] = brepgeom.StackedWallSegment{From: p[i], To: p[(i+1)%len(p)]}
		}
		return out
	}

	regions, level, err := brepgeom.StackedWallRegions(xy, [][]brepgeom.StackedWallSegment{
		square(0, 0, 4, 4, 3, true), square(5, 0, 6, 1, 3, true),
	})
	require.NoError(t, err)
	require.Equal(t, 3.0, level)
	require.Len(t, regions, 2)
	for _, r := range regions {
		require.Empty(t, r.Holes)
		require.Len(t, r.Outer.Segments, 4)
	}

	regions, _, err = brepgeom.StackedWallRegions(xy, [][]brepgeom.StackedWallSegment{
		square(1, 1, 2, 2, 3, false), square(0, 0, 4, 4, 3, true),
	})
	require.NoError(t, err)
	require.Len(t, regions, 1)
	require.Len(t, regions[0].Holes, 1, "the clockwise loop is the hole")
	require.Equal(t, sectionrecord.Point2{U: 0, V: 0}, regions[0].Outer.Segments[0].(sectionrecord.LineSeg).Start)
	require.Equal(t, sectionrecord.Point2{U: 1, V: 1}, regions[0].Holes[0].Segments[0].(sectionrecord.LineSeg).Start)

	for _, loops := range [][][]brepgeom.StackedWallSegment{
		{square(0, 0, 4, 4, 3, true), square(5, 0, 9, 4, 3, true), square(1, 1, 2, 2, 3, false)},
		{square(1, 1, 2, 2, 3, false)},
		{square(0, 0, 4, 4, 3, true), square(5, 0, 6, 1, 2, true)},
	} {
		_, _, err := brepgeom.StackedWallRegions(xy, loops)
		require.ErrorIs(t, err, brepgeom.ErrStackedWallMiss)
	}
}
