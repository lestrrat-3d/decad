package retainedblend

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// TestColumnAtCorner checks the exact identity that lets a cross-plane chain's end column take a held corner of its
// end plane (agent-docs/modify-design.md Part M §15b.1): a step's wall edge between the side y = 10 and the
// wall x = 30 ends on the 45° bevel of the top rim, whose far corner on the wall is where the bevel, the wall and the
// top z = 20 meet.
func TestColumnAtCorner(t *testing.T) {
	t.Parallel()
	a := math.Sqrt(0.5)
	side := plane{n: r3.NewVec(0, 1, 0), o: r3.NewVec(0, 10, 0)}
	wall := plane{n: r3.NewVec(-1, 0, 0), o: r3.NewVec(30, 0, 0)}
	top := plane{n: r3.NewVec(0, 0, 1), o: r3.NewVec(0, 0, 20)}
	bevel := func(d float64) plane {
		return plane{n: r3.NewVec(0, a, a), o: r3.NewVec(40, 10-d, 20)}
	}
	edge := [2]plane{side, wall}

	t.Run("the same setback meets the old bevel's corner on the wall", func(t *testing.T) {
		t.Parallel()
		corner, ok := columnAtCorner(edge, bevel(0.5), top, 1, 0.5)
		require.True(t, ok)
		require.Equal(t, r3.NewVec(30, 9.5, 20), corner)
	})
	t.Run("a wider old bevel leaves its corner beyond the column", func(t *testing.T) {
		t.Parallel()
		_, ok := columnAtCorner(edge, bevel(0.8), top, 1, 0.5)
		require.False(t, ok)
	})
	t.Run("a setback one float step off is a near miss, which proves nothing", func(t *testing.T) {
		t.Parallel()
		_, ok := columnAtCorner(edge, bevel(0.5), top, 1, math.Nextafter(0.5, 1))
		require.False(t, ok)
	})
	t.Run("the column on the side face meets no corner of the top", func(t *testing.T) {
		t.Parallel()
		_, ok := columnAtCorner(edge, bevel(0.5), top, 0, 0.5)
		require.False(t, ok)
	})
	t.Run("planes with dependent normals meet in no point", func(t *testing.T) {
		t.Parallel()
		_, ok := columnAtCorner(edge, top, top, 1, 0.5)
		require.False(t, ok)
	})
}

// TestLeavesOutward checks the direction test for a kept chord that leaves a corner of a removed region.
func TestLeavesOutward(t *testing.T) {
	t.Parallel()
	// The region a step's wall loses near its end: the edge from (12, 10) up to (19.5, 10), the old bevel's edge to
	// the held corner (20, 9.5), and the setback line back down, in (z, y) coordinates.
	region := []pt2{{12, 10}, {19.5, 10}, {20, 9.5}, {12, 9.5}}
	corner := 2
	require.NoError(t, leavesOutward(region, corner, pt2{20, -9.5}), "along the top face, away from the region")
	require.Error(t, leavesOutward(region, corner, pt2{15, 9.8}), "into the region")
	require.Error(t, leavesOutward(region, corner, pt2{12, 9.5}), "along the region's own edge")
	// The same region wound the other way reads the same.
	reversed := []pt2{{12, 9.5}, {20, 9.5}, {19.5, 10}, {12, 10}}
	require.NoError(t, leavesOutward(reversed, 1, pt2{20, -9.5}))
	require.Error(t, leavesOutward(reversed, 1, pt2{15, 9.8}))
}

func TestInPolygon(t *testing.T) {
	t.Parallel()
	square := []pt2{{0, 0}, {1, 0}, {1, 1}, {0, 1}}
	require.True(t, inPolygon(square, pt2{0.5, 0.5}))
	require.False(t, inPolygon(square, pt2{1.5, 0.5}))
	require.False(t, inPolygon(square, pt2{0.5, -0.1}))
}
