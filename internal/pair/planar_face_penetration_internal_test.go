package pair

import (
	"math/big"
	"testing"

	"github.com/stretchr/testify/require"
)

// The region test of §9.6's condition 3, read directly: an overlap whose sunk
// part leaves the crossed face generically also crosses the face's rim, which
// condition 1 refuses first, so these legs are shown on the helper itself.
//
// Legs shown to fail (each deleted in turn, fixture red, then restored):
//   - every point strictly inside: the points inside the hole are accepted;
//   - no hull edge meeting a loop: the L region's two arm points, whose
//     segment cuts across the notch, are accepted;
//   - no loop vertex in the hull: the points around the hole are accepted.

func pt(x, y float64) Point2 { return Point2{X: ratOf(x), Y: ratOf(y)} }

func ratOf(f float64) *big.Rat { return new(big.Rat).SetFloat64(f) }

func noPollInternal() error { return nil }

func TestHullInsideRegion(t *testing.T) {
	square := []Point2{pt(0, 0), pt(8, 0), pt(8, 8), pt(0, 8)}
	inside := func(points []Point2, loops ...[]Point2) bool {
		t.Helper()
		ok, err := hullInsideRegion(points, loops, new(big.Rat), noPollInternal)
		require.NoError(t, err)
		return ok
	}

	require.True(t, inside([]Point2{pt(1, 1), pt(7, 1), pt(4, 6)}, square))
	require.False(t, inside([]Point2{pt(1, 1), pt(8, 1), pt(4, 6)}, square), "a point on the rim")

	// The L region [0,8]² less [4,8]×[4,8]: (6.5,3.5) and (3.5,6.5) lie in
	// its arms, but the segment between them passes through the missing
	// corner, beyond its tip at (4,4).
	l := []Point2{pt(0, 0), pt(8, 0), pt(8, 4), pt(4, 4), pt(4, 8), pt(0, 8)}
	require.True(t, inside([]Point2{pt(1, 1), pt(6, 1), pt(1, 6)}, l))
	require.False(t, inside([]Point2{pt(6.5, 3.5), pt(3.5, 6.5)}, l))

	// A hole [3,5]² wound clockwise: points around it lie in the region, but
	// their hull covers the hole.
	hole := []Point2{pt(3, 3), pt(3, 5), pt(5, 5), pt(5, 3)}
	around := []Point2{pt(1, 1), pt(7, 1), pt(7, 7), pt(1, 7)}
	require.False(t, inside(around, square, hole))
	require.True(t, inside([]Point2{pt(1, 1), pt(2, 1), pt(1, 2)}, square, hole))
	// Points inside the hole: their hull meets no loop and holds no loop
	// vertex, but no point lies on material.
	require.False(t, inside([]Point2{pt(3.5, 3.5), pt(4.5, 3.5), pt(4, 4.5)}, square, hole))
}

// TestHullInsideRegionMargin grows the hull by a margin: a point nearer the
// rim than the margin, a hull edge passing nearer a loop vertex than it, and
// a hull nearer a hole than it each refuse, compared exactly. Each of the two
// distance checks was deleted in turn and its case went red.
func TestHullInsideRegionMargin(t *testing.T) {
	square := []Point2{pt(0, 0), pt(8, 0), pt(8, 8), pt(0, 8)}
	inside := func(margin float64, points []Point2, loops ...[]Point2) bool {
		t.Helper()
		ok, err := hullInsideRegion(points, loops, ratOf(margin), noPollInternal)
		require.NoError(t, err)
		return ok
	}
	near := []Point2{pt(1, 1), pt(7.5, 1), pt(4, 6)}
	require.True(t, inside(.25, near, square))
	require.False(t, inside(.5, near, square), "the corner (7.5, 1) lies exactly 0.5 from the rim")

	// The L region's notch tip (4, 4) lies 0.5/√2 from the segment between
	// (3, 4.5) and (4.5, 3).
	l := []Point2{pt(0, 0), pt(8, 0), pt(8, 4), pt(4, 4), pt(4, 8), pt(0, 8)}
	diagonal := []Point2{pt(1, 1), pt(4.5, 3), pt(3, 4.5)}
	require.True(t, inside(.25, diagonal, l))
	require.False(t, inside(.5, diagonal, l))

	// A hole [3,5]² beside a hull whose nearest corner is (2, 2): √2 away,
	// while the hull stays 1.5 from the square's rim.
	hole := []Point2{pt(3, 3), pt(3, 5), pt(5, 5), pt(5, 3)}
	beside := []Point2{pt(1.5, 1.5), pt(2, 1.5), pt(2, 2)}
	require.True(t, inside(1, beside, square, hole))
	require.False(t, inside(1.45, beside, square, hole))
}

func TestConvexHull2(t *testing.T) {
	hull := convexHull2([]Point2{pt(0, 0), pt(2, 0), pt(1, 1), pt(2, 2), pt(0, 2), pt(1, 0), pt(2, 2)})
	require.Len(t, hull, 4, "the interior point and the edge midpoint are dropped")
	require.True(t, inConvexHull(pt(1, 0), hull))
	require.True(t, inConvexHull(pt(1, 1), hull))
	require.False(t, inConvexHull(pt(3, 1), hull))

	segment := convexHull2([]Point2{pt(0, 0), pt(1, 1), pt(2, 2)})
	require.Len(t, segment, 2)
	require.True(t, inConvexHull(pt(1, 1), segment))
	require.False(t, inConvexHull(pt(1, 0), segment))

	point := convexHull2([]Point2{pt(1, 1), pt(1, 1)})
	require.Len(t, point, 1)
	require.True(t, inConvexHull(pt(1, 1), point))
}
