package classbgeom

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// TestCrossingOffsetUpperCoversTheCrossing holds the bound to the exact
// distance from a vertex on the trace v = 3 to the crossing of that trace
// with the radius-5 circle about the origin, which is (4, 3): zero at the
// crossing itself, and at least the vertex's distance from 4 off it, for a
// circle and for an arc whose Start states the same radius. A trace that
// misses the circle states no bound.
func TestCrossingOffsetUpperCoversTheCrossing(t *testing.T) {
	t.Parallel()
	circle := sectionrecord.CircleSeg{Radius: units.Millimeters(5), TStart: 0, TEnd: 1, CCW: true}
	arc := sectionrecord.ArcSeg{Start: sectionrecord.Point2{U: 5}, End: sectionrecord.Point2{V: 5}, TStart: 0, TEnd: 1}
	for _, seg := range []sectionrecord.CurveSegment{circle, arc} {
		require.Zero(t, CrossingOffsetUpper(seg, 4, 0, 3, 0))
		require.Zero(t, CrossingOffsetUpper(seg, -4, 0, 3, 0), `the crossing on the other side`)

		off := 4.0
		for range 1000 {
			off = math.Nextafter(off, 5)
		}
		got := CrossingOffsetUpper(seg, off, 0, 3, 0)
		exact, _ := new(big.Rat).Sub(new(big.Rat).SetFloat64(off), big.NewRat(4, 1)).Float64()
		require.GreaterOrEqual(t, got, exact)
		require.Less(t, got, 2*exact)

		require.True(t, math.IsInf(CrossingOffsetUpper(seg, 4, 0, 6, 0), 1), `v = 6 misses the circle`)
	}
}
