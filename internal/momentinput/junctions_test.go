package momentinput

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/stretchr/testify/require"
)

// TestEndsOfWalkCoversArcNaturalEnd holds an arc's natural end to the point
// the arc denotes there. The record pins End at a radius about 500 ulps past
// Start's, so the walk holds End verbatim at zero bound while the arc denotes
// (0, 10) — Start's radius at End's angle — and the end's bound must reach
// it. The t == 0 end is Start itself and stays at zero.
//
// Shown-to-fail: dropping the radial residual from endsOfWalk leaves the
// t == 1 bound at zero, and the coverage leg goes red.
func TestEndsOfWalkCoversArcNaturalEnd(t *testing.T) {
	t.Parallel()
	top := 10.0
	for range 500 {
		top = math.Nextafter(top, 11)
	}
	arc := ArcSeg{Center: Point2{}, Start: Point2{U: 10}, End: Point2{U: 0, V: top}, TStart: 0, TEnd: 1}
	walk, err := boundarywalk.WalkOf(arc, nil)
	require.NoError(t, err)
	ends := endsOfWalk(arc, walk)

	require.Equal(t, Point2{U: 0, V: top}, ends.end, `the walk holds the recorded End`)
	require.GreaterOrEqual(t, ends.endBound.V, top-10, `the bound reaches the denoted (0, 10)`)
	require.Equal(t, Point2{U: 10}, ends.start)
	require.Zero(t, ends.startBound.U)
	require.Zero(t, ends.startBound.V)
}

// TestLineCornerClosesTJunction closes a T-junction at the exact crossing of
// the two supports. The line along v = 0 ends naturally at (10, 0), which is
// not on the slightly tilted line it meets, and that line's fragment starts
// at a cut parameter off the crossing. The corner is the point where the
// tilted support crosses v = 0, stated exactly from the recorded endpoints.
func TestLineCornerClosesTJunction(t *testing.T) {
	t.Parallel()
	top := math.Nextafter(10, 11)
	for range 4000 {
		top = math.Nextafter(top, 11)
	}
	stem := LineSeg{Start: Point2{U: 0, V: 0}, End: Point2{U: 10, V: 0}, TStart: 0, TEnd: 1}
	bar := LineSeg{Start: Point2{U: 10, V: -5}, End: Point2{U: top, V: 5}, TStart: 0.5000001, TEnd: 1}
	_, p := lineExactEnds(stem)
	q, _ := lineExactEnds(bar)
	corner, ok := lineCorner(stem, bar, p, q)
	require.True(t, ok)

	// The tilted support meets v = 0 halfway along its recorded endpoints.
	want := new(big.Rat).Add(big.NewRat(10, 1), new(big.Rat).Quo(new(big.Rat).Sub(new(big.Rat).SetFloat64(top), big.NewRat(10, 1)), big.NewRat(2, 1)))
	require.Zero(t, corner.U.Cmp(want))
	require.Zero(t, corner.V.Sign())
	require.NotZero(t, corner.U.Cmp(p.U), `the stem's own end is off the bar`)
	require.NotZero(t, corner.U.Cmp(q.U), `the bar's cut start is off the crossing`)
}

// TestLineCornerFallsBackToChord leaves a junction to the chord where the
// crossing is not near it: two supports that never cross, and two supports
// so nearly parallel that their crossing lies 1e4 mm from a 1e-9 mm gap.
//
// Shown-to-fail: without lineCorner's reach gate the slanted pair closes at
// its distant crossing.
func TestLineCornerFallsBackToChord(t *testing.T) {
	t.Parallel()
	a := LineSeg{Start: Point2{U: 0, V: 0}, End: Point2{U: 10, V: 0}, TStart: 0, TEnd: 1}
	parallel := LineSeg{Start: Point2{U: 10, V: 1e-9}, End: Point2{U: 20, V: 1e-9}, TStart: 0, TEnd: 1}
	slanted := LineSeg{Start: Point2{U: 10, V: 1e-9}, End: Point2{U: 20, V: 1e-9 + 1e-12}, TStart: 0, TEnd: 1}
	for _, b := range []LineSeg{parallel, slanted} {
		_, p := lineExactEnds(a)
		q, _ := lineExactEnds(b)
		_, ok := lineCorner(a, b, p, q)
		require.False(t, ok)
	}
}
