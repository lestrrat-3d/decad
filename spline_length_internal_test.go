package decad

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/sketch/geom"
	"github.com/stretchr/testify/require"
)

// The bracket is only useful if it both ENCLOSES the true length and NARROWS
// with subdivision. A dense polyline is the falsifier for the first: it must
// land inside every depth's interval, and it must not be used as the answer.

func denseSplineLength(t *testing.T, coords [][2]float64) float64 {
	t.Helper()
	ring, err := geom.SampleCubicBSpline(coords, 400000)
	require.NoError(t, err)
	total := 0.0
	for i := 0; i+1 < len(ring); i++ {
		total += math.Hypot(ring[i+1][0]-ring[i][0], ring[i+1][1]-ring[i][1])
	}
	return total
}

func TestFreeformArcLengthBracketEnclosesAndNarrows(t *testing.T) {
	control := []Point2{{U: 0, V: 0}, {U: 1, V: 2}, {U: 3, V: 2}, {U: 4, V: 0}, {U: 6, V: 1}, {U: 7, V: -2}}
	coords := make([][2]float64, len(control))
	for i, point := range control {
		coords[i] = [2]float64{point.U, point.V}
	}
	spans, err := splineBezierSpans(SplineSeg{Control: control, TStart: 0, TEnd: 1}, &freeformWork{})
	require.NoError(t, err)

	reference := denseSplineLength(t, coords)

	previous := math.Inf(1)
	for _, depth := range []int{0, 2, 4, 6, 8, 10} {
		lo, hi := 0.0, 0.0
		for _, span := range spans {
			spanLo, spanHi := spanLengthBracket(span, depth)
			lo += spanLo
			hi += spanHi
		}
		require.LessOrEqual(t, lo, reference, "depth %d lower bound must not exceed the true length", depth)
		require.GreaterOrEqual(t, hi, reference, "depth %d upper bound must not fall below the true length", depth)
		width := hi - lo
		require.Less(t, width, previous, "depth %d must narrow the bracket", depth)
		previous = width
	}
	// At full depth the bracket is ~1.5e-6 mm on an 11 mm curve — a relative
	// width near 1e-7, far below any tolerance a caller would set.
	require.Less(t, previous, 1e-5, "the bracket closes to well under 10 nm at full depth")
	require.Less(t, previous/reference, 1e-6, "the relative bracket width closes past 1e-6")
}

func TestFreeformArcLengthReportsPositiveBound(t *testing.T) {
	control := []Point2{{U: 0, V: 0}, {U: 1, V: 2}, {U: 3, V: 2}, {U: 4, V: 0}}
	spans, err := splineBezierSpans(SplineSeg{Control: control, TStart: 0, TEnd: 1}, &freeformWork{})
	require.NoError(t, err)

	value, bound, err := freeformArcLength(spans, &freeformWork{})
	require.NoError(t, err)
	require.Positive(t, value)
	require.Positive(t, bound, "an arc length is never exact, so its bound is never zero")

	coords := [][2]float64{{0, 0}, {1, 2}, {3, 2}, {4, 0}}
	reference := denseSplineLength(t, coords)
	require.InDelta(t, reference, value, bound+1e-9, "the reported interval encloses the true length")
}

// A straight-line degenerate case pins the directed rounding: the exact length
// of the unit-diagonal chord is irrational, so the lower bound must sit at or
// below it and the upper at or above.
func TestDirectedSqrtBracketsIrrationalLength(t *testing.T) {
	a := ratPoint{u: mustRatOf(0), v: mustRatOf(0)}
	b := ratPoint{u: mustRatOf(1), v: mustRatOf(1)}
	squared := ratSquaredDistance(a, b)

	lo := ratSqrtDown(squared)
	hi := ratSqrtUp(squared)
	require.LessOrEqual(t, lo*lo, 2.0)
	require.GreaterOrEqual(t, hi*hi, 2.0)
	require.LessOrEqual(t, lo, hi)
	require.InDelta(t, math.Sqrt2, lo, 1e-15)
}

// Every consumer that has no free-form construction must refuse a free-form
// walk rather than take its line branch. walkElem is the shared 2D conversion,
// so its refusal covers the wall survey, the section audit and the clearance
// trims at once.
func TestFreeformWalkRefusedByAnalyticConsumers(t *testing.T) {
	control := []Point2{{U: 0, V: 0}, {U: 1, V: 2}, {U: 3, V: 2}, {U: 4, V: 0}}
	walk, err := walkOf(SplineSeg{Control: control, TStart: 0, TEnd: 1})
	require.NoError(t, err, "a Tier A segment resolves into a walk")
	require.Equal(t, walkFreeform, walk.kind)
	require.False(t, walk.isLine(), "a free-form walk is not a line")
	require.False(t, walk.isCircular(), "a free-form walk is not circular")

	_, ok := walkElem(walk)
	require.False(t, ok, "there is no 2D boundary element for a free-form walk yet")

	err = requireAnalyticWalk(walk, "the test consumer")
	require.Error(t, err)
	require.ErrorIs(t, err, ErrUnsupported)
}
