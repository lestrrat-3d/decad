package decad

import (
	"math"
	"testing"
	"time"

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

// A control net that has collapsed to a single point brackets to [0, 0], and a
// zero half width IS the Exact zero §6.1 forbids. It must refuse instead — with
// the same sentinel the moments path already gives the identical record, so the
// two paths agree on what the record is.
func TestFreeformCoincidentControlNetRefused(t *testing.T) {
	for _, tc := range []struct {
		name    string
		control []Point2
	}{
		{name: "at the origin", control: []Point2{{}, {}, {}, {}}},
		{
			name: "away from the origin",
			control: []Point2{
				{U: 5, V: 7}, {U: 5, V: 7}, {U: 5, V: 7}, {U: 5, V: 7},
			},
		},
	} {
		t.Run(tc.name, func(t *testing.T) {
			seg := SplineSeg{Control: tc.control, TStart: 0, TEnd: 1}

			spans, err := splineBezierSpans(seg, &freeformWork{})
			require.NoError(t, err, "the record itself converts")
			_, _, err = freeformArcLength(spans, &freeformWork{})
			require.ErrorIs(t, err, ErrDegenerate)
			require.Contains(t, err.Error(), "coincide")

			_, err = walkOf(seg)
			require.ErrorIs(t, err, ErrDegenerate, "no walk carries a zero length bound")

			_, _, _, err = validateFreeformMomentSegment(seg, &freeformWork{})
			require.ErrorIs(t, err, ErrDegenerate, "the moments path refuses the same record")
		})
	}
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

// The bracket's charge must grow with the span DEGREE, not only with the fixed
// subdivision depth: one split blends every de Casteljau pair, so a charge that
// counts leaves alone admits a span whose splits run for hours.
func TestFreeformBracketCostGrowsWithDegree(t *testing.T) {
	require.Less(t, freeformBracketCost(4), freeformWorkLimit, "a cubic span's bracket is affordable")
	require.Greater(t, freeformBracketCost(8), 3*freeformBracketCost(4),
		"doubling the degree more than triples the charge")
	require.Equal(t, freeformCostCeiling, freeformBracketCost(1025), "the finding's degree-1024 span")
	require.Equal(t, freeformCostCeiling, freeformBracketCost(1<<20), "an absurd degree saturates, never wraps")
	require.Less(t, uint64(1)<<freeformLengthDepth, freeformWorkLimit,
		"a charge counting only the leaves is what let that degree through")
}

// The measured defect: a validator-accepted single-span NURBS of high degree
// subdivided 1023 times, each split blending half a million rational pairs,
// with no charge past the leaf count. The preflight must refuse it before the
// first split — and public Extrude reaches this through walkOf, so the walk must
// refuse it too rather than run the bracket on the way to its staging refusal.
func TestWideSpanBracketRefusesBeforeSubdividing(t *testing.T) {
	const degree = 1024
	seg := oneSpanNURBS(degree)
	require.NoError(t, validateNURBSSegment(seg), "the record itself is well formed")

	work := &freeformWork{}
	spans, _, err := freeformBezierSpans(seg, work)
	require.NoError(t, err, "a single span needs no knot insertion")
	require.Len(t, spans, 1)

	start := time.Now()
	_, _, err = freeformArcLength(spans, work)
	require.ErrorIs(t, err, ErrUnsupported)
	require.Contains(t, err.Error(), "work budget")
	require.Less(t, time.Since(start), 10*time.Second, "the refusal precedes the subdivision")

	start = time.Now()
	_, err = walkOf(seg)
	require.ErrorIs(t, err, ErrUnsupported)
	require.Contains(t, err.Error(), "work budget")
	require.Less(t, time.Since(start), 10*time.Second, "the walk refuses on the same preflight")
}

// oneSpanNURBS is a valid non-rational clamped NURBS of the given degree over
// degree+1 control points: no interior knot, so it converts to exactly one
// Bézier span and the conversion pass itself charges almost nothing.
func oneSpanNURBS(degree int) NURBSSeg {
	control := make([]Point2, degree+1)
	weights := make([]float64, degree+1)
	knots := make([]float64, 0, 2*(degree+1))
	for i := range control {
		control[i] = Point2{U: float64(i), V: float64(i % 7)}
		weights[i] = 1
	}
	for range degree + 1 {
		knots = append(knots, 0)
	}
	for range degree + 1 {
		knots = append(knots, 1)
	}
	return NURBSSeg{Degree: degree, Control: control, Knots: knots, Weights: weights, TStart: 0, TEnd: 1}
}
