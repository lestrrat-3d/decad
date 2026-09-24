package decad_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is docs/loft-design.md §16's public-surface tests for the first
// chain-loft increment — docs/surface-design.md §15's T196 through T199:
// LoftChain over a LineSeg-only correspondence between two open walks on
// exactly parallel planes. A curved pair, and a plane pair that is not exactly
// parallel or sits on the from-plane's negative side, stay refused.

// chainLoftPair builds two open walks on parallel planes gap mm apart, each
// from the plane-local points handed in, and returns both sketches with their
// one published chain. Every point is fixed, so the solver moves nothing and
// the published walk is the one the caller drew.
func chainLoftPair(tb testing.TB, gap float64, lower, upper [][2]float64) (*sketch.Sketch, *sketch.Chain, *sketch.Sketch, *sketch.Chain) {
	tb.Helper()
	w := sketch.NewWorld()
	top, err := w.CreateOffsetPlane(w.XY(), gap)
	require.NoError(tb, err)
	s0, ch0 := openWalkSketch(tb, w.XY(), w, lower)
	s1, ch1 := openWalkSketch(tb, top, w, upper)
	return s0, ch0, s1, ch1
}

// openWalkSketch draws one open polyline on plane and returns its sketch and
// the single chain sketch publishes for it.
func openWalkSketch(tb testing.TB, plane *sketch.Plane, w *sketch.World, pts [][2]float64) (*sketch.Sketch, *sketch.Chain) {
	tb.Helper()
	s, err := w.CreateSketch(plane)
	require.NoError(tb, err)
	points := make([]*sketch.Point, len(pts))
	for i, p := range pts {
		points[i] = s.CreatePoint(p[0], p[1])
		s.Fix(points[i])
	}
	for i := range len(points) - 1 {
		s.CreateLine(points[i], points[i+1])
	}
	_, err = s.Solve(tb.Context())
	require.NoError(tb, err)
	require.Empty(tb, s.Profiles(), "an open polyline closes no region")
	chains := s.Chains()
	require.Len(tb, chains, 1, "one connected open run")
	require.Len(tb, chains[0].Edges, len(pts)-1)
	return s, chains[0]
}

// TestLoftChainOneCellRibbonPinsTheSideToExtrudeChain is
// docs/surface-design.md's T196: two 40 mm line walks on parallel planes 10 mm
// apart rule into a two-triangle ribbon. Beside the structural readings, the
// row's own assertion is that every wall normal equals the normal
// ExtrudeChain's wall over the SAME recorded segment publishes — which is what
// pins docs/loft-design.md §16.2's stated positive side, T x N0, to the landed
// Table G rule rather than to this build's own winding convention.
func TestLoftChainOneCellRibbonPinsTheSideToExtrudeChain(t *testing.T) {
	t.Parallel()
	s0, ch0, s1, ch1 := chainLoftPair(t, 10,
		[][2]float64{{0, 0}, {40, 0}},
		[][2]float64{{0, 0}, {40, 0}},
	)

	doc := decad.New()
	body, err := doc.LoftChain(t.Context(), s0, ch0, s1, ch1)
	require.NoError(t, err)

	require.Equal(t, decad.BodySheet, body.Kind())
	require.False(t, body.IsSolid())
	_, err = body.Volume()
	require.ErrorIs(t, err, decad.ErrNotSolid)
	_, err = body.Centroid()
	require.ErrorIs(t, err, decad.ErrNotSolid)

	require.Len(t, body.Faces(), 2, "one chord cell builds two flat wall triangles")

	// The two walk rims plus the two END rungs: 2n + 2 at n == 1.
	free, err := decad.Edges(decad.Free()).Exactly(4).SelectEdges(body)
	require.NoError(t, err)
	for _, e := range free {
		require.True(t, e.IsFree())
	}

	area, err := body.Area()
	require.NoError(t, err)
	require.Equal(t, 400.0, area.Value.Base(), "a 40 x 10 mm ribbon is two 200 mm² triangles")
	require.Equal(t, decad.Approximate, area.Exactness,
		"docs/loft-design.md §8: a triangle area is a square root of a rational, so Area is never Exact")

	decadtestBoundsChainLoft(t, body)

	// The side pin: the same from-walk extruded 10 mm along its own plane
	// normal builds one wall whose normal Table G states as T x N. Both of the
	// ribbon's triangles are coplanar with it, so all three normals agree.
	sibling := decad.New()
	ribbon, err := sibling.ExtrudeChain(s0, ch0, decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	require.NoError(t, err)
	require.Len(t, ribbon.Faces(), 1)
	want, err := ribbon.Faces()[0].NormalAt(r3.NewVec(20, 0, 5))
	require.NoError(t, err)
	for i, f := range body.Faces() {
		got, err := f.NormalAt(r3.NewVec(20, 0, 5))
		require.NoError(t, err)
		require.Equal(t, want.Value, got.Value,
			"chain loft wall %d must publish the same normal as the chain prism wall over the same recorded segment", i)
	}
}

// decadtestBoundsChainLoft asserts the ribbon's own box. The bound is read as a
// computed quantity rather than pinned to a literal: a LineSeg pair under the
// identity motion publishes a zero station displacement, so the box is exact.
func decadtestBoundsChainLoft(tb testing.TB, body *decad.Body) {
	tb.Helper()
	box, err := body.Bounds()
	require.NoError(tb, err)
	require.Equal(tb, r3.NewVec(0, 0, 0), box.Min)
	require.Equal(tb, r3.NewVec(40, 0, 10), box.Max)
	require.Equal(tb, decad.Exact, box.Exactness,
		"every station of an unplaced LineSeg pair is pinned, so the box carries no displacement")
	require.Zero(tb, box.Bound.Base())
}

// TestLoftChainMultiCellRibbonFreeEdgesAndArea rules a three-segment walk pair
// and asserts the edge families docs/loft-design.md §16.5 states: n bottom
// rims, n top rims and the two END rungs are free, while every diagonal and
// every INTERIOR rung bounds two triangles. It also asserts the published area
// against the analytic ruled area, which for a translated walk is the walk
// length times the gap.
func TestLoftChainMultiCellRibbonFreeEdgesAndArea(t *testing.T) {
	t.Parallel()
	pts := [][2]float64{{0, 0}, {10, 0}, {10, 8}, {22, 8}}
	s0, ch0, s1, ch1 := chainLoftPair(t, 5, pts, pts)

	doc := decad.New()
	body, err := doc.LoftChain(t.Context(), s0, ch0, s1, ch1)
	require.NoError(t, err)

	require.Len(t, body.Faces(), 6, "three chord cells build two triangles each")
	_, err = decad.Edges(decad.Free()).Exactly(8).SelectEdges(body)
	require.NoError(t, err, "2n + 2 free edges at n == 3")

	interior := 0
	for _, e := range body.Edges() {
		if !e.IsFree() {
			interior++
			require.Len(t, e.Faces(), 2)
		}
	}
	require.Equal(t, 5, interior, "three diagonals plus two interior rungs")

	area, err := body.Area()
	require.NoError(t, err)
	analytic := (10.0 + 8.0 + 12.0) * 5.0
	require.LessOrEqual(t, math.Abs(area.Value.Base()-analytic), math.Max(area.Bound.Base(), 0),
		"the ruled area of a translated walk is its own length times the plane gap")
}

// TestLoftChainRefusesANonParallelOrNegativePlanePair is
// docs/surface-design.md's T197: docs/loft-design.md §16.2's gate refuses a
// plane pair that is not exactly parallel, and one whose to-plane sits on the
// from-plane's negative side. Each message is read, not only its sentinel, so
// both refusals are known to fire at this gate rather than at a later one.
//
// Shown-to-fail: removing chainLoftPlaneSideGate's negative-side arm turns the
// below-case refusal below red, and the ribbon it then builds publishes every
// wall normal as the NEGATION of the chain prism's over the same recorded
// segment — measured at (20, 0, -5): (0, 1, 0) against ExtrudeChain's
// (0, -1, 0). That negation is the whole reason the arm exists, since the side
// a chain loft states is pinned to Table G's T x N and to nothing else. See
// this PR's report for the exact failure text.
func TestLoftChainRefusesANonParallelOrNegativePlanePair(t *testing.T) {
	t.Parallel()
	pts := [][2]float64{{0, 0}, {40, 0}}

	// A tilted second plane: parallel is an exact rational comparison, so a
	// plane the caller tilted at all refuses.
	w := sketch.NewWorld()
	s0, ch0 := openWalkSketch(t, w.XY(), w, pts)
	tiltedFrame, err := r3.NewFrame(r3.NewVec(0, 0, 10), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 1))
	require.NoError(t, err)
	tilted, err := w.CreatePlaneFromFrame(tiltedFrame)
	require.NoError(t, err)
	s1, ch1 := openWalkSketch(t, tilted, w, pts)

	doc := decad.New()
	_, err = doc.LoftChain(t.Context(), s0, ch0, s1, ch1)
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.ErrorContains(t, err, "not exactly parallel")
	require.Empty(t, doc.Bodies())

	// A second plane 10 mm BELOW the first: exactly parallel, wrong side.
	belowS0, belowCh0, belowS1, belowCh1 := chainLoftPair(t, -10, pts, pts)
	_, err = doc.LoftChain(t.Context(), belowS0, belowCh0, belowS1, belowCh1)
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.ErrorContains(t, err, "positive side")
	require.Empty(t, doc.Bodies())

	// Swapping the argument order puts the same two planes the right way
	// round, which pins the refusal to the ORDER rather than to the pair.
	body, err := doc.LoftChain(t.Context(), belowS1, belowCh1, belowS0, belowCh0)
	require.NoError(t, err)
	require.Equal(t, decad.BodySheet, body.Kind())
}

// TestLoftChainCrossingCorrespondenceIsRefused is docs/surface-design.md's
// T198, in the two legs the behaviour actually has.
//
// The first leg is the load-bearing one: docs/loft-design.md §6's crossing
// audit is a pure function of the assembled vertex and triangle sets, so it
// transfers to an open ribbon with the cap triangles simply absent. A
// correspondence whose walls pass through one another is proven and refused
// with ErrDegenerate, and the document is unchanged.
//
// The second leg records what an OPPOSED pair does, which is not the same
// question. Two walks sketch publishes from opposite ends rule each side
// against the other's reversed walk, and the ribbon that produces is twisted —
// but twisted is not the same as self-crossing, and this pair builds. decad
// never guesses intent: the correspondence the two records state is the one it
// rules, and the audit refuses a crossing rather than a correspondence the
// caller did not mean.
//
// Producing an opposed pair at all takes work, because sketch's own canonical
// walk direction starts every walk at its lexicographically smaller free end:
// two walks whose free ends hold the SAME plane-local coordinates therefore
// always publish in the same direction, whatever order the caller drew them
// in. An opposed pair needs end coordinates whose lexicographic order
// disagrees with the geometric correspondence, which is what the second leg's
// two end sets arrange.
func TestLoftChainCrossingCorrespondenceIsRefused(t *testing.T) {
	t.Parallel()

	// An open rectangle against its own mirrored walk: rung 1 runs
	// (20,0,0)->(0,20,10) and rung 3 runs (0,20,0)->(20,0,10), so the two
	// cells carrying them meet at the middle of the ribbon.
	s0, ch0, s1, ch1 := chainLoftPair(t, 10,
		[][2]float64{{0, 0}, {20, 0}, {20, 20}, {0, 20}},
		[][2]float64{{0, 0}, {0, 20}, {20, 20}, {20, 0}},
	)
	doc := decad.New()
	_, err := doc.LoftChain(t.Context(), s0, ch0, s1, ch1)
	require.ErrorIs(t, err, decad.ErrDegenerate)
	require.ErrorContains(t, err, "share no recorded vertex, but make contact")
	require.Empty(t, doc.Bodies())

	// An opposed pair that does not cross: the lower walk is published from
	// y = 0 and the upper from y = 20, the end above the lower walk's own
	// finish.
	oppS0, oppCh0, oppS1, oppCh1 := chainLoftPair(t, 10,
		[][2]float64{{0, 0}, {2, 10}, {5, 20}},
		[][2]float64{{5, 0}, {3, 10}, {0, 20}},
	)
	require.Equal(t, 0.0, oppCh0.Edges[0].Polyline[0][1], "the lower walk starts at y = 0")
	require.Equal(t, 20.0, oppCh1.Edges[0].Polyline[0][1], "the upper walk starts at y = 20")

	twisted, err := doc.LoftChain(t.Context(), oppS0, oppCh0, oppS1, oppCh1)
	require.NoError(t, err, "an opposed walk pair is twisted, not necessarily self-crossing")
	require.Equal(t, decad.BodySheet, twisted.Kind())
	require.Len(t, twisted.Faces(), 4)
}

// TestLoftChainRefusesAMismatchedOrCurvedPairing is docs/surface-design.md's
// T199: a segment-count mismatch has no one-to-one pairing, a curved pair has
// no side proof for its computed stations yet, and the coplanar pose is the
// closed loft's own ErrDegenerate. Each message is read so the three refusals
// are known to be distinct.
func TestLoftChainRefusesAMismatchedOrCurvedPairing(t *testing.T) {
	t.Parallel()
	doc := decad.New()

	// A segment-count mismatch.
	s0, ch0, s1, ch1 := chainLoftPair(t, 10,
		[][2]float64{{0, 0}, {20, 0}, {40, 0}},
		[][2]float64{{0, 0}, {40, 0}},
	)
	_, err := doc.LoftChain(t.Context(), s0, ch0, s1, ch1)
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.ErrorContains(t, err, "segment-count mismatch")

	// A same-kind ArcSeg pair: admitted by Table P, staged here.
	w := sketch.NewWorld()
	top, err := w.CreateOffsetPlane(w.XY(), 10)
	require.NoError(t, err)
	arcS0, arcCh0 := arcWalkSketch(t, w.XY(), w)
	arcS1, arcCh1 := arcWalkSketch(t, top, w)
	_, err = doc.LoftChain(t.Context(), arcS0, arcCh0, arcS1, arcCh1)
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.ErrorContains(t, err, "LineSeg pair only")

	// The coplanar pose: the same chain twice, which is S5's own refusal.
	flatS, flatCh, _, _ := chainLoftPair(t, 10,
		[][2]float64{{0, 0}, {40, 0}},
		[][2]float64{{0, 0}, {40, 0}},
	)
	_, err = doc.LoftChain(t.Context(), flatS, flatCh, flatS, flatCh)
	require.ErrorIs(t, err, decad.ErrDegenerate)
	require.ErrorContains(t, err, "same geometric plane")

	require.Empty(t, doc.Bodies())
}

// arcWalkSketch draws a one-arc open walk on plane, so its single recorded
// segment is an ArcSeg rather than a LineSeg.
func arcWalkSketch(tb testing.TB, plane *sketch.Plane, w *sketch.World) (*sketch.Sketch, *sketch.Chain) {
	tb.Helper()
	s, err := w.CreateSketch(plane)
	require.NoError(tb, err)
	center := s.CreatePoint(0, 0)
	start := s.CreatePoint(10, 0)
	end := s.CreatePoint(0, 10)
	s.Fix(center)
	s.Fix(start)
	s.Fix(end)
	s.CreateArc(center, start, end)
	_, err = s.Solve(tb.Context())
	require.NoError(tb, err)
	chains := s.Chains()
	require.Len(tb, chains, 1)
	return s, chains[0]
}

// TestLoftChainPlacedRibbonKeepsItsAreaAndChargesItsVertices places an
// admitted ribbon by a motion that is neither axis-aligned nor origin-centred.
// A rigid motion moves no area, and every vertex then carries the placement's
// own proven rounding, so the box stops being exact — the two readings
// together are what show the payload's replay charges its own delta rather
// than reproducing the unplaced build.
func TestLoftChainPlacedRibbonKeepsItsAreaAndChargesItsVertices(t *testing.T) {
	t.Parallel()
	pts := [][2]float64{{0, 0}, {40, 0}}
	s0, ch0, s1, ch1 := chainLoftPair(t, 10, pts, pts)

	doc := decad.New()
	body, err := doc.LoftChain(t.Context(), s0, ch0, s1, ch1)
	require.NoError(t, err)
	before, err := body.Area()
	require.NoError(t, err)

	spin, err := r3.RotationAround(r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 1), units.Degrees(30))
	require.NoError(t, err)
	placed, err := body.Placed(t.Context(), spin)
	require.NoError(t, err)

	after, err := placed.Area()
	require.NoError(t, err)
	require.InDelta(t, before.Value.Base(), after.Value.Base(), math.Max(after.Bound.Base(), 1e-9),
		"a rigid motion moves no area")

	box, err := placed.Bounds()
	require.NoError(t, err)
	require.Equal(t, decad.Approximate, box.Exactness)
	require.Greater(t, box.Bound.Base(), 0.0,
		"a placed build holds every vertex only within the motion's own proven rounding")
}
